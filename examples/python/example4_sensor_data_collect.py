"""
注意：运行本示例时，请确认机器人的左臂相机驱动`/data/galbot/bin/left_arm_camera_capture`
    和雷达驱动`/data/galbot/bin/service_livox_capture`已加载;
"""
try:
    from galbot_sdk.g1  import GalbotRobot, SensorType
except ImportError:
    print("import galbot_sdk failed, please install it first or check if it is in the PYTHONPATH")
    exit(1)

import time
import open3d as o3d
from typing import Dict
import cv2
import numpy as np


def convert_pointcloud(cloud):
    """
    Convert cloud dict to NumPy array dictionary

    Args:
        pointcloud_msg: PointCloud2 protobuf message object

    Returns:
        Dictionary: {field_name: NumPy array}
        - Single-element fields: shape (N,)
        - Multi-element fields: shape (N, count) or (N,)
        - N = width * height (total number of points)
    """

    if not cloud:
        return {}

    num_points = cloud["height"] * cloud["width"]
    if num_points == 0:
        return {}

    DTYPE_MAP = {
        1: np.int8,
        2: np.uint8,
        3: np.int16,
        4: np.uint16,
        5: np.int32,
        6: np.uint32,
        7: np.float32,
        8: np.float64
    }
    dtype_list = []
    for field in cloud["fields"]:
        # Get base data type
        np_dtype_class = DTYPE_MAP.get(field["datatype"])
        if np_dtype_class is None:
            raise ValueError(f"Unsupported data type: {field['datatype']}")

        dtype_inst = np.dtype(np_dtype_class)

        # Handle byte order (endianness)
        if dtype_inst.itemsize > 1:
            byteorder = '>' if cloud["is_bigendian"] else '<'
            dtype_inst = dtype_inst.newbyteorder(byteorder)

        # Add to dtype list
        if field["count"] == 1:
            dtype_list.append((field["name"], dtype_inst))
        else:
            # Multi-element fields (e.g., rgb)
            dtype_list.append((field["name"], dtype_inst, field["count"]))

    # Create structured dtype
    dtype = np.dtype(dtype_list)

    # Data integrity check
    expected_size = num_points * cloud["point_step"]
    if len(cloud["data"]) < expected_size:
        raise ValueError(
            f"Insufficient data length: expected {expected_size} bytes, "
            f"actual {len(cloud['data'])} bytes"
        )

    # Create NumPy structured array from binary data
    # count parameter ensures only expected number of points are read
    arr = np.frombuffer(cloud["data"], dtype=dtype, count=num_points)

    # Convert to regular dictionary (copy data to avoid modifying original)
    result = {}
    for field in cloud["fields"]:
        field_data = arr[field["name"]]

        # Handle shape of multi-element fields
        if field["count"] == 1:
            result[field["name"]] = field_data.copy()
        else:
            # Keep original shape or flatten, choose according to needs
            result[field["name"]] = field_data.copy()

    return result


def get_xyz_array(pointcloud_dict: Dict[str, np.ndarray], 
                remove_nan: bool = False) -> np.ndarray:
    """
    Extract XYZ coordinate array from converted point cloud dictionary

    Args:
        pointcloud_dict: Dictionary returned by pointcloud2_to_numpy()
        remove_nan: Whether to remove points containing NaN (for FLOAT32/FLOAT64 types)

    Returns:
        Nx3 point coordinate array
    """
    required = ['x', 'y', 'z']
    if not all(k in pointcloud_dict for k in required):
        raise ValueError("Point cloud data missing required xyz fields")

    points = np.stack([pointcloud_dict['x'], 
                    pointcloud_dict['y'], 
                    pointcloud_dict['z']], axis=1)

    if remove_nan:
        mask = ~np.isnan(points).any(axis=1)
        points = points[mask]

    return points

def save_xyz_to_pcd(xyz_array: np.ndarray, filename: str, binary: bool = False) -> None:
    """
    Save XYZ coordinates to PCD file format (simplest option for coordinate-only data)

    Args:
        xyz_array: Nx3 array of XYZ coordinates
        filename: Output PCD file path
        binary: If True, saves in binary format; otherwise ASCII
    """
    if xyz_array.ndim != 2 or xyz_array.shape[1] != 3:
        raise ValueError(f"xyz_array must have shape (N, 3), got {xyz_array.shape}")

    num_points = xyz_array.shape[0]
    header = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",  # F = float32
        "COUNT 1 1 1",
        f"WIDTH {num_points}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {num_points}",
        f"DATA {'binary' if binary else 'ascii'}"
    ]

    if binary:
        with open(filename, 'wb') as f:
            f.write(('\n'.join(header) + '\n').encode('ascii'))
            f.write(xyz_array.astype(np.float32).tobytes())
    else:
        with open(filename, 'w') as f:
            f.write('\n'.join(header) + '\n')
            np.savetxt(f, xyz_array, fmt='%f')


def decode_compressed_image(compressed_image, camera_info={}):
    """
    decode CompressedImage image

    Params:
        compressed_image: image dict, keys:[header, format, data, "depth_scale"]

    Returns:
        numpy.ndarray: decoded image
    """
    image_data = compressed_image["data"]
    if compressed_image["format"] == "rgb8":
        return decode_rgb_image(image_data)
    elif compressed_image["format"] == "16UC1":
        return decode_depth_image(image_data, compressed_image["depth_scale"], camera_info)
    else:
        raise ValueError(f"Unsupport data format: {compressed_image['format']}")

def decode_rgb_image(image_data):
    """decode rgb image"""
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Fail to Decode RGB Image")
    return img

def decode_depth_image(image_data, depth_scale, camera_info):
    """decode depth image"""
    depth_img = np.frombuffer(image_data, dtype=np.uint16).copy()

    if not camera_info:
        depth_img = depth_img.reshape((720, 1280))
    else:
        depth_img = depth_img.reshape((camera_info["height"], camera_info["width"]))
    depth_img = depth_img.astype(np.float32) / depth_scale

    return depth_img

def depth_rgb_to_pointcloud(depth, rgb, fx, fy, cx, cy, depth_scale=1.0):
    """
    depth: (H, W) 深度图
    rgb:   (H, W, 3) RGB图
    depth_scale: 深度单位缩放（mm->m 用 0.001）
    """
    assert depth.shape[:2] == rgb.shape[:2]

    H, W = depth.shape

    # 像素坐标
    u, v = np.meshgrid(np.arange(W), np.arange(H))

    # 深度（转为米）
    Z = depth.astype(np.float32) * depth_scale

    # 过滤无效深度
    valid = Z > 0

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    points = np.stack((X, Y, Z), axis=-1)
    colors = rgb.astype(np.float32) / 255.0

    # 只保留有效点
    points = points[valid]
    colors = colors[valid]

    return points, colors

def check_robot_safety():
    """检查机器人是否安全"""
    # 提示注意事项
    print("⚠️  注意: 1.请确保机器人的急停按钮已放开; 2.请确保机器人前后左右无遮挡，避免发生意外情况。")
    while True:
        key = input("请确认机器人急停按钮已放开，且无遮挡，是否继续（y/n）...")
        if key == 'y':
            print("用户确认，继续运行...")
            break
        elif key == 'n':
            print("用户未确认，程序退出...")
            exit(1)
        else:
            print("输入错误，请输入'y'或'n'")

def main():
    check_robot_safety()
    try:
        # 获取 GalbotRobot 的单例并初始化
        robot = GalbotRobot.get_instance()

        # 获取左臂的RGB图像和深度图像，右臂的深度图像，底座的激光雷达数据，躯干的IMU数据
        enable_sensor_set = {SensorType.LEFT_ARM_CAMERA, # 左臂深度相机
                            SensorType.LEFT_ARM_DEPTH_CAMERA, # 左臂RGB相机
                            SensorType.BASE_LIDAR, # 底座激光雷达
                            SensorType.TORSO_IMU} # 躯干IMU传感器

        # 为节省资源开销，只有初始化中传入的相机与雷达传感器可获取数据
        robot.init(enable_sensor_set)
        print("初始化成功")
        # 程序立即启动，稍等数据就绪时间
        time.sleep(5)

        # 获取左臂的RGB图像
        rgb_image_data = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA)
        if not rgb_image_data:
            print("No rgb image data!")
        else:
            print("get rgb image suceess")
            print(rgb_image_data['header'])
            img = decode_compressed_image(rgb_image_data)
            
            # 保存RGB图像
            cv2.imwrite("rgb_image_data.jpg", img)
            # 可视化RGB图像
            cv2.namedWindow("rgb image", cv2.WINDOW_NORMAL)
            cv2.imshow("rgb image", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        # 获取左臂的深度图像
        depth_data = robot.get_depth_data(SensorType.LEFT_ARM_DEPTH_CAMERA)
        if not depth_data:
            print("No depth_data!")
        else:
            print("get depth data suceess")
            print(depth_data['header'])
            depth_img_raw = decode_compressed_image(depth_data)
            depth_img = cv2.normalize(depth_img_raw, None, 0, 255, cv2.NORM_MINMAX) # 归一化，将深度值映射到0-1范围
            depth_img = depth_img.astype(np.uint8)
            
            # 保存深度图
            cv2.imwrite("depth_data.jpg", depth_img)
            # 可视化深度图
            cv2.namedWindow("depth image", cv2.WINDOW_NORMAL)
            cv2.imshow("depth image", depth_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        # 获取底座的激光雷达数据
        lidar_data = robot.get_lidar_data(SensorType.BASE_LIDAR)
        if not lidar_data:
            print("No lidar data!")
        else:
            pointcloud_dict = convert_pointcloud(lidar_data)
            xyz_points = get_xyz_array(pointcloud_dict)
            save_xyz_to_pcd(xyz_points, "output_xyz.pcd")
            print(pointcloud_dict)
            print("get lidar data success")

            # 可视化雷达点云
            vis = o3d.visualization.Visualizer()
            vis.create_window()
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz_points)
            vis.add_geometry(pcd)
            vis.run()
            vis.destroy_window()
        
        # 获取躯干的IMU数据
        imu_data = robot.get_imu_data(SensorType.TORSO_IMU)
        if not imu_data:
            print("No imu data!")
        else:
            print("get imu data suceess")
        
        try:
            camera_info = robot.get_camera_intrinsic(SensorType.LEFT_ARM_DEPTH_CAMERA)
            if not camera_info:
                print("No camera info!")
            else:
                print(camera_info)
        except Exception as e:
            camera_info = {
                "width": 1280,
                "height": 720,
                "distortion_model": "plumb_bob",
                "camera_type": "D405",
                "k": [653.4349365234375, 0.0, 639.95159912109375, 
                    0.0, 652.48858642578125, 365.29425048828125, 
                    0.0, 0.0, 1.0],
            }
        
        # 深度图和RGB图转点云并保存
        if depth_data and rgb_image_data:
            points, colors = depth_rgb_to_pointcloud(
                depth_img_raw,
                img,
                fx=camera_info['k'][0],
                fy=camera_info['k'][4],
                cx=camera_info['k'][2],
                cy=camera_info['k'][5],
                depth_scale=0.1   # 如果 depth 是 mm，则设置为 0.001
            )
            save_xyz_to_pcd(points, "left_arm_camera_pointcloud.pcd", binary=True)
            print(f"RGB融合深度图点云保存至left_arm_camera_pointcloud.pcd, 点云数量: {points.shape[0]}")
    except Exception as e:
        print(f"❌ 出现异常: {e}")
    finally:
        # 主动发出SIGINT退出信号
        robot.request_shutdown()
        # 等待进入shutdown状态
        robot.wait_for_shutdown()
        # 进行SDK资源释放
        robot.destroy()
        print('资源释放成功')


if __name__=="__main__":
    main()