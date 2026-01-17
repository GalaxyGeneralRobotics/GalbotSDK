"""
Note: When running this example, please confirm that the robot's left arm camera driver `/data/galbot/bin/left_arm_camera_capture`
    and radar driver `/data/galbot/bin/service_livox_capture` have been loaded;
"""
try:
    from galbot_sdk.g1  import GalbotRobot, SensorType
except ImportError:
    print("import galbot_sdk failed, please install it first or check if it is in the PYTHONPATH")
    exit(1)

import os

try:
    import open3d as o3d
except ImportError:
    os.system("pip install open3d")
    import open3d as o3d

try:
    import cv2
except ImportError:
    os.system("pip install opencv-python")
    import cv2

try:
    import numpy as np
except ImportError:
    os.system("pip install numpy")
    import numpy as np

import time
from typing import Dict


def convert_pointcloud(cloud):
    """
    Convert cloud dict to NumPy array dictionary

    Parameters:
        cloud (dict): PointCloud2 protobuf message object

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

    Parameters:
        pointcloud_dict (Dict[str, np.ndarray]): Dictionary returned by pointcloud2_to_numpy()
        remove_nan (bool, optional): Whether to remove points containing NaN (for FLOAT32/FLOAT64 types). Defaults to False.

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

    Parameters:
        xyz_array (np.ndarray): Nx3 array of XYZ coordinates
        filename (str): Output PCD file path
        binary (bool, optional): If True, saves in binary format; otherwise ASCII. Defaults to False.
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


def decode_compressed_image(compressed_image):
    """
    decode CompressedImage image

    Parameters:
        compressed_image (dict): image dict, keys:[header, format, data, "depth_scale"]

    Returns:
        numpy.ndarray: decoded image
    """
    image_data = compressed_image["data"]
    if compressed_image["format"] == "rgb8":
        return decode_rgb_image(image_data)
    elif compressed_image["format"] == "16UC1":
        return decode_depth_image(compressed_image)
    else:
        raise ValueError(f"Unsupport data format: {compressed_image['format']}")

def decode_rgb_image(image_data):
    """decode rgb image"""
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Fail to Decode RGB Image")
    return img

def decode_depth_image(image_data):
    """decode depth image"""
    depth_img = np.frombuffer(image_data["data"], dtype=np.uint16).copy()

    # Check if height and width exist
    if "height" not in image_data or "width" not in image_data:
        raise ValueError("Missing 'height' or 'width' in depth image metadata.")
    if image_data["height"] == 0 or image_data["width"] == 0:
        raise ValueError(f"Invalid 'height' ({image_data['height']}) or 'width' ({image_data['width']}) in depth image metadata.")

    # Parse depth image
    depth_img = depth_img.reshape((image_data["height"], image_data["width"]))
    depth_img = depth_img.astype(np.float32) / image_data["depth_scale"]

    return depth_img

def depth_rgb_to_pointcloud(depth, rgb, fx, fy, cx, cy, depth_scale=1.0):
    """
    Convert depth map and RGB image to point cloud

    Parameters:
        depth: (H, W) depth map
        rgb:   (H, W, 3) RGB image
        depth_scale: Depth unit scaling (use 0.001 for mm->m conversion)
    
    Returns:
        points: (N, 3) point cloud coordinate array
        colors: (N, 3) point cloud color array (0-1 range)
    """
    assert depth.shape[:2] == rgb.shape[:2]

    H, W = depth.shape

    # Pixel coordinates
    u, v = np.meshgrid(np.arange(W), np.arange(H))

    # Depth (converted to meters)
    Z = depth.astype(np.float32) * depth_scale

    # Filter invalid depths
    valid = Z > 0

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    points = np.stack((X, Y, Z), axis=-1)
    colors = rgb.astype(np.float32) / 255.0

    # Keep only valid points
    points = points[valid]
    colors = colors[valid]

    return points, colors

def check_robot_safety():
    """Check if robot is safe"""
    # Prompt important notes
    print("⚠️  Note: 1. Please ensure the emergency stop button of the robot is released; 2. Please ensure there are no obstructions around the robot to avoid unexpected situations.")
    while True:
        key = input("Please confirm that the robot's emergency stop button is released and there are no obstructions, continue? (y/n)...")
        if key == 'y':
            print("User confirmed, continuing...")
            break
        elif key == 'n':
            print("User did not confirm, exiting program...")
            exit(1)
        else:
            print("Invalid input, please enter 'y' or 'n'")

def main():
    check_robot_safety()
    try:
        # Get and initialize the GalbotRobot singleton
        robot = GalbotRobot.get_instance()

        # Get RGB and depth images from the left arm, depth images from the right arm, 
        # base LiDAR data, and torso IMU data
        enable_sensor_set = {SensorType.LEFT_ARM_CAMERA, # Left arm depth camera
                            SensorType.LEFT_ARM_DEPTH_CAMERA, # Left arm RGB camera
                            SensorType.BASE_LIDAR,} # Base LiDAR

        # To save resource overhead, only cameras and radar sensors passed during initialization can acquire data
        robot.init(enable_sensor_set)
        print("Initialization successful")
        # Program starts immediately, wait for data readiness
        time.sleep(5)

        # Get RGB image from the left arm
        rgb_image_data = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA)
        if not rgb_image_data:
            print("No rgb image data!")
        else:
            print("get rgb image suceess")
            print(rgb_image_data['header'])
            img = decode_compressed_image(rgb_image_data)
            
            # Save RGB image
            cv2.imwrite("rgb_image_data.jpg", img)
            # Visualize RGB image
            cv2.namedWindow("rgb image", cv2.WINDOW_NORMAL)
            cv2.imshow("rgb image", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        # Get depth image from the left arm
        depth_data = robot.get_depth_data(SensorType.LEFT_ARM_DEPTH_CAMERA)
        if not depth_data or "data" not in depth_data:
            print("Depth camera not ready")
        else:
            print("get depth data suceess")
            print(depth_data['header'])
            depth_img_raw = decode_compressed_image(depth_data)
            depth_img = cv2.normalize(depth_img_raw, None, 0, 255, cv2.NORM_MINMAX) # Normalize, mapping depth values to 0-1 range
            depth_img = depth_img.astype(np.uint8)

            # Save depth image
            cv2.imwrite("depth_data.jpg", depth_img)
            # Visualize depth image
            cv2.namedWindow("depth image", cv2.WINDOW_NORMAL)
            cv2.imshow("depth image", depth_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        # Get base LiDAR data
        lidar_data = robot.get_lidar_data(SensorType.BASE_LIDAR)
        if not lidar_data:
            print("No lidar data!")
        else:
            pointcloud_dict = convert_pointcloud(lidar_data)
            xyz_points = get_xyz_array(pointcloud_dict)
            save_xyz_to_pcd(xyz_points, "output_xyz.pcd")
            print(pointcloud_dict)
            print("get lidar data success")

            # Visualize LiDAR point cloud
            vis = o3d.visualization.Visualizer()
            vis.create_window()
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz_points)
            vis.add_geometry(pcd)
            vis.run()
            vis.destroy_window()
        
        # Get torso IMU data
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
        
        # Convert depth map and RGB image to point cloud and save
        if depth_data and rgb_image_data:
            points, colors = depth_rgb_to_pointcloud(
                depth_img_raw,
                img,
                fx=camera_info['k'][0],
                fy=camera_info['k'][4],
                cx=camera_info['k'][2],
                cy=camera_info['k'][5],
                depth_scale=0.1   # If depth is in mm, set to 0.001
            )
            save_xyz_to_pcd(points, "left_arm_camera_pointcloud.pcd", binary=True)
            print(f"RGB fused depth map point cloud saved to left_arm_camera_pointcloud.pcd, number of points: {points.shape[0]}")
    except Exception as e:
        print(f"❌ Exception occurred: {e}")
    finally:
        # Actively send SIGINT exit signal
        robot.request_shutdown()
        # Wait to enter shutdown state
        robot.wait_for_shutdown()
        # Release SDK resources
        robot.destroy()
        print('Resource release successful')


if __name__=="__main__":
    main()