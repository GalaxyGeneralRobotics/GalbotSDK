from galbot_sdk.g1 import GalbotRobot, SensorType
from typing import Dict
import time
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

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
enable_sensor_set = {SensorType.BASE_LIDAR}
# 为节省资源开销，只有初始化中传入的相机与雷达传感器可获取数据
robot.init(enable_sensor_set)
print("初始化成功")
# 程序立即启动，稍等数据就绪时间
time.sleep(4)

cloud = robot.get_lidar_data(SensorType.BASE_LIDAR)
if not cloud:
    print("No lidar data!")
else:
    pointcloud_dict = convert_pointcloud(cloud)
    xyz_points = get_xyz_array(pointcloud_dict)
    save_xyz_to_pcd(xyz_points, "output_xyz.pcd")
    print(pointcloud_dict)
    print("get lidar data success")

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')