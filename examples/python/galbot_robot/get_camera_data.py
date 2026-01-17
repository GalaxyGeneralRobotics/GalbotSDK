try:
    from galbot_sdk.g1  import GalbotRobot, SensorType
except ImportError:
    print("import galbot_sdk failed, please install it first or check if it is in the PYTHONPATH")
    exit(1)

import os

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

    # 检查是否存在 height 和 width
    if "height" not in image_data or "width" not in image_data:
        raise ValueError("Missing 'height' or 'width' in depth image metadata.")
    if image_data["height"] == 0 or image_data["width"] == 0:
        raise ValueError(f"Invalid 'height' ({image_data['height']}) or 'width' ({image_data['width']}) in depth image metadata.")

    # 解析深度图像
    depth_img = depth_img.reshape((image_data["height"], image_data["width"]))
    depth_img = depth_img.astype(np.float32) / image_data["depth_scale"]

    return depth_img

def main():
    robot = GalbotRobot.get_instance()

    # 获取左臂的RGB图像和深度图像，右臂的深度图像，底座的激光雷达数据，躯干的IMU数据
    enable_sensor_set = {SensorType.LEFT_ARM_CAMERA, # 左臂深度相机
                        SensorType.LEFT_ARM_DEPTH_CAMERA,} # 左臂RGB相机
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
    if not depth_data or "data" not in depth_data:
        print("Depth camera not ready")
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

    # 主动发出SIGINT退出信号
    robot.request_shutdown()
    # 等待进入shutdown状态
    robot.wait_for_shutdown()
    # 进行SDK资源释放
    robot.destroy()
    print('资源释放成功')
    
if __name__=="__main__":
    main()