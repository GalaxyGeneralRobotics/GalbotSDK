try:
    from galbot_sdk.s1 import GalbotRobot, SensorType
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

def decode_ir_image(ir_image):
    """decode IR (infrared) grayscale image.
    Supports JPEG-compressed mono8 and raw mono8 byte streams.
    """
    data = ir_image["data"]
    fmt = ir_image.get("format", "")

    # Raw mono8: plain pixel bytes, use height/width directly
    if fmt == "mono8":
        h = ir_image.get("height", 0)
        w = ir_image.get("width", 0)
        if h == 0 or w == 0:
            raise ValueError(f"decode_ir_image: height/width unavailable for mono8, got {h}x{w}")
        return np.frombuffer(data, dtype=np.uint8).reshape(h, w).copy()

    # Compressed (JPEG / PNG)
    nparr = np.frombuffer(data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError(f"decode_ir_image: imdecode failed, format='{fmt}', size={len(data)}")
    return img

def decode_depth_image(image_data):
    """decode depth image"""
    depth_img = np.frombuffer(image_data["data"], dtype=np.uint16).copy()

    # Check whether height and width exist
    if "height" not in image_data or "width" not in image_data:
        raise ValueError("Missing 'height' or 'width' in depth image metadata.")
    if image_data["height"] == 0 or image_data["width"] == 0:
        raise ValueError(f"Invalid 'height' ({image_data['height']}) or 'width' ({image_data['width']}) in depth image metadata.")

    # Parse depth image
    depth_img = depth_img.reshape((image_data["height"], image_data["width"]))
    depth_img = depth_img.astype(np.float32) / image_data["depth_scale"]

    return depth_img

def main():
    SHOW_IMAGE = False
    robot = GalbotRobot()

    # Get left arm RGB and depth images, right arm depth image, chassis lidar data, torso IMU data
    enable_sensor_set = {SensorType.LEFT_ARM_CAMERA,
                        SensorType.LEFT_ARM_DEPTH_CAMERA,
                        SensorType.LEFT_ARM_INFRA_CAMERA_1,
                        SensorType.LEFT_ARM_INFRA_CAMERA_2,}
    robot.init(enable_sensor_set)
    print("Initialization succeeded")
    # Program started, waiting for data
    time.sleep(5)
    # Get left arm RGB image
    rgb_image_data = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA)
    if not rgb_image_data:
        print("No rgb image data!")
    else:
        print("get rgb image suceess")
        print(rgb_image_data['header'])
        img = decode_compressed_image(rgb_image_data)
        
        # Save RGB image
        cv2.imwrite("rgb_image_data.jpg", img)
        # RGBimage
        if SHOW_IMAGE:
            cv2.namedWindow("rgb image", cv2.WINDOW_NORMAL)
            cv2.imshow("rgb image", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    # Get left arm depth image
    depth_data = robot.get_depth_data(SensorType.LEFT_ARM_DEPTH_CAMERA)
    if not depth_data or "data" not in depth_data:
        print("Depth camera not ready")
    else:
        print("get depth data suceess")
        print(depth_data['header'])
        depth_img_raw = decode_compressed_image(depth_data)
        depth_img = cv2.normalize(depth_img_raw, None, 0, 255, cv2.NORM_MINMAX) # Normalize depth values to 0-1 range
        depth_img = depth_img.astype(np.uint8)

        # Save depth image
        cv2.imwrite("depth_data.jpg", depth_img)
        # depth
        if SHOW_IMAGE:
            cv2.namedWindow("depth image", cv2.WINDOW_NORMAL)
            cv2.imshow("depth image", depth_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    # Get left arm IR images
    for sensor_type, filename in [
        (SensorType.LEFT_ARM_INFRA_CAMERA_1, "ir_infra1.jpg"),
        (SensorType.LEFT_ARM_INFRA_CAMERA_2, "ir_infra2.jpg"),
    ]:
        ir_data = robot.get_ir_data(sensor_type)
        if not ir_data or "data" not in ir_data:
            print(f"IR camera {sensor_type} not ready (ir_enabled may be false)")
        else:
            print(f"get ir data {sensor_type} success")
            print(ir_data['header'])
            ir_img = decode_ir_image(ir_data)

            cv2.imwrite(filename, ir_img)
            if SHOW_IMAGE:
                cv2.namedWindow(f"ir image {sensor_type}", cv2.WINDOW_NORMAL)
                cv2.imshow(f"ir image {sensor_type}", ir_img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

    # send SIGINT shutdown signal
    robot.request_shutdown()
    # Wait until entering shutdown state
    robot.wait_for_shutdown()
    # Perform SDK resource release
    robot.destroy()
    print('Resources released successfully')
    
if __name__=="__main__":
    main()