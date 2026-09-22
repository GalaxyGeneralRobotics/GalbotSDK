try:
    from galbot_sdk.g3 import GalbotRobot, RgbOutputFormat, SensorType
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


def decode_compressed_image(compressed_image):
    """
    decode CompressedImage image

    Parameters:
        compressed_image (dict): image dict, keys: [header, format, data]

    Returns:
        numpy.ndarray: decoded image
    """
    image_data = compressed_image["data"]
    if compressed_image["format"].lower() in ("jpeg", "jpg", "rgb8"):
        return decode_rgb_image(image_data)
    raise ValueError(f"Unsupported data format: {compressed_image['format']}")

def decode_rgb_image(image_data):
    """decode rgb image"""
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Fail to Decode RGB Image")
    return img

def main():
    robot = GalbotRobot()

    
    # G3 provides RGB cameras but no arm-mounted depth cameras.
    enable_sensor_set = {SensorType.HEAD_LEFT_CAMERA}
    robot.init(enable_sensor_set)
    print("Initialization succeeded")
    
    # Program started, waiting for data
    time.sleep(5)
    
    # Get head left RGB image
    rgb_image_data = robot.get_rgb_data(SensorType.HEAD_LEFT_CAMERA, RgbOutputFormat.JPEG, True)
    if not rgb_image_data:
        print("No rgb image data!")
    else:
        print("get rgb image suceess")
        print(rgb_image_data['header'])
        img = decode_compressed_image(rgb_image_data)
        
        # Save RGB image
        cv2.imwrite("rgb_image_data.jpg", img)

    # Get head left camera intrinsics
    camera_intrinsics = robot.get_camera_intrinsic(SensorType.HEAD_LEFT_CAMERA)
    if not camera_intrinsics:
        print("No camera intrinsics data!")
    else:
        print("get camera intrinsics suceess")
        print(camera_intrinsics)

    # Extrinsics
    time.sleep(2)
    camera_extrinsics, timestamp_ns = robot.get_sensor_extrinsic(SensorType.HEAD_LEFT_CAMERA)
    if not camera_extrinsics:
        print("No camera extrinsics data!")
    else:
        print("get camera extrinsics suceess")
        print(camera_extrinsics)

    # send SIGINT shutdown signal
    robot.request_shutdown()
    # Wait until entering shutdown state
    robot.wait_for_shutdown()
    # Perform SDK resource release
    robot.destroy()
    print('Resources released successfully')
    
if __name__=="__main__":
    main()
