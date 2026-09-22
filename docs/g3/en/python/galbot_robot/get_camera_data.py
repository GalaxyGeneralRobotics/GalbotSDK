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

def decode_rgb_image(image_data):
    """decode rgb image"""
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Fail to Decode RGB Image")
    return img

def main():
    SHOW_IMAGE = False
    robot = GalbotRobot()

    # Get left arm RGB and IR images. G3 does not have arm-mounted depth cameras.
    enable_sensor_set = {SensorType.LEFT_ARM_CAMERA,
                        SensorType.LEFT_ARM_INFRA_CAMERA_1,
                        SensorType.LEFT_ARM_INFRA_CAMERA_2,}
    robot.init(enable_sensor_set)
    print("Initialization succeeded")
    # Program started, waiting for data
    time.sleep(5)
    # Get left arm RGB image
    rgb_image_data = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA, RgbOutputFormat.JPEG, True)
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
            print(ir_data['format'])
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
