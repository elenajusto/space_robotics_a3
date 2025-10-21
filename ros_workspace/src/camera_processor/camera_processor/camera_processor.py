#### Working with all effects ###
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os
import numpy as np
from rclpy.qos import qos_profile_sensor_data

SAVE_DIR = "advanced_1_raw_and_effects"

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.cv_bridge_ = CvBridge()

        # Publishers: before/after streams for RViz
        self.pub_before = self.create_publisher(Image, 'camera/image_before', 1)
        self.pub_after  = self.create_publisher(Image, 'camera/image_after',  1)

        # Runtime parameters (set with ros2 param set …)
        self.declare_parameter('effect', 'none')      # none | dust | motion_blur | low_light | low_res | blur
        self.declare_parameter('severity', 0)         # 0..3
        self.declare_parameter('save_every_n', 0)     # 0 = no save, else save every N frames
        self.frame_idx = 0

        # Sub (sensor QoS recommended)
        self.sub = self.create_subscription(
            Image, 'camera/image', self.image_callback, qos_profile_sensor_data
        )

        # CSV log for your report (what effect/level was active when an image was saved)
        os.makedirs(SAVE_DIR, exist_ok=True)
        self.log_path = os.path.join(SAVE_DIR, 'advanced1_log.csv')
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w') as f:
                f.write('timestamp,effect,severity,frame_idx,filename\n')

        self.get_logger().info("CameraProcessor (Advanced 1) initialised")

    def image_callback(self, msg):
        img = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Publish BEFORE stream
        self.pub_before.publish(self.cv_bridge_.cv2_to_imgmsg(img, encoding='rgb8'))

        # Build AFTER (degraded) image
        effect  = self.get_parameter('effect').get_parameter_value().string_value
        level   = int(self.get_parameter('severity').get_parameter_value().integer_value)
        level   = max(0, min(level, 3))

        degraded = self.apply_effect(img, effect, level)

        # Publish AFTER stream
        self.pub_after.publish(self.cv_bridge_.cv2_to_imgmsg(degraded, encoding='rgb8'))

        # Optional saving cadence
        n = int(self.get_parameter('save_every_n').get_parameter_value().integer_value)
        if n > 0 and (self.frame_idx % n == 0):
            ts = time.strftime("%Y%m%d-%H%M%S")
            fn_before = f"before_{ts}.jpg"
            fn_after  = f"after_{ts}_{effect}_L{level}.jpg"
            p_before = os.path.join(SAVE_DIR, fn_before)
            p_after  = os.path.join(SAVE_DIR, fn_after)
            cv2.imwrite(p_before, cv2.cvtColor(img,      cv2.COLOR_BGR2RGB))
            cv2.imwrite(p_after,  cv2.cvtColor(degraded, cv2.COLOR_BGR2RGB))
            with open(self.log_path, 'a') as f:
                f.write(f'{ts},{effect},{level},{self.frame_idx},{fn_after}\n')
            self.get_logger().info(f"Saved {fn_before}, {fn_after}")

        self.frame_idx += 1

    def apply_effect(self, image, effect, level):
        if level == 0 or effect == 'none':
            return image

        if effect == 'blur':
            # median blur to simulate mild defocus
            k = {1:3, 2:5, 3:7}[level]
            return cv2.medianBlur(image, k)

        if effect == 'dust':
            return self.dust_filter(image, level)

        if effect == 'motion_blur':
            return self.motion_blur(image, level)

        if effect == 'low_light':
            return self.low_light(image, level)

        if effect == 'low_res':
            return self.low_res(image, level)

        self.get_logger().warn(f"Unknown effect '{effect}', passing through")
        return image

    def low_res(self, image, level):
        scale = {1: 0.75, 2: 0.5, 3: 0.25}[level]
        h, w = image.shape[:2]
        small = cv2.resize(image, (int(w*scale), int(h*scale)), interpolation=cv2.INTER_AREA)
        return cv2.resize(small, (w, h), interpolation=cv2.INTER_NEAREST)

    def low_light(self, image, level):
        # shift brightness negative; combine w/ contrast clip (CLAHE) in compensation step before YOLO
        shift = {1: -30, 2: -60, 3: -90}[level]
        return cv2.add(image, np.array([shift], dtype=np.int16)).clip(0,255).astype(np.uint8)

    def dust_filter(self, image, level):
        # non-destructive: work on a copy & pepper with brown specs
        out = image.copy()
        density = {1:0.005, 2:0.01, 3:0.02}[level]
        h, w = out.shape[:2]
        n = int(h * w * density)
        rows = np.random.randint(0, h, size=n)
        cols = np.random.randint(0, w, size=n)
        out[rows, cols] = (88, 57, 39)  # brown specks
        # (optional) smear a few specks to simulate streaks:
        return out

    def motion_blur(self, image, level):
        k = {1:9, 2:15, 3:25}[level]
        kernel = np.zeros((k, k), dtype=np.float32)
        kernel[k//2, :] = 1.0
        kernel /= kernel.sum()
        return cv2.filter2D(image, -1, kernel)

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

