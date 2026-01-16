#!/usr/bin/env python3
import os, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from behav3d_interfaces.srv import CaptureRGBD
import cv2
import numpy as np
from threading import Lock

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        # Params (topic names and default save dir)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('default_save_dir', '/home/create_ros/captures')

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.default_save_dir = self.get_parameter('default_save_dir').value

        # QoS: subscribe with both Reliable and BestEffort in parallel; whichever receives wins
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)

        self.bridge = CvBridge()
        self._lock = Lock()
        self._last_color = None   # (msg, t_float)
        self._last_depth = None   # (msg, t_float)

        # two subs for each topic, to handle QoS mismatch
        self.create_subscription(Image, self.color_topic, lambda m: self._store('color', m), qos_be)
        self.create_subscription(Image, self.color_topic, lambda m: self._store('color', m), qos_rel)
        self.create_subscription(Image, self.depth_topic, lambda m: self._store('depth', m), qos_be)
        self.create_subscription(Image, self.depth_topic, lambda m: self._store('depth', m), qos_rel)

        # Service
        self._srv = self.create_service(CaptureRGBD, 'capture_rgbd', self._capture_cb)

        self.get_logger().info(f"ImageProcessor up. Color: {self.color_topic}, Depth: {self.depth_topic}")
        self.get_logger().info("Service: /capture_rgbd (behav3d_interfaces/srv/CaptureRGBD)")

    def _store(self, which: str, msg: Image):
        # store most recent with wall time float for pairing
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            if which == 'color':
                self._last_color = (msg, t)
            else:
                self._last_depth = (msg, t)

    def _closest_pair(self, max_dt=0.033):
        """Return (color_msg, depth_msg) whose stamps are closest, within max_dt seconds."""
        with self._lock:
            c = self._last_color
            d = self._last_depth
        if not c or not d:
            return None, None
        cmsg, ct = c
        dmsg, dt = d
        if abs(ct - dt) <= max_dt:
            return cmsg, dmsg
        # pick whichever is latest and search the other; we only cache latest, so if not within tolerance, fail
        return None, None

    def _save_depth(self, arr_f32, path, image_format):
        if image_format == 'npy':
            np.save(path, arr_f32)
            return path + '.npy'
        # default: PNG 16UC1 millimeters
        arr_u16 = np.clip(arr_f32 * 1000.0, 0, 65535).astype(np.uint16)
        cv2.imwrite(path, arr_u16)
        return path

    def _capture_cb(self, req: CaptureRGBD.Request, res: CaptureRGBD.Response):
        save_dir = req.save_dir.strip() or self.default_save_dir
        os.makedirs(save_dir, exist_ok=True)
        prefix = req.prefix.strip() or 'capture'
        fmt = (req.image_format or 'png').lower()
        want_color = bool(req.save_color)
        want_depth = bool(req.save_depth)
        wait_sync = bool(req.wait_for_sync)

        # timeout handling
        deadline = self.get_clock().now() + Duration(seconds=req.timeout.sec,
                                                     nanoseconds=req.timeout.nanosec)

        color_msg = depth_msg = None
        while rclpy.ok():
            if wait_sync:
                color_msg, depth_msg = self._closest_pair(max_dt=0.050)  # 50ms tolerance
            else:
                with self._lock:
                    color_msg = self._last_color[0] if self._last_color else None
                    depth_msg = self._last_depth[0] if self._last_depth else None

            if (not want_color or color_msg) and (not want_depth or depth_msg):
                break

            if self.get_clock().now() > deadline:
                res.success = False
                res.message = "Timeout waiting for frames"
                return res
            rclpy.sleep(0.01)  # small yield

        # Convert and save
        stamp = color_msg.header.stamp if color_msg else (depth_msg.header.stamp if depth_msg else None)
        tstr = f"{stamp.sec}_{stamp.nanosec:09d}" if stamp else str(int(time.time()))

        color_path = depth_path = ""
        cw = ch = dw = dh = 0

        try:
            if want_color and color_msg:
                color_cv = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
                cw, ch = color_cv.shape[1], color_cv.shape[0]
                if fmt == 'npy':
                    np.save(os.path.join(save_dir, f"{prefix}_color_{tstr}.npy"), color_cv)
                    color_path = os.path.join(save_dir, f"{prefix}_color_{tstr}.npy")
                else:
                    ext = 'jpg' if fmt == 'jpg' else 'png'
                    color_path = os.path.join(save_dir, f"{prefix}_color_{tstr}.{ext}")
                    cv2.imwrite(color_path, color_cv, [cv2.IMWRITE_JPEG_QUALITY, 95] if ext == 'jpg' else [])
            if want_depth and depth_msg:
                # depth passthrough → either 16UC1 or 32FC1; convert to float meters first
                dcv = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
                if dcv.dtype == np.uint16:
                    d_m = dcv.astype(np.float32) / 1000.0
                else:
                    d_m = dcv.astype(np.float32)
                dw, dh = d_m.shape[1], d_m.shape[0]
                base = os.path.join(save_dir, f"{prefix}_depth_{tstr}")
                depth_path = self._save_depth(d_m, base + ".png", fmt)
        except Exception as e:
            res.success = False
            res.message = f"Capture failed: {e}"
            return res

        # Fill response
        res.success = True
        res.message = f"Saved: {color_path or '-'} {depth_path or '-'}"
        if stamp:
            res.stamp = stamp
        res.color_path = color_path
        res.depth_path = depth_path
        res.color_width = cw
        res.color_height = ch
        res.depth_width = dw
        res.depth_height = dh
        self.get_logger().info(res.message)
        return res

def main():
    rclpy.init()
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
