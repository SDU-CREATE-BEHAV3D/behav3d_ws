#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

from message_filters import Subscriber, ApproximateTimeSynchronizer

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None


def stamp_to_ns(msg: Image) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class FemtoCapture:
    """Camera handler for Orbbec Femto Bolt: subscriptions, sync and saving."""

    def __init__(self, node: Node):
        self.node = node

        # ---- Parameters (camera + saving) ----
        node.declare_parameter('color_topic', '/camera/color/image_raw')
        node.declare_parameter('depth_topic', '/camera/depth/image_raw')
        node.declare_parameter('ir_topic', '/camera/ir/image_raw')
        node.declare_parameter('color_info_topic', '/camera/color/camera_info')
        node.declare_parameter('depth_info_topic', '/camera/depth/camera_info')

        node.declare_parameter('decimate', 1)
        node.declare_parameter('save_depth_as_npy', False)
        node.declare_parameter('depth32_to_png16_mm', False)
        node.declare_parameter('pair_slop_ms', 40)
        node.declare_parameter('status_log_every', 0)  # 0=off, >0 logs every N synced pairs

        self.color_topic = node.get_parameter('color_topic').get_parameter_value().string_value
        self.depth_topic = node.get_parameter('depth_topic').get_parameter_value().string_value
        self.ir_topic = node.get_parameter('ir_topic').get_parameter_value().string_value
        self.color_info_topic = node.get_parameter('color_info_topic').get_parameter_value().string_value
        self.depth_info_topic = node.get_parameter('depth_info_topic').get_parameter_value().string_value

        self.decimate = max(1, int(node.get_parameter('decimate').get_parameter_value().integer_value))
        self.save_depth_as_npy = node.get_parameter('save_depth_as_npy').get_parameter_value().bool_value
        self.depth32_to_png16_mm = node.get_parameter('depth32_to_png16_mm').get_parameter_value().bool_value
        self.pair_slop_ns = int(node.get_parameter('pair_slop_ms').get_parameter_value().integer_value) * 1_000_000
        self.status_log_every = int(node.get_parameter('status_log_every').get_parameter_value().integer_value)

        # ---- cv_bridge ----
        if CvBridge is None:
            node.get_logger().warn('cv_bridge not available. Install ros-jazzy-cv-bridge and python3-opencv.')
            self.bridge = None
        else:
            self.bridge = CvBridge()

        # ---- QoS ----
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # ---- message_filters sync (color+depth) ----
        self.sub_color_sync = Subscriber(node, Image, self.color_topic, qos_profile=img_qos)
        self.sub_depth_sync = Subscriber(node, Image, self.depth_topic, qos_profile=img_qos)
        self.sync = ApproximateTimeSynchronizer([self.sub_color_sync, self.sub_depth_sync], queue_size=30, slop=0.05)
        self.sync.registerCallback(self._cb_images_synced)

        # ---- raw subscribers for fallback ----
        self.latest_color_raw: Optional[Image] = None
        self.latest_depth_raw: Optional[Image] = None
        self.latest_ir: Optional[Image] = None

        self._got_color_raw = False
        self._got_depth_raw = False
        self._got_ir = False
        self._got_pair = False

        node.create_subscription(Image, self.color_topic, self._cb_color_raw, img_qos)
        node.create_subscription(Image, self.depth_topic, self._cb_depth_raw, img_qos)
        node.create_subscription(Image, self.ir_topic, self._cb_ir, img_qos)

        # CameraInfo (optional)
        self.color_info: Optional[CameraInfo] = None
        self.depth_info: Optional[CameraInfo] = None
        node.create_subscription(CameraInfo, self.color_info_topic, self._cb_color_info, info_qos)
        node.create_subscription(CameraInfo, self.depth_info_topic, self._cb_depth_info, info_qos)

        # State
        self.latest_pair: Optional[Tuple[Image, Image]] = None
        self.frame_count = 0

        node.get_logger().info(
            f'FemtoCapture subscribed:\n'
            f'  color={self.color_topic}, depth={self.depth_topic}, ir={self.ir_topic}\n'
            f'  infos: color_info={self.color_info_topic}, depth_info={self.depth_info_topic}\n'
            f'  decimate={self.decimate}, save_depth_as_npy={self.save_depth_as_npy}, '
            f'depth32_to_png16_mm={self.depth32_to_png16_mm}, pair_slop_ms={self.pair_slop_ns/1e6:.0f}'
        )

    # ---------- Callbacks ----------
    def _cb_color_info(self, msg: CameraInfo):
        self.color_info = msg

    def _cb_depth_info(self, msg: CameraInfo):
        self.depth_info = msg

    def _cb_color_raw(self, msg: Image):
        self.latest_color_raw = msg
        if not self._got_color_raw:
            self._got_color_raw = True
            self.node.get_logger().info(f'First COLOR frame: enc={msg.encoding}, {msg.width}x{msg.height}')

    def _cb_depth_raw(self, msg: Image):
        self.latest_depth_raw = msg
        if not self._got_depth_raw:
            self._got_depth_raw = True
            self.node.get_logger().info(f'First DEPTH frame: enc={msg.encoding}, {msg.width}x{msg.height}')

    def _cb_ir(self, msg: Image):
        self.latest_ir = msg
        if not self._got_ir:
            self._got_ir = True
            self.node.get_logger().info(f'First IR frame: enc={msg.encoding}, {msg.width}x{msg.height}')

    def _cb_images_synced(self, color_msg: Image, depth_msg: Image):
        self.latest_pair = (color_msg, depth_msg)
        self.frame_count += 1
        if not self._got_pair:
            self._got_pair = True
            self.node.get_logger().info(
                f'First COLOR/DEPTH pair: color enc={color_msg.encoding} {color_msg.width}x{color_msg.height} | '
                f'depth enc={depth_msg.encoding} {depth_msg.width}x{depth_msg.height}'
            )
        if self.status_log_every > 0 and (self.frame_count % self.status_log_every == 0):
            ci = f"{self.color_info.width}x{self.color_info.height}" if self.color_info else "none"
            di = f"{self.depth_info.width}x{self.depth_info.height}" if self.depth_info else "none"
            self.node.get_logger().info(
                f'#{self.frame_count} synced pair present | info(color={ci}, depth={di})'
            )

    # ---------- Public helpers ----------
    def compose_pair_from_raw(self) -> Optional[Tuple[Image, Image]]:
        """Try to compose a pair from latest raw color & depth if stamps are close."""
        if self.latest_color_raw is None or self.latest_depth_raw is None:
            return None
        c_ns = stamp_to_ns(self.latest_color_raw)
        d_ns = stamp_to_ns(self.latest_depth_raw)
        if abs(c_ns - d_ns) <= self.pair_slop_ns:
            return (self.latest_color_raw, self.latest_depth_raw)
        return None

    # ---------- Saving ----------
    def _idx_tag(self, idx: int) -> str:
        return f'i{idx}'

    def _decimate(self, img: np.ndarray, factor: int) -> np.ndarray:
        if factor <= 1:
            return img
        return img[::factor, ::factor].copy()

    def _to_bgr8(self, msg: Image) -> np.ndarray:
        """Convert many common encodings to BGR8 for saving."""
        if self.bridge is None:
            raise RuntimeError('cv_bridge not available')
        enc = msg.encoding.lower()
        if enc == 'bgr8':
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if enc == 'rgb8':
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if enc == 'bgra8':
            bgra = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
            return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        if enc == 'rgba8':
            rgba = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgba8')
            return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        if enc in ('yuyv', 'yuy2', 'yuv422'):
            yuyv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
        if enc in ('uyvy', 'uyvy422'):
            uyvy = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            return cv2.cvtColor(uyvy, cv2.COLOR_YUV2BGR_UYVY)
        if enc in ('mono8', '8uc1'):
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if arr.ndim == 2:
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        if arr.shape[2] == 3:
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if arr.shape[2] == 4:
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        raise RuntimeError(f'Unsupported color encoding: {msg.encoding}')

    def save_color(self, session_dir: Path, idx: int, color_msg: Image) -> Optional[str]:
        if self.bridge is None or color_msg is None:
            return None
        try:
            img = self._to_bgr8(color_msg)
            img = self._decimate(img, self.decimate)
            tag = self._idx_tag(idx)
            fpath = session_dir / 'color_raw' / f'rgb_{tag}.png'
            ok = cv2.imwrite(str(fpath), img)
            if not ok:
                raise RuntimeError('cv2.imwrite returned False for color')
            return str(fpath.relative_to(session_dir))
        except Exception as e:
            self.node.get_logger().warn(f'Failed to save color ({color_msg.encoding}): {e}')
            return None

    def save_ir(self, session_dir: Path, idx: int, ir_msg: Image) -> Optional[str]:
        if self.bridge is None or ir_msg is None:
            return None
        try:
            img = self.bridge.imgmsg_to_cv2(ir_msg, desired_encoding='passthrough')
            img = self._decimate(img, self.decimate)
            tag = self._idx_tag(idx)
            fpath = session_dir / 'ir_raw' / f'ir_{tag}.png'
            ok = cv2.imwrite(str(fpath), img)
            if not ok:
                raise RuntimeError('cv2.imwrite returned False for IR')
            return str(fpath.relative_to(session_dir))
        except Exception as e:
            self.node.get_logger().warn(f'Failed to save IR ({ir_msg.encoding}): {e}')
            return None

    def save_depth(self, session_dir: Path, idx: int, depth_msg: Image) -> Optional[str]:
        if self.bridge is None or depth_msg is None:
            return None
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth = self._decimate(depth, self.decimate)
            tag = self._idx_tag(idx)

            if depth.dtype == np.uint16:
                fpath = session_dir / 'depth_raw' / f'depth_{tag}.png'
                ok = cv2.imwrite(str(fpath), depth)
                if not ok:
                    raise RuntimeError('cv2.imwrite returned False for depth16')
                if self.save_depth_as_npy:
                    np.save(str(fpath.with_suffix('.npy')), depth)
                return str(fpath.relative_to(session_dir))

            if depth.dtype == np.float32:
                fpath = session_dir / 'depth_raw' / f'depth_{tag}.npy'
                np.save(str(fpath), depth)
                if self.depth32_to_png16_mm:
                    mm = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
                    fpath_png = session_dir / 'depth_raw' / f'depth_mm16_{tag}.png'
                    cv2.imwrite(str(fpath_png), mm)
                return str(fpath.relative_to(session_dir))

            # Fallback
            fpath = session_dir / 'depth_raw' / f'depth_{tag}.png'
            ok = cv2.imwrite(str(fpath), depth)
            if not ok:
                raise RuntimeError(f'cv2.imwrite failed for depth dtype={depth.dtype}')
            return str(fpath.relative_to(session_dir))

        except Exception as e:
            self.node.get_logger().warn(f'Failed to save depth ({depth_msg.encoding}): {e}')
            return None
