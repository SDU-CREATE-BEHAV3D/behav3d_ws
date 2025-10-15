#!/usr/bin/env python3
import os
from pathlib import Path
from datetime import datetime
import yaml
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from sensor_msgs.msg import Image, CameraInfo
from behav3d_interfaces.srv import Capture

from message_filters import Subscriber, ApproximateTimeSynchronizer

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None


def stamp_to_ns(msg) -> int:
    # sensor_msgs/Image header stamp -> epoch ns
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class SenseNode(Node):
    def __init__(self):
        super().__init__('sense_node')

        # ---------- Parameters ----------
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('ir_topic', '/camera/ir/image_raw')
        self.declare_parameter('color_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_info_topic', '/camera/depth/camera_info')

        self.declare_parameter('decimate', 1)
        self.declare_parameter('save_depth_as_npy', False)
        self.declare_parameter('depth32_to_png16_mm', False)
        self.declare_parameter('pair_slop_ms', 40)   # max stamp diff to compose a pair from raw streams
        self.declare_parameter('filename_index_width', 6)
        self.color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.ir_topic = self.get_parameter('ir_topic').get_parameter_value().string_value
        self.color_info_topic = self.get_parameter('color_info_topic').get_parameter_value().string_value
        self.depth_info_topic = self.get_parameter('depth_info_topic').get_parameter_value().string_value

        self.decimate = max(1, int(self.get_parameter('decimate').get_parameter_value().integer_value))
        self.save_depth_as_npy = self.get_parameter('save_depth_as_npy').get_parameter_value().bool_value
        self.depth32_to_png16_mm = self.get_parameter('depth32_to_png16_mm').get_parameter_value().bool_value
        self.pair_slop_ns = int(self.get_parameter('pair_slop_ms').get_parameter_value().integer_value) * 1_000_000

        # ---------- Captures root and session ----------
        env_root = os.environ.get('BEHAV3D_CAPTURES_ROOT', '')
        self.captures_root = Path(env_root).expanduser() if env_root else (Path.home() / 'behav3d_ws' / 'captures')
        self.captures_root.mkdir(parents=True, exist_ok=True)

        self.current_capture_dir = self._create_new_session_dir()
        self._ensure_manifest(self.current_capture_dir)

        # ---------- cv_bridge ----------
        if CvBridge is None:
            self.get_logger().warn('cv_bridge not available. Install ros-jazzy-cv-bridge and python3-opencv.')
            self.bridge = None
        else:
            self.bridge = CvBridge()

        # ---------- QoS ----------
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

        # ---------- message_filters (synced color+depth) ----------
        self.sub_color_sync = Subscriber(self, Image, self.color_topic, qos_profile=img_qos)
        self.sub_depth_sync = Subscriber(self, Image, self.depth_topic, qos_profile=img_qos)
        self.sync = ApproximateTimeSynchronizer([self.sub_color_sync, self.sub_depth_sync], queue_size=30, slop=0.05)
        self.sync.registerCallback(self.cb_images_synced)

        # ---------- plain subscribers (raw latest for fallback) ----------
        self.latest_color_raw = None
        self.latest_depth_raw = None
        self.latest_ir = None

        self._got_color_raw = False
        self._got_depth_raw = False
        self._got_ir = False
        self._got_pair = False

        self.create_subscription(Image, self.color_topic, self._cb_color_raw, img_qos)
        self.create_subscription(Image, self.depth_topic, self._cb_depth_raw, img_qos)
        self.create_subscription(Image, self.ir_topic, self._cb_ir, img_qos)

        # CameraInfo (optional)
        self.color_info = None
        self.depth_info = None
        self.create_subscription(CameraInfo, self.color_info_topic, self._cb_color_info, info_qos)
        self.create_subscription(CameraInfo, self.depth_info_topic, self._cb_depth_info, info_qos)

        # Last synced pair
        self.latest_pair = None  # (color_msg, depth_msg)
        self.frame_count = 0

        self.get_logger().info(
            "behav3d_sense ready.\n"
            f"  session: {self.current_capture_dir}\n"
            f"  topics: color={self.color_topic}, depth={self.depth_topic}, ir={self.ir_topic}\n"
            f"  infos:  color_info={self.color_info_topic}, depth_info={self.depth_info_topic}\n"
            f"  decimate={self.decimate}, save_depth_as_npy={self.save_depth_as_npy}, depth32_to_png16_mm={self.depth32_to_png16_mm}, pair_slop_ms={self.pair_slop_ns/1e6:.0f}"
        )

        # ---------- Service ----------
        self.srv = self.create_service(Capture, 'capture', self.handle_capture)

        # TF params
        self.declare_parameter('tf_base_frame', 'ur20_base_link')
        self.declare_parameter('tf_tool_frame', 'ur20_tool0')
        self.declare_parameter('tf_timeout_sec', 0.3)

        self.tf_base = self.get_parameter('tf_base_frame').get_parameter_value().string_value
        self.tf_tool = self.get_parameter('tf_tool_frame').get_parameter_value().string_value
        # accept int or float param
        p = self.get_parameter('tf_timeout_sec')
        self.tf_timeout = float(p.get_parameter_value().double_value if p.type_ == 11 else p.get_parameter_value().integer_value)

        # TF2 buffer + listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    # ================= Folder helpers =================
    def _timestamp_dir(self) -> str:
        return datetime.now().strftime('%y%m%d_%H%M%S')

    def _ensure_capture_folder(self, folder: Path) -> Path:
        subfolders = ['ir_raw', 'color_raw', 'depth_raw']
        folder.mkdir(parents=True, exist_ok=True)
        for sub in subfolders:
            (folder / sub).mkdir(parents=True, exist_ok=True)
        return folder

    def _create_new_session_dir(self) -> Path:
        session_dir = self.captures_root / self._timestamp_dir()
        return self._ensure_capture_folder(session_dir)

    def _resolve_folder_arg(self, folder_arg: str) -> Path:
        p = Path(folder_arg).expanduser()
        return p if p.is_absolute() else (self.captures_root / p)

    # ================= Manifest helpers =================
    def _manifest_path(self, session_dir: Path) -> Path:
        return session_dir / 'manifest.yaml'

    def _now_iso(self) -> str:
        return datetime.now().strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]

    def _ensure_manifest(self, session_dir: Path) -> None:
        mpath = self._manifest_path(session_dir)
        if not mpath.exists():
            now_ns = self.get_clock().now().nanoseconds
            data = {
                'session': {
                    'created_at': self._now_iso(),
                    'created_ns': int(now_ns),
                    'path': str(session_dir),
                },
                'captures': []
            }
            self._write_manifest(mpath, data)

    def _read_manifest(self, mpath: Path) -> dict:
        if not mpath.exists():
            return {'session': {'created_at': self._now_iso(), 'path': str(mpath.parent)}, 'captures': []}
        with mpath.open('r', encoding='utf-8') as f:
            loaded = yaml.safe_load(f) or {}
        if 'captures' not in loaded:
            loaded['captures'] = []
        if 'session' not in loaded:
            loaded['session'] = {'created_at': self._now_iso(), 'path': str(mpath.parent)}
        return loaded

    def _write_manifest(self, mpath: Path, data: dict) -> None:
        tmp = mpath.with_suffix('.yaml.tmp')
        with tmp.open('w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False, allow_unicode=True)
        tmp.replace(mpath)

    def _append_capture_entry(self, session_dir: Path, stamp_ns: int, initial_fields: dict) -> dict:
        mpath = self._manifest_path(session_dir)
        data = self._read_manifest(mpath)
        idx = len(data['captures'])
        entry = {
            'index': idx,
            'stamp_ns': int(stamp_ns),
            'timestamp': self._now_iso(),
            'rgb': initial_fields.get('rgb', None),
            'depth': initial_fields.get('depth', None),
            'ir': initial_fields.get('ir', None),
            'pose_tool0': initial_fields.get('pose_tool0', None),  
        }
        data['captures'].append(entry)
        self._write_manifest(mpath, data)
        return entry
    def _to_bgr8(self, msg: Image):
        """Convert many common encodings to BGR8 for saving."""
        enc = msg.encoding.lower()
        # Try fast paths first
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

        # YUV packed formats some drivers use
        if enc in ('yuyv', 'yuy2', 'yuv422'):
            yuyv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
        if enc in ('uyvy', 'uyvy422'):
            uyvy = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            return cv2.cvtColor(uyvy, cv2.COLOR_YUV2BGR_UYVY)

        # Grayscale as last resort
        if enc in ('mono8', '8uc1'):
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # Fall back: passthrough then try to coerce
        arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if arr.ndim == 2:  # grayscale
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        if arr.shape[2] == 3:
            # assume already BGR or RGB; try RGB->BGR swap conservatively
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if arr.shape[2] == 4:
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        raise RuntimeError(f'Unsupported color encoding: {msg.encoding}')

    # ================= Subscribers / callbacks =================
    def _cb_color_info(self, msg: CameraInfo):
        self.color_info = msg

    def _cb_depth_info(self, msg: CameraInfo):
        self.depth_info = msg

    def _cb_color_raw(self, msg: Image):
        self.latest_color_raw = msg
        if not self._got_color_raw:
            self._got_color_raw = True
            self.get_logger().info(f'First COLOR frame: enc={msg.encoding}, {msg.width}x{msg.height}')

    def _cb_depth_raw(self, msg: Image):
        self.latest_depth_raw = msg
        if not self._got_depth_raw:
            self._got_depth_raw = True
            self.get_logger().info(f'First DEPTH frame: enc={msg.encoding}, {msg.width}x{msg.height}')

    def _cb_ir(self, msg: Image):
        self.latest_ir = msg
        if not self._got_ir:
            self._got_ir = True
            self.get_logger().info(f'First IR frame: enc={msg.encoding}, {msg.width}x{msg.height}')

    def cb_images_synced(self, color_msg: Image, depth_msg: Image):
        self.latest_pair = (color_msg, depth_msg)
        self.frame_count += 1
        if not self._got_pair:
            self._got_pair = True
            self.get_logger().info(
                f'First COLOR/DEPTH pair: color enc={color_msg.encoding} {color_msg.width}x{color_msg.height} | '
                f'depth enc={depth_msg.encoding} {depth_msg.width}x{depth_msg.height}'
            )

    # ================= Saving helpers =================
    def _decimate(self, img: np.ndarray, factor: int) -> np.ndarray:
        if factor <= 1:
            return img
        return img[::factor, ::factor].copy()

    def _save_color_msg(self, session_dir: Path, idx: int, color_msg: Image, stamp_ns_for_name: int) -> str:
        if self.bridge is None or color_msg is None:
            return None
        try:
            cv_img = self._to_bgr8(color_msg)  # tu helper robusto
            cv_img = self._decimate(cv_img, self.decimate)
            tag = self._idx_tag(idx)
            fname = f'rgb_{tag}.png'
            fpath = session_dir / 'color_raw' / fname
            ok = cv2.imwrite(str(fpath), cv_img)
            if not ok:
                raise RuntimeError('cv2.imwrite returned False for color')
            return str(fpath.relative_to(session_dir))
        except Exception as e:
            self.get_logger().warn(f'Failed to save color ({color_msg.encoding}): {e}')
            return None


    def _save_ir_msg(self, session_dir: Path, idx: int, ir_msg: Image, stamp_ns_for_name: int) -> str:
        if self.bridge is None or ir_msg is None:
            return None
        try:
            img = self.bridge.imgmsg_to_cv2(ir_msg, desired_encoding='passthrough')
            img = self._decimate(img, self.decimate)
            tag = self._idx_tag(idx)
            fname = f'ir_{tag}.png'
            fpath = session_dir / 'ir_raw' / fname
            ok = cv2.imwrite(str(fpath), img)
            if not ok:
                raise RuntimeError('cv2.imwrite returned False for IR')
            return str(fpath.relative_to(session_dir))
        except Exception as e:
            self.get_logger().warn(f'Failed to save IR ({ir_msg.encoding}): {e}')
            return None



    def _save_depth_msg(self, session_dir: Path, idx: int, depth_msg: Image, stamp_ns_for_name: int) -> str:
        if self.bridge is None or depth_msg is None:
            return None
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth = self._decimate(depth, self.decimate)
            tag = self._idx_tag(idx)

            if depth.dtype == np.uint16:
                fname = f'depth_{tag}.png'
                fpath = session_dir / 'depth_raw' / fname
                ok = cv2.imwrite(str(fpath), depth)
                if not ok:
                    raise RuntimeError('cv2.imwrite returned False for depth16')
                if self.save_depth_as_npy:
                    np.save(str(fpath.with_suffix('.npy')), depth)
                return str(fpath.relative_to(session_dir))

            elif depth.dtype == np.float32:
                fname = f'depth_{tag}.npy'
                fpath = session_dir / 'depth_raw' / fname
                np.save(str(fpath), depth)
                if self.depth32_to_png16_mm:
                    mm = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
                    fpath_png = session_dir / 'depth_raw' / f'depth_mm16_{tag}.png'
                    cv2.imwrite(str(fpath_png), mm)
                return str(fpath.relative_to(session_dir))

            else:
                fname = f'depth_{tag}.png'
                fpath = session_dir / 'depth_raw' / fname
                ok = cv2.imwrite(str(fpath), depth)
                if not ok:
                    raise RuntimeError(f'cv2.imwrite failed for depth dtype={depth.dtype}')
                return str(fpath.relative_to(session_dir))

        except Exception as e:
            self.get_logger().warn(f'Failed to save depth ({depth_msg.encoding}): {e}')
            return None

    def _idx_tag(self, idx: int) -> str:
        # No padding: i1, i12, i3432
        return f'i{idx}'
    
    def _lookup_pose_tool0(self):
        """Return dict with pose of tf_tool in tf_base or None if TF unavailable."""
        base = self.tf_base
        tool = self.tf_tool
        timeout = Duration(seconds=self.tf_timeout)
        stamp = Time()  # latest available

        try:
            if not self.tf_buffer.can_transform(base, tool, stamp, timeout):
                self.get_logger().warn(f'TF not available: {base} <- {tool}')
                return None
            tf = self.tf_buffer.lookup_transform(base, tool, stamp, timeout)
        except Exception as e:
            self.get_logger().warn(f'lookup_transform failed: {e}')
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        return {
            'child_frame_id': tool,
            'frame_id': base,
            'orientation_xyzw': [float(q.x), float(q.y), float(q.z), float(q.w)],
            'position': [float(t.x), float(t.y), float(t.z)],
        }


    # ================= Pair composition helper =================
    def _compose_pair_from_raw(self):
        """Try to compose a pair from latest raw color & depth if stamps are close."""
        if self.latest_color_raw is None or self.latest_depth_raw is None:
            return None
        c_ns = stamp_to_ns(self.latest_color_raw)
        d_ns = stamp_to_ns(self.latest_depth_raw)
        if abs(c_ns - d_ns) <= self.pair_slop_ns:
            return (self.latest_color_raw, self.latest_depth_raw)
        return None

    # ================= Service handler =================
    def handle_capture(self, request: Capture.Request, response: Capture.Response):
        # Optional folder change
        if request.set_folder and request.folder.strip():
            try:
                new_dir = self._resolve_folder_arg(request.folder.strip())
                self._ensure_capture_folder(new_dir)
                self.current_capture_dir = new_dir
                self._ensure_manifest(self.current_capture_dir)
            except Exception as e:
                response.success = False
                response.message = f'Failed to set folder: {e}'
                return response

        stamp_now_ns = int(self.get_clock().now().nanoseconds)
        fields = {'rgb': None, 'depth': None, 'ir': None, 'pose_tool0': None}
        entry = self._append_capture_entry(self.current_capture_dir, stamp_now_ns, fields)
        idx = entry['index']

        # Choose color/depth sources
        color_msg = None
        depth_msg = None

        if self.latest_pair is not None:
            color_msg, depth_msg = self.latest_pair
        else:
            pair = self._compose_pair_from_raw()
            if pair is not None:
                color_msg, depth_msg = pair

        # Save streams as requested
        if request.do_rgb:
            if color_msg is not None:
                fields['rgb'] = self._save_color_msg(self.current_capture_dir, idx, color_msg, stamp_to_ns(color_msg))
            elif self.latest_color_raw is not None:
                self.get_logger().warn('Using unsynced latest COLOR frame (no synced pair).')
                cm = self.latest_color_raw
                fields['rgb'] = self._save_color_msg(self.current_capture_dir, idx, cm, stamp_to_ns(cm))
            else:
                self.get_logger().warn('No COLOR frame available to save.')

        if request.do_depth:
            if depth_msg is not None:
                fields['depth'] = self._save_depth_msg(self.current_capture_dir, idx, depth_msg, stamp_to_ns(depth_msg))
            elif self.latest_depth_raw is not None:
                self.get_logger().warn('Using unsynced latest DEPTH frame (no synced pair).')
                dm = self.latest_depth_raw
                fields['depth'] = self._save_depth_msg(self.current_capture_dir, idx, dm, stamp_to_ns(dm))
            else:
                self.get_logger().warn('No DEPTH frame available to save.')

        if request.do_ir:
            if self.latest_ir is not None:
                fields['ir'] = self._save_ir_msg(self.current_capture_dir, idx, self.latest_ir, stamp_to_ns(self.latest_ir))
            else:
                self.get_logger().warn('No IR frame available to save.')

        if request.do_pose:
            fields['pose_tool0'] = self._lookup_pose_tool0()

        # Update manifest entry with saved paths
        mpath = self._manifest_path(self.current_capture_dir)
        data = self._read_manifest(mpath)
        try:
            data['captures'][idx].update(fields)
            self._write_manifest(mpath, data)
        except Exception as e:
            self.get_logger().warn(f'Failed to update manifest with file paths: {e}')

        saved = [k for k, v in fields.items() if v]
        self.get_logger().info(f'Capture #{idx} saved -> {", ".join(saved) if saved else "none"} in {self.current_capture_dir}')
        response.success = True
        response.message = f'Captured #{idx} in {self.current_capture_dir}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
