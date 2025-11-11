#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from pathlib import Path
from datetime import datetime
import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from behav3d_interfaces.srv import Capture
from sensor_msgs.msg import Image  # only for type hints

from .femto_capture import FemtoCapture, stamp_to_ns


class SenseNode(Node):
    def __init__(self):
        super().__init__('sense_node')

        # ---------- Captures root and session ----------
        env_root = os.environ.get('BEHAV3D_CAPTURES_ROOT', '')
        self.captures_root = Path(env_root).expanduser() if env_root else (Path.home() / 'behav3d_ws' / 'captures')
        self.captures_root.mkdir(parents=True, exist_ok=True)

        self.current_capture_dir = self._create_new_session_dir()
        self.session_root_dir = self.current_capture_dir  # keep the timestamp root pinned

        # ---------- Camera handler (all subs/sync live here) ----------
        self.cam = FemtoCapture(self)

        # ---------- One-time session intrinsics write ----------
        self._intrinsics_written = False

        def _try_write_session_intrinsics():
            if self._intrinsics_written:
                return
            if (self.cam.color_info is not None) or (self.cam.depth_info is not None) or (getattr(self.cam, "ir_info", None) is not None):
                try:
                    self.cam.save_intrinsics_yaml(self.session_root_dir)
                    self._intrinsics_written = True
                    self.get_logger().info("Session intrinsics saved under config/.")
                    self.destroy_timer(self._intrinsics_timer)
                except Exception as e:
                    self.get_logger().warn(f"Failed to write session intrinsics: {e}")

        self._intrinsics_timer = self.create_timer(0.5, _try_write_session_intrinsics)

        self._extrinsics_written = False
        def _try_write_extrinsics():
            if self._extrinsics_written:
                return
            ok = self._save_extrinsics_yaml(self.session_root_dir)
            if ok:
                self._extrinsics_written = True
                self.get_logger().info("Session extrinsics saved under config/.")
                self.destroy_timer(self._extrinsics_timer)

        self._extrinsics_timer = self.create_timer(0.5, _try_write_extrinsics)


        self.get_logger().info(
            "behav3d_sense ready.\n"
            f"  session: {self.current_capture_dir}"
        )

        # ---------- Service ----------
        self.srv = self.create_service(Capture, 'capture', self.handle_capture)

        # ---------- TF params ----------
        self.declare_parameter('tf_base_frame', 'ur20_base_link')
        self.declare_parameter('tf_tool_frame', 'ur20_tool0')
        self.declare_parameter('tf_timeout_sec', 0.3)
        self.declare_parameter('tf_color_calib_frame', 'femto_color_optical_calib')
        self.declare_parameter('tf_ir_calib_frame', 'femto_ir_optical_calib')

        self.tf_base = self.get_parameter('tf_base_frame').get_parameter_value().string_value
        self.tf_tool = self.get_parameter('tf_tool_frame').get_parameter_value().string_value

        self.tf_color_calib = self.get_parameter('tf_color_calib_frame').get_parameter_value().string_value
        self.tf_ir_calib    = self.get_parameter('tf_ir_calib_frame').get_parameter_value().string_value


        p = self.get_parameter('tf_timeout_sec')
        self.tf_timeout = float(p.get_parameter_value().double_value if p.type_ == 11 else p.get_parameter_value().integer_value)

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
        """Create the timestamp-named session directory and an empty 'config' subfolder."""
        session_dir = self.captures_root / self._timestamp_dir()
        session_dir.mkdir(parents=True, exist_ok=True)
        (session_dir / 'config').mkdir(parents=True, exist_ok=True)
        return session_dir

    def _resolve_folder_arg(self, folder_arg: str) -> Path:
        """Resolve folder arg with session-aware rules:
        ''            -> keep current dir
        '.'           -> current dir
        './...'       -> under current dir
        '..'          -> parent of current, clamped at session root
        '../...'      -> relative to current, clamped under session root
        '@session'    -> session root
        '@session/...'-> under session root
        other relative-> under captures_root
        absolute      -> use as-is
        """
        raw = (folder_arg or '').strip()

        # Keep current
        if raw == '' or raw == '.':
            return self.current_capture_dir

        # Absolute
        if raw.startswith('/'):
            return Path(raw).expanduser().resolve()

        # Explicit session-root tokens
        if raw == '@session' or raw == '@':
            return self.session_root_dir
        if raw.startswith('@session/'):
            sub = raw[len('@session/'):]
            return (self.session_root_dir / sub).expanduser().resolve()
        if raw.startswith('@/'):
            sub = raw[2:]
            return (self.session_root_dir / sub).expanduser().resolve()

        # Current-dir relative
        if raw.startswith('./'):
            return (self.current_capture_dir / raw[2:]).expanduser().resolve()

        # Parent-relative with clamping to session root
        if raw == '..':
            cand = (self.current_capture_dir / '..').resolve()
            try:
                _ = cand.relative_to(self.session_root_dir)
                return cand
            except ValueError:
                return self.session_root_dir

        if raw.startswith('../'):
            cand = (self.current_capture_dir / raw).resolve()
            # Clamp: keep inside the session root
            try:
                _ = cand.relative_to(self.session_root_dir)
                return cand
            except ValueError:
                # stick the target's basename under the session root
                return (self.session_root_dir / Path(raw).name).resolve()

        # Default: under captures_root
        return (self.captures_root / raw).expanduser().resolve()



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
            return yaml.safe_load(f) or {'captures': []}

    def _write_manifest(self, mpath: Path, data: dict) -> None:
        tmp = mpath.with_suffix('.yaml.tmp')
        with tmp.open('w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False, allow_unicode=True)
        tmp.replace(mpath)

    def _append_capture_entry(self, session_dir: Path, stamp_ns: int, initial_fields: dict) -> dict:
        mpath = self._manifest_path(session_dir)
        data = self._read_manifest(mpath)
        data.setdefault('captures', [])
        idx = len(data['captures'])
        entry = {
            'index': idx,
            'stamp_ns': int(stamp_ns),
            'timestamp': self._now_iso(),
            'color': initial_fields.get('color', None),
            'depth': initial_fields.get('depth', None),
            'ir': initial_fields.get('ir', None),
            'T_base_tool0': initial_fields.get('T_base_tool0', None),
        }
        data['captures'].append(entry)
        self._write_manifest(mpath, data)
        return entry

    # ================= TF pose =================
    def _lookup_T_base_tool0(self):
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
            'child': tool,
            'parent': base,
            'orientation_xyzw': [float(q.x), float(q.y), float(q.z), float(q.w)],
            'position': [float(t.x), float(t.y), float(t.z)],
        }

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
        fields = {'color': None, 'depth': None, 'ir': None, 'T_base_tool0': None}
        entry = self._append_capture_entry(self.current_capture_dir, stamp_now_ns, fields)
        idx = entry['index']

        # Pick color/depth
        color_msg = None
        depth_msg = None
        if self.cam.latest_pair is not None:
            color_msg, depth_msg = self.cam.latest_pair
        else:
            pair = self.cam.compose_pair_from_raw()
            if pair is not None:
                color_msg, depth_msg = pair

        # Save as requested
        if request.do_rgb:
            if color_msg is not None:
                fields['color'] = self.cam.save_color(self.current_capture_dir, idx, color_msg)
            elif self.cam.latest_color_raw is not None:
                self.get_logger().warn('Using unsynced latest COLOR frame (no synced pair).')
                fields['color'] = self.cam.save_color(self.current_capture_dir, idx, self.cam.latest_color_raw)
            else:
                self.get_logger().warn('No COLOR frame available to save.')

        if request.do_depth:
            if depth_msg is not None:
                fields['depth'] = self.cam.save_depth(self.current_capture_dir, idx, depth_msg)
            elif self.cam.latest_depth_raw is not None:
                self.get_logger().warn('Using unsynced latest DEPTH frame (no synced pair).')
                fields['depth'] = self.cam.save_depth(self.current_capture_dir, idx, self.cam.latest_depth_raw)
            else:
                self.get_logger().warn('No DEPTH frame available to save.')

        if request.do_ir:
            if self.cam.latest_ir is not None:
                fields['ir'] = self.cam.save_ir(self.current_capture_dir, idx, self.cam.latest_ir)
            else:
                self.get_logger().warn('No IR frame available to save.')

        if request.do_pose:
            fields['T_base_tool0'] = self._lookup_T_base_tool0()

        # Update manifest
        mpath = self._manifest_path(self.current_capture_dir)
        data = self._read_manifest(mpath)
        try:
            data['captures'][idx].update(fields)
            self._write_manifest(mpath, data)
        except Exception as e:
            self.get_logger().warn(f'Failed to update manifest: {e}')

        saved = [k for k, v in fields.items() if v]
        self.get_logger().info(f'Capture #{idx} saved -> {", ".join(saved) if saved else "none"} in {self.current_capture_dir}')
        response.success = True
        response.message = f'Captured #{idx} in {self.current_capture_dir}'
        return response

    def _quat_to_rpy(self, x, y, z, w):
        # roll (x), pitch (y), yaw (z), convención URDF (extrinsic XYZ)
        t0 = +2.0*(w*x + y*z)
        t1 = +1.0 - 2.0*(x*x + y*y)
        roll = math.atan2(t0, t1)

        t2 = +2.0*(w*y - z*x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0*(w*z + x*y)
        t4 = +1.0 - 2.0*(y*y + z*z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]

    def _lookup_tool0_to(self, child_frame: str):
        parent = self.tf_tool
        timeout = Duration(seconds=self.tf_timeout)
        stamp = Time()  # latest
        try:
            if not self.tf_buffer.can_transform(parent, child_frame, stamp, timeout):
                self.get_logger().warn(f'TF not available: {parent} -> {child_frame}')
                return None
            tf = self.tf_buffer.lookup_transform(parent, child_frame, stamp, timeout)
        except Exception as e:
            self.get_logger().warn(f'lookup_transform({parent}->{child_frame}) failed: {e}')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        rpy = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        return {
            'parent': parent,
            'child': child_frame,
            'xyz': [float(t.x), float(t.y), float(t.z)],
            'rpy': [float(rpy[0]), float(rpy[1]), float(rpy[2])],
            'quat_xyzw': [float(q.x), float(q.y), float(q.z), float(q.w)],
        }

    def _write_yaml_safe(self, path: Path, data: dict) -> None:
        tmp = path.with_suffix(path.suffix + '.tmp')
        with tmp.open('w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True, default_flow_style=False)
        tmp.replace(path)

    def _save_extrinsics_yaml(self, session_dir: Path) -> bool:
        """Write config/extrinsics.yaml with tool0→color/ir calib frames from TF."""
        color = self._lookup_tool0_to(self.tf_color_calib)
        ir    = self._lookup_tool0_to(self.tf_ir_calib)
        if color is None or ir is None:
            return False
        data = {
            'session_created_at': self._now_iso(),
            'frames': {
                'T_tool0_color': color,
                'T_tool0_ir': ir,
            }
        }
        out = session_dir / 'config' / 'extrinsics.yaml'
        self._write_yaml_safe(out, data)
        self.get_logger().info(f"Extrinsics written: {out}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
