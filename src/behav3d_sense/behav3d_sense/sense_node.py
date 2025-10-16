#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from pathlib import Path
from datetime import datetime
import yaml

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
        self._ensure_manifest(self.current_capture_dir)

        # ---------- Camera handler (all subs/sync live here) ----------
        self.cam = FemtoCapture(self)

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

        self.tf_base = self.get_parameter('tf_base_frame').get_parameter_value().string_value
        self.tf_tool = self.get_parameter('tf_tool_frame').get_parameter_value().string_value
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
        session_dir = self.captures_root / self._timestamp_dir()
        return self._ensure_capture_folder(session_dir)

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
            'rgb': initial_fields.get('rgb', None),
            'depth': initial_fields.get('depth', None),
            'ir': initial_fields.get('ir', None),
            'pose_tool0': initial_fields.get('pose_tool0', None),
        }
        data['captures'].append(entry)
        self._write_manifest(mpath, data)
        return entry

    # ================= TF pose =================
    def _lookup_pose_tool0(self):
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
                fields['rgb'] = self.cam.save_color(self.current_capture_dir, idx, color_msg)
            elif self.cam.latest_color_raw is not None:
                self.get_logger().warn('Using unsynced latest COLOR frame (no synced pair).')
                fields['rgb'] = self.cam.save_color(self.current_capture_dir, idx, self.cam.latest_color_raw)
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
            fields['pose_tool0'] = self._lookup_pose_tool0()

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


def main(args=None):
    rclpy.init(args=args)
    node = SenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
