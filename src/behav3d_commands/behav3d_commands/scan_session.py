#!/usr/bin/env python3
from __future__ import annotations

from typing import Any, Dict, List, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped

from .session import Session


TOOL_PLUS_Z_POINTS_OUTWARD = False


class ScanSession(Session):
    """
    Session extension with scan-specific orchestration helpers.
    """

    def run(
        self,
        *,
        target: PoseStamped,
        distance: float,
        cap_rad: float,
        samples: int,
        prompt: Optional[str] = None,
        folder: Optional[str] = None,
        settle_s: float = 0.2,
        debug: bool = False,
        z_jitter: float = 0.0,
    ) -> Dict[str, Any]:
        return self.fib_scan(
            target=target,
            distance=distance,
            cap_rad=cap_rad,
            samples=samples,
            prompt=prompt,
            folder=folder,
            settle_s=settle_s,
            debug=debug,
            z_jitter=z_jitter,
        )

    def fib_scan(
        self,
        *,
        target: PoseStamped,
        distance: float,
        cap_rad: float,
        samples: int,
        prompt: Optional[str] = None,
        folder: Optional[str] = None,
        settle_s: float = 0.2,
        debug: bool = False,
        z_jitter: float = 0.0,
    ) -> Dict[str, Any]:
        if not isinstance(target, PoseStamped):
            raise TypeError("target must be geometry_msgs.msg.PoseStamped in 'world'.")
        if target.header.frame_id not in ("world", "", None):
            raise ValueError("target.header.frame_id must be 'world'.")

        if samples <= 0:
            return {
                "ok": True,
                "msg": "No samples requested (samples <= 0).",
                "poses_world": [],
                "params": {
                    "distance": distance,
                    "cap_rad": cap_rad,
                    "samples": samples,
                    "settle_s": settle_s,
                    "debug": debug,
                    "z_jitter": z_jitter,
                },
                "folder": folder,
            }

        p_t = np.array(
            [target.pose.position.x, target.pose.position.y, target.pose.position.z],
            dtype=float,
        )
        q_t = np.array(
            [
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ],
            dtype=float,
        )
        qn = np.linalg.norm(q_t)
        R_t = R.identity() if qn < 1e-12 else R.from_quat(q_t / qn)

        dirs_local = _fibonacci_cap_dirs_np(cap_rad, samples)
        order = np.arange(samples - 1, -1, -1, dtype=int)

        poses_world: List[PoseStamped] = []
        Rz_adjust = R.from_rotvec([0.0, 0.0, -np.pi / 2])

        for idx in order:
            d = dirs_local[idx]
            p_loc = distance * d

            z_axis = d if TOOL_PLUS_Z_POINTS_OUTWARD else -d
            x_guess = np.array([1.0, 0.0, 0.0])
            x_axis = x_guess - np.dot(x_guess, z_axis) * z_axis
            nx = np.linalg.norm(x_axis)
            if nx < 1e-9:
                y_guess = np.array([0.0, 1.0, 0.0])
                x_axis = y_guess - np.dot(y_guess, z_axis) * z_axis
                nx = np.linalg.norm(x_axis)
                if nx < 1e-12:
                    x_axis = _any_orthonormal(z_axis)
                    nx = np.linalg.norm(x_axis)
            x_axis /= nx
            y_axis = np.cross(z_axis, x_axis)
            y_axis /= max(np.linalg.norm(y_axis), 1e-12)

            R_loc = R.from_matrix(np.column_stack((x_axis, y_axis, z_axis))) * Rz_adjust

            p_w = p_t + R_t.apply(p_loc)
            R_w = R_t * R_loc

            if z_jitter > 0.0:
                ray_dir = p_t - p_w
                n_ray = np.linalg.norm(ray_dir)
                if n_ray > 1e-9:
                    ray_dir /= n_ray
                    delta = np.random.uniform(-z_jitter, +z_jitter)
                    p_w = p_w + delta * ray_dir

            ps_w = PoseStamped()
            ps_w.header.frame_id = "world"
            ps_w.pose.position.x, ps_w.pose.position.y, ps_w.pose.position.z = p_w.tolist()
            q_w = R_w.as_quat()
            ps_w.pose.orientation.x = float(q_w[0])
            ps_w.pose.orientation.y = float(q_w[1])
            ps_w.pose.orientation.z = float(q_w[2])
            ps_w.pose.orientation.w = float(q_w[3])
            poses_world.append(ps_w)

        first = True
        def _prepend_goto_for_index(i: int, cb):
            ps = poses_world[i]
            rpy = R.from_quat([
                ps.pose.orientation.x,
                ps.pose.orientation.y,
                ps.pose.orientation.z,
                ps.pose.orientation.w,
            ]).as_euler("xyz", degrees=False)

            self.prepend(
                "goto",
                {
                    "pose": ps,
                    "exec": (not debug),
                },
                cmd_kind="goto",
                on_done=cb,
            )

        def _after_goto(i: int):
            def _cb(res):
                nonlocal first
                ok = bool(res.get("ok", False))

                next_i = i - 1
                if next_i >= 0:
                    _prepend_goto_for_index(next_i, _after_goto(next_i))

                if not ok:
                    self.node.get_logger().warn(
                        f"[fib_scan] Move failed at viewpoint {i}; skipping capture/input."
                    )
                    return
                if debug:
                    return

                self.prepend(
                    "capture",
                    {
                        "rgb": True,
                        "depth": True,
                        "ir": True,
                        "pose": True,
                        "folder": (folder if first else None),
                    },
                    cmd_kind="capture",
                    on_done=None,
                )
                if prompt:
                    self.prepend(
                        "wait_input",
                        {"key": None, "prompt": prompt},
                        cmd_kind="input",
                        on_done=None,
                    )
                if settle_s > 0.0:
                    self.prepend(
                        "wait",
                        {"secs": float(settle_s)},
                        cmd_kind="wait",
                        on_done=None,
                    )

                first = False

            return _cb

        if poses_world:
            i0 = len(poses_world) - 1
            ps0 = poses_world[i0]
            rpy0 = R.from_quat([
                ps0.pose.orientation.x,
                ps0.pose.orientation.y,
                ps0.pose.orientation.z,
                ps0.pose.orientation.w,
            ]).as_euler("xyz", degrees=False)

            self.goto(
                x=ps0.pose.position.x,
                y=ps0.pose.position.y,
                z=ps0.pose.position.z,
                rx=float(rpy0[0]),
                ry=float(rpy0[1]),
                rz=float(rpy0[2]),
                exec=(not debug),
                on_done=_after_goto(i0),
            )

        self.wait(0.2)

        return {
            "ok": True,
            "poses_world": poses_world,
            "params": {
                "distance": distance,
                "cap_rad": cap_rad,
                "samples": samples,
                "settle_s": settle_s,
                "debug": debug,
            },
            "folder": folder,
        }


def _fibonacci_cap_dirs_np(cap_rad: float, n: int) -> np.ndarray:
    if n <= 0:
        return np.zeros((0, 3), dtype=float)

    cos_cap = float(np.cos(cap_rad))
    i = np.arange(n, dtype=float)
    z = cos_cap + (1.0 - cos_cap) * ((i + 0.5) / n)
    r_xy = np.sqrt(np.maximum(0.0, 1.0 - z * z))

    golden = (1.0 + np.sqrt(5.0)) * 0.5
    lon = 2.0 * np.pi * (i + 0.5) / golden

    x = r_xy * np.cos(lon)
    y = r_xy * np.sin(lon)

    dirs = np.stack([x, y, z], axis=1)
    norms = np.linalg.norm(dirs, axis=1, keepdims=True)
    norms = np.where(norms < 1e-12, 1.0, norms)
    return dirs / norms


def _any_orthonormal(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    idx = np.argmin(np.abs(v))
    basis = np.zeros(3, dtype=float)
    basis[idx] = 1.0
    u = basis - np.dot(basis, v) * v
    n = np.linalg.norm(u)
    return u / (n if n > 1e-12 else 1.0)
