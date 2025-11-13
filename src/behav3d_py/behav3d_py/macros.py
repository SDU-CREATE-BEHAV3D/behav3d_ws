# macros.py
# High-level, queued macros built on top of Commands primitives.
# NumPy + SciPy version (concise, reliable rotations).

from typing import Any, Dict, List, Optional
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped

# Set this according to your TCP convention:
# True  -> tool +Z points OUTWARD (common when optical axis is -Z looking to target)
# False -> tool +Z points INWARD (directly at the target center)
TOOL_PLUS_Z_POINTS_OUTWARD = False


class Macros:
    """
    Compose higher-level routines from Commands.
    Initialize with a Commands instance that provides: goto, wait, input, capture.
    """

    def __init__(self, commands):
        self.cmd = commands  # Commands instance

    def fibScan(
        self,
        target: PoseStamped,          # must be in 'world'
        distance: float,
        cap_rad: float,
        samples: int,
        *,
        prompt: Optional[str] = None, # ENTER-only gate text
        folder: Optional[str] = None, # applied only on first capture
        settle_s: float = 0.2,
        debug: bool = False,          # plan-only (no exec)
        z_jitter: float = 0.0,        # +/- random offset on world Z (in meters)
    ) -> Dict[str, Any]:
        """
        Enqueue: for each viewpoint on a spherical cap (half-angle cap_rad) around 'target':
        goto -> wait(settle_s) -> input(prompt) -> capture

        Adds a uniform random offset in [-z_jitter, +z_jitter] to the world Z coordinate
        of each viewpoint if z_jitter > 0.
        """
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
                    "distance": distance, "cap_rad": cap_rad, "samples": samples,
                    "settle_s": settle_s, "debug": debug, "z_jitter": z_jitter
                },
                "folder": folder,
            }

        # Target pose (world)
        p_t = np.array(
            [target.pose.position.x, target.pose.position.y, target.pose.position.z],
            dtype=float,
        )
        q_t = np.array(
            [target.pose.orientation.x, target.pose.orientation.y,
            target.pose.orientation.z, target.pose.orientation.w],
            dtype=float,
        )
        qn = np.linalg.norm(q_t)
        R_t = R.identity() if qn < 1e-12 else R.from_quat(q_t / qn)

        # Fibonacci directions on spherical cap (target-local)
        dirs_local = _fibonacci_cap_dirs_np(cap_rad, samples)
        order = np.arange(samples - 1, -1, -1, dtype=int)

        poses_world: List[PoseStamped] = []
        Rz_pi = R.from_rotvec([0.0, 0.0, np.pi])

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

            R_loc = R.from_matrix(np.column_stack((x_axis, y_axis, z_axis))) * Rz_pi

            # Transform to world coordinates
            p_w = p_t + R_t.apply(p_loc)
            R_w = R_t * R_loc

            # Apply jitter along the ray to the target instead of world Z
            if z_jitter > 0.0:
                # direction from viewpoint towards the target (unit vector)
                ray_dir = p_t - p_w
                n_ray = np.linalg.norm(ray_dir)
                if n_ray > 1e-9:
                    ray_dir /= n_ray
                    delta = np.random.uniform(-z_jitter, +z_jitter)
                    p_w = p_w + delta * ray_dir


            # Create PoseStamped
            ps_w = PoseStamped()
            ps_w.header.frame_id = "world"
            ps_w.pose.position.x, ps_w.pose.position.y, ps_w.pose.position.z = p_w.tolist()
            q_w = R_w.as_quat()
            ps_w.pose.orientation.x = float(q_w[0])
            ps_w.pose.orientation.y = float(q_w[1])
            ps_w.pose.orientation.z = float(q_w[2])
            ps_w.pose.orientation.w = float(q_w[3])
            poses_world.append(ps_w)
        # Conditional chain via prepend:
        # After each move finishes, decide whether to prepend settle/input/capture,
        # and always prepend the next goto to keep the scan contiguous.

        first = True  # apply 'folder' only on the first successful capture

        def _prepend_goto_for_index(i: int, cb):
            """Build and prepend a plan_motion for poses_world[i]."""
            ps = poses_world[i]
            rpy = R.from_quat([
                ps.pose.orientation.x,
                ps.pose.orientation.y,
                ps.pose.orientation.z,
                ps.pose.orientation.w,
            ]).as_euler("xyz", degrees=False)

            self.cmd._prepend("plan_motion", {
                "pose": ps,
                "eef": self.cmd._default_eef,
                "vel_scale": self.cmd._default_vel_scale,
                "accel_scale": self.cmd._default_accel_scale,
                "exec": (not debug),
                "start_print": None,
                "on_move_done": cb,
                "motion": self.cmd._motion_mode,
            })

        def _after_goto(i: int):
            """Return a callback that decides the next steps for index i."""
            def _cb(res):
                nonlocal first
                ok = bool(res.get("ok", False))
                # Always schedule the next viewpoint first (so it executes last in the bundle)
                next_i = i - 1
                if next_i >= 0:
                    _prepend_goto_for_index(next_i, _after_goto(next_i))

                if not ok:
                    # Move failed: skip settle/input/capture for this viewpoint
                    self.cmd.node.get_logger().warn(f"[fibScan] Move failed at viewpoint {i}; skipping capture/input.")
                    return

                # Move succeeded: prepend capture chain in reverse order so it executes as:
                # wait(settle) -> input(prompt?) -> capture -> next goto
                self.cmd._prepend("capture", {
                    "rgb": True, "depth": True, "ir": True, "pose": True,
                    "folder": (folder if first else None),
                    "on_done": None,
                })
                if prompt:
                    self.cmd._prepend("wait_input", {
                        "key": None, "prompt": prompt, "on_done": None
                    })
                if settle_s > 0.0:
                    self.cmd._prepend("wait", {"secs": float(settle_s), "on_done": None})

                first = False
            return _cb

        # Kick off the chain by enqueuing (to tail) the first goto (last index n-1)
        if poses_world:
            i0 = len(poses_world) - 1
            ps0 = poses_world[i0]
            rpy0 = R.from_quat([
                ps0.pose.orientation.x, ps0.pose.orientation.y,
                ps0.pose.orientation.z, ps0.pose.orientation.w,
            ]).as_euler("xyz", degrees=False)

            self.cmd.goto(
                x=ps0.pose.position.x,
                y=ps0.pose.position.y,
                z=ps0.pose.position.z,
                rx=float(rpy0[0]),
                ry=float(rpy0[1]),
                rz=float(rpy0[2]),
                exec=(not debug),
                on_move_done=_after_goto(i0),
            )

        # Barrier
        self.cmd.wait(0.2)

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


# ---------------------------- Helpers ---------------------------------------

def _fibonacci_cap_dirs_np(cap_rad: float, n: int) -> np.ndarray:
    """Vectorized Fibonacci sampling on a spherical cap (half-angle cap_rad).
    Returns (n,3) outward unit directions in target-local coordinates.
    """
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
    """Return any unit vector orthonormal to v."""
    v = np.asarray(v, dtype=float)
    idx = np.argmin(np.abs(v))
    basis = np.zeros(3, dtype=float)
    basis[idx] = 1.0
    u = basis - np.dot(basis, v) * v
    n = np.linalg.norm(u)
    return u / (n if n > 1e-12 else 1.0)
