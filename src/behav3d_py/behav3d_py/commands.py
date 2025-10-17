#!/usr/bin/env python3
# Commands: internal async FIFO for motion; single executor for FollowJointTrajectory.

import math
from typing import Callable, Optional, Dict, Any, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from behav3d_interfaces.srv import PlanPilzPtp, PlanPilzLin, GetLinkPose, Capture  

from behav3d_interfaces.action import PrintTime


OnMoveDone = Optional[Callable[[Dict[str, Any]], None]]

class Commands:
    """
    Minimal async command layer with a single FollowJointTrajectory executor.

    Public API:
      - home(duration_s=..., on_move_done=...)
      - goto(x, y, z, eef=..., vel_scale=..., accel_scale=..., exec=True, on_move_done=...)

    Queue item kinds:
      - "follow_traj": {'jt': JointTrajectory, 'on_move_done', 'kind': 'home'|'goto'}
      - "plan_ptp": {'pose': PoseStamped, 'eef', 'vel_scale', 'accel_scale', 'exec', 'on_move_done'}
    """

    def __init__(self, node: Node,
                 *,
                 controller_action: str = "/scaled_joint_trajectory_controller/follow_joint_trajectory"):
        self.node = node

        # Action client (controller)
        self._ctrl_client = ActionClient(node, FollowJointTrajectory, controller_action)

        # Planning and logging services  (Pilz PTP, LIN and Pose via Motion Bridge)
        self._ptp_cli = node.create_client(PlanPilzPtp, "/behav3d/plan_pilz_ptp")
        self._lin_cli = node.create_client(PlanPilzLin, "/behav3d/plan_pilz_lin") 
        self._pose_cli = node.create_client(GetLinkPose, "/behav3d/get_link_pose")
        self._capture_cli = node.create_client(Capture, "/capture")                                              
        # PrintTime action client (process / extruder)
        self._print_ac = ActionClient(node, PrintTime, "print")
        #TF buffer and listener
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)

        # DEFAULT
        self._motion_mode = "PTP"  # default
        self._default_eef = "extruder_tcp"
        self._default_vel_scale = 0.1
        self._default_accel_scale = 0.1

        # UR20 joint names
        self._joint_names = [
            "ur20_shoulder_pan_joint",
            "ur20_shoulder_lift_joint",
            "ur20_elbow_joint",
            "ur20_wrist_1_joint",
            "ur20_wrist_2_joint",
            "ur20_wrist_3_joint",
        ]

        # Default "home" (deg → rad)
        home_deg = [-90.0, -120.0, 120.0, 0.0, 90.0, -180.0]
        self._home_rad = [math.radians(d) for d in home_deg]

        # FIFO queue & busy flag
        self._queue: List[Tuple[str, Dict[str, Any]]] = []
        self._busy: bool = False

    # ---------------- Public API ----------------

    def home(self, *, duration_s: float = 10.0, on_move_done: OnMoveDone = None):
        """Enqueue a single-point trajectory to the predefined HOME configuration."""
        jt = JointTrajectory()
        jt.joint_names = list(self._joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = list(self._home_rad)
        pt.time_from_start.sec = int(duration_s)
        pt.time_from_start.nanosec = int((duration_s - int(duration_s)) * 1e9)
        jt.points.append(pt)
        self._enqueue("follow_traj", {"jt": jt, "on_move_done": on_move_done, "kind": "home"})
   
    def goto(self, *,
            x: float, y: float, z: float,
            rx: Optional[float] = None,
            ry: Optional[float] = None,
            rz: Optional[float] = None,
            eef: Optional[str] = None,
            vel_scale: Optional[float] = None,
            accel_scale: Optional[float] = None,
            exec: bool = True,
            on_move_done: OnMoveDone = None,
            start_print: Optional[Dict[str, Any]] = None,
            motion: Optional[str] = None):
      
        """Enqueue plan (Pilz PTP) to (x, y, z) in 'world'. Optional absolute RPY in radians."""
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)

        # Orientation: if any of rx/ry/rz provided, use them (others default to 0); else identity
        use_rpy = (rx is not None) or (ry is not None) or (rz is not None)
        if use_rpy:
            r = 0.0 if rx is None else float(rx)
            p = 0.0 if ry is None else float(ry)
            y_ = 0.0 if rz is None else float(rz)
            qx, qy, qz, qw = _quat_from_euler(r, p, y_)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
        else:
            ps.pose.orientation.w = 1.0

        self._enqueue("plan_motion", {
            "pose": ps,
            "eef": eef or self._default_eef,
            "vel_scale": self._default_vel_scale if vel_scale is None else float(vel_scale),
            "accel_scale": self._default_accel_scale if accel_scale is None else float(accel_scale),
            "exec": bool(exec),
            "start_print": start_print,
            "on_move_done": on_move_done,
            "motion": (motion or self._motion_mode)
        })

    def print(self,
            *,
            secs: float,
            speed: int,
            use_previous_speed: bool = False,
            on_done: OnMoveDone = None):
        """
        Enqueue a print-time action using the 'print' action server.
        Reuses the unified callback with kind='print', phase='exec'.
        """
        self._enqueue("print_time", {
            "secs": float(secs),
            "speed": int(speed),
            "use_prev": bool(use_previous_speed),
            "on_done": on_done
        })
 
    def PTP(self):  
        self._motion_mode = "PTP"

    def LIN(self):
        self._motion_mode = "LIN"
   
    def wait(self, secs: float, on_done: OnMoveDone = None):
        """Enqueue a time delay (secs) before the next command."""
        self._enqueue("wait", {"secs": float(secs), "on_done": on_done})

    def EEF(self, name: str):
        """Set default end-effector for all subsequent goto() calls."""
        self._default_eef = str(name)

    def SPD(self, val: float):
        """Set default velocity scale (clamped 0..1)."""
        self._default_vel_scale = max(0.0, min(1.0, float(val)))

    def ACC(self, val: float):
        """Set default accel scale (clamped 0..1)."""
        self._default_accel_scale = max(0.0, min(1.0, float(val)))

    def getPose(self,
                eef: str,
                base_frame: Optional[str] = "world",
                *,
                use_tf: bool = False,
                on_done: Optional[Callable[[Dict[str, Any]], None]] = None):
        """
        ALWAYS-QUEUED: enqueue a pose query so it respects FIFO order.
        - eef: link name (e.g., 'extruder_tcp')
        - base_frame: reference frame (default 'world'); None -> planning frame ('')
        - use_tf: reserved; TF path not implemented yet
        on_done will receive a dict with keys:
        ok, kind='get_pose', phase='exec', metrics{source, base_frame, link}, pose?, error?
        """
        self._enqueue("get_pose", {
            "link": str(eef),
            "base_frame": ("" if base_frame is None else str(base_frame)),
            "use_tf": bool(use_tf),
            "on_done": on_done,
        })

    def capture(self,
                *,
                rgb: bool = False,
                depth: bool = False,
                ir: bool = False,
                pose: bool = False,
                folder: Optional[str] = None,
                on_done: OnMoveDone = None):
        """
        Enqueue a capture request to the image manager.

        Semantics:
        - Each of {rgb, depth, ir, pose}: if True => set corresponding do_* = True, else False.
        - folder:
            * If folder is None  -> set_folder = False (leave current folder unchanged).
            * If folder is ""    -> set_folder = True and folder = "" (clear/reset).
            * If folder is "abc" -> set_folder = True and folder = "abc" (change).
        """
        self._enqueue("capture", {
            "rgb": bool(rgb),
            "depth": bool(depth),
            "ir": bool(ir),
            "pose": bool(pose),
            "folder": folder,      # None, "" or a path
            "on_done": on_done
        })

    def input(self,
            *,
            key: Optional[str] = None,
            prompt: Optional[str] = None,
            on_done: OnMoveDone = None):
        """
        Enqueue a keyboard input wait command.
        - key=None: waits for ENTER.
        - key='q': waits until the user types 'q' + ENTER. (not working in yaml yet)
        'on_done' will receive metrics={'value': <typed text>}.
        """
        self._enqueue("wait_input", {
            "key": (None if key is None else str(key)),
            "prompt": (prompt if prompt is not None else
                    ("Press ENTER to continue..." if key is None
                        else f"Type '{key}' + ENTER to continue...")),
            "on_done": on_done
        })

    # ---------------- Queue core ----------------
    def _prepend(self, kind: str, payload: Dict[str, Any]):
        """Insert a queue item at the head so it runs immediately after the current one."""
        self._queue.insert(0, (kind, payload))

    def _enqueue(self, kind: str, payload: Dict[str, Any]):
        """Push intent to FIFO and start processing if idle."""
        self._queue.append((kind, payload))
        if not self._busy:
            self._process_next()

    def _process_next(self):
        """Process a single queued item; called again only after completion."""
        if not self._queue:
            self._busy = False
            return
        self._busy = True
        kind, p = self._queue.pop(0)

        if kind == "follow_traj":
            self._do_follow_traj(p)
        elif kind == "plan_motion":
            self._do_plan_motion(p)
        elif kind == "print_time":
            self._do_print_time(p)
        elif kind == "wait":
            self._do_wait(p)
        elif kind == "get_pose":
            self._do_get_pose(p)
        elif kind == "capture":
            self._do_capture(p)
        elif kind == "wait_input":
            self._do_wait_input(p)

        else:
            self.node.get_logger().error(f"Unknown queue item kind: {kind}")
            self._busy = False
            self._process_next()  # continue with next item if any

    # ---------------- Executors ----------------

    def _do_follow_traj(self, p: Dict[str, Any]):
        """Single executor for any FollowJointTrajectory request."""
        jt: JointTrajectory = p["jt"]
        on_move_done: OnMoveDone = p.get("on_move_done")
        kind: str = p.get("kind", "goto")  # label for callback
        start_print: Optional[Dict[str, Any]] = p.get("start_print") 
        
        if not self._ctrl_client.wait_for_server(timeout_sec=2.0):
            self._finish_move(on_move_done, kind, ok=False, phase="exec",
                              error="controller not available")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        self.node.get_logger().info(f"{kind.upper()}: sending trajectory ({len(jt.points)} pts)...")
        send_fut = self._ctrl_client.send_goal_async(goal)

        def _on_goal_sent(f):
            handle = f.result()
            if not handle or not handle.accepted:
                self._finish_move(on_move_done, kind, ok=False, phase="exec",
                                  error="exec goal rejected")
                return
            if start_print:
                sync = (start_print.get("sync") or "accept").lower()
                if sync == "accept":  # start at motion start
                    offset_s = float(start_print.get("offset_s", 0.0))
                    secs = float(start_print["secs"])
                    speed = int(start_print["speed"])
                    self._start_print_concurrent(secs=secs, speed=speed, offset_s=offset_s)
            res_fut = handle.get_result_async()
            res_fut.add_done_callback(_on_result)

        def _on_result(rf):
            res = rf.result().result
            # control_msgs/FollowJointTrajectoryResult: 0 == SUCCESS
            err = int(getattr(res, "error_code", -999))
            is_ok = (err == 0)
            self._finish_move(on_move_done, kind, ok=is_ok, phase="exec",
                            metrics={"error_code": err},
                            error=(None if is_ok else f"exec failed (error_code={err})"))


        send_fut.add_done_callback(_on_goal_sent)

    def _do_plan_motion(self, p: Dict[str, Any]):
        ps = p["pose"]; eef = p["eef"]
        vs = p["vel_scale"]; ac = p["accel_scale"]
        do_exec = p["exec"]; on_move_done = p.get("on_move_done")
        start_print = p.get("start_print")
        motion = (p.get("motion") or "PTP").upper()

        # Pick service
        if motion == "PTP":
            cli = self._ptp_cli
            Req = PlanPilzPtp.Request
            label = "GOTO(PTP)"
        elif motion == "LIN":
            cli = self._lin_cli
            Req = PlanPilzLin.Request
            label = "GOTO(LIN)"
        else:
            self._finish_move(on_move_done, "goto", ok=False, phase="plan",
                            error=f"unknown motion mode '{motion}'")
            return

        if not cli.wait_for_service(timeout_sec=2.0):
            self._finish_move(on_move_done, "goto", ok=False, phase="plan",
                            error=f"{motion.lower()} planner not available")
            return

        req = Req()
        req.group_name = "ur_arm"
        req.eef_link = eef
        req.velocity_scale = vs
        req.accel_scale = ac
        req.preview_only = True
        req.target = ps

        self.node.get_logger().info(
            f"{label}: planning to ({ps.pose.position.x:.3f}, {ps.pose.position.y:.3f}, {ps.pose.position.z:.3f}) eef={eef}..."
        )
        fut = cli.call_async(req)

        def _on_planned(fr):
            resp = fr.result()
            if resp is None or not resp.success:
                self._finish_move(on_move_done, "goto", ok=False, phase="plan",
                                error="planning failed")
                return

            jt = resp.trajectory.joint_trajectory
            if len(jt.points) == 0:
                self._finish_move(on_move_done, "goto", ok=False, phase="plan",
                                error="empty trajectory")
                return

            if not do_exec:
                self._finish_move(on_move_done, "goto", ok=True, phase="plan",
                                metrics={"points": len(jt.points), "mode": motion},
                                planned_only=True)
                return

            # Atomic execution (same as before), pass through start_print
            self._do_follow_traj({"jt": jt, "on_move_done": on_move_done, "kind": "goto",
                                "start_print": start_print})

        fut.add_done_callback(_on_planned)

    def _do_print_time(self, p: Dict[str, Any]):
        """Execute PrintTime action (process op) in FIFO."""
        secs = p["secs"]
        speed = p["speed"]
        use_prev = p["use_prev"]
        on_done = p.get("on_done")

        if not self._print_ac.wait_for_server(timeout_sec=2.0):
            self._finish_move(on_done, "print", ok=False, phase="exec",
                            error="print action server not available")
            return

        goal = PrintTime.Goal()
        goal.duration.sec = int(secs)
        goal.duration.nanosec = int((secs - int(secs)) * 1e9)
        goal.speed = int(speed)
        goal.use_previous_speed = bool(use_prev)

        self.node.get_logger().info(f"PRINT: sending goal secs={secs:.2f} speed={speed} use_prev={use_prev}")
        fut = self._print_ac.send_goal_async(goal)  # omit feedback for now; add later if needed

        def _on_goal_response(gf):
            gh = gf.result()
            if not gh or not gh.accepted:
                self._finish_move(on_done, "print", ok=False, phase="exec",
                                error="print goal rejected")
                return
            res_fut = gh.get_result_async()
            res_fut.add_done_callback(_on_result)

        def _on_result(rf):
            try:
                wrap = rf.result()
                result = wrap.result
                status = wrap.status
                ok = bool(result.success)
                metrics = {
                    "status": int(status),
                    "elapsed_ms": int(result.elapsed_ms),
                    "reason": str(result.reason),
                }
                self._finish_move(on_done, "print", ok=ok, phase="exec",
                                metrics=metrics,
                                error=None if ok else result.reason)
            except Exception as e:
                self._finish_move(on_done, "print", ok=False, phase="exec",
                                error=f"exception waiting result: {e}")

        fut.add_done_callback(_on_goal_response)

    def _start_print_concurrent(self, *, secs: float, speed: int, offset_s: float = 0.0):
        """Fire PrintTime action without touching the FIFO (runs alongside motion)."""
        def _send_print_goal():
            if not self._print_ac.wait_for_server(timeout_sec=1.0):
                self.node.get_logger().warn("PRINT(concurrent): action server not available; skipping")
                return

            goal = PrintTime.Goal()
            goal.duration.sec = int(secs)
            goal.duration.nanosec = int((secs - int(secs)) * 1e9)
            goal.speed = int(speed)
            goal.use_previous_speed = False

            #self.node.get_logger().info(f"PRINT(concurrent): sending secs={secs:.2f} speed={speed}")
            fut = self._print_ac.send_goal_async(goal)

            def _on_goal_resp(gf):
                gh = gf.result()
                if not gh or not gh.accepted:
                    self.node.get_logger().error("PRINT(concurrent): goal rejected")
                    return
                res_fut = gh.get_result_async()
                def _on_res(rf):
                    try:
                        wrap = rf.result()
                        res = wrap.result
                        self.node.get_logger().info(
                            f"PRINT(concurrent): done status={wrap.status} "
                            f"elapsed_ms={res.elapsed_ms} reason='{res.reason}'speed={speed}")
                    except Exception as e:
                        self.node.get_logger().error(f"PRINT(concurrent): result exception: {e}")
                res_fut.add_done_callback(_on_res)

            fut.add_done_callback(_on_goal_resp)

        if offset_s > 0.0:
            def _oneshot():
                _send_print_goal()
                t.cancel()
            t = self.node.create_timer(offset_s, _oneshot)
        else:
            _send_print_goal()

    def _do_wait(self, p: Dict[str, Any]):
        secs = float(p["secs"])
        cb = p.get("on_done")
        self.node.get_logger().info(f"WAIT: delaying for {secs:.2f} s")

        t = None  # will hold the timer so we can cancel it

        def _one_shot():
            nonlocal t
            if t is not None:
                t.cancel()     # stop it from firing again
                t = None
            # now finish the command and advance FIFO
            self._finish_move(cb, "wait", ok=True, phase="exec", metrics={"secs": secs})

        t = self.node.create_timer(secs, _one_shot)

    def _do_get_pose(self, p: Dict[str, Any]):
        link = p["link"]
        base_frame = p["base_frame"]          # '' means "planning frame" for the MoveIt path
        use_tf = bool(p.get("use_tf", False))
        cb = p.get("on_done")

        # ---------- TF path ----------
        if use_tf:
            # Choose frames: if base_frame is empty, default to 'world' (adjust to your base if needed)
            target = base_frame if base_frame else "world"
            source = link

            timeout = Duration(seconds=0.3)
            stamp = Time()  # latest available transform

            try:
                ok = self._tf_buffer.can_transform(target, source, stamp, timeout)
            except Exception:
                ok = False

            if not ok:
                self._finish_move(cb, "get_pose", ok=False, phase="exec",
                                error=f"TF not available: {target} <- {source}",
                                metrics={"source": "tf", "base_frame": target, "link": source})
                return

            try:
                tf = self._tf_buffer.lookup_transform(target, source, stamp, timeout)
            except Exception as e:
                self._finish_move(cb, "get_pose", ok=False, phase="exec",
                                error=f"lookup_transform exception: {e}",
                                metrics={"source": "tf", "base_frame": target, "link": source})
                return

            ps = PoseStamped()
            ps.header = tf.header
            ps.header.frame_id = target
            ps.pose.position.x = tf.transform.translation.x
            ps.pose.position.y = tf.transform.translation.y
            ps.pose.position.z = tf.transform.translation.z
            ps.pose.orientation = tf.transform.rotation

            payload = {
                "ok": True,
                "kind": "get_pose",
                "phase": "exec",
                "planned_only": False,
                "error": None,
                "metrics": {"source": "tf"},
                "base_frame": target,
                "link": source,
                "pose": ps,
            }
            try:
                if cb:
                    cb(payload)
            finally:
                self._busy = False
                self._process_next()
            return

        # ---------- MoveIt service path ----------
        if not self._pose_cli.wait_for_service(timeout_sec=2.0):
            self._finish_move(cb, "get_pose", ok=False, phase="exec",
                            error="GetLinkPose service not available",
                            metrics={"source": "moveit", "base_frame": base_frame, "link": link})
            return

        req = GetLinkPose.Request()
        req.base_frame = base_frame  # '' -> planning frame on the backend
        req.link = link

        fut = self._pose_cli.call_async(req)

        def _on_resp(fr):
            try:
                resp = fr.result()
            except Exception as e:
                self._finish_move(cb, "get_pose", ok=False, phase="exec",
                                error=f"exception: {e}",
                                metrics={"source": "moveit", "base_frame": req.base_frame, "link": req.link})
                return

            if not resp or not getattr(resp, "success", False):
                self._finish_move(cb, "get_pose", ok=False, phase="exec",
                                error=getattr(resp, "message", "no response / failed"),
                                metrics={"source": "moveit", "base_frame": req.base_frame, "link": req.link})
                return

            payload = {
                "ok": True,
                "kind": "get_pose",
                "phase": "exec",
                "planned_only": False,
                "error": None,
                "metrics": {"source": "moveit"},
                "base_frame": req.base_frame,
                "link": req.link,
                "pose": resp.pose,
            }
            try:
                if cb:
                    cb(payload)
            finally:
                self._busy = False
                self._process_next()

        fut.add_done_callback(_on_resp)

    def _do_capture(self, p: Dict[str, Any]):
        on_done: OnMoveDone = p.get("on_done")

        if not self._capture_cli.wait_for_service(timeout_sec=2.0):
            self._finish_move(on_done, "capture", ok=False, phase="exec",
                            error="capture service not available")
            return

        req = Capture.Request()
        # booleans
        req.do_rgb   = bool(p.get("rgb", False))
        req.do_depth = bool(p.get("depth", False))
        req.do_ir    = bool(p.get("ir", False))
        req.do_pose  = bool(p.get("pose", False))

        # folder semantics
        folder = p.get("folder", None)
        if folder is None:
            req.set_folder = False
            req.folder = ""          # ignored by server when set_folder == False
        else:
            req.set_folder = True
            req.folder = str(folder) # may be "" to clear, or a path to set

        self.node.get_logger().info(
            f"CAPTURE: rgb={req.do_rgb} depth={req.do_depth} ir={req.do_ir} pose={req.do_pose} "
            f"set_folder={req.set_folder} folder='{req.folder}'"
        )

        fut = self._capture_cli.call_async(req)

        def _on_resp(fr):
            try:
                resp = fr.result()
            except Exception as e:
                self._finish_move(on_done, "capture", ok=False, phase="exec",
                                error=f"exception: {e}")
                return

            ok = bool(getattr(resp, "success", False))
            metrics = {"message": getattr(resp, "message", "")}
            self._finish_move(on_done, "capture", ok=ok, phase="exec",
                            metrics=metrics,
                            error=None if ok else metrics["message"])

        fut.add_done_callback(_on_resp)

    def _do_wait_input(self, p: Dict[str, Any]):
        import sys
        import threading

        key = p.get("key", None)
        prompt = p.get("prompt", "Press ENTER to continue...")
        cb = p.get("on_done")

        # If running without a TTY (e.g., launched headless), continue automatically.
        if not sys.stdin or not sys.stdin.isatty():
            self.node.get_logger().warn("WAIT_INPUT: no TTY detected; continuing automatically.")
            self._finish_move(cb, "input", ok=True, phase="exec",
                            metrics={"value": "", "reason": "no-tty-autocontinue"})
            return

        self.node.get_logger().info(f"WAIT_INPUT: {prompt}")

        def _reader():
            try:
                text = input()
                value = text.strip()
                if key is not None:
                    ok = (value == str(key))
                    self._finish_move(cb, "input", ok=ok, phase="exec",
                                    metrics={"value": value},
                                    error=None if ok else f"expected '{key}', got '{value}'")
                else:
                    self._finish_move(cb, "input", ok=True, phase="exec",
                                    metrics={"value": value})
            except Exception as e:
                self._finish_move(cb, "input", ok=False, phase="exec",
                                error=f"stdin exception: {e}")

        t = threading.Thread(target=_reader, daemon=True)
        t.start()

    # ---------------- Finish & advance ----------------

    def _finish_move(self,
                     cb: OnMoveDone,
                     kind: str,
                     *,
                     ok: bool,
                     phase: str,
                     error: Optional[str] = None,
                     metrics: Optional[Dict[str, Any]] = None,
                     planned_only: bool = False):
        """Common completion for moves (home/goto). Releases busy and advances FIFO."""
        try:
            if cb:
                cb({
                    "ok": ok,
                    "kind": kind,             # 'home' | 'goto'
                    "phase": phase,           # 'plan' | 'exec'
                    "planned_only": planned_only,
                    "error": error,
                    "metrics": metrics or {},
                })
        finally:
            self._busy = False
            self._process_next()

# ---------------- HELPERS ----------------


def _quat_from_euler(roll: float, pitch: float, yaw: float):
    """RPY (rad) -> quaternion (x,y,z,w), extrinsic XYZ (ROS-style)."""
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*cp*cy
    return x, y, z, w