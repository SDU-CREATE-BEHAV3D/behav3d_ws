#!/usr/bin/env python3
# Commands: internal async FIFO for motion; single executor for FollowJointTrajectory.

import math
from typing import Callable, Optional, Dict, Any, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from behav3d_interfaces.srv import PlanPilzPtp
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

        # Planning service (Pilz PTP via Motion Bridge)
        self._ptp_cli = node.create_client(PlanPilzPtp, "/behav3d/plan_pilz_ptp")
        
        # PrintTime action client (process / extruder)
        self._print_ac = ActionClient(node, PrintTime, "print")
 
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

    def goto(self,
             *,
             x: float, y: float, z: float,
             eef: str = "extruder_tcp",
             vel_scale: float = 0.1,
             accel_scale: float = 0.1,
             exec: bool = True,
             on_move_done: OnMoveDone = None,
             start_print: Optional[Dict[str, Any]] = None):
        """
        Enqueue plan (Pilz PTP) to (x, y, z) in 'world'.
        If exec=True, execution happens immediately after planning (atomic).
        """
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose = Pose()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0  # TODO: add Rx/Ry/Rz support

        self._enqueue("plan_ptp", {
            "pose": ps,
            "eef": eef,
            "vel_scale": float(vel_scale),
            "accel_scale": float(accel_scale),
            "exec": bool(exec),
            "start_print": start_print, 
            "on_move_done": on_move_done,
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

    # ---------------- Queue core ----------------

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
        elif kind == "plan_ptp":
            self._do_plan_ptp(p)
        elif kind == "print_time":
            self._do_print_time(p)

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
            self._finish_move(on_move_done, kind, ok=True, phase="exec",
                              metrics={"error_code": res.error_code})

        send_fut.add_done_callback(_on_goal_sent)

    def _do_plan_ptp(self, p: Dict[str, Any]):
        """
        Plan Pilz PTP via service. If exec=True, execute immediately (atomic plan+exec),
        keeping _busy=True until execution finishes.
        """
        ps: PoseStamped = p["pose"]
        eef = p["eef"]
        vs = p["vel_scale"]
        ac = p["accel_scale"]
        do_exec = p["exec"]
        on_move_done: OnMoveDone = p.get("on_move_done")
        start_print = p.get("start_print")

        if not self._ptp_cli.wait_for_service(timeout_sec=2.0):
            self._finish_move(on_move_done, "goto", ok=False, phase="plan",
                              error="plan_pilz_ptp not available")
            return

        req = PlanPilzPtp.Request()
        req.group_name = "ur_arm"
        req.eef_link = eef
        req.velocity_scale = vs
        req.accel_scale = ac
        req.preview_only = True
        req.target = ps

        self.node.get_logger().info(
            f"GOTO: planning Pilz PTP to ({ps.pose.position.x:.3f}, "
            f"{ps.pose.position.y:.3f}, {ps.pose.position.z:.3f}) eef={eef}..."
        )
        fut = self._ptp_cli.call_async(req)

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
                # Plan-only: finish here (planned_only=True); release busy and continue queue.
                self._finish_move(on_move_done, "goto", ok=True, phase="plan",
                                  metrics={"points": len(jt.points)}, planned_only=True)
                return

            # Atomic path: execute immediately without enqueueing or releasing _busy.
            # _do_follow_traj will call _finish_move when execution finishes, which releases _busy and advances the queue.
            self._do_follow_traj({"jt": jt, "on_move_done": on_move_done, "kind": "goto","start_print": start_print})

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
