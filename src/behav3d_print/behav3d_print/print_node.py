# behav3d_print/print_node.py

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,CancelResponse
from pymodbus.client import ModbusTcpClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import UInt32

from behav3d_interfaces.action import PrintTime  
from behav3d_interfaces.action import PrintSteps 
from behav3d_interfaces.srv import UpdatePrintConfig
from behav3d_interfaces.srv import GetPrintStatus

# --- Static config (no ROS params) ---
MODBUS_IP      = "192.168.1.10"
MODBUS_PORT    = 502
COIL_EXTRUDE   = 10   # ON/OFF coil
COIL_DIR       = 4    # direction coil
REG_SPEED      = 1    # holding register for speed
POLL_MS        = 500  # For getting info back from the Controllino
DIR_FORWARD    = True
DIR_REVERSE    = False

# --- Modbus addresses ---
COIL_QUEUE_PUSH = 13    # rising-edge enqueue
COIL_CANCEL_ALL = 14    # optional explicit cancel not implemented
REG_STEPS_LO    = 3     # HR3 low 16 bits
REG_STEPS_HI    = 4     # HR4 high 16 bits
IR_STEPS_LO   = 0   # input register 0: low 16 bits
IR_STEPS_HI   = 1   # input register 1: high 16 bits

class PrintNode(Node):
    def __init__(self):
        super().__init__("print_node")

        # --- Internal state (minimum) ---
        self.current_speed = 0          # active speed
        self.current_amount = 0         # target amount (placeholder, not used yet)
        self.extrude_enabled = False    # extruder ON/OFF
        self.reverse = False            # False=forward/away-from-limit, True=reverse/toward-limit
        self.state = "IDLE"             # IDLE | PRINTING | STOPPED | ERROR
        self._printing = False # internal handle to know if printing is active
        self._force_stop = False    
        
        # --- Modbus client ---
        self._mb_lock = threading.Lock()
        self.client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT, timeout=2.0)

        if not self._mb_connect():
            self.state = "ERROR"
            self.get_logger().error(f"Failed to connect to {MODBUS_IP}:{MODBUS_PORT}")
            raise SystemExit(1)

        self.get_logger().info(f"Connected to Modbus TCP {MODBUS_IP}:{MODBUS_PORT}")

        # --- Polling timer (500 ms) ---
        self._last_coil = None
        self._last_speed = None
        self._last_reverse = None
        self.timer = self.create_timer(POLL_MS / 1000.0, self._poll)

        # Initial safe state: extruder OFF (delay to avoid early reset during bringup)
        self._init_safe_done = False
        self._init_timer = self.create_timer(1.0, self._init_safe_state_once)


        # Service: update print configuration
        self.srv_update = self.create_service(
            UpdatePrintConfig,
            "update_print_config",
            self._on_update_config )
        # Service: Get print status
        self.srv_get_status = self.create_service(
            GetPrintStatus,
            "get_print_status",
            self._on_get_status
        )
    


        self._steps_action = ActionServer(
            self,
            PrintSteps,
            "print_steps",
            execute_callback=self._exec_print_steps,
            cancel_callback=self._on_cancel_steps,
        )

        # Action server: time-based print
        self._print_action = ActionServer(
            self,
            PrintTime,
            "print",
            execute_callback=self._exec_print_time,
            cancel_callback=self._on_cancel,
        )

    # -------- Modbus resilience helpers --------
    def _mb_connect(self) -> bool:
        """Connect with a couple retries. Call under lock."""
        with self._mb_lock:
            for _ in range(3):
                try:
                    if self.client.connect():
                        return True
                except Exception as e:
                    self.get_logger().warning(f"Modbus connect exception: {type(e).__name__}")
                time.sleep(0.3)
            return False

    def _mb_call(self, fn, *args, **kwargs):
        """
        Serialize Modbus calls and auto-reconnect once on broken socket/reset.
        Returns fn(...) result or None if it failed hard.
        """
        with self._mb_lock:
            try:
                return fn(*args, **kwargs)
            except (ConnectionResetError, BrokenPipeError, OSError) as e:
                # socket died -> reconnect and retry once
                self.get_logger().warning(f"Modbus socket error: {type(e).__name__}. Reconnecting...")
                try:
                    self.client.close()
                except Exception:
                    pass
                if not self.client.connect():
                    self.state = "ERROR"
                    return None
                try:
                    return fn(*args, **kwargs)
                except Exception as e2:
                    self.get_logger().error(f"Modbus retry failed: {type(e2).__name__}")
                    self.state = "ERROR"
                    return None
            except Exception as e:
                self.get_logger().error(f"Modbus call exception: {type(e).__name__}")
                self.state = "ERROR"
                return None

    def _init_safe_state_once(self):
        if self._init_safe_done:
            return
        self._init_safe_done = True
        try:
            self._init_timer.cancel()
        except Exception:
            pass
        # Best-effort OFF (won't crash node if server is still waking up)
        self.set_extrude(False)

    # -------- Minimal Modbus interface --------
    def set_extrude(self, on: bool) -> bool:
        """Turn extrusion ON/OFF via coil."""
        rr = self._mb_call(self.client.write_coil, COIL_EXTRUDE, bool(on))
        ok = (rr is not None) and (not rr.isError())
        if ok:
            self.extrude_enabled = bool(on)
            self.get_logger().info(f"coil[{COIL_EXTRUDE}] <- {on}")
        else:
            self.get_logger().error(f"Error writing coil {COIL_EXTRUDE}: {rr}")
            self.state = "ERROR"
        return ok

    def set_speed(self, value: int) -> bool:
        """Set speed in holding register (apply scaling if firmware requires)."""
        wr = self._mb_call(self.client.write_register, REG_SPEED, int(value))
        ok = (wr is not None) and (not wr.isError())
        if ok:
            self.current_speed = int(value)
            self.get_logger().info(f"reg[{REG_SPEED}] <- {value}")
        else:
            self.get_logger().error(f"Error writing reg {REG_SPEED}: {wr}")
            self.state = "ERROR"
        return ok

    def set_direction(self, reverse: bool) -> bool:
        """Set extrusion direction via coil 4. reverse=False is the normal print direction."""
        coil_value = DIR_REVERSE if bool(reverse) else DIR_FORWARD
        rr = self._mb_call(self.client.write_coil, COIL_DIR, bool(coil_value))
        ok = (rr is not None) and (not rr.isError())
        if ok:
            self.reverse = bool(reverse)
            self.get_logger().info(f"coil[{COIL_DIR}] <- {coil_value} (reverse={self.reverse})")
        else:
            self.get_logger().error(f"Error writing coil {COIL_DIR}: {rr}")
            self.state = "ERROR"
        return ok
    
    # -------- Low-level helpers --------
    def _write_u32(self, reg_lo: int, reg_hi: int, value: int) -> bool:
        lo = int(value & 0xFFFF)
        hi = int((value >> 16) & 0xFFFF)
        wr1 = self._mb_call(self.client.write_register, reg_lo, lo)
        wr2 = self._mb_call(self.client.write_register, reg_hi, hi)
        return (wr1 is not None) and (wr2 is not None) and (not wr1.isError()) and (not wr2.isError())

    def _pulse_coil(self, addr: int) -> bool:
        # momentary pulse: 1 then 0
        w1 = self._mb_call(self.client.write_coil, addr, True)
        w2 = self._mb_call(self.client.write_coil, addr, False)
        return (w1 is not None) and (w2 is not None) and (not w1.isError()) and (not w2.isError())


    def enqueue_steps(self, steps: int, ensure_on: bool = True) -> bool:
        """Queue a counted-step job at current HR1 speed."""
        if steps <= 0:
            self.get_logger().warn("enqueue_steps: steps<=0, ignored")
            return False
        if ensure_on and not self.extrude_enabled:
            if not self.set_extrude(True):
                return False
        if not self._write_u32(REG_STEPS_LO, REG_STEPS_HI, int(steps)):
            self.get_logger().error("Failed writing HR3|HR4")
            return False
        if not self._pulse_coil(COIL_QUEUE_PUSH):
            self.get_logger().error("Failed pulsing QUEUE_PUSH")
            return False
        self.get_logger().info(f"Queued {int(steps)} steps")
        return True
    #-------- check steps remaining--------

    def _read_steps_remaining_now(self) -> int:
        """Read steps_accum from IR0..1 atomically and return uint32."""
        ri = self._mb_call(self.client.read_input_registers, IR_STEPS_LO, count=2)
        if ri is None:
            return -1
        if ri.isError():
            self.get_logger().warning(f"Error reading IR0..1: {ri}")
            return -1
        lo = int(ri.registers[0])
        hi = int(ri.registers[1])
        return (hi << 16) | lo

    # -------- Polling (500 ms) --------
    def _poll(self):
        # Read coil (extrusion ON/OFF)
        rr = self._mb_call(self.client.read_coils, COIL_EXTRUDE, count=1)
        if rr is None:
            return
        if not rr.isError():
            coil = bool(rr.bits[0])
            if coil != self._last_coil:
                self._last_coil = coil
                self.get_logger().info(f"coil[{COIL_EXTRUDE}] => {coil}")
                self.extrude_enabled = coil
        else:
            self.get_logger().warning(f"Error reading coil {COIL_EXTRUDE}: {rr}")

        # Read direction coil
        rd = self._mb_call(self.client.read_coils, COIL_DIR, count=1)
        if rd is None:
            return
        if not rd.isError():
            dir_coil = bool(rd.bits[0])
            reverse = (dir_coil == DIR_REVERSE)
            if reverse != self._last_reverse:
                self._last_reverse = reverse
                self.reverse = reverse
                self.get_logger().info(f"coil[{COIL_DIR}] => {dir_coil} (reverse={reverse})")
        else:
            self.get_logger().warning(f"Error reading coil {COIL_DIR}: {rd}")

        # Read speed (holding register)
        rs = self._mb_call(self.client.read_holding_registers, REG_SPEED, count=1)
        if rs is None:
            return
        if not rs.isError():
            speed_val = int(rs.registers[0])
            if speed_val != self._last_speed:
                self._last_speed = speed_val
                # muted to avoid spamming; uncomment to log changes
                # self.get_logger().info(f"reg[{REG_SPEED}] => {speed_val}")
                self.current_speed = speed_val
        else:
            self.get_logger().warning(f"Error reading reg {REG_SPEED}: {rs}")

    # --- service callback ---
    def _on_update_config(self, req: UpdatePrintConfig.Request, res: UpdatePrintConfig.Response):
        """Apply selective updates to speed, direction, and/or extrude using flags."""
        # Apply speed if requested
        if req.set_speed:
            if not self.set_speed(req.speed):
                res.success = False
                res.message = f"Failed to set speed={req.speed}"
                return res

        if req.set_reverse:
            if not self.set_direction(req.reverse):
                res.success = False
                res.message = f"Failed to set reverse={req.reverse}"
                return res

        # Apply extrude if requested
        if req.set_extrude:
            if not self.set_extrude(req.extrude_on):
                res.success = False
                res.message = f"Failed to set extrude_on={req.extrude_on}"
                return res

        res.success = True
        res.message = "Config updated"
        return res

    def _on_get_status(self, req: GetPrintStatus.Request, res: GetPrintStatus.Response):
        # Read from internal, authoritative state
        res.speed = int(self.current_speed)
        res.extrude_on = bool(self.extrude_enabled)
        res.reverse = bool(self.reverse)
        res.state = str(self.state)
        return res

    # --- Action cancel callback ---
    def _on_cancel(self, goal_handle):
        # Accept cancel quickly; STOP must be done in execute loop
        return CancelResponse.ACCEPT

    # --- Action execute callback (sync) ---
    def _exec_print_time(self, goal_handle):
        goal = goal_handle.request
        target_speed = self.current_speed if goal.use_previous_speed else int(goal.speed)
        target_ms = int(goal.duration.sec * 1000 + goal.duration.nanosec / 1e6)

        if target_ms <= 0:
            goal_handle.abort()
            return PrintTime.Result(success=False, elapsed_ms=0, reason="invalid_duration")

        if not goal.use_previous_speed:
            if not self.set_speed(target_speed):
                goal_handle.abort()
                return PrintTime.Result(success=False, elapsed_ms=0, reason="modbus_speed_error")

        if not self.set_direction(bool(goal.reverse)):
            goal_handle.abort()
            return PrintTime.Result(success=False, elapsed_ms=0, reason="modbus_direction_error")

        if not self.set_extrude(True):
            goal_handle.abort()
            return PrintTime.Result(success=False, elapsed_ms=0, reason="modbus_extrude_on_error")

        self.state = "PRINTING"
        self._printing = True

        t0 = time.monotonic()
        t_stop = t0
        feedback = PrintTime.Feedback()

        FEED_DT = 0.05  # 50 ms feedback cadence

        try:
            while True:
                # --- early stops: cancel, service-forced OFF, or external OFF detected by polling ---
                if goal_handle.is_cancel_requested or self._force_stop or (self.state == "PRINTING" and self.extrude_enabled is False):
                    t_stop = time.monotonic()  # timestamp BEFORE cleanup
                    # ensure OFF (idempotent)
                    self.set_extrude(False)
                    self.state = "IDLE"
                    self._printing = False

                    # decide reason
                    if goal_handle.is_cancel_requested:
                        self._force_stop = False
                        goal_handle.canceled()
                        return PrintTime.Result(
                            success=False,
                            elapsed_ms=int((t_stop - t0) * 1000),
                            reason="canceled"
                        )
                    else:
                        reason = "stopped_by_service" if self._force_stop else "stopped_external"
                        self._force_stop = False
                        goal_handle.abort()
                        return PrintTime.Result(
                            success=False,
                            elapsed_ms=int((t_stop - t0) * 1000),
                            reason=reason
                        )

                # timing
                elapsed_ms = int((time.monotonic() - t0) * 1000)
                remaining_ms = target_ms - elapsed_ms

                if remaining_ms <= 0:
                    t_stop = time.monotonic()  # timestamp BEFORE cleanup
                    break

                # feedback
                feedback.elapsed_ms = elapsed_ms
                feedback.progress = min(1.0, elapsed_ms / float(max(1, target_ms)))
                goal_handle.publish_feedback(feedback)

                # sleep only what's left (cap to FEED_DT)
                sleep_s = FEED_DT if remaining_ms > (FEED_DT * 1000.0) else max(0.0, remaining_ms / 1000.0)
                time.sleep(sleep_s)

        except Exception as e:
            try:
                self.set_extrude(False)
            finally:
                self.state = "IDLE"
                self._printing = False
                self._force_stop = False
            goal_handle.abort()
            return PrintTime.Result(
                success=False,
                elapsed_ms=int((t_stop - t0) * 1000),
                reason=f"exception:{type(e).__name__}"
            )

        # cleanup AFTER timestamping normal completion
        try:
            self.set_extrude(False)
        finally:
            self.state = "IDLE"
            self._printing = False
            self._force_stop = False

        goal_handle.succeed()
        return PrintTime.Result(
            success=True,
            elapsed_ms=int((t_stop - t0) * 1000),
            reason="completed"
        )
    
    def _on_cancel_steps(self, goal_handle):
        # command for cancelling queued steps
        self.get_logger().info("Cancel requested for print_steps action")
        self.set_extrude(False)
        return CancelResponse.ACCEPT
    
    def _exec_print_steps(self, goal_handle):
        goal = goal_handle.request
        target_steps = int(goal.steps)
        if target_steps <= 0:
            goal_handle.abort()
            return PrintSteps.Result(success=False, accepted_steps=0, reason="invalid_steps")

        # Speed setup (if requested)
        if not goal.use_previous_speed:
            if not self.set_speed(int(goal.speed)):
                goal_handle.abort()
                return PrintSteps.Result(success=False, accepted_steps=0, reason="modbus_speed_error")

        if not self.set_direction(bool(goal.reverse)):
            goal_handle.abort()
            return PrintSteps.Result(success=False, accepted_steps=0, reason="modbus_direction_error")

        # Ensure ON
        if not self.set_extrude(True):
            goal_handle.abort()
            return PrintSteps.Result(success=False, accepted_steps=0, reason="modbus_extrude_on_error")

        # Reflect state for external status consumers
        self.state = "PRINTING"
        self._printing = True

        # Read baseline steps_accum
        baseline = self._read_steps_remaining_now()
        if baseline < 0:
            self.state = "IDLE"
            self._printing = False
            self._force_stop = False
            goal_handle.abort()
            return PrintSteps.Result(success=False, accepted_steps=0, reason="read_steps_error")

        # Enqueue (HR3|HR4 + QUEUE_PUSH)
        if (not self._write_u32(REG_STEPS_LO, REG_STEPS_HI, target_steps)) or (not self._pulse_coil(COIL_QUEUE_PUSH)):
            self.state = "IDLE"
            self._printing = False
            self._force_stop = False
            goal_handle.abort()
            return PrintSteps.Result(success=False, accepted_steps=0, reason="enqueue_failed")

        # Wait until steps_remaining <= baseline
        FEED_DT = 0.05
        try:
            while True:
                if goal_handle.is_cancel_requested:
                    self.set_extrude(False)
                    self.state = "IDLE"
                    self._printing = False
                    self._force_stop = False
                    goal_handle.canceled()
                    return PrintSteps.Result(success=False, accepted_steps=0, reason="canceled")

                current = self._read_steps_remaining_now()
                if current < 0:
                    time.sleep(FEED_DT)
                    continue

                if current <= baseline:
                    break

                time.sleep(FEED_DT)
        finally:
            # Ensure consistent state flags even on exceptions
            self.state = "IDLE"
            self._printing = False
            self._force_stop = False
            self.set_extrude(False)
        goal_handle.succeed()
        return PrintSteps.Result(success=True, accepted_steps=target_steps, reason="completed")

def main():
    rclpy.init()
    node = PrintNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.client.close()
        node.destroy_node()
        rclpy.shutdown()
