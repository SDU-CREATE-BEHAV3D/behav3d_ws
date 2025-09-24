# behav3d_print/print_node.py

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,CancelResponse
from pymodbus.client import ModbusTcpClient
from rclpy.executors import MultiThreadedExecutor

from behav3d_interfaces.action import PrintTime   
from behav3d_interfaces.srv import UpdatePrintConfig

# --- Static config (no ROS params) ---
MODBUS_IP      = "192.168.1.10"
MODBUS_PORT    = 502
COIL_EXTRUDE   = 10   # ON/OFF coil
REG_SPEED      = 1    # holding register for speed
POLL_MS        = 500  # For getting info back from the Controllino

class PrintNode(Node):
    def __init__(self):
        super().__init__("print_node")

        # --- Internal state (minimum) ---
        self.current_speed = 0          # active speed
        self.current_amount = 0         # target amount (placeholder, not used yet)
        self.extrude_enabled = False    # extruder ON/OFF
        self.state = "IDLE"             # IDLE | PRINTING | STOPPED | ERROR
        self._printing = False # internal handle to know if printing is active
        self._force_stop = False    
        # --- Modbus client ---
        self.client = ModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
        if not self.client.connect():
            self.state = "ERROR"
            self.get_logger().error(f"Failed to connect to {MODBUS_IP}:{MODBUS_PORT}")
            raise SystemExit(1)
        self.get_logger().info(f"Connected to Modbus TCP {MODBUS_IP}:{MODBUS_PORT}")

        # --- Polling timer (500 ms) ---
        self._last_coil = None
        self._last_speed = None
        self.timer = self.create_timer(POLL_MS / 1000.0, self._poll)

        # Initial safe state: extruder OFF
        self.set_extrude(False)

        # Service: update print configuration
        self.srv_update = self.create_service(
            UpdatePrintConfig,
            "update_print_config",
            self._on_update_config )

        # Action server: time-based print
        self._print_action = ActionServer(
            self,
            PrintTime,
            "print",
            execute_callback=self._exec_print,
            cancel_callback=self._on_cancel,
        )

    # -------- Minimal Modbus interface --------
    def set_extrude(self, on: bool) -> bool:
        """Turn extrusion ON/OFF via coil."""
        rr = self.client.write_coil(COIL_EXTRUDE, bool(on))
        ok = (not rr.isError())
        if ok:
            self.extrude_enabled = bool(on)
            self.get_logger().info(f"coil[{COIL_EXTRUDE}] <- {on}")
        else:
            self.get_logger().error(f"Error writing coil {COIL_EXTRUDE}: {rr}")
            self.state = "ERROR"
        return ok

    def set_speed(self, value: int) -> bool:
        """Set speed in holding register (apply scaling if firmware requires)."""
        wr = self.client.write_register(REG_SPEED, int(value))
        ok = (not wr.isError())
        if ok:
            self.current_speed = int(value)
            self.get_logger().info(f"reg[{REG_SPEED}] <- {value}")
        else:
            self.get_logger().error(f"Error writing reg {REG_SPEED}: {wr}")
            self.state = "ERROR"
        return ok

    # (placeholder) high-level API, amount/time/feedback not implemented yet
    def start_print(self, amount: int, speed: int) -> bool:
        """Skeleton: set speed and enable extrusion. (Stopped via stop_print)."""
        self.current_amount = int(amount)
        if not self.set_speed(speed):
            return False
        if not self.set_extrude(True):
            return False
        self.state = "PRINTING"
        return True

    def stop_print(self) -> None:
        """Turn extrusion OFF and set state back to IDLE."""
        self.set_extrude(False)
        self.state = "IDLE"

    # -------- Polling (500 ms) --------
    def _poll(self):
        # Read coil (extrusion ON/OFF)
        rr = self.client.read_coils(COIL_EXTRUDE, count=1)
        if not rr.isError():
            coil = bool(rr.bits[0])
            if coil != self._last_coil:
                self._last_coil = coil
                self.get_logger().info(f"coil[{COIL_EXTRUDE}] => {coil}")
                self.extrude_enabled = coil
        else:
            self.get_logger().warning(f"Error reading coil {COIL_EXTRUDE}: {rr}")

        # Read speed (holding register)
        rs = self.client.read_holding_registers(REG_SPEED, count=1)
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
        """Apply selective updates to speed and/or extrude using flags."""
        # Apply speed if requested
        if req.set_speed:
            if not self.set_speed(req.speed):
                res.success = False
                res.message = f"Failed to set speed={req.speed}"
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

    # --- Action cancel callback ---
    def _on_cancel(self, goal_handle):
        # Accept cancel quickly; STOP must be done in execute loop
        return CancelResponse.ACCEPT

    # --- Action execute callback (sync) ---
    def _exec_print(self, goal_handle):
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
