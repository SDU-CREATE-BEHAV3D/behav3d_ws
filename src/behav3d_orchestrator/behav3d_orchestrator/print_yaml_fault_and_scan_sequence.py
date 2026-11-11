"""Print unchanged YAML with endpoint-selected underextrusion, then scan."""
import rclpy

from .print_yaml_and_scan_sequence import PrintYamlAndScanSequenceNode
from .src.fault_session import FaultSession


class PrintYamlFaultAndScanSequenceNode(PrintYamlAndScanSequenceNode):
    config_filename = "print_yaml_fault_and_scan_sequence_config.yaml"
    default_print_mode = "auto"

    def __init__(self):
        self.fault_session = FaultSession(self)
        self._loaded_segments = []
        super().__init__("print_yaml_fault_and_scan_sequence")

    def _configure_run(self, cfg):
        super()._configure_run(cfg)
        self._refresh_execution_config(cfg)

    def _refresh_execution_config(self, cfg):
        super()._refresh_execution_config(cfg)
        if getattr(self, "fault_session", None) is None:
            self.fault_session = FaultSession(self)
        self.fault_session.configure(
            cfg, config_path=self._default_config_path(), frame_id=self._execution_frame)
        self.print_session.fault_session = self.fault_session

    def _print_chunk(self, targets, *, volumes_mm3, plane_types, cfg, timeout_s,
                     target_start_index, total_target_count):
        if self._resolve_print_mode(cfg) == "segments":
            return self._print_segment_chunk(
                cfg=cfg, timeout_s=timeout_s,
                target_start_index=target_start_index, target_count=len(targets),
                total_target_count=total_target_count,
            )

        def before_target(local_index):
            if not self._cfg_bool(cfg, "prompt_before_print"):
                return True
            return not self._operator_requested_stop(self._target_print_prompt(
                target_index=target_start_index + local_index, total_target_count=total_target_count,
                first=local_index == 0))

        common = dict(steps_per_mm3=self._cfg_float(cfg, "extrusion_steps_per_mm3"),
                      eef_link=self._cfg_str(cfg, "print_eef_link"), timeout_s=timeout_s,
                      before_target=before_target)
        defaults = [self._cfg_int(cfg, "dot_steps_e" if t == "E" else "dot_steps_n") for t in plane_types]
        return self.print_session.run_print_dots_targets(
            targets=targets, volumes_mm3=volumes_mm3, dot_steps_per_target=defaults,
            target_indices=self._loaded_target_indices[target_start_index:target_start_index + len(targets)],
            dot_speed=self._cfg_int(cfg, "dot_speed"),
            approach_z_offset_m=self._cfg_float(cfg, "approach_z_offset_m"),
            dot_z_offset_m=self._cfg_float(cfg, "dot_z_offset_m"),
            pre_dot_vel_scale=self._cfg_float(cfg, "pre_dot_vel_scale"),
            dot_vel_scale=self._cfg_float(cfg, "dot_vel_scale"),
            accel_scale=self._cfg_float(cfg, "print_accel_scale"),
            dwell_s=self._cfg_float(cfg, "dwell_s"),
            post_dot_retract_steps=self._cfg_int(cfg, "post_dot_retract_steps"),
            post_dot_retract_speed=self._cfg_int(cfg, "post_dot_retract_speed"), **common)


def main(args=None):
    rclpy.init(args=args)
    node = PrintYamlFaultAndScanSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
