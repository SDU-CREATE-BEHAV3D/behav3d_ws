# Experiment Campaign

## General Parameters

- Bead height: 20 mm
- Bead Width: 40 mm (+- 2)
- Bead spacing: 35 mm
- Net dot steps: 19002
- Nominal volume per bead: 23.32 mL
- Scan every: 7 Beads
## E01

- [x] **Offline Rule-Based**

- **Targets:** `/home/lab/behav3d_ws/yaml/E01/targets_rule_based.yaml`
- **Config:** `/home/lab/behav3d_ws/src/behav3d_orchestrator/config/print_yaml_and_scan_sequence_config.yaml`
- **Sequence:** `/home/lab/behav3d_ws/src/behav3d_orchestrator/behav3d_orchestrator/print_yaml_and_scan_sequence.py`

- **Pilot run:** `/home/lab/behav3d_ws/captures/261113_112808`

- [ ] **Iterative**

- **Field:** `/home/lab/behav3d_ws/mesh/fields/field_state_centered_30.npz`
- **Config:** `/home/lab/behav3d_ws/src/behav3d_orchestrator/config/geometry_representation_scan_print_loop_config.yaml`
- **Sequence:** `/home/lab/behav3d_ws/src/behav3d_orchestrator/behav3d_orchestrator/geometry_representation_scan_print_loop.py`

## E02
-  Extrusion_reduction_pct: 40.0%

- [x] **Offline Rule-Based (Fault Region)**

- **Targets:** `/home/lab/behav3d_ws/yaml/E01/targets_rule_based.yaml`
- **Mesh:** `/home/lab/behav3d_ws/mesh/fault_region.stl`
- **Config:** `/home/lab/behav3d_ws/src/behav3d_orchestrator/config/print_yaml_fault_and_scan_sequence_config.yaml`
- **Sequence:** `/home/lab/behav3d_ws/src/behav3d_orchestrator/behav3d_orchestrator/print_yaml_fault_and_scan_sequence.py`

- [ ] **Iterative (Fault Region)**

- **Field:** `/home/lab/behav3d_ws/mesh/fields/field_state_centered_30.npz`
- **Config:** `/home/lab/behav3d_ws/src/behav3d_orchestrator/config/geometry_representation_fault_scan_print_loop_config.yaml`
- **Mesh:** `/home/lab/behav3d_ws/mesh/fault_region.stl`
- **Sequence:** `/home/lab/behav3d_ws/src/behav3d_orchestrator/behav3d_orchestrator/geometry_representation_fault_scan_print_loop.py`

## E03

- [ ] **Iterative Addition**

- **Field:** `/home/lab/behav3d_ws/mesh/fields/field_state_centered_30.npz`
- **Config:** ``
- **Sequence:** ``

- [ ] **Iterative Removal**

- **Field:** `/home/lab/behav3d_ws/mesh/fields/field_state_centered_30.npz`
- **Config:** ``
- **Sequence:** ``