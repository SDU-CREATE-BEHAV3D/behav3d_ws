# Cheatsheet

```bash
colcon build && source ~/.bashrc
```

```bash
ros2 launch behav3d_bringup print_move.launch.py robot_ip:=192.168.1.8 use_mock_hardware:=false
```

```bash
ros2 action send_goal /print_steps behav3d_interfaces/action/PrintSteps "{steps: 64000, use_previous_speed: false, speed: 7000, reverse: true}"
```

```bash
ros2 run behav3d_orchestrator print_field_oriented_sequence_v2
```

```bash
ros2 topic pub --once /behav3d/control std_msgs/msg/String "{data: stop}"
```

```bash
python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_scan_loop_simulator_dds_v2.py \
  --field-mesh /home/lab/behav3d_ws/mesh/doublewall_iterative_loop_340mm.obj\
  --scan-mesh /home/lab/behav3d_ws/captures/260701_172314/field_loop/cycle_0000/scan/reconstruct/tsdf_surface_mesh.stl \
  --output-dir /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/output/loop_sim \
  --seed-level 5 \
  --position-target-x 0.00 \
  --position-target-y -0.80 \
  --t-coef 2000 \
  --field-subdivide-iter 0 \
  --field-scale 0.001 \
  --clearance 0.00 \
  --candidate-mode gradient_lift \
  --offset-distance-mm 12 \
  --offset-geodesic-delta-mm 0.6 \
  --beads-per-step 10 \
  --bead-separation-mm 36\
  --bead-width-mm 42 \
  --bead-height-mm 19 \
  --bead-shape cylinder \
  --walk-distance-mm 12 \
  --walk-step-mm 1.0 \
  --walk-max-steps 32 \
  --walk-tangent-sign 1.0 \
  --walk-start-fraction 0.45 \
  --clamp-to-cone \
  --cone-max-tilt-deg 45 \
  --positioning-attempts 3 \
  --search-step-x 0.01 \
  --search-step-y 0.01 \
  --base-z-offset 0.005\
  --axis-size -1 \
  --dds-voxel-size-mm 3.0 \
  --dds-threshold 0.5 \
  --dds-padding-mm 42 \
  --dds-surface-step-size 1 \
  --dds-view-mode surface \
  --dds-domain-source field \
  --dds-deposit-mode line \
  --dds-line-fraction 0.22 \
```

```bash
# FIXED WIDTH MODE

python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_state_scan_loop_simulator_dds_v2.py \
  --field-state /home/lab/behav3d_ws/mesh/fields/field_state_init2.npz \
  --scan-mesh /home/lab/behav3d_ws/mesh/ScanMesh.stl \
  --scan-scale 0.001 \
  --scan-yaw-deg 180 \
  --candidate-mode gradient_lift \
  --beads-per-step 7 \
  --candidate-width-mode fixed \
  --bead-separation-mm 30 \
  --bead-width-mm 36 \
  --bead-height-mm 16 \
  --clamp-to-cone \
  --cone-max-tilt-deg 30
```
```bash
# VARIABLE WIDTH FIELD MODE
# - width_norm=0 maps to bead-width-min-mm.
# - width_norm=1 maps to bead-width-max-mm.
# - Center spacing = width_i/2 + width_j/2 - overlap.
# - Overlap is enforced only between candidates from the current cycle.
# - bead-width-mm and bead-separation-mm do not control spacing in this mode.

python3 /home/lab/behav3d_ws/src/behav3d_py/behav3d_py/scalar_field/field_state_scan_loop_simulator_dds_v2.py \
  --field-state /home/lab/behav3d_ws/mesh/fields/field_state_init2.npz \
  --scan-mesh /home/lab/behav3d_ws/mesh/ScanMesh.stl \
  --scan-scale 0.001 \
  --scan-yaw-deg 180 \
  --candidate-mode gradient_lift \
  --beads-per-step 7 \
  --candidate-width-mode field \
  --width-field /home/lab/behav3d_ws/mesh/fields/width_field.npz \
  --bead-width-min-mm 16 \
  --bead-width-max-mm 36 \
  --bead-overlap-mm 4 \
  --bead-height-mm 14 \
  --clamp-to-cone \
  --cone-max-tilt-deg 30
```

```bash
  THIS ARE FOR ADJUSTING MESH WHEN IT COMES FROM RHINO WORLD COORDINTAES IN MM>
    --scan-scale 0.001 \
    --scan-yaw-deg 180 \
```

```bash
  ros2 service call /capture behav3d_interfaces/srv/Capture "{do_rgb: true, do_depth: true, do_ir: true, do_pose: true, set_folder: true, folder: '/home/lab/behav3d_ws/captures/test_capture'}"
```