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
ros2 topic pub --once /print_field_oriented_sequence/control std_msgs/msg/String "{data: stop}"
```
