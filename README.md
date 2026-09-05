# EL05 ROS 2 Motor Control

ROS 2 Jazzy workspace for RobStride EL05 motors over a USB-CANHUB SocketCAN interface. Includes the motor protocol library, five-DOF controller, hardware test nodes, CAN setup script, and the EL05 manual.

## Hardware

- EL05 motors with CAN IDs `1` to `5`
- CAN master ID `253` (`0xFD`)
- USB-CANHUB exposed as Linux `can0`
- CAN bitrate: `1 Mbps`; positions are in radians

All motors share `can0`. Verify wiring, mechanical clearance, direction, limits, and an accessible emergency stop before enabling hardware.

## Build

Requires ROS 2 Jazzy and `colcon`:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/el05_serial_ws
colcon build --symlink-install
source install/setup.bash
```

After reconnecting the USB-CANHUB:

```bash
~/el05_serial_ws/scripts/setup_can0.sh
```

## Hardware tests

Single motor (ID 1), small motion:

```bash
ros2 run rs_motor_ros2 el05_joint1_sine --confirm-hardware --duration 10 \
  --amplitude 0.005 --frequency 0.1 --speed 0.1
```

Five motors, each centered on its measured current position:

```bash
ros2 run rs_motor_ros2 el05_all_sine --confirm-hardware --duration 60 \
  --amplitude 0.02 --frequency 0.1 --speed 0.1
```

Large amplitudes up to 30 degrees require a second explicit confirmation. Simultaneous five-motor test at ±30 degrees:

```bash
ros2 run rs_motor_ros2 el05_all_sine --confirm-hardware --confirm-large-motion \
  --duration 60 --amplitude 0.523599 --frequency 0.05 --speed 0.2
```

Leave a motor stationary with `--skip-id`; this example moves IDs 1, 3, 4, and 5:

```bash
ros2 run rs_motor_ros2 el05_all_sine --confirm-hardware --confirm-large-motion \
  --skip-id 2 --duration 60 --amplitude 0.523599 --frequency 0.05 --speed 0.2
```

Press `Ctrl+C` to stop. The nodes disable every motor they created on normal exit, interruption, or control errors. `0.523599` rad means ±30 degrees around the measured center; use `0.261799` for a 30-degree total stroke.

## Package layout

- `src/robstride_ros_sample`: `rs_motor_ros2` C++ package and motor protocol
- `src/el05_serial_tools`: optional serial monitor utility
- `scripts/setup_can0.sh`: SocketCAN initialization
- `docs/EL05使用说明书2600428.pdf`: hardware reference manual

Generated `build/`, `install/`, and `log/` directories are excluded from version control.
