# Wheelbot_RDK-main Deployment Notes

This workspace keeps Wheelbot content optional and non-invasive. Do not copy the
whole Wheelbot source tree into `src`.

## What Was Imported

- Optional lidar packages are stored under:
  `/home/sunrise/rdk_transport_ws/third_party/wheelbot_lslidar`
- The optional lidar driver has ROS diagnostics removed so it can build on the
  current board image without installing extra system packages.
- A cleaned Wheelbot reference model is available inside:
  `rdk_transport_description/urdf/wheelbot_reference`
- The launch file uses a static URDF so it does not require `xacro` to be
  installed on the board. The cleaned xacro files are kept only as references.
- `mycar_control` was intentionally not imported because it writes to a serial
  port and current testing must not touch the STM32 serial link.

## Optional N10 Lidar Validation

The optional Wheelbot lidar packages are ignored by normal colcon builds. To
validate them explicitly on the board:

```bash
cd /home/sunrise/rdk_transport_ws
bash src/rdk_transport_bringup/scripts/prepare_wheelbot_lslidar.sh
source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
colcon build --packages-select lslidar_msgs lslidar_driver --symlink-install
source install/setup.bash
ros2 launch lslidar_driver rdk_n10_ftdi.launch.py
```

The validation launch uses:

```text
/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0
```

It publishes `/scan` with frame `laser`.

## Reference URDF Check

```bash
cd /home/sunrise/rdk_transport_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rdk_transport_description wheelbot_reference_tf.launch.py
```

Use this only as a reference model. The normal robot model remains
`rdk_transport.urdf`.

## Safety Notes

- Do not run `mycar_control`.
- Do not write to any STM32 serial device during this phase.
- Do not run `wheeltec_udev.sh`.
- Do not run Orbbec FirmwareUpgrade.
- Do not use fuzzy `pgrep -f` kills; inspect `/proc/$pid/comm` and
  `/proc/$pid/cmdline` before stopping processes.
