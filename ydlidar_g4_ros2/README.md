# ydlidar_g4_ros2

ROS 2 Jazzy driver for a YDLIDAR G4 configured for 230400 baud, 9 kHz sampling,
and 10 Hz rotation. It publishes the acquisition chronology explicitly: raw G4
clockwise angles are converted to ROS counter-clockwise angles while the scan
array retains first-ray order with a negative `angle_increment` and positive
`time_increment`.

## Run

```bash
ros2 launch ydlidar_g4_ros2 ydlidar_g4.launch.py port:=/dev/ydlidar_g4
```

Parameters are `port`, `frame_id`, `scan_topic`, `frequency`, and `sample_rate`.
`sample_rate` is expressed in kHz. The defaults are `/dev/ydlidar_g4`, `laser_link`,
`/scan`, `10.0`, and `9.0` respectively.

The scan range is fixed to the G4's 0.28--16 m operating range at 9 kHz. The SDK
is configured with `fixed_resolution=false`; this node performs the ROS angular
binning without sorting SDK points or changing their acquisition order.

The first SDK result has no complete rotation period. That partial result and
the first complete rotation are intentionally not published; publishing starts
after two consecutive complete rotations establish a stable timestamp baseline.
Later period values outside 80--120% of the configured rotation period use the
configured-frequency fallback. Published scan timestamps are kept monotonic
with a 1 microsecond float-rounding guard so downstream SLAM does not receive
rays projected into an earlier scan.

## Vendored SDK

The required `core/`, `src/`, and `LICENSE.txt` files are vendored mechanically
from the official [YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK) at commit
`01cdda4f2b36dff2a706d0535c64228d863c7411` (SDK 1.2.20). Vendored files remain
under their upstream license. The ROS node and scan conversion helpers are
project code and do not copy the official ROS publisher.

For JD-AMR, install `udev/99-ydlidar-g4.rules` under `/etc/udev/rules.d/` and
reload udev. The rule binds the robot's CP2102 adapter serial `0001` to the
stable `/dev/ydlidar_g4` name.
