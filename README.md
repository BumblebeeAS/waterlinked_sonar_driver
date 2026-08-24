# waterlinked_sonar_driver

[![ci](https://github.com/BumblebeeAS/waterlinked_sonar_driver/actions/workflows/ci.yml/badge.svg)](https://github.com/BumblebeeAS/waterlinked_sonar_driver/actions/workflows/ci.yml)

ROS 2 lifecycle driver for the [Water Linked Sonar 3D-15](https://docs.waterlinked.com/sonar-3d/sonar-3d-15-introduction/), built on [BumblebeeAS/waterlinkedsonar](https://github.com/BumblebeeAS/waterlinkedsonar).

## Setup

```bash
git clone https://github.com/BumblebeeAS/waterlinked_sonar_driver.git src/waterlinked_sonar_driver
vcs import src < src/waterlinked_sonar_driver/dependencies.repos
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to waterlinked_sonar_driver
```

## Run

```bash
ros2 launch waterlinked_sonar_driver sonar.launch.py
```

The launch file configures and activates the lifecycle node. Deactivation stops the receiver and disables the sonar's acoustics.

## Topics

| Topic | Type | Content |
|---|---|---|
| `~/point_cloud` | `sensor_msgs/PointCloud2` | XYZ float32; valid points only, or full height x width with NaN when `organized_cloud` is set |
| `~/range_image` | `sensor_msgs/Image` (`32FC1`) | distance in meters per pixel; 0.0 = no data |
| `~/intensity_image` | `sensor_msgs/Image` (`8UC1`) | logarithmic signal strength |
| `~/camera_info` | `sensor_msgs/CameraInfo` | pinhole-equivalent intrinsics derived from the FOV |
| `~/imu` | `sensor_msgs/Imu` | ~100 Hz raw samples; published when `imu_enabled` is set |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | receiver counters and sonar temperature/status |

## Services

| Service | Type | Action |
|---|---|---|
| `~/enable_acoustics` | `std_srvs/SetBool` | turn acoustic pinging on or off |
| `~/enable_high_frequency` | `std_srvs/SetBool` | `true` = high-frequency, `false` = low-frequency |
| `~/set_range` | `waterlinked_sonar_driver/SetRange` | set the imaging range, `float64 min, max` in meters |

## Parameters

All parameters are read-only. The node applies them once during the configure transition; restart or cycle the lifecycle to change them. The full list with descriptions and validation is in [`src/waterlinked_sonar_driver_parameters.yaml`](src/waterlinked_sonar_driver_parameters.yaml).

## License

MIT
