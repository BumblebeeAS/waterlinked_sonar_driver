# waterlinked_sonar_driver

[![CI](https://github.com/BumblebeeAS/waterlinked_sonar_driver/actions/workflows/ci.yml/badge.svg)](https://github.com/BumblebeeAS/waterlinked_sonar_driver/actions/workflows/ci.yml)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue)](https://docs.ros.org/)
[![Release](https://img.shields.io/github/v/release/BumblebeeAS/waterlinked_sonar_driver)](https://github.com/BumblebeeAS/waterlinked_sonar_driver/releases)
[![License](https://img.shields.io/github/license/BumblebeeAS/waterlinked_sonar_driver)](LICENSE)

ROS 2 lifecycle driver for the [Water Linked Sonar 3D-15](https://docs.waterlinked.com/sonar-3d/sonar-3d-15-introduction/), built on [waterlinkedsonar](https://github.com/BumblebeeAS/waterlinkedsonar). Supports ROS 2 Humble and Jazzy.

## Setup

From the root of a colcon workspace:

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

The launch file configures and activates the lifecycle node. It accepts `namespace` and `params_file` arguments; the default parameter file is [`config/sonar.yaml`](config/sonar.yaml).

On activation the node sets the sonar's acoustics to `acoustics_enabled` and starts receiving. Deactivation stops the receiver and disables the acoustics.

The node is also available as a component, `waterlinked::sonar::ros::WaterlinkedSonarDriver`.

## Topics

| Topic | Type | Content |
|---|---|---|
| `~/point_cloud` | `sensor_msgs/PointCloud2` | XYZ float32; valid points only, or full height x width with NaN when `organized_cloud` is set |
| `~/range_image` | `sensor_msgs/Image` (`32FC1`) | distance in meters per pixel; 0.0 = no data |
| `~/intensity_image` | `sensor_msgs/Image` (`8UC1`) | logarithmic signal strength |
| `~/camera_info` | `sensor_msgs/CameraInfo` | pinhole-equivalent intrinsics derived from the FOV, stamped like the range or intensity image it accompanies |
| `~/imu` | `sensor_msgs/Imu` | ~100 Hz raw samples, without orientation; published when `imu_enabled` is set |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | receiver counters and sonar temperature/status |

The `~/` topics use reliable QoS with a depth of 10. Messages are stamped with their receipt time, or with the sonar clock when `time_source` is `device`. With receipt time, the IMU samples of one batch keep their device-time spacing and the newest is stamped with the receipt time. The IMU sits at (-22, -46, -3) mm from the point-cloud origin in `frame_id`; model that offset in the URDF.

## Services

| Service | Type | Action |
|---|---|---|
| `~/enable_acoustics` | `std_srvs/SetBool` | turn acoustic pinging on or off |
| `~/enable_high_frequency` | `std_srvs/SetBool` | `true` = high-frequency, `false` = low-frequency |
| `~/set_range` | `waterlinked_sonar_driver/SetRange` | set the imaging range, `float64 min, max` in meters |

## Parameters

Parameters are read-only and applied when the node is configured; change them by restarting the node. The full list, with descriptions and validation, is in [`src/waterlinked_sonar_driver_parameters.yaml`](src/waterlinked_sonar_driver_parameters.yaml).

When `ntp_address` is set, the node queries that server before configuring it on the sonar. If the server does not answer, or answers with a time before 2024, the sonar is given the unreachable address `192.0.2.1` instead, because a Water Linked device synchronized to a bad clock can become unreachable until power-cycled. `auto` is passed to the sonar without the check.

## Tests

`colcon test` runs the node against a fake sonar ([`test/fake_sonar.py`](test/fake_sonar.py)) and checks the lifecycle transitions, published messages, services and parameter validation.

## Attribution

[`test/data/ship_short.sonar`](test/data/ship_short.sonar) is from [waterlinked/wlsonar](https://github.com/waterlinked/wlsonar), Copyright (c) 2026 Water Linked, under the MIT license in [`test/data/LICENSE`](test/data/LICENSE).

## License

MIT. See [LICENSE](LICENSE).
