# FAST-LIO for LASER

`fast_lio` estimates vehicle motion and builds a map from LiDAR scans and IMU measurements. This LASER fork provides the ROS 2 estimator used by the LUS LiDAR-odometry integration, including scan-corrected and high-frequency odometry outputs.

FAST-LIO2 uses an iterated error-state Kalman filter and incremental mapping with ikd-Tree. It can register raw points without a separate feature-extraction stage.

## Build and run

LUS supplies this repository through `laser_fast_lio_core`. With its dependencies available in a ROS 2 workspace:

```bash
colcon build --packages-up-to fast_lio
source install/setup.bash
```

Use the LASER wrapper to select a calibrated sensor profile:

```bash
export UAV_NAME=uav1
ros2 launch laser_fast_lio_core fast_lio.launch.py topic_pcl:=/uav1/livox/lidar topic_imu:=/uav1/livox/imu
```

To start the node through this package's own launch, pass a full configuration path:

```bash
ros2 launch fast_lio mapping.launch.py namespace:=uav1 config_file_path:=/absolute/path/to/sensor.yaml topic_pcl:=/uav1/livox/lidar topic_imu:=/uav1/livox/imu
```

The launch uses `config_file_path`; the older upstream `config_file` and `config_path` examples do not match this fork's launch arguments. `use_sim_time` defaults to `true` when `REAL_UAV=false`.

## Interfaces

The supplied mapping launch remaps node topics as follows for `uav1`:

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| Input | `topic_pcl` | `sensor_msgs/msg/PointCloud2` | LiDAR scan. |
| Input | `topic_imu` | `sensor_msgs/msg/Imu` | IMU samples. |
| Output | `/uav1/fast_lio/odometry` | `nav_msgs/msg/Odometry` | Scan-corrected state. |
| Output | `/uav1/fast_lio/odometry_high_freq` | `nav_msgs/msg/Odometry` | High-frequency propagated state. |
| Output | `/uav1/fast_lio/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Registered point cloud. |
| Output | `/uav1/fast_lio/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | Cloud in the body frame. |
| Output | `/uav1/fast_lio/cloud_effected`, `/uav1/fast_lio/laser_map` | `sensor_msgs/msg/PointCloud2` | Selected points and map output. |
| Output | `/uav1/fast_lio/imu/out` | `sensor_msgs/msg/Imu` | Estimator IMU output. |
| Output | `/uav1/fast_lio/path` | `nav_msgs/msg/Path` | Estimated path. |
| Service | `/uav1/map_save` | `std_srvs/srv/Trigger` | Saves the recorded map if PCD saving is enabled. |

This implementation subscribes to `PointCloud2`. The Livox `CustomMsg` subscription in the source is commented out; configure the driver or simulator to publish the point fields expected by the selected preprocessing mode.

## Configuration

The sensor YAML is passed directly to the node. LUS profiles live in `laser_fast_lio_core/params/`.

| Parameter | Meaning |
| --- | --- |
| `preprocess.lidar_type` | `2` selects the Velodyne handler, `3` Ouster and `4` Mid-360. Other values use the generic handler in this fork. |
| `preprocess.timestamp_unit` | Per-point time unit: `0` seconds, `1` milliseconds, `2` microseconds or `3` nanoseconds. |
| `preprocess.scan_line`, `preprocess.scan_rate`, `preprocess.blind` | Scan structure, nominal rate and minimum range. |
| `feature_extract_enable`, `point_filter_num` | Feature extraction and point thinning. |
| `filter_size_surf`, `filter_size_map`, `cube_side_length`, `max_iteration` | Surface/map resolution, local map size and filter iteration limit. |
| `common.time_sync_en`, `common.time_offset_lidar_to_imu` | Software synchronization and calibrated sensor time offset. |
| `mapping.acc_cov`, `gyr_cov`, `b_acc_cov`, `b_gyr_cov` | IMU measurement and bias noise settings. |
| `mapping.extrinsic_T`, `mapping.extrinsic_R` | LiDAR translation and rotation in the IMU frame. |
| `mapping.extrinsic_est_en` | Enables online extrinsic estimation. |
| `mapping.fov_degree`, `mapping.det_range` | Mapping field of view and range. |
| `publish.*` | Enables path, cloud and map outputs. |
| `pcd_save.pcd_save_en`, `pcd_save.interval`, `map_file_path` | Map recording, scan grouping and save path. |
| `imu_in_gravity_unit` | Acceleration is in g when `true`, m/s² when `false`. |
| `velocity_in_body_frame` | Selects the velocity output frame. |
| `transform.enable`, `transform.enable_imu` | Enables the configured TF handling. |
| `transform.fcu_frame`, `world_frame`, `lidar_frame`, `imu_frame` | Frame names within the `transform` group. |

LiDAR and IMU timestamps must agree, and scans need usable per-point timing for motion compensation. Prefer hardware synchronization; use the software sync option only when external synchronization is unavailable. Supply measured extrinsics and the TF relationships for your sensor mounting.

Enable PCD saving before calling `map_save`. An interval of `-1` retains all scans in one recording and can exhaust memory on long runs.

## References and credits

- [FAST-LIO2: Fast Direct LiDAR-inertial Odometry](https://doi.org/10.1109/TRO.2022.3141876).
- [FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196).
- [FAST-LIO2 video](https://youtu.be/2OvjGnxszf8) and [FAST-LIO1 video](https://youtu.be/iYCY6T79oNU).

Upstream contributors include Wei Xu, Yixi Cai, Dongjiao He, Fangcheng Zhu, Jiarong Lin, Zheng Liu and Borong Yuan. The ROS 2 fork credits [Ericsiii](https://github.com/Ericsii). FAST-LIO builds on work including LOAM, Livox Mapping, LINS, Loam Livox and [ikd-Tree](https://github.com/hku-mars/ikd-Tree).

The repository retains its [GPL version 2 license](LICENSE) and bundled dependency notices.
