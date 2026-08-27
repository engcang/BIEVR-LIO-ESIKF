<h1 align="center"><span>BIEVR-LIO-ESIKF</span></h1>

+ This repository tightly couples the [BIEVR](https://github.com/ethz-asl/BIEVR-LIO) oriented height-image measurement model to a
  [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)-family iterated error-state Kalman filter implemented with Eigen.
+ The LiDAR update uses BIEVR bump-image residuals, image-gradient Jacobians, robust Huber
  weighting, and map-informed sampling instead.
  + BIEVR voxel-wise oriented height images with bilinear residual sampling and central-difference
    image gradients.
  + BIEVR map-informed sampling: all points from high-score voxels and one representative point from
    the remaining voxels.
  + TBB-parallel preprocessing, sampling, residual construction, and map integration.
+ MTK, `MTK_BUILD_MANIFOLD`, IKFoM, ikd-Tree, and OpenMP are not used.
  + Explicit fixed-size Eigen implementation of the 17-dimensional error state, including the SO(3) perturbation and two-dimensional gravity error.
+ The proposed-method rows use the corrected right-error SO(3) covariance propagation, complete S² gravity transport, final S² covariance reset, and explicit first synchronized-scan discard.
+ Initial-IMU gravity alignment published as map_frame → odometry_frame without changing the estimator state or map.
+ A ROS-independent C++ core is shared by separate ROS1 and ROS2 wrappers.


<br>

## Dependencies

+ `C++` >= 17
+ `Eigen3`
+ `PCL`
+ `oneTBB`
+ `ROS1 Noetic` or `ROS2 Jazzy`
+ `livox_ros_driver` for ROS1 or `livox_ros_driver2` for ROS2

<br>

## How to install

### ROS2
  ```bash
  cd ~/<your_ros2_workspace>/src
  git clone <repository-url> BIEVR-LIO-ESIKF

  # ROS2
  cd ~/<your_catkin_workspace>
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  source install/setup.bash

  # ROS1
  cd ~/<your_catkin_workspace>
  source /opt/ros/noetic/setup.bash
  catkin build bievr_lio_esikf --cmake-args -DCMAKE_BUILD_TYPE=Release
  source devel/setup.bash
  ```

<br>

## How to use

### ROS2
  ```bash
  ros2 launch bievr_lio_esikf run.launch.py \
      config_file:=mid360.yaml \
      rviz:=true
  ```

### ROS1
  ```bash
  roslaunch bievr_lio_esikf run.launch \
      config_file:=mid360.yaml \
      rviz:=true
  ```

<br>

## Measurement model
- For a world-frame point transformed into an oriented cell as $p_C = T_{CW} p_W = [u, v, z]$, the residual is
  $$
  r = z - I(u / pixel\_size, v / pixel\_size).
  $$
- With $A = e_{zᵀ} R_{CW} - [I_u, I_v] R_{CW}(0:2,:) / pixel\_size$, the Jacobian used by the iterated ESIKF is
  $$
  H_{position} = A
  H_{rotation} = A (-R_{WI} [p_I]x).
  $$
- The rotation term follows this estimator's right SO(3) perturbation convention. The residual and Jacobian are both multiplied by the square root of the Huber weight before they enter the filter.

<br>

# Performance - non-degenerate environments

<p align="center">
  <img src="assets/benchmark_overview.svg" alt="BIEVR-LIO-ESIKF and FAST-LIO2 Original benchmark comparison" width="80%" />
</p>

<p align="center">
  <img src="assets/bievr_lio_official_comparison.svg" alt="BIEVR-LIO-ESIKF and BIEVR-LIO official benchmark comparison" width="80%" />
</p>

### FAST-LIO2 Original comparison

| Sequence | FAST-LIO2 Original APE RMSE (m) | BIEVR-LIO-ESIKF APE RMSE (m) | Original runtime (ms/scan) | BIEVR-LIO-ESIKF runtime (ms/scan) | Original CPU (% one core) | BIEVR-LIO-ESIKF CPU (% one core) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `HILTI Basement_1 (Mid70 / Ouster)` | 0.1646 | **0.1592** | **1.500** | 6.403 | **23.49** | 79.64 |
| `HILTI Construction_Site_2 (Mid70 / Ouster)` | 0.2922 | **0.0950** | **2.807** | 4.669 | **27.89** | 67.25 |
| `HILTI uzh_tracking_area_run2 (Mid70 / Ouster)` | 0.2368 | **0.1808** | **2.334** | 6.190 | **25.58** | 86.43 |
| `Newer College 01_short` | **0.3800** | 0.3982 | 13.225 | **5.107** | **57.68** | 101.84 |
| `Newer College 02_long` | **0.3485** | 0.4970 | 14.503 | **5.412** | **61.64** | 106.64 |
| `Newer College 05_quad` | 0.1030 | **0.0998** | 12.847 | **5.420** | **57.19** | 98.15 |
| `NTU VIRAL eee_02` | **0.0713** | 0.1245 | 8.285 | **2.683** | **43.69** | 59.53 |
| `NTU VIRAL nya_03` | **0.1029** | 0.1071 | 5.921 | **2.475** | **36.50** | 51.96 |
| `NTU VIRAL rtp_02` | **0.3966** | 2.5417 | 8.557 | **3.462** | **43.70** | 66.05 |
| `NTU VIRAL sbs_02` | 0.0723 | **0.0720** | 6.798 | **2.451** | **39.51** | 54.97 |
| `NTU VIRAL spms_01` | 0.2271 | **0.2094** | 8.716 | **2.947** | **45.53** | 62.81 |
| `NTU VIRAL tnp_02` | **0.0873** | 0.7506 | 6.071 | **3.063** | **36.75** | 56.29 |


### BIEVR-LIO official comparison

+ Official BIEVR-LIO values are the two-run means from its reference campaign. The
  BIEVR-LIO-ESIKF column uses the current corrected one-run results.

| Sequence | BIEVR-LIO official APE RMSE (m) | BIEVR-LIO-ESIKF APE RMSE (m) |
| --- | ---: | ---: |
| `GrandTour arc1 gt` | 0.0149 | **0.0111** |
| `GrandTour eig2 gt` | **0.0278** | 0.0299 |
| `HILTI LAB Survey 2 (Ouster)` | 0.0184 | **0.0168** |
| `HILTI UZH tracking area run 2 (Ouster)` | 0.1809 | **0.1808** |
| `Newer College 01 short experiment` | 0.4235 | **0.3982** |
| `Newer College 05 quad with dynamics` | **0.0963** | 0.0998 |
| `NTU VIRAL eee 03` | 0.1126 | **0.1106** |
| `NTU VIRAL spms 01` | **0.2069** | 0.2094 |
| `SubT MRS Hawkins Long Corridor RC` | **1.8381** | 1.9247 |


| Sequence | Implementation | Samples (scans) | Mean runtime (ms/scan) | p50 runtime (ms/scan) | p95 runtime (ms/scan) | p99 runtime (ms/scan) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `GrandTour arc1 gt` | BIEVR-LIO official | 4332 | 5.541 | 5.572 | 6.952 | 7.630 |
| `GrandTour arc1 gt` | BIEVR-LIO-ESIKF | 4329 | 4.607 | 4.675 | 5.622 | 6.672 |
| `Newer College 01 short experiment` | BIEVR-LIO official | 15301 | 8.694 | 8.503 | 11.914 | 13.700 |
| `Newer College 01 short experiment` | BIEVR-LIO-ESIKF | 15299 | 5.107 | 5.180 | 6.090 | 6.832 |

<br>

## LICENSE

+ The oriented height-image map, residual/Jacobian, and map-informed sampling follow [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO). BIEVR-derived portions remain subject to their compatible upstream BSD 3-Clause terms
+ The iterated ESIKF and LiDAR-inertial processing lineage follows [FAST-LIO2](https://github.com/hku-mars/FAST_LIO). The repository as a whole is therefore distributed under GNU GPL version 2.
+ The BIEVR-derived implementation preserves the upstream BSD 3-Clause notice directly in
  `bievr_lio_esikf/include/bievr_voxel_map.hpp`.
+ `ankerl::unordered_dense` retains its upstream MIT notice in the vendored source headers.
