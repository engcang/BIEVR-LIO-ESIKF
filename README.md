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
+ **Added** (09/27/2026): map-update sorting correction is now included in this repository. `BievrVoxelMap::update()` sorts by `(voxel x, y, z, point x, y, z, range)` so finite points with equal x have a canonical accumulation order.


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

# Performance - degenerate environments

<p align="center">
  <img src="assets/degenerate_benchmark_overview.svg" alt="BIEVR-LIO-ESIKF and FAST-LIO2 degenerate-environment benchmark comparison" width="80%" />
</p>

<p align="center">
  <img src="assets/degenerate_bievr_lio_official_comparison.svg" alt="BIEVR-LIO-ESIKF and BIEVR-LIO official degenerate-environment benchmark comparison" width="80%" />
</p>

### FAST-LIO2 comparison

+ FAST-LIO2 completed six of the eight sequences. It diverged on
  `SubT MRS Hawkins Multi Floor LegRobot` and `AgriLiRa4D NJFlatC04`; runtime and
  CPU measurements from those incomplete runs are omitted.

| Sequence | FAST-LIO2 APE RMSE (m) | BIEVR-LIO-ESIKF APE RMSE (m) | FAST-LIO2 runtime (ms/scan) | BIEVR-LIO-ESIKF runtime (ms/scan) | FAST-LIO2 CPU (% one core) | BIEVR-LIO-ESIKF CPU (% one core) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `SubT MRS Hawkins Long Corridor RC` | 3905.4493 | **1.9293** | 4.955 | **3.543** | **28.45** | 49.93 |
| `SubT MRS Hawkins Multi Floor LegRobot` | Failed | **0.4741** | — | **3.606** | — | **49.61** |
| `AgriLiRa4D NJFlatB03` | 23.1101 | **1.9511** | 16.547 | **6.842** | **56.98** | 94.85 |
| `AgriLiRa4D NJFlatC04` | Failed | **1.1624** | — | **5.802** | — | **84.26** |
| `AgriLiRa4D NJHillB03` | 49.6606 | **2.2454** | 18.116 | **6.964** | **61.29** | 95.40 |
| `AgriLiRa4D NJHillC03` | 130.3016 | **2.3123** | 19.049 | **7.033** | **62.50** | 95.86 |
| `AgriLiRa4D NJTerrB04` | 11.2758 | **0.4496** | 11.793 | **5.436** | **47.19** | 78.15 |
| `AgriLiRa4D NJTerrC05` | 14.0998 | **0.4733** | 12.499 | **5.690** | **50.01** | 81.20 |


### BIEVR-LIO official comparison

+ Runtime values are method-native wall-time latency: BIEVR-LIO official measures its complete
  `processFrame` step, while BIEVR-LIO-ESIKF measures synchronized LiDAR/IMU processing through
  odometry publication through BIEVR-map update completion.

| Sequence | BIEVR-LIO official APE RMSE (m) | BIEVR-LIO-ESIKF APE RMSE (m) | BIEVR-LIO official CPU (% one core) | BIEVR-LIO-ESIKF CPU (% one core) |
| --- | ---: | ---: | ---: | ---: |
| `SubT MRS Hawkins Long Corridor RC` | **1.8551** | 1.9293 | 52.71 | 49.93 |
| `SubT MRS Hawkins Multi Floor LegRobot` | 0.5000 | **0.4741** | 53.79 | 49.61 |
| `AgriLiRa4D NJFlatB03` | 2.9902 | **1.9511** | 111.15 | 94.85 |
| `AgriLiRa4D NJFlatC04` | **0.9588** | 1.1624 | 98.26 | 84.26 |
| `AgriLiRa4D NJHillB03` | **1.9502** | 2.2454 | 119.06 | 95.40 |
| `AgriLiRa4D NJHillC03` | **1.9631** | 2.3123 | 119.09 | 95.86 |
| `AgriLiRa4D NJTerrB04` | **0.4128** | 0.4496 | 89.83 | 78.15 |
| `AgriLiRa4D NJTerrC05` | **0.4379** | 0.4733 | 95.36 | 81.20 |


| Sequence | Implementation | Samples (scans) | Mean runtime (ms/scan) | p50 runtime (ms/scan) | p95 runtime (ms/scan) | p99 runtime (ms/scan) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `SubT MRS Hawkins Long Corridor RC` | BIEVR-LIO official | 2776 | 4.027 | 3.951 | 4.910 | 5.466 |
| `SubT MRS Hawkins Long Corridor RC` | BIEVR-LIO-ESIKF | 2774 | 3.543 | 3.523 | 4.252 | 4.786 |
| `SubT MRS Hawkins Multi Floor LegRobot` | BIEVR-LIO official | 4137 | 4.096 | 3.909 | 5.393 | 6.164 |
| `SubT MRS Hawkins Multi Floor LegRobot` | BIEVR-LIO-ESIKF | 4136 | 3.606 | 3.465 | 5.020 | 5.754 |
| `AgriLiRa4D NJFlatB03` | BIEVR-LIO official | 1588 | 6.448 | 6.365 | 8.060 | 9.364 |
| `AgriLiRa4D NJFlatB03` | BIEVR-LIO-ESIKF | 1586 | 6.842 | 6.808 | 8.246 | 9.211 |
| `AgriLiRa4D NJFlatC04` | BIEVR-LIO official | 2787 | 5.871 | 5.747 | 7.471 | 8.738 |
| `AgriLiRa4D NJFlatC04` | BIEVR-LIO-ESIKF | 2785 | 5.802 | 5.745 | 6.950 | 7.910 |
| `AgriLiRa4D NJHillB03` | BIEVR-LIO official | 1675 | 6.779 | 6.695 | 8.694 | 10.141 |
| `AgriLiRa4D NJHillB03` | BIEVR-LIO-ESIKF | 1673 | 6.964 | 6.918 | 8.588 | 9.735 |
| `AgriLiRa4D NJHillC03` | BIEVR-LIO official | 2374 | 6.781 | 6.667 | 8.936 | 10.590 |
| `AgriLiRa4D NJHillC03` | BIEVR-LIO-ESIKF | 2371 | 7.033 | 6.983 | 8.866 | 9.747 |
| `AgriLiRa4D NJTerrB04` | BIEVR-LIO official | 957 | 5.366 | 5.174 | 7.439 | 8.603 |
| `AgriLiRa4D NJTerrB04` | BIEVR-LIO-ESIKF | 955 | 5.436 | 5.023 | 7.878 | 9.128 |
| `AgriLiRa4D NJTerrC05` | BIEVR-LIO official | 1452 | 5.626 | 5.331 | 8.202 | 9.459 |
| `AgriLiRa4D NJTerrC05` | BIEVR-LIO-ESIKF | 1450 | 5.690 | 5.219 | 8.179 | 9.091 |

<br>

# Performance - non-degenerate environments

<p align="center">
  <img src="assets/benchmark_overview.svg" alt="BIEVR-LIO-ESIKF and FAST-LIO2 benchmark comparison" width="80%" />
</p>

<p align="center">
  <img src="assets/bievr_lio_official_comparison.svg" alt="BIEVR-LIO-ESIKF and BIEVR-LIO official benchmark comparison" width="80%" />
</p>

### FAST-LIO2 comparison

| Sequence | FAST-LIO2 APE RMSE (m) | BIEVR-LIO-ESIKF APE RMSE (m) | FAST-LIO2 runtime (ms/scan) | BIEVR-LIO-ESIKF runtime (ms/scan) | FAST-LIO2 CPU (% one core) | BIEVR-LIO-ESIKF CPU (% one core) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `HILTI Basement_1 (Mid70 / Ouster)` | 0.1646 | **0.1594** | **1.661** | 8.712 | **23.49** | 81.09 |
| `HILTI Construction_Site_2 (Mid70 / Ouster)` | 0.2922 | **0.0924** | **3.348** | 6.335 | **27.89** | 69.70 |
| `HILTI uzh_tracking_area_run2 (Mid70 / Ouster)` | 0.2368 | **0.1807** | **2.742** | 8.657 | **25.58** | 88.15 |
| `Newer College 01_short` | **0.3800** | 0.3988 | 14.264 | **8.509** | **57.68** | 103.23 |
| `Newer College 02_long` | **0.3485** | 0.4957 | 15.543 | **9.013** | **61.64** | 107.64 |
| `Newer College 05_quad` | 0.1030 | **0.0997** | 13.889 | **8.397** | **57.19** | 98.31 |
| `NTU VIRAL eee_02` | **0.0713** | 0.4181 | 9.248 | **4.225** | **43.69** | 61.27 |
| `NTU VIRAL nya_03` | **0.1029** | 0.1079 | 6.508 | **3.603** | **36.50** | 53.15 |
| `NTU VIRAL rtp_02` | 0.3966 | **0.2879** | 9.424 | **5.035** | **43.70** | 66.28 |
| `NTU VIRAL sbs_02` | 0.0723 | **0.0716** | 7.598 | **3.814** | **39.51** | 56.32 |
| `NTU VIRAL spms_01` | 0.2271 | **0.2092** | 10.051 | **4.585** | **45.53** | 63.87 |
| `NTU VIRAL tnp_02` | **0.0873** | 0.7544 | 6.673 | **4.360** | **36.75** | 56.38 |


### BIEVR-LIO official comparison

+ Official BIEVR-LIO values are the two-run means from its reference campaign. The
  BIEVR-LIO-ESIKF column uses the current one-run results.

| Sequence | BIEVR-LIO official APE RMSE (m) | BIEVR-LIO-ESIKF APE RMSE (m) | BIEVR-LIO official CPU (% one core) | BIEVR-LIO-ESIKF CPU (% one core) |
| --- | ---: | ---: | ---: | ---: |
| `GrandTour arc1 gt` | 0.0149 | **0.0110** | 77.32 | 75.51 |
| `GrandTour eig2 gt` | **0.0278** | 0.0299 | 70.02 | 72.61 |
| `HILTI LAB Survey 2 (Ouster)` | 0.0184 | **0.0168** | 95.86 | 86.81 |
| `HILTI UZH tracking area run 2 (Ouster)` | 0.1809 | **0.1807** | 104.12 | 88.15 |
| `Newer College 01 short experiment` | 0.4235 | **0.3988** | 150.11 | 103.23 |
| `Newer College 05 quad with dynamics` | **0.0963** | 0.0997 | 137.76 | 98.31 |
| `NTU VIRAL eee 03` | **0.1126** | 0.1131 | 68.25 | 60.56 |
| `NTU VIRAL spms 01` | **0.2069** | 0.2092 | 79.53 | 63.87 |
| `SubT MRS Hawkins Long Corridor RC` | **1.8381** | 1.9293 | 51.43 | 49.93 |


| Sequence | Implementation | Samples (scans) | Mean runtime (ms/scan) | p50 runtime (ms/scan) | p95 runtime (ms/scan) | p99 runtime (ms/scan) |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `GrandTour arc1 gt` | BIEVR-LIO official | 4332 | 5.541 | 5.572 | 6.952 | 7.630 |
| `GrandTour arc1 gt` | BIEVR-LIO-ESIKF | 4329 | 6.388 | 6.466 | 8.077 | 9.233 |
| `Newer College 01 short experiment` | BIEVR-LIO official | 15301 | 8.694 | 8.503 | 11.914 | 13.700 |
| `Newer College 01 short experiment` | BIEVR-LIO-ESIKF | 15299 | 8.509 | 8.481 | 11.026 | 12.202 |

<br>

## LICENSE

+ The oriented height-image map, residual/Jacobian, and map-informed sampling follow [BIEVR-LIO](https://github.com/ethz-asl/BIEVR-LIO). BIEVR-derived portions remain subject to their compatible upstream BSD 3-Clause terms
+ The iterated ESIKF and LiDAR-inertial processing lineage follows [FAST-LIO2](https://github.com/hku-mars/FAST_LIO). The repository as a whole is therefore distributed under GNU GPL version 2.
+ The BIEVR-derived implementation preserves the upstream BSD 3-Clause notice directly in
  `bievr_lio_esikf/include/bievr_voxel_map.hpp`.
+ `ankerl::unordered_dense` retains its upstream MIT notice in the vendored source headers.
