#ifndef COMMON_HPP
#define COMMON_HPP

#include <array>
#include <deque>
#include <utility>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

using LidarPoint = pcl::PointXYZINormal;
using LidarPointCloud = pcl::PointCloud<LidarPoint>;

struct State15D
{
    std::array<double, 3> pos_{};
    std::array<double, 9> rot_{};
    std::array<double, 3> vel_{};
    std::array<double, 3> acc_{};
    std::array<double, 3> gyro_{};
    double offset_time_ = 0.0;
};

struct ImuSample
{
    double timestamp_ = 0.0;
    std::array<double, 3> linear_acceleration_{};
    std::array<double, 3> angular_velocity_{};
};

enum LidarType
{
    LIVOX = 1,
    VELODYNE,
    OUSTER,
    MARSIM,
    HESAI
};

struct MeasureGroup // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        lidar_beg_time_ = 0.0;
        lidar_end_time_ = 0.0;
        lidar_measured_.reset(new LidarPointCloud());
    };

    double lidar_beg_time_;
    double lidar_end_time_;
    LidarPointCloud::Ptr lidar_measured_;
    std::deque<ImuSample> imu_measured_;
};

template<typename T>
auto setState15D(const double _t,
                 const Eigen::Matrix<T, 3, 1> &_a,
                 const Eigen::Matrix<T, 3, 1> &_g,
                 const Eigen::Matrix<T, 3, 1> &_v,
                 const Eigen::Matrix<T, 3, 1> &_p,
                 const Eigen::Matrix<T, 3, 3> &_r)
{
    State15D state;
    state.offset_time_ = _t;
    for (int i = 0; i < 3; ++i)
    {
        state.acc_[i] = _a(i);
        state.gyro_[i] = _g(i);
        state.vel_[i] = _v(i);
        state.pos_[i] = _p(i);
        for (int j = 0; j < 3; ++j)
        {
            state.rot_[i * 3 + j] = _r(i, j);
        }
    }
    return state;
}

#endif
