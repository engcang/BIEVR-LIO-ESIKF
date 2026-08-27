#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common.hpp"
#include "eigen_lie.hpp"
#include "error_state_iterative_kalman_filter.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

inline bool comparePointTime(LidarPoint &_x,
                             LidarPoint &_y)
{
    return (_x.curvature < _y.curvature);
};

/// *************IMU process and undistortion
class ImuProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess() = default;

    void resetInitialize();
    void setExtrinsic(const Eigen::Vector3d &_transl, const Eigen::Matrix3d &_rot);
    void setGyroCov(const Eigen::Vector3d &_scaler);
    void setAccelCov(const Eigen::Vector3d &_scaler);
    void setGyroBiasCov(const Eigen::Vector3d &_b_g);
    void setAccelBiasCov(const Eigen::Vector3d &_b_a);
    void setLidarType(int _lidar_type);
    const Eigen::Vector3d &getLidarTranslationWrtImu() const;
    const Eigen::Matrix3d &getLidarRotationWrtImu() const;
    const Eigen::Quaterniond &getGravityAlignmentRotation() const;
    void forwardBackwardPropagation(const MeasureGroup &_meas,
                                    ErrorStateIterativeKalmanFilter &_esikf,
                                    LidarPointCloud::Ptr _undistorted_points);

private:
    void initializeImu(const MeasureGroup &_meas,
                       ErrorStateIterativeKalmanFilter &_esikf,
                       int &_sample_count);
    void undistortPoints(const MeasureGroup &_meas,
                         ErrorStateIterativeKalmanFilter &_esikf,
                         LidarPointCloud &_undistorted_points);

    ProcessNoiseCovariance process_noise_covariance_;
    Eigen::Vector3d accel_covariance_;
    Eigen::Vector3d gyro_covariance_;
    Eigen::Vector3d accel_covariance_scale_;
    Eigen::Vector3d gyro_covariance_scale_;
    Eigen::Vector3d gyro_bias_covariance_;
    Eigen::Vector3d accel_bias_covariance_;
    ImuSample last_imu_;
    std::vector<State15D> imu_poses_;
    Eigen::Matrix3d lidar_to_imu_rotation_;
    Eigen::Vector3d lidar_to_imu_translation_;
    Eigen::Quaterniond gravity_alignment_rotation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d mean_acc_;
    Eigen::Vector3d mean_gyro_;
    Eigen::Vector3d last_gyro_;
    Eigen::Vector3d last_world_acc_ = Eigen::Vector3d::Zero();
    double last_lidar_end_time_ = 0.0;
    int init_sample_count_ = 1;
    int lidar_type_ = LIVOX;
    bool if_first_frame_ = true;
    bool if_imu_need_init_ = true;
};

inline ImuProcess::ImuProcess()
{
    init_sample_count_ = 1;
    process_noise_covariance_ = setProcessNoiseCov();
    accel_covariance_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    gyro_covariance_ = Eigen::Vector3d(0.1, 0.1, 0.1);
    gyro_bias_covariance_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
    accel_bias_covariance_ = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
    mean_acc_ = Eigen::Vector3d(0, 0, -1.0);
    mean_gyro_ = Eigen::Vector3d(0, 0, 0);
    last_gyro_ = Eigen::Vector3d::Zero();
    last_world_acc_ = Eigen::Vector3d::Zero();
    last_lidar_end_time_ = 0.0;
    gravity_alignment_rotation_.setIdentity();
    lidar_to_imu_translation_ = Eigen::Vector3d::Zero();
    lidar_to_imu_rotation_ = Eigen::Matrix3d::Identity();
    last_imu_ = ImuSample{};
}

inline void ImuProcess::resetInitialize()
{
    // ROS_WARN("reset ImuProcess");
    mean_acc_ = Eigen::Vector3d(0, 0, -1.0);
    mean_gyro_ = Eigen::Vector3d(0, 0, 0);
    last_gyro_ = Eigen::Vector3d::Zero();
    last_world_acc_ = Eigen::Vector3d::Zero();
    last_lidar_end_time_ = 0.0;
    gravity_alignment_rotation_.setIdentity();
    if_imu_need_init_ = true;
    init_sample_count_ = 1;
    imu_poses_.clear();
    last_imu_ = ImuSample{};
}

inline void ImuProcess::setExtrinsic(const Eigen::Vector3d &_transl,
                                     const Eigen::Matrix3d &_rot)
{
    lidar_to_imu_translation_ = _transl;
    lidar_to_imu_rotation_ = _rot;
}

inline void ImuProcess::setGyroCov(const Eigen::Vector3d &_scaler)
{
    gyro_covariance_scale_ = _scaler;
}

inline void ImuProcess::setAccelCov(const Eigen::Vector3d &_scaler)
{
    accel_covariance_scale_ = _scaler;
}

inline void ImuProcess::setGyroBiasCov(const Eigen::Vector3d &_b_g)
{
    gyro_bias_covariance_ = _b_g;
}

inline void ImuProcess::setAccelBiasCov(const Eigen::Vector3d &_b_a)
{
    accel_bias_covariance_ = _b_a;
}

inline void ImuProcess::setLidarType(int _lidar_type)
{
    lidar_type_ = _lidar_type;
}

inline const Eigen::Vector3d &ImuProcess::getLidarTranslationWrtImu() const
{
    return lidar_to_imu_translation_;
}

inline const Eigen::Matrix3d &ImuProcess::getLidarRotationWrtImu() const
{
    return lidar_to_imu_rotation_;
}

inline const Eigen::Quaterniond &ImuProcess::getGravityAlignmentRotation() const
{
    return gravity_alignment_rotation_;
}

inline void ImuProcess::initializeImu(const MeasureGroup &_meas,
                                      ErrorStateIterativeKalmanFilter &_esikf,
                                      int &_sample_count)
{
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/

    Eigen::Vector3d cur_acc, cur_gyr;

    if (if_first_frame_)
    {
        resetInitialize();
        _sample_count = 1;
        if_first_frame_ = false;
        const auto &imu_acc = _meas.imu_measured_.front().linear_acceleration_;
        const auto &gyr_acc = _meas.imu_measured_.front().angular_velocity_;
        mean_acc_ << imu_acc[0], imu_acc[1], imu_acc[2];
        mean_gyro_ << gyr_acc[0], gyr_acc[1], gyr_acc[2];
    }

    for (const auto &imu : _meas.imu_measured_)
    {
        const auto &imu_acc = imu.linear_acceleration_;
        const auto &gyr_acc = imu.angular_velocity_;
        cur_acc << imu_acc[0], imu_acc[1], imu_acc[2];
        cur_gyr << gyr_acc[0], gyr_acc[1], gyr_acc[2];

        mean_acc_ += (cur_acc - mean_acc_) / _sample_count;
        mean_gyro_ += (cur_gyr - mean_gyro_) / _sample_count;

        accel_covariance_ = accel_covariance_ * (_sample_count - 1.0) / _sample_count + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (_sample_count - 1.0) / (_sample_count * _sample_count);
        gyro_covariance_ = gyro_covariance_ * (_sample_count - 1.0) / _sample_count + (cur_gyr - mean_gyro_).cwiseProduct(cur_gyr - mean_gyro_) * (_sample_count - 1.0) / (_sample_count * _sample_count);

        // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc_.norm()<<endl;

        _sample_count++;
    }
    LioState init_state = _esikf.getState();
    init_state.gravity_direction_ = -mean_acc_ / mean_acc_.norm() * kGravityNorm;

    const Eigen::Matrix3d gravity_rotation = Eigen::Quaterniond::FromTwoVectors(mean_acc_.normalized(),
                                                                                Eigen::Vector3d::UnitZ())
                                                 .toRotationMatrix();
    const double gravity_rotation_yaw = std::atan2(gravity_rotation(1, 0), gravity_rotation(0, 0));
    gravity_alignment_rotation_ = Eigen::Quaterniond(Eigen::AngleAxisd(-gravity_rotation_yaw, Eigen::Vector3d::UnitZ()) * gravity_rotation);
    gravity_alignment_rotation_.normalize();

    // state_inout.rotation_ = Eigen::Matrix3d::Identity(); // Exp(mean_acc_.cross(Eigen::Vector3d(0, 0, -1 / scale_gravity)));
    init_state.gyro_bias_ = mean_gyro_;
    _esikf.setState(init_state);

    StateCovariance init_p = _esikf.getCovariance();
    init_p.setIdentity();
    init_p.block<3, 3>(K_GYRO_BIAS, K_GYRO_BIAS).diagonal().setConstant(0.0001);
    init_p.block<3, 3>(K_ACC_BIAS, K_ACC_BIAS).diagonal().setConstant(0.001);
    init_p.block<2, 2>(K_GRAVITY, K_GRAVITY).diagonal().setConstant(0.00001);
    _esikf.setCovariance(init_p);
    last_imu_ = _meas.imu_measured_.back();
}

inline void ImuProcess::undistortPoints(const MeasureGroup &_meas,
                                        ErrorStateIterativeKalmanFilter &_esikf,
                                        LidarPointCloud &_undistorted_points)
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = _meas.imu_measured_;
    v_imu.push_front(last_imu_);
    const double imu_end_time = v_imu.back().timestamp_;

    double pcl_beg_time = _meas.lidar_beg_time_;
    double pcl_end_time = _meas.lidar_end_time_;

    if (lidar_type_ == MARSIM)
    {
        pcl_beg_time = last_lidar_end_time_;
        pcl_end_time = _meas.lidar_beg_time_;
    }

    /*** sort point clouds by offset time ***/
    _undistorted_points = *(_meas.lidar_measured_);
    std::sort(_undistorted_points.points.begin(), _undistorted_points.points.end(), comparePointTime);
    /*** Initialize IMU pose ***/
    LioState imu_state = _esikf.getState();
    imu_poses_.clear();
    imu_poses_.push_back(setState15D(0.0, last_world_acc_, last_gyro_, imu_state.velocity_, imu_state.position_, imu_state.rotation_.toRotationMatrix()));

    /*** forward propagation at each imu point ***/
    Eigen::Vector3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    Eigen::Matrix3d r_imu;

    double dt = 0;

    ImuInput in;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
    {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);

        if (tail.timestamp_ < last_lidar_end_time_)
            continue;

        angvel_avr << 0.5 * (head.angular_velocity_[0] + tail.angular_velocity_[0]),
            0.5 * (head.angular_velocity_[1] + tail.angular_velocity_[1]),
            0.5 * (head.angular_velocity_[2] + tail.angular_velocity_[2]);
        acc_avr << 0.5 * (head.linear_acceleration_[0] + tail.linear_acceleration_[0]),
            0.5 * (head.linear_acceleration_[1] + tail.linear_acceleration_[1]),
            0.5 * (head.linear_acceleration_[2] + tail.linear_acceleration_[2]);


        acc_avr = acc_avr * kGravityNorm / mean_acc_.norm(); // - state_inout.acc_bias_;

        if (head.timestamp_ < last_lidar_end_time_)
        {
            dt = tail.timestamp_ - last_lidar_end_time_;
            // dt = tail->header.stamp.toSec() - pcl_beg_time;
        }
        else
        {
            dt = tail.timestamp_ - head.timestamp_;
        }

        in.acc_ = acc_avr;
        in.gyro_ = angvel_avr;
        process_noise_covariance_.block<3, 3>(0, 0).diagonal() = gyro_covariance_;
        process_noise_covariance_.block<3, 3>(3, 3).diagonal() = accel_covariance_;
        process_noise_covariance_.block<3, 3>(6, 6).diagonal() = gyro_bias_covariance_;
        process_noise_covariance_.block<3, 3>(9, 9).diagonal() = accel_bias_covariance_;
        _esikf.predict(dt, process_noise_covariance_, in);

        /* save the poses at each IMU measurements */
        imu_state = _esikf.getState();
        last_gyro_ = angvel_avr - imu_state.gyro_bias_;
        last_world_acc_ = imu_state.rotation_ * (acc_avr - imu_state.acc_bias_);
        for (int i = 0; i < 3; i++)
        {
            last_world_acc_[i] += imu_state.gravity_direction_[i];
        }
        double &&offs_t = tail.timestamp_ - pcl_beg_time;
        imu_poses_.push_back(setState15D(offs_t, last_world_acc_, last_gyro_, imu_state.velocity_, imu_state.position_, imu_state.rotation_.toRotationMatrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    _esikf.predict(dt, process_noise_covariance_, in);

    imu_state = _esikf.getState();
    last_imu_ = _meas.imu_measured_.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** undistort each lidar point (backward propagation) ***/
    if (_undistorted_points.points.begin() == _undistorted_points.points.end())
        return;

    if (lidar_type_ != MARSIM)
    {
        auto it_pcl = _undistorted_points.points.end() - 1;
        for (auto it_kp = imu_poses_.end() - 1; it_kp != imu_poses_.begin(); it_kp--)
        {
            auto head = it_kp - 1;
            auto tail = it_kp;
            r_imu << MAT_FROM_ARRAY(head->rot_);
            // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
            vel_imu << VEC_FROM_ARRAY(head->vel_);
            pos_imu << VEC_FROM_ARRAY(head->pos_);
            acc_imu << VEC_FROM_ARRAY(tail->acc_);
            angvel_avr << VEC_FROM_ARRAY(tail->gyro_);

            for (; it_pcl->curvature / static_cast<double>(1000) > head->offset_time_; it_pcl--)
            {
                dt = it_pcl->curvature / static_cast<double>(1000) - head->offset_time_;

                /* Transform to the 'end' frame, using only the rotation
                 * Note: Compensation direction is INVERSE of Frame's moving direction
                 * So if we want to compensate a point at timestamp-i to the frame-e
                 * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
                const Eigen::Matrix3d r_i = r_imu * lie::exp(angvel_avr * dt).toRotationMatrix();

                Eigen::Vector3d p_i(it_pcl->x, it_pcl->y, it_pcl->z);
                Eigen::Vector3d t_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.position_);
                Eigen::Vector3d p_compensate = lidar_to_imu_rotation_.transpose() * (imu_state.rotation_.conjugate() * (r_i * (lidar_to_imu_rotation_ * p_i + lidar_to_imu_translation_) + t_ei) - lidar_to_imu_translation_); // not accurate!

                // save Undistorted points and their rotation
                it_pcl->x = p_compensate(0);
                it_pcl->y = p_compensate(1);
                it_pcl->z = p_compensate(2);

                if (it_pcl == _undistorted_points.points.begin())
                    break;
            }
        }
    }
}

inline void ImuProcess::forwardBackwardPropagation(const MeasureGroup &_meas,
                                                   ErrorStateIterativeKalmanFilter &_esikf,
                                                   LidarPointCloud::Ptr _undistorted_points)
{
    if (_meas.imu_measured_.empty())
    {
        return;
    };
    assert(_meas.lidar_measured_ != nullptr);

    if (if_imu_need_init_)
    {
        /// The very first lidar frame
        initializeImu(_meas, _esikf, init_sample_count_);

        if_imu_need_init_ = true;

        last_imu_ = _meas.imu_measured_.back();

        if (init_sample_count_ > MAX_INI_COUNT)
        {
            accel_covariance_ *= std::pow(kGravityNorm / mean_acc_.norm(), 2);
            if_imu_need_init_ = false;

            accel_covariance_ = accel_covariance_scale_;
            gyro_covariance_ = gyro_covariance_scale_;
        }

        return;
    }

    undistortPoints(_meas, _esikf, *_undistorted_points);
}
