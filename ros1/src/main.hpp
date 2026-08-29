// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <algorithm>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>

#include "bievr_voxel_map.hpp"
#include "eigen_lie.hpp"
#include "imu_processing.hpp"
#include "ros_converter.hpp"
#include "runtime_profiler.hpp"

#define LASER_POINT_COV (0.001)

class BievrLioApplication
{
public:
    BievrLioApplication()
    {
        active_application_ = this;
    }

    ~BievrLioApplication()
    {
        if (active_application_ == this)
        {
            active_application_ = nullptr;
        }
    }

private:
    inline static BievrLioApplication *active_application_ = nullptr;
    inline static volatile std::sig_atomic_t exit_requested_ = 0;

    static void measurementJacobianCallback(LioState &_state,
                                            DynamicSharedData &_measurement_data)
    {
        active_application_->buildMeasurementModelJacobianMatrix(_state, _measurement_data);
    }

    template<typename T>
    T parameter(ros::NodeHandle &_node,
                const std::string &_name,
                const T &_default_value)
    {
        std::string ros_name = _name;
        std::replace(ros_name.begin(), ros_name.end(), '.', '/');
        T value;
        _node.param<T>(ros_name, value, _default_value);
        return value;
    }


    bool path_enabled_ = true;

    std::string lidar_topic_, imu_topic_;
    std::string map_frame_ = "map";
    std::string odometry_frame_ = "odom";

    double last_timestamp_lidar_ = 0, last_timestamp_imu_ = -1.0;
    double gyroscope_covariance_ = 0.1, accelerometer_covariance_ = 0.1, gyroscope_bias_covariance_ = 0.0001, accelerometer_bias_covariance_ = 0.0001;
    double voxel_resolution_ = 0;
    double lidar_end_time_ = 0;
    double lidar_mean_scantime_ = 0.0;
    double huber_delta_ = 0.1;
    int count_lidar_scan_ = 0;
    int num_effective_points_ = 0;
    int num_measurement_points_ = 0, maximum_iterations_ = 0;
    bool lidar_pushed_ = false;
    bool scan_publish_enabled_ = false, dense_publish_enabled_ = false, body_scan_publish_enabled_ = false;
    int lidar_type_ = LIVOX;

    std::deque<double> time_buffer_;
    std::deque<LidarPointCloud::Ptr> lidar_buffer_;
    std::deque<ImuSample> imu_buffer_;

    LidarPointCloud::Ptr points_undistorted_lidar_ = LidarPointCloud::Ptr(new LidarPointCloud());
    LidarPointCloud::Ptr points_registration_lidar_ = LidarPointCloud::Ptr(new LidarPointCloud());
    LidarPointCloud::Ptr points_fine_lidar_ = LidarPointCloud::Ptr(new LidarPointCloud());
    LidarPointCloud::Ptr points_voxel_lidar_ = LidarPointCloud::Ptr(new LidarPointCloud());
    LidarPointCloud::Ptr points_world_ = LidarPointCloud::Ptr(new LidarPointCloud());
    LidarPointCloud::Ptr points_voxel_lidar_effective_ = LidarPointCloud::Ptr(new LidarPointCloud(100000, 1));
    std::vector<std::uint8_t> point_has_valid_measurement_;
    std::vector<BievrMeasurement> point_measurements_;
    std::vector<BievrMeasurement> effective_measurements_;
    std::vector<double> point_ranges_;

    BievrVoxelMap bievr_map_;
    BievrVoxelMap::SamplingParameters sampling_parameters_;

    /*** EKF inputs and output ***/
    MeasureGroup measurements_;
    RuntimeProfiler runtime_profiler_;
    ErrorStateIterativeKalmanFilter esikf_;
    LioState esikf_state_;

    nav_msgs::Path lio_path_;
    nav_msgs::Odometry mapped_odometry_;
    geometry_msgs::PoseStamped body_pose_message_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    std::shared_ptr<RosConverter> points_preprocessor_ = std::make_shared<RosConverter>();
    std::shared_ptr<ImuProcess> imu_processor_ = std::make_shared<ImuProcess>();

    inline ros::Time secondsToStamp(const double _seconds)
    {
        ros::Time stamp;
        stamp.fromSec(_seconds);
        return stamp;
    }

    inline double stampToSeconds(const ros::Time &_stamp)
    {
        return _stamp.toSec();
    }

    static double maximumPointTimeSeconds(const LidarPointCloud &_cloud)
    {
        double maximum_point_time_milliseconds = 0.0;
        for (const LidarPoint &point : _cloud.points)
        {
            const double point_time_milliseconds = static_cast<double>(point.curvature);
            if (std::isfinite(point_time_milliseconds) &&
                point_time_milliseconds >= 0.0)
            {
                maximum_point_time_milliseconds = std::max(maximum_point_time_milliseconds,
                                                           point_time_milliseconds);
            }
        }
        return maximum_point_time_milliseconds / 1.0e3;
    }

    void pointLidarToWorld(LidarPoint const *const _pi, LidarPoint *const _po)
    {
        const Eigen::Vector3d &lidar_to_imu_translation = imu_processor_->getLidarTranslationWrtImu();
        const Eigen::Matrix3d &lidar_to_imu_rotation = imu_processor_->getLidarRotationWrtImu();
        Eigen::Vector3d p_body(_pi->x, _pi->y, _pi->z);
        Eigen::Vector3d p_global(esikf_state_.rotation_ * (lidar_to_imu_rotation * p_body + lidar_to_imu_translation) + esikf_state_.position_);

        _po->x = p_global(0);
        _po->y = p_global(1);
        _po->z = p_global(2);
        _po->intensity = _pi->intensity;
    }

    void pointLidarToImu(LidarPoint const *const _pi, LidarPoint *const _po)
    {
        const Eigen::Vector3d &lidar_to_imu_translation = imu_processor_->getLidarTranslationWrtImu();
        const Eigen::Matrix3d &lidar_to_imu_rotation = imu_processor_->getLidarRotationWrtImu();
        Eigen::Vector3d p_body_lidar(_pi->x, _pi->y, _pi->z);
        Eigen::Vector3d p_body_imu(lidar_to_imu_rotation * p_body_lidar + lidar_to_imu_translation);

        _po->x = p_body_imu(0);
        _po->y = p_body_imu(1);
        _po->z = p_body_imu(2);
        _po->intensity = _pi->intensity;
    }

    void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr &_msg)
    {
        const double lidar_timestamp = stampToSeconds(_msg->header.stamp);
        if (lidar_timestamp < last_timestamp_lidar_)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer_.clear();
        }
        last_timestamp_lidar_ = lidar_timestamp;

        if (!imu_buffer_.empty() && std::abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0)
        {
            std::printf("IMU and LiDAR not synced, IMU time: %lf, LiDAR time: %lf\n", last_timestamp_imu_, last_timestamp_lidar_);
        }

        LidarPointCloud::Ptr ptr(new LidarPointCloud());
        points_preprocessor_->preProcessPoints(_msg, ptr);
        lidar_buffer_.push_back(ptr);
        time_buffer_.push_back(last_timestamp_lidar_);
    }

    void pcdLivoxCallback(const livox_ros_driver::CustomMsg::ConstPtr &_msg)
    {
        const double lidar_timestamp = stampToSeconds(_msg->header.stamp);
        if (lidar_timestamp < last_timestamp_lidar_)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer_.clear();
        }
        last_timestamp_lidar_ = lidar_timestamp;

        if (!imu_buffer_.empty() && std::abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0)
        {
            std::printf("IMU and LiDAR not synced, IMU time: %lf, LiDAR time: %lf\n", last_timestamp_imu_, last_timestamp_lidar_);
        }

        LidarPointCloud::Ptr ptr(new LidarPointCloud());
        points_preprocessor_->preProcessPoints(_msg, ptr);
        lidar_buffer_.push_back(ptr);
        time_buffer_.push_back(last_timestamp_lidar_);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &_msg_in)
    {
        const double timestamp = stampToSeconds(_msg_in->header.stamp);
        ImuSample sample;
        sample.timestamp_ = timestamp;
        sample.linear_acceleration_ = {_msg_in->linear_acceleration.x,
                                       _msg_in->linear_acceleration.y,
                                       _msg_in->linear_acceleration.z};
        sample.angular_velocity_ = {_msg_in->angular_velocity.x,
                                    _msg_in->angular_velocity.y,
                                    _msg_in->angular_velocity.z};

        if (timestamp < last_timestamp_imu_)
        {
            ROS_WARN("imu loop back, clear buffer");
            imu_buffer_.clear();
        }

        last_timestamp_imu_ = timestamp;

        imu_buffer_.push_back(sample);
    }

    bool synchronizeMeasurements(MeasureGroup &_meas)
    {
        if (lidar_buffer_.empty() || imu_buffer_.empty())
        {
            return false;
        }

        /*** push a lidar scan ***/
        if (!lidar_pushed_)
        {
            _meas.lidar_measured_ = lidar_buffer_.front();
            _meas.lidar_beg_time_ = time_buffer_.front();

            const double scan_duration_seconds = maximumPointTimeSeconds(*_meas.lidar_measured_);

            if (_meas.lidar_measured_->points.size() <= 1) // time too little
            {
                lidar_end_time_ = _meas.lidar_beg_time_ + lidar_mean_scantime_;
                ROS_WARN("Too few input point cloud!");
            }
            else if (scan_duration_seconds < 0.5 * lidar_mean_scantime_)
            {
                lidar_end_time_ = _meas.lidar_beg_time_ + lidar_mean_scantime_;
            }
            else
            {
                count_lidar_scan_++;
                lidar_end_time_ = _meas.lidar_beg_time_ + scan_duration_seconds;
                lidar_mean_scantime_ += (scan_duration_seconds - lidar_mean_scantime_) / count_lidar_scan_;
            }
            if (lidar_type_ == MARSIM)
                lidar_end_time_ = _meas.lidar_beg_time_;

            _meas.lidar_end_time_ = lidar_end_time_;

            lidar_pushed_ = true;
        }

        if (last_timestamp_imu_ < lidar_end_time_)
        {
            return false;
        }

        /*** push imu data, and pop from imu buffer ***/
        double imu_time = imu_buffer_.front().timestamp_;
        _meas.imu_measured_.clear();
        while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_))
        {
            imu_time = imu_buffer_.front().timestamp_;
            if (imu_time > lidar_end_time_)
                break;
            _meas.imu_measured_.push_back(imu_buffer_.front());
            imu_buffer_.pop_front();
        }

        lidar_buffer_.pop_front();
        time_buffer_.pop_front();
        lidar_pushed_ = false;
        return true;
    }

    void updateMap()
    {
        const int point_count = static_cast<int>(points_undistorted_lidar_->size());
        points_world_->resize(point_count);
        point_ranges_.resize(point_count);
        //clang-format off
        tbb::parallel_for(tbb::blocked_range<int>(0, point_count),
                          [this](const tbb::blocked_range<int> &_range)
                          {
                              for (int index = _range.begin(); index != _range.end(); ++index)
                              {
                                  const LidarPoint &point_lidar = points_undistorted_lidar_->points[index];
                                  pointLidarToWorld(&point_lidar, &points_world_->points[index]);
                                  point_ranges_[index] = std::sqrt(static_cast<double>(point_lidar.x) * point_lidar.x +
                                                                   static_cast<double>(point_lidar.y) * point_lidar.y +
                                                                   static_cast<double>(point_lidar.z) * point_lidar.z);
                              }
                          });
        //clang-format on

        bievr_map_.update(*points_world_, &point_ranges_);
    }

    void publishPointCloudWorld(const ros::Publisher &_pub_laser_cloud_full)
    {
        if (scan_publish_enabled_)
        {
            LidarPointCloud::Ptr laser_cloud_full_res(dense_publish_enabled_ ? points_undistorted_lidar_ : points_voxel_lidar_);
            int size = laser_cloud_full_res->points.size();
            LidarPointCloud::Ptr laser_cloud_world(new LidarPointCloud(size, 1));

            for (int i = 0; i < size; i++)
            {
                pointLidarToWorld(&laser_cloud_full_res->points[i],
                                  &laser_cloud_world->points[i]);
            }

            sensor_msgs::PointCloud2 laser_cloudmsg;
            pcl::toROSMsg(*laser_cloud_world, laser_cloudmsg);
            laser_cloudmsg.header.stamp = secondsToStamp(lidar_end_time_);
            laser_cloudmsg.header.frame_id = odometry_frame_;
            _pub_laser_cloud_full.publish(laser_cloudmsg);
        }
    }

    void publishPointCloudBody(const ros::Publisher &_pub_laser_cloud_full_body)
    {
        int size = points_undistorted_lidar_->points.size();
        LidarPointCloud::Ptr laser_cloud_imu_body(new LidarPointCloud(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointLidarToImu(&points_undistorted_lidar_->points[i],
                            &laser_cloud_imu_body->points[i]);
        }

        sensor_msgs::PointCloud2 laser_cloudmsg;
        pcl::toROSMsg(*laser_cloud_imu_body, laser_cloudmsg);
        laser_cloudmsg.header.stamp = secondsToStamp(lidar_end_time_);
        laser_cloudmsg.header.frame_id = "body";
        _pub_laser_cloud_full_body.publish(laser_cloudmsg);
    }

    template<typename T>
    void setPoseStamp(T &_out)
    {
        _out.pose.position.x = esikf_state_.position_(0);
        _out.pose.position.y = esikf_state_.position_(1);
        _out.pose.position.z = esikf_state_.position_(2);
        _out.pose.orientation.x = esikf_state_.rotation_.x();
        _out.pose.orientation.y = esikf_state_.rotation_.y();
        _out.pose.orientation.z = esikf_state_.rotation_.z();
        _out.pose.orientation.w = esikf_state_.rotation_.w();
    }

    void publishOdometry(const ros::Publisher &_pub_odom_aft_mapped)
    {
        mapped_odometry_.header.frame_id = odometry_frame_;
        mapped_odometry_.child_frame_id = "body";
        mapped_odometry_.header.stamp = secondsToStamp(lidar_end_time_);
        setPoseStamp(mapped_odometry_.pose);
        const auto &p = esikf_.getCovariance();
        for (int row = 0; row < 6; ++row)
        {
            for (int column = 0; column < 6; ++column)
            {
                mapped_odometry_.pose.covariance[row * 6 + column] = p(row, column);
            }
        }
        _pub_odom_aft_mapped.publish(mapped_odometry_);

        geometry_msgs::TransformStamped transform;
        transform.header = mapped_odometry_.header;
        transform.child_frame_id = "body";
        transform.transform.translation.x = mapped_odometry_.pose.pose.position.x;
        transform.transform.translation.y = mapped_odometry_.pose.pose.position.y;
        transform.transform.translation.z = mapped_odometry_.pose.pose.position.z;
        transform.transform.rotation = mapped_odometry_.pose.pose.orientation;
        transform_broadcaster_->sendTransform(transform);

        if (map_frame_ != odometry_frame_)
        {
            const Eigen::Quaterniond &gravity_alignment = imu_processor_->getGravityAlignmentRotation();
            geometry_msgs::TransformStamped gravity_transform;
            gravity_transform.header.stamp = mapped_odometry_.header.stamp;
            gravity_transform.header.frame_id = map_frame_;
            gravity_transform.child_frame_id = odometry_frame_;
            gravity_transform.transform.rotation.x = gravity_alignment.x();
            gravity_transform.transform.rotation.y = gravity_alignment.y();
            gravity_transform.transform.rotation.z = gravity_alignment.z();
            gravity_transform.transform.rotation.w = gravity_alignment.w();
            transform_broadcaster_->sendTransform(gravity_transform);
        }
    }

    void publishPath(const ros::Publisher &_pub_path)
    {
        setPoseStamp(body_pose_message_);
        body_pose_message_.header.stamp = secondsToStamp(lidar_end_time_);
        body_pose_message_.header.frame_id = odometry_frame_;

        /*** if path is too large, the rvis will crash ***/
        static int jjj = 0;
        jjj++;
        if (jjj % 10 == 0)
        {
            lio_path_.poses.push_back(body_pose_message_);
            _pub_path.publish(lio_path_);
        }
    }

    void buildMeasurementModelJacobianMatrix(LioState &_s, DynamicSharedData &_measurement_data)
    {
        const Eigen::Vector3d &lidar_to_imu_translation = imu_processor_->getLidarTranslationWrtImu();
        const Eigen::Matrix3d &lidar_to_imu_rotation = imu_processor_->getLidarRotationWrtImu();
        points_voxel_lidar_effective_->clear();
        point_has_valid_measurement_.assign(num_measurement_points_, 0U);
        point_measurements_.resize(num_measurement_points_);
        points_voxel_lidar_effective_->resize(num_measurement_points_);

        //clang-format off
        tbb::parallel_for(tbb::blocked_range<int>(0, num_measurement_points_),
                          [this, &_s, &lidar_to_imu_translation, &lidar_to_imu_rotation](const tbb::blocked_range<int> &_range)
                          {
                              for (int index = _range.begin(); index != _range.end(); ++index)
                              {
                                  const LidarPoint &point_body = points_voxel_lidar_->points[index];
                                  const Eigen::Vector3d point_lidar(point_body.x, point_body.y, point_body.z);
                                  const Eigen::Vector3d point_global = _s.rotation_ *
                                                                           (lidar_to_imu_rotation * point_lidar + lidar_to_imu_translation) +
                                                                       _s.position_;
                                  BievrMeasurement measurement;
                                  if (!bievr_map_.sample(point_global, measurement))
                                  {
                                      continue;
                                  }

                                  const double absolute_residual = std::abs(measurement.residual_);
                                  const double robust_weight = absolute_residual <= huber_delta_ ? 1.0 : huber_delta_ / absolute_residual;
                                  const double square_root_weight = std::sqrt(robust_weight);
                                  measurement.position_jacobian_ *= square_root_weight;
                                  measurement.residual_ *= square_root_weight;
                                  point_measurements_[index] = measurement;
                                  point_has_valid_measurement_[index] = 1U;
                              }
                          });
        //clang-format on

        num_effective_points_ = 0;
        effective_measurements_.clear();
        effective_measurements_.reserve(num_measurement_points_);
        for (int i = 0; i < num_measurement_points_; ++i)
        {
            if (point_has_valid_measurement_[i] != 0U)
            {
                points_voxel_lidar_effective_->points[num_effective_points_] = points_voxel_lidar_->points[i];
                effective_measurements_.push_back(point_measurements_[i]);
                ++num_effective_points_;
            }
        }

        if (num_effective_points_ < 1)
        {
            _measurement_data.valid_ = false;
            ROS_WARN("No Effective Points!");
            return;
        }

        /*** Computation of measurement Jacobian matrix H and measurement vector ***/
        _measurement_data.jacobian_.setZero(num_effective_points_, kMeasurementStateDim);
        _measurement_data.residual_.resize(num_effective_points_);

        //clang-format off
        tbb::parallel_for(tbb::blocked_range<int>(0, num_effective_points_),
                          [this, &_s, &_measurement_data, &lidar_to_imu_translation, &lidar_to_imu_rotation](const tbb::blocked_range<int> &_range)
                          {
                              for (int index = _range.begin(); index != _range.end(); ++index)
                              {
                                  const LidarPoint &laser_point = points_voxel_lidar_effective_->points[index];
                                  const Eigen::Vector3d point_lidar(laser_point.x, laser_point.y, laser_point.z);
                                  const Eigen::Vector3d point_imu = lidar_to_imu_rotation * point_lidar + lidar_to_imu_translation;
                                  const Eigen::Matrix3d point_imu_cross = lie::hat(point_imu);

                                  const BievrMeasurement &measurement = effective_measurements_[index];
                                  const Eigen::RowVector3d rotation_jacobian = measurement.position_jacobian_ * (-_s.rotation_.toRotationMatrix() * point_imu_cross);
                                  _measurement_data.jacobian_.block<1, kMeasurementStateDim>(index, 0) << measurement.position_jacobian_, rotation_jacobian;
                                  _measurement_data.residual_(index) = -measurement.residual_;
                              }
                          });
        //clang-format on
    }

public:
    static void requestExit()
    {
        exit_requested_ = 1;
    }

    int mainFunction()
    {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");
        std::vector<double> lidar_to_imu_translation_values(3, 0.0);
        std::vector<double> lidar_to_imu_rotation_values(9, 0.0);

        path_enabled_ = parameter<bool>(private_node, "publish.path_en", true);
        scan_publish_enabled_ = parameter<bool>(private_node, "publish.scan_publish_en", true);
        dense_publish_enabled_ = parameter<bool>(private_node, "publish.dense_publish_en", true);
        body_scan_publish_enabled_ = parameter<bool>(private_node, "publish.scan_bodyframe_pub_en", true);
        const bool runtime_enabled = parameter<bool>(private_node, "runtime.enabled", false);
        const std::string runtime_output_path = parameter<std::string>(private_node, "runtime.output_path", "runtime.csv");
        runtime_profiler_.configure(runtime_enabled, runtime_output_path);
        maximum_iterations_ = parameter<int>(private_node, "filter.maximum_iterations", 4);
        lidar_topic_ = parameter<std::string>(private_node, "common.lidar_topic", "/livox/lidar");
        imu_topic_ = parameter<std::string>(private_node, "common.imu_topic", "/livox/imu");
        map_frame_ = parameter<std::string>(private_node, "common.map_frame", "map");
        odometry_frame_ = parameter<std::string>(private_node, "common.odometry_frame", "odom");
        voxel_resolution_ = parameter<double>(private_node, "preprocess.voxel_resolution", 0.1);
        gyroscope_covariance_ = parameter<double>(private_node, "mapping.gyroscope_covariance", 0.1);
        accelerometer_covariance_ = parameter<double>(private_node, "mapping.accelerometer_covariance", 0.1);
        gyroscope_bias_covariance_ = parameter<double>(private_node, "mapping.gyroscope_bias_covariance", 0.0001);
        accelerometer_bias_covariance_ = parameter<double>(private_node, "mapping.accelerometer_bias_covariance", 0.0001);
        points_preprocessor_->minimum_range_ = parameter<double>(private_node, "preprocess.min_range", 0.01);
        lidar_type_ = parameter<int>(private_node, "preprocess.lidar_type", LIVOX);
        points_preprocessor_->scan_channels_ = parameter<int>(private_node, "preprocess.scan_line_count", 16);
        points_preprocessor_->point_timestamp_unit_ = parameter<int>(private_node, "preprocess.point_timestamp_unit", US);
        points_preprocessor_->scan_rate_ = parameter<int>(private_node, "preprocess.scan_rate_hz", 10);
        points_preprocessor_->point_stride_ = parameter<int>(private_node, "preprocess.point_stride", 2);
        lidar_to_imu_translation_values = parameter<std::vector<double>>(private_node, "mapping.lidar_to_imu_translation", lidar_to_imu_translation_values);
        lidar_to_imu_rotation_values = parameter<std::vector<double>>(private_node, "mapping.lidar_to_imu_rotation", lidar_to_imu_rotation_values);
        if (lidar_to_imu_translation_values.size() != 3U || lidar_to_imu_rotation_values.size() != 9U)
        {
            ROS_FATAL("Invalid LiDAR-IMU extrinsic dimensions: lidar_to_imu_translation requires 3 values and lidar_to_imu_rotation requires 9 values (received %zu and %zu)",
                      lidar_to_imu_translation_values.size(),
                      lidar_to_imu_rotation_values.size());
            ros::shutdown();
            return 1;
        }
        BievrVoxelMap::Parameters bievr_parameters;
        bievr_parameters.voxel_size_ = parameter<double>(private_node, "bievr.voxel_size", 0.5);
        bievr_parameters.pixel_size_ = parameter<double>(private_node, "bievr.pixel_size", 0.05);
        bievr_parameters.normal_tolerance_degrees_ = parameter<double>(private_node, "bievr.normal_tolerance_degrees", 3.0);
        bievr_parameters.smooth_image_ = parameter<bool>(private_node, "bievr.smooth_image", true);
        bievr_parameters.use_range_weight_ = parameter<bool>(private_node, "bievr.use_range_weight", true);
        bievr_parameters.maximum_voxel_count_ = static_cast<std::size_t>(parameter<int>(private_node,
                                                                                        "bievr.maximum_voxel_count",
                                                                                        1500000));
        sampling_parameters_.fine_voxel_size_ = voxel_resolution_;
        sampling_parameters_.informed_sampling_enabled_ = parameter<bool>(private_node, "bievr.informed_sampling_enabled", true);
        sampling_parameters_.informed_voxel_count_ = static_cast<std::size_t>(parameter<int>(private_node,
                                                                                             "bievr.informed_voxel_count",
                                                                                             300));
        huber_delta_ = parameter<double>(private_node, "bievr.huber_delta", 0.1);
        bievr_map_.configure(bievr_parameters);
        ROS_INFO("BIEVR map: scan leaf %.3f m, voxel %.3f m, pixel %.3f m, informed voxels %zu",
                 voxel_resolution_,
                 bievr_parameters.voxel_size_,
                 bievr_parameters.pixel_size_,
                 sampling_parameters_.informed_voxel_count_);

        points_preprocessor_->lidar_type_ = lidar_type_;

        lio_path_.header.stamp = ros::Time::now();
        lio_path_.header.frame_id = odometry_frame_;


        Eigen::Vector3d lidar_to_imu_translation;
        Eigen::Matrix3d lidar_to_imu_rotation;
        lidar_to_imu_translation << VEC_FROM_ARRAY(lidar_to_imu_translation_values);
        lidar_to_imu_rotation << MAT_FROM_ARRAY(lidar_to_imu_rotation_values);
        imu_processor_->setExtrinsic(lidar_to_imu_translation, lidar_to_imu_rotation);
        imu_processor_->setGyroCov(Eigen::Vector3d(gyroscope_covariance_, gyroscope_covariance_, gyroscope_covariance_));
        imu_processor_->setAccelCov(Eigen::Vector3d(accelerometer_covariance_, accelerometer_covariance_, accelerometer_covariance_));
        imu_processor_->setGyroBiasCov(Eigen::Vector3d(gyroscope_bias_covariance_, gyroscope_bias_covariance_, gyroscope_bias_covariance_));
        imu_processor_->setAccelBiasCov(Eigen::Vector3d(accelerometer_bias_covariance_, accelerometer_bias_covariance_, accelerometer_bias_covariance_));
        imu_processor_->setLidarType(lidar_type_);
        const ErrorStateVector convergence_limits = ErrorStateVector::Constant(0.001);
        esikf_.initialize(measurementJacobianCallback,
                          maximum_iterations_,
                          convergence_limits);


        /*** ROS subscribe initialization ***/
        ros::Subscriber sub_livox;
        ros::Subscriber sub_pcl;
        if (points_preprocessor_->lidar_type_ == LIVOX)
        {
            sub_livox = node.subscribe<livox_ros_driver::CustomMsg>(lidar_topic_,
                                                                    200000,
                                                                    &BievrLioApplication::pcdLivoxCallback,
                                                                    this,
                                                                    ros::TransportHints().tcpNoDelay());
        }
        else
        {
            sub_pcl = node.subscribe<sensor_msgs::PointCloud2>(lidar_topic_,
                                                               200000,
                                                               &BievrLioApplication::pcdCallback,
                                                               this,
                                                               ros::TransportHints().tcpNoDelay());
        }
        auto sub_imu = node.subscribe<sensor_msgs::Imu>(imu_topic_,
                                                        200000,
                                                        &BievrLioApplication::imuCallback,
                                                        this,
                                                        ros::TransportHints().tcpNoDelay());
        auto pub_laser_cloud_full = node.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
        auto pub_laser_cloud_full_body = node.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
        auto pub_odom_aft_mapped = node.advertise<nav_msgs::Odometry>("/Odometry", 100000);
        auto pub_path = node.advertise<nav_msgs::Path>("/path", 100000);
        transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        //------------------------------------------------------------------------------------------------------
        ros::Rate rate(5000.0);
        bool status = ros::ok();
        bool first_synchronized_measurement = true;
        while (status)
        {
            if (exit_requested_ != 0)
            {
                break;
            }
            ros::spinOnce();
            if (synchronizeMeasurements(measurements_))
            {
                if (first_synchronized_measurement)
                {
                    first_synchronized_measurement = false;
                    continue;
                }

                RuntimeProfiler::Measurement runtime_measurement(runtime_profiler_);
                imu_processor_->forwardBackwardPropagation(measurements_, esikf_, points_undistorted_lidar_);
                esikf_state_ = esikf_.getState();

                if (points_undistorted_lidar_->empty() || (points_undistorted_lidar_ == NULL))
                {
                    ROS_WARN("No point, skip this scan!");
                    continue;
                }

                // Keep the dense undistorted scan for mapping and apply the legacy stride only to registration.
                const LidarPointCloud *registration_input = points_undistorted_lidar_.get();
                if (points_preprocessor_->point_stride_ > 1)
                {
                    points_preprocessor_->sampleRegistrationPoints(*points_undistorted_lidar_,
                                                                   *points_registration_lidar_);
                    registration_input = points_registration_lidar_.get();
                }

                /*** BIEVR fine downsampling and map-informed point selection ***/
                BievrVoxelMap::downsampleNearestToVoxelCenter(*registration_input,
                                                              sampling_parameters_.fine_voxel_size_,
                                                              *points_fine_lidar_);
                if (bievr_map_.empty())
                {
                    if (points_fine_lidar_->size() > 5U)
                    {
                        updateMap();
                        ROS_INFO("Initialized BIEVR map from %zu points: %zu voxels",
                                 points_undistorted_lidar_->size(),
                                 bievr_map_.voxelCount());
                    }
                    continue;
                }

                bievr_map_.selectMeasurementPoints(*points_fine_lidar_,
                                                   esikf_state_.rotation_,
                                                   esikf_state_.position_,
                                                   imu_processor_->getLidarRotationWrtImu(),
                                                   imu_processor_->getLidarTranslationWrtImu(),
                                                   sampling_parameters_,
                                                   *points_voxel_lidar_);
                num_measurement_points_ = static_cast<int>(points_voxel_lidar_->size());

                /*** ICP and iterated Kalman filter update ***/
                if (num_measurement_points_ < 5)
                {
                    ROS_WARN("No point, skip this scan!");
                    updateMap();
                    continue;
                }


                /*** iterated state estimation ***/
                esikf_.updateIterated(LASER_POINT_COV);
                esikf_state_ = esikf_.getState();


                /******* Publish odometry *******/
                publishOdometry(pub_odom_aft_mapped);
                runtime_measurement.finish();

                /*** add the registered full scan to the BIEVR map ***/
                updateMap();

                /******* Publish points *******/
                if (path_enabled_)
                    publishPath(pub_path);
                if (scan_publish_enabled_)
                    publishPointCloudWorld(pub_laser_cloud_full);
                if (scan_publish_enabled_ && body_scan_publish_enabled_)
                    publishPointCloudBody(pub_laser_cloud_full_body);
            }

            status = ros::ok();
            rate.sleep();
        }

        transform_broadcaster_.reset();
        if (runtime_profiler_.enabled())
        {
            ROS_INFO("Runtime: %s", runtime_profiler_.summary().c_str());
            if (!runtime_profiler_.writeCsv())
            {
                ROS_ERROR("Failed to write runtime CSV '%s'.",
                          runtime_profiler_.outputPath().c_str());
            }
        }
        ros::shutdown();
        return 0;
    }
};
