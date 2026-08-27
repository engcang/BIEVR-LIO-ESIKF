#ifndef ROS_CONVERTER_HPP
#define ROS_CONVERTER_HPP

#include <cmath>
#include <cstdio>
#include <vector>

#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "preprocess.hpp"

class RosConverter: public Preprocess
{
public:
    void preProcessPoints(const livox_ros_driver::CustomMsg::ConstPtr &_msg,
                          LidarPointCloud::Ptr &_pcl_out);
    void preProcessPoints(const sensor_msgs::PointCloud2::ConstPtr &_msg,
                          LidarPointCloud::Ptr &_pcl_out);

private:
    void livoxHandler(const livox_ros_driver::CustomMsg::ConstPtr &_msg);
    void ousterHandler(const sensor_msgs::PointCloud2::ConstPtr &_msg);
    void velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &_msg);
    void simulationHandler(const sensor_msgs::PointCloud2::ConstPtr &_msg);
};

inline void RosConverter::preProcessPoints(const livox_ros_driver::CustomMsg::ConstPtr &_msg,
                                           LidarPointCloud::Ptr &_pcl_out)
{
    livoxHandler(_msg);
    *_pcl_out = preprocessed_cloud_;
}

inline void RosConverter::preProcessPoints(const sensor_msgs::PointCloud2::ConstPtr &_msg,
                                           LidarPointCloud::Ptr &_pcl_out)
{
    switch (point_timestamp_unit_)
    {
        case SEC:
            point_timestamp_unit_scale_ = 1.e3f;
            break;
        case MS:
            point_timestamp_unit_scale_ = 1.f;
            break;
        case US:
            point_timestamp_unit_scale_ = 1.e-3f;
            break;
        case NS:
            point_timestamp_unit_scale_ = 1.e-6f;
            break;
        default:
            point_timestamp_unit_scale_ = 1.f;
            break;
    }

    switch (lidar_type_)
    {
        case OUSTER:
            ousterHandler(_msg);
            break;
        case VELODYNE:
            velodyneHandler(_msg);
            break;
        case MARSIM:
            simulationHandler(_msg);
            break;
        default:
            std::printf("Error LiDAR Type");
            break;
    }
    *_pcl_out = preprocessed_cloud_;
}

inline void RosConverter::livoxHandler(const livox_ros_driver::CustomMsg::ConstPtr &_msg)
{
    preprocessed_cloud_.clear();
    const std::uint32_t point_count = _msg->point_num;
    preprocessed_cloud_.reserve(point_count);

    for (std::uint32_t i = 1; i < point_count; ++i)
    {
        if ((_msg->points[i].line >= scan_channels_) ||
            (((_msg->points[i].tag & 0x30) != 0x10) && ((_msg->points[i].tag & 0x30) != 0x00)))
        {
            continue;
        }

        const auto &input_point = _msg->points[i];
        const auto &previous_point = _msg->points[i - 1];
        LidarPoint output_point;
        output_point.x = input_point.x;
        output_point.y = input_point.y;
        output_point.z = input_point.z;
        output_point.intensity = input_point.reflectivity;
        output_point.curvature = input_point.offset_time / static_cast<float>(1000000); // use curvature as time of each laser points, curvature unit: ms

        const bool differs_from_previous = (std::abs(input_point.x - previous_point.x) > 1e-7) ||
                                           (std::abs(input_point.y - previous_point.y) > 1e-7) ||
                                           (std::abs(input_point.z - previous_point.z) > 1e-7);
        const double squared_range = output_point.x * output_point.x +
                                     output_point.y * output_point.y +
                                     output_point.z * output_point.z;
        if (differs_from_previous &&
            std::isfinite(squared_range) &&
            squared_range > minimum_range_ * minimum_range_)
        {
            preprocessed_cloud_.push_back(output_point);
        }
    }
}

inline void RosConverter::ousterHandler(const sensor_msgs::PointCloud2::ConstPtr &_msg)
{
    preprocessed_cloud_.clear();
    pcl::PointCloud<ouster_ros::Point> original_cloud;
    pcl::fromROSMsg(*_msg, original_cloud);
    preprocessed_cloud_.reserve(original_cloud.size());

    for (std::size_t i = 0; i < original_cloud.size(); ++i)
    {
        const auto &input_point = original_cloud.points[i];
        const double squared_range = input_point.x * input_point.x +
                                     input_point.y * input_point.y +
                                     input_point.z * input_point.z;
        if (!std::isfinite(squared_range) ||
            squared_range < minimum_range_ * minimum_range_)
        {
            continue;
        }

        LidarPoint output_point;
        output_point.x = input_point.x;
        output_point.y = input_point.y;
        output_point.z = input_point.z;
        output_point.intensity = input_point.intensity;
        output_point.normal_x = 0;
        output_point.normal_y = 0;
        output_point.normal_z = 0;
        output_point.curvature = input_point.t * point_timestamp_unit_scale_; // curvature unit: ms
        preprocessed_cloud_.points.push_back(output_point);
    }
}

inline void RosConverter::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &_msg)
{
    preprocessed_cloud_.clear();
    pcl::PointCloud<velodyne_ros::Point> original_cloud;
    pcl::fromROSMsg(*_msg, original_cloud);
    const int point_count = static_cast<int>(original_cloud.size());
    if (point_count == 0)
    {
        return;
    }
    preprocessed_cloud_.reserve(point_count);

    /*** These variables only works when no point timestamps given ***/
    const double angular_velocity = 0.361 * scan_rate_; // scan angular velocity
    std::vector<bool> is_first(scan_channels_, true);
    std::vector<double> first_yaw(scan_channels_, 0.0); // yaw of first scan point
    std::vector<float> last_time(scan_channels_, 0.0);  // last offset time
    /*****************************************************************/

    if_given_offset_time_ = original_cloud.points[point_count - 1].time > 0;
    for (int i = 0; i < point_count; ++i)
    {
        const auto &input_point = original_cloud.points[i];
        LidarPoint output_point;
        output_point.normal_x = 0;
        output_point.normal_y = 0;
        output_point.normal_z = 0;
        output_point.x = input_point.x;
        output_point.y = input_point.y;
        output_point.z = input_point.z;
        output_point.intensity = input_point.intensity;
        output_point.curvature = input_point.time * point_timestamp_unit_scale_; // curvature unit: ms

        if (!if_given_offset_time_)
        {
            const int layer = input_point.ring;
            const double yaw_angle = std::atan2(output_point.y, output_point.x) * 57.2957;
            if (is_first[layer])
            {
                first_yaw[layer] = yaw_angle;
                is_first[layer] = false;
                output_point.curvature = 0.0;
                last_time[layer] = output_point.curvature;
                continue;
            }

            if (yaw_angle <= first_yaw[layer])
            {
                output_point.curvature = (first_yaw[layer] - yaw_angle) / angular_velocity;
            }
            else
            {
                output_point.curvature = (first_yaw[layer] - yaw_angle + 360.0) / angular_velocity;
            }

            if (output_point.curvature < last_time[layer])
            {
                output_point.curvature += 360.0 / angular_velocity;
            }
            last_time[layer] = output_point.curvature;
        }

        const double squared_range = output_point.x * output_point.x +
                                     output_point.y * output_point.y +
                                     output_point.z * output_point.z;
        if (std::isfinite(squared_range) &&
            squared_range > minimum_range_ * minimum_range_)
        {
            preprocessed_cloud_.points.push_back(output_point);
        }
    }
}

inline void RosConverter::simulationHandler(const sensor_msgs::PointCloud2::ConstPtr &_msg)
{
    preprocessed_cloud_.clear();
    pcl::PointCloud<pcl::PointXYZI> original_cloud;
    pcl::fromROSMsg(*_msg, original_cloud);
    preprocessed_cloud_.reserve(original_cloud.size());

    for (const auto &input_point : original_cloud.points)
    {
        const double squared_range = input_point.x * input_point.x +
                                     input_point.y * input_point.y +
                                     input_point.z * input_point.z;
        if (!std::isfinite(squared_range) ||
            squared_range < minimum_range_ * minimum_range_)
        {
            continue;
        }

        LidarPoint output_point;
        output_point.x = input_point.x;
        output_point.y = input_point.y;
        output_point.z = input_point.z;
        output_point.intensity = input_point.intensity;
        output_point.normal_x = 0;
        output_point.normal_y = 0;
        output_point.normal_z = 0;
        output_point.curvature = 0.0;
        preprocessed_cloud_.points.push_back(output_point);
    }
}

#endif
