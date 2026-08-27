#ifndef PREPROCESS_HPP
#define PREPROCESS_HPP

#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "common.hpp"

enum TimeUnit
{
    SEC = 0,
    MS = 1,
    US = 2,
    NS = 3
};

namespace velodyne_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity; // NOLINT(readability-identifier-naming) PCL schema field.
        float time;      // NOLINT(readability-identifier-naming) PCL schema field.
        uint16_t ring;   // NOLINT(readability-identifier-naming) PCL schema field.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace velodyne_ros

//clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring))
//clang-format on

namespace ouster_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;       // NOLINT(readability-identifier-naming) PCL schema field.
        uint32_t t;            // NOLINT(readability-identifier-naming) PCL schema field.
        uint16_t reflectivity; // NOLINT(readability-identifier-naming) PCL schema field.
        uint8_t ring;          // NOLINT(readability-identifier-naming) PCL schema field.
        uint16_t ambient;      // NOLINT(readability-identifier-naming) PCL schema field.
        uint32_t range;        // NOLINT(readability-identifier-naming) PCL schema field.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace ouster_ros

//clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range))
//clang-format on

namespace hesai_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;    // NOLINT(readability-identifier-naming) PCL schema field.
        std::uint16_t ring; // NOLINT(readability-identifier-naming) PCL schema field.
        double timestamp;   // NOLINT(readability-identifier-naming) PCL schema field.
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace hesai_ros

//clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp))
//clang-format on

class Preprocess
{
public:
    void sampleRegistrationPoints(const LidarPointCloud &_dense_cloud,
                                  LidarPointCloud &_registration_cloud) const
    {
        if (point_stride_ <= 0)
        {
            throw std::invalid_argument("LiDAR point stride must be positive");
        }
        if (&_dense_cloud == &_registration_cloud)
        {
            throw std::invalid_argument("Dense and registration point clouds must be distinct");
        }

        _registration_cloud.clear();
        _registration_cloud.header = _dense_cloud.header;
        _registration_cloud.sensor_origin_ = _dense_cloud.sensor_origin_;
        _registration_cloud.sensor_orientation_ = _dense_cloud.sensor_orientation_;
        _registration_cloud.is_dense = _dense_cloud.is_dense;
        const std::size_t stride = static_cast<std::size_t>(point_stride_);
        _registration_cloud.reserve((_dense_cloud.size() + stride - 1U) / stride);
        for (std::size_t index = 0U; index < _dense_cloud.size(); index += stride)
        {
            _registration_cloud.push_back(_dense_cloud.points[index]);
        }
    }

    LidarPointCloud preprocessed_cloud_;
    float point_timestamp_unit_scale_ = 1.0f;
    int lidar_type_ = LIVOX;
    int point_stride_ = 1;
    int scan_channels_ = 6;
    int scan_rate_ = 10;
    int point_timestamp_unit_ = US;
    double minimum_range_ = 0.01;
    bool if_given_offset_time_ = false;
};

#endif
