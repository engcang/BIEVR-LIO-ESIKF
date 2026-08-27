#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lie
{

    using Matrix3d = Eigen::Matrix3d;
    using Quaterniond = Eigen::Quaterniond;
    using Vector3d = Eigen::Vector3d;

    inline Matrix3d hat(const Vector3d &_vector)
    {
        Matrix3d result;
        result << 0.0, -_vector.z(), _vector.y(),
            _vector.z(), 0.0, -_vector.x(),
            -_vector.y(), _vector.x(), 0.0;
        return result;
    }

    inline Quaterniond exp(const Vector3d &_rotation_vector)
    {
        const double scale = 0.5;
        const double scaled_squared_angle = scale * scale * _rotation_vector.squaredNorm();
        const double taylor_bound = std::sqrt(std::sqrt(std::numeric_limits<double>::epsilon()));
        double real;
        double sinc;
        if (scaled_squared_angle >= taylor_bound)
        {
            const double scaled_angle = std::sqrt(scaled_squared_angle);
            real = std::cos(scaled_angle);
            sinc = std::sin(scaled_angle) / scaled_angle;
        }
        else
        {
            static constexpr double inverse[] = {1.0 / 3.0, 1.0 / 4.0, 1.0 / 5.0, 1.0 / 6.0, 1.0 / 7.0, 1.0 / 8.0, 1.0 / 9.0};
            real = 1.0;
            sinc = 1.0;
            double term = -0.5 * scaled_squared_angle;
            for (int index = 0; index < 3; ++index)
            {
                real += term;
                term *= inverse[2 * index];
                sinc += term;
                term *= -inverse[2 * index + 1] * scaled_squared_angle;
            }
        }

        const double imaginary_scale = sinc * scale;

        return Quaterniond(real,
                           imaginary_scale * _rotation_vector.x(),
                           imaginary_scale * _rotation_vector.y(),
                           imaginary_scale * _rotation_vector.z());
    }

    inline Vector3d log(const Quaterniond &_quaternion)
    {
        Quaterniond quaternion = _quaternion.normalized();
        if (quaternion.w() < 0.0)
        {
            quaternion.coeffs() = -quaternion.coeffs();
        }

        const double imaginary_norm = quaternion.vec().norm();
        if (imaginary_norm < 1.0e-11)
        {
            return 2.0 * quaternion.vec();
        }
        const double scale = 2.0 * std::atan2(imaginary_norm, quaternion.w()) / imaginary_norm;
        return scale * quaternion.vec();
    }

    inline Matrix3d leftJacobian(const Vector3d &_rotation_vector)
    {
        const double squared_angle = _rotation_vector[0] * _rotation_vector[0] +
                                     _rotation_vector[1] * _rotation_vector[1] +
                                     _rotation_vector[2] * _rotation_vector[2];
        const double angle = std::sqrt(squared_angle);
        if (angle < 1.0e-11)
        {
            return Matrix3d::Identity();
        }

        const Matrix3d rotation_hat = hat(_rotation_vector);
        return Matrix3d::Identity() + ((1.0 - std::cos(angle)) / squared_angle) * rotation_hat + ((1.0 - std::sin(angle) / angle) / squared_angle) * rotation_hat * rotation_hat;
    }

} // namespace lie
