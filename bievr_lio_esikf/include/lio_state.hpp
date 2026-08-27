#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "eigen_lie.hpp"

constexpr double kGravityNorm = 9.81;
constexpr int kErrorStateDim = 17;
constexpr int kProcessNoiseDim = 12;
constexpr int kMeasurementStateDim = 6;

enum ErrorStateIndex
{
    K_POSITION = 0,
    K_ROTATION = 3,
    K_VELOCITY = 6,
    K_GYRO_BIAS = 9,
    K_ACC_BIAS = 12,
    K_GRAVITY = 15
};

using ErrorStateVector = Eigen::Matrix<double, kErrorStateDim, 1>;
using StateCovariance = Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>;
using ProcessNoiseCovariance = Eigen::Matrix<double, kProcessNoiseDim, kProcessNoiseDim>;

class GravityDirection
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GravityDirection():
        direction_vector_(kGravityNorm, 0.0, 0.0)
    {
    }

    explicit GravityDirection(const Eigen::Vector3d &_vector)
    {
        setVector(_vector);
    }

    GravityDirection &operator=(const Eigen::Vector3d &_vector)
    {
        setVector(_vector);
        return *this;
    }

    double operator[](const int _index) const
    {
        return direction_vector_[_index];
    }

    const Eigen::Vector3d &vector() const
    {
        return direction_vector_;
    }

    Eigen::Matrix<double, 3, 2> basis() const
    {
        Eigen::Matrix<double, 3, 2> result;
        const double denominator = kGravityNorm + direction_vector_.x();
        if (denominator > 1.0e-11)
        {
            result << -direction_vector_.y(), -direction_vector_.z(),
                kGravityNorm - direction_vector_.y() * direction_vector_.y() / denominator, -direction_vector_.z() * direction_vector_.y() / denominator,
                -direction_vector_.z() * direction_vector_.y() / denominator, kGravityNorm - direction_vector_.z() * direction_vector_.z() / denominator;
            result /= kGravityNorm;
        }
        else
        {
            result.setZero();
            result(1, 1) = -1.0;
            result(2, 0) = 1.0;
        }
        return result;
    }

    Eigen::Matrix<double, 2, 3> nx() const
    {
        return (1.0 / kGravityNorm / kGravityNorm) * basis().transpose() * lie::hat(direction_vector_);
    }

    Eigen::Matrix<double, 3, 2> mx(const Eigen::Vector2d &_delta) const
    {
        const Eigen::Matrix<double, 3, 2> gravity_basis = basis();
        if (_delta.norm() < 1.0e-11)
        {
            return -lie::hat(direction_vector_) * gravity_basis;
        }

        const Eigen::Vector3d tangent_rotation = gravity_basis * _delta;
        return -lie::exp(tangent_rotation).toRotationMatrix() * lie::hat(direction_vector_) *
               lie::leftJacobian(tangent_rotation).transpose() * gravity_basis;
    }

    void boxPlus(const Eigen::Vector2d &_delta)
    {
        const Eigen::Vector3d tangent_rotation = basis() * _delta;
        direction_vector_ = lie::exp(tangent_rotation).toRotationMatrix() * direction_vector_;
    }

    Eigen::Vector2d boxMinus(const GravityDirection &_other) const
    {
        const Eigen::Vector3d cross = lie::hat(direction_vector_) * _other.direction_vector_;
        const double sine = cross.norm();
        const double cosine = direction_vector_.dot(_other.direction_vector_);
        const double angle = std::atan2(sine, cosine);

        if (sine < 1.0e-11)
        {
            return std::abs(angle) > 1.0e-11 ? Eigen::Vector2d(M_PI, 0.0) : Eigen::Vector2d::Zero();
        }

        return (angle / sine) * _other.basis().transpose() * lie::hat(_other.direction_vector_) * direction_vector_;
    }

private:
    void setVector(const Eigen::Vector3d &_vector)
    {
        direction_vector_ = _vector;
        if (direction_vector_.norm() > 1.0e-12)
        {
            direction_vector_.normalize();
            direction_vector_ = direction_vector_ * kGravityNorm;
        }
        else
        {
            direction_vector_ = Eigen::Vector3d(kGravityNorm, 0.0, 0.0);
        }
    }

    Eigen::Vector3d direction_vector_;
};

struct LioState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero();
    GravityDirection gravity_direction_;

    void boxPlus(const ErrorStateVector &_delta)
    {
        position_ += _delta.segment<3>(K_POSITION);
        rotation_ = rotation_ * lie::exp(_delta.segment<3>(K_ROTATION));
        velocity_ += _delta.segment<3>(K_VELOCITY);
        gyro_bias_ += _delta.segment<3>(K_GYRO_BIAS);
        acc_bias_ += _delta.segment<3>(K_ACC_BIAS);
        gravity_direction_.boxPlus(_delta.segment<2>(K_GRAVITY));
    }

    ErrorStateVector boxMinus(const LioState &_other) const
    {
        ErrorStateVector delta;
        delta.segment<3>(K_POSITION) = position_ - _other.position_;
        delta.segment<3>(K_ROTATION) = lie::log(_other.rotation_.conjugate() * rotation_);
        delta.segment<3>(K_VELOCITY) = velocity_ - _other.velocity_;
        delta.segment<3>(K_GYRO_BIAS) = gyro_bias_ - _other.gyro_bias_;
        delta.segment<3>(K_ACC_BIAS) = acc_bias_ - _other.acc_bias_;
        delta.segment<2>(K_GRAVITY) = gravity_direction_.boxMinus(_other.gravity_direction_);
        return delta;
    }
};

struct ImuInput
{
    Eigen::Vector3d acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
};

inline ProcessNoiseCovariance setProcessNoiseCov()
{
    ProcessNoiseCovariance covariance = ProcessNoiseCovariance::Zero();
    covariance.block<3, 3>(0, 0).diagonal().setConstant(0.0001);
    covariance.block<3, 3>(3, 3).diagonal().setConstant(0.0001);
    covariance.block<3, 3>(6, 6).diagonal().setConstant(0.00001);
    covariance.block<3, 3>(9, 9).diagonal().setConstant(0.00001);
    return covariance;
}
