#pragma once

#include <Eigen/Core>

#include "lio_state.hpp"


struct DynamicSharedData
{
    bool valid_ = true;
    bool converged_ = true;
    Eigen::VectorXd residual_;
    Eigen::Matrix<double, Eigen::Dynamic, kMeasurementStateDim> jacobian_;
};

class ErrorStateIterativeKalmanFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using MeasurementModel = void (*)(LioState &, DynamicSharedData &);

    ErrorStateIterativeKalmanFilter():
        covariance_(StateCovariance::Identity()), convergence_limits_(ErrorStateVector::Constant(0.001))
    {
    }

    void initialize(MeasurementModel _measurement_model,
                    const int _maximum_iterations,
                    const ErrorStateVector &_convergence_limits)
    {
        measurement_model_ = _measurement_model;
        maximum_iterations_ = _maximum_iterations;
        convergence_limits_ = _convergence_limits;
    }

    const LioState &getState() const
    {
        return state_;
    }

    const StateCovariance &getCovariance() const
    {
        return covariance_;
    }

    void setState(const LioState &_state)
    {
        state_ = _state;
    }

    void setCovariance(const StateCovariance &_covariance)
    {
        covariance_ = _covariance;
    }

    void predict(const double _dt,
                 const ProcessNoiseCovariance &_process_noise,
                 const ImuInput &_input)
    {
        const Eigen::Vector3d angular_velocity = _input.gyro_ - state_.gyro_bias_;
        const Eigen::Vector3d acceleration = _input.acc_ - state_.acc_bias_;
        const Eigen::Matrix3d rotation = state_.rotation_.toRotationMatrix();
        const Eigen::Vector3d world_acceleration = state_.rotation_ * acceleration + state_.gravity_direction_.vector();

        StateCovariance derivative_jacobian = StateCovariance::Zero();
        Eigen::Matrix<double, kErrorStateDim, kProcessNoiseDim> noise_jacobian = Eigen::Matrix<double, kErrorStateDim, kProcessNoiseDim>::Zero();

        const Eigen::Vector3d negative_rotation_increment = -angular_velocity * _dt;
        const Eigen::Matrix3d rotation_jacobian = lie::leftJacobian(negative_rotation_increment);

        derivative_jacobian.block<3, 3>(K_POSITION, K_VELOCITY).setIdentity();
        derivative_jacobian.block<3, 3>(K_ROTATION, K_GYRO_BIAS) = -rotation_jacobian;
        derivative_jacobian.block<3, 3>(K_VELOCITY, K_ROTATION) = -rotation * lie::hat(acceleration);
        derivative_jacobian.block<3, 3>(K_VELOCITY, K_ACC_BIAS) = -rotation;
        derivative_jacobian.block<3, 2>(K_VELOCITY, K_GRAVITY) = state_.gravity_direction_.mx(Eigen::Vector2d::Zero());

        noise_jacobian.block<3, 3>(K_ROTATION, 0) = -rotation_jacobian;
        noise_jacobian.block<3, 3>(K_VELOCITY, 3) = -rotation;
        noise_jacobian.block<3, 3>(K_GYRO_BIAS, 6).setIdentity();
        noise_jacobian.block<3, 3>(K_ACC_BIAS, 9).setIdentity();

        const Eigen::Vector3d old_velocity = state_.velocity_;
        state_.position_ += old_velocity * _dt;
        state_.rotation_ = state_.rotation_ * lie::exp(angular_velocity * _dt);
        state_.velocity_ += world_acceleration * _dt;

        StateCovariance transition = StateCovariance::Identity();
        transition.block<3, 3>(K_ROTATION, K_ROTATION) = lie::exp(negative_rotation_increment).toRotationMatrix();
        transition.block<2, 2>(K_GRAVITY, K_GRAVITY) = state_.gravity_direction_.nx() * state_.gravity_direction_.mx(Eigen::Vector2d::Zero());
        transition += derivative_jacobian * _dt;
        covariance_ = transition * covariance_ * transition.transpose() +
                      (_dt * noise_jacobian) * _process_noise * (_dt * noise_jacobian).transpose();
    }

    void updateIterated(const double _measurement_noise)
    {
        if (measurement_model_ == nullptr)
        {
            return;
        }

        DynamicSharedData shared_data;
        const LioState propagated_state = state_;
        const StateCovariance propagated_covariance = covariance_;
        int convergence_count = 0;

        for (int iteration = -1; iteration < maximum_iterations_; ++iteration)
        {
            shared_data.valid_ = true;
            measurement_model_(state_, shared_data);
            if (!shared_data.valid_)
            {
                continue;
            }

            const auto &measurement_jacobian = shared_data.jacobian_;
            const ErrorStateVector delta = state_.boxMinus(propagated_state);
            ErrorStateVector transported_delta = delta;
            StateCovariance iteration_covariance = propagated_covariance;
            transportIteration(state_,
                               propagated_state,
                               delta,
                               transported_delta,
                               iteration_covariance);

            Eigen::Matrix<double, kMeasurementStateDim, kMeasurementStateDim> hessian;
            hessian = measurement_jacobian.transpose() * measurement_jacobian;
            StateCovariance information = (iteration_covariance / _measurement_noise).inverse();
            information.topLeftCorner<kMeasurementStateDim, kMeasurementStateDim>() += hessian;
            const StateCovariance posterior_information_inverse = information.inverse();

            ErrorStateVector kalman_residual;
            kalman_residual = posterior_information_inverse.leftCols<kMeasurementStateDim>() *
                              measurement_jacobian.transpose() * shared_data.residual_;
            StateCovariance kalman_jacobian = StateCovariance::Zero();
            kalman_jacobian.leftCols<kMeasurementStateDim>() = posterior_information_inverse.leftCols<kMeasurementStateDim>() * hessian;

            const ErrorStateVector correction = kalman_residual + (kalman_jacobian - StateCovariance::Identity()) * transported_delta;
            const LioState state_before_correction = state_;
            state_.boxPlus(correction);

            shared_data.converged_ = (correction.cwiseAbs().array() <= convergence_limits_.array()).all();
            if (shared_data.converged_)
            {
                ++convergence_count;
            }
            if (convergence_count == 0 && iteration == maximum_iterations_ - 2)
            {
                shared_data.converged_ = true;
            }

            if (convergence_count > 1 || iteration == maximum_iterations_ - 1)
            {
                finalizeCovariance(state_,
                                   state_before_correction,
                                   correction,
                                   iteration_covariance,
                                   kalman_jacobian);
                return;
            }
        }
    }

private:
    static void transportIteration(const LioState &_current,
                                   const LioState &_reference,
                                   const ErrorStateVector &_delta,
                                   ErrorStateVector &_transported_delta,
                                   StateCovariance &_covariance)
    {
        const int rotation_indices[] = {K_ROTATION};
        for (const int index : rotation_indices)
        {
            const Eigen::Matrix3d transport = lie::leftJacobian(_delta.segment<3>(index)).transpose();
            _transported_delta.segment<3>(index) = transport * _transported_delta.segment<3>(index);
            for (int column = 0; column < kErrorStateDim; ++column)
            {
                _covariance.block<3, 1>(index, column) = transport * _covariance.block<3, 1>(index, column);
            }
            for (int row = 0; row < kErrorStateDim; ++row)
            {
                _covariance.block<1, 3>(row, index) = _covariance.block<1, 3>(row, index) * transport.transpose();
            }
        }

        const Eigen::Matrix2d gravity_transport = _current.gravity_direction_.nx() * _reference.gravity_direction_.mx(_delta.segment<2>(K_GRAVITY));
        _transported_delta.segment<2>(K_GRAVITY) = gravity_transport * _transported_delta.segment<2>(K_GRAVITY);
        for (int column = 0; column < kErrorStateDim; ++column)
        {
            _covariance.block<2, 1>(K_GRAVITY, column) = gravity_transport * _covariance.block<2, 1>(K_GRAVITY, column);
        }
        for (int row = 0; row < kErrorStateDim; ++row)
        {
            _covariance.block<1, 2>(row, K_GRAVITY) = _covariance.block<1, 2>(row, K_GRAVITY) * gravity_transport.transpose();
        }
    }

    void finalizeCovariance(const LioState &_current,
                            const LioState &_pre_correction,
                            const ErrorStateVector &_correction,
                            StateCovariance _covariance,
                            StateCovariance _kalman_jacobian)
    {
        StateCovariance left_covariance = _covariance;
        const int rotation_indices[] = {K_ROTATION};
        for (const int index : rotation_indices)
        {
            const Eigen::Matrix3d transport = lie::leftJacobian(_correction.segment<3>(index)).transpose();
            for (int column = 0; column < kErrorStateDim; ++column)
            {
                left_covariance.block<3, 1>(index, column) = transport * _covariance.block<3, 1>(index, column);
            }
            for (int column = 0; column < kMeasurementStateDim; ++column)
            {
                _kalman_jacobian.block<3, 1>(index, column) = transport * _kalman_jacobian.block<3, 1>(index, column);
            }
            for (int row = 0; row < kErrorStateDim; ++row)
            {
                left_covariance.block<1, 3>(row, index) = left_covariance.block<1, 3>(row, index) * transport.transpose();
                _covariance.block<1, 3>(row, index) = _covariance.block<1, 3>(row, index) * transport.transpose();
            }
        }

        const Eigen::Matrix2d gravity_transport = _current.gravity_direction_.nx() * _pre_correction.gravity_direction_.mx(_correction.segment<2>(K_GRAVITY));
        for (int column = 0; column < kErrorStateDim; ++column)
        {
            left_covariance.block<2, 1>(K_GRAVITY, column) = gravity_transport * _covariance.block<2, 1>(K_GRAVITY, column);
        }
        for (int column = 0; column < kMeasurementStateDim; ++column)
        {
            _kalman_jacobian.block<2, 1>(K_GRAVITY, column) = gravity_transport * _kalman_jacobian.block<2, 1>(K_GRAVITY, column);
        }
        for (int row = 0; row < kErrorStateDim; ++row)
        {
            left_covariance.block<1, 2>(row, K_GRAVITY) = left_covariance.block<1, 2>(row, K_GRAVITY) * gravity_transport.transpose();
            _covariance.block<1, 2>(row, K_GRAVITY) = _covariance.block<1, 2>(row, K_GRAVITY) * gravity_transport.transpose();
        }

        covariance_ = left_covariance -
                      _kalman_jacobian.leftCols<kMeasurementStateDim>() *
                          _covariance.topRows<kMeasurementStateDim>();
    }

    LioState state_;
    StateCovariance covariance_;
    ErrorStateVector convergence_limits_;
    MeasurementModel measurement_model_ = nullptr;
    int maximum_iterations_ = 0;
};
