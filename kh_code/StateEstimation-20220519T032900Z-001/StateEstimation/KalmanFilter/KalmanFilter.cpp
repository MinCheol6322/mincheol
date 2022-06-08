/*
 * KalmanFilter.cpp
 *
 *  Created on: 2022. 1. 26.
 *      Author: KimKyungHwan (kkh9764@naver.com)
 *      Seoultech HRRLAB
 *      
 */
#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(int state_dimension, int measure_dimesnion, int input_dimesnion)
    : n(state_dimension), m(measure_dimesnion), k(input_dimesnion),
      F(n, n), B(n, k), H(m,n), Q(n, n), R(m, m),K(m,m), P(n, n),
      I_n(n, n), x_hat(n), x_hat_new(n), u(k) {}
KalmanFilter::~KalmanFilter() {}
void KalmanFilter::Initialize(
    const Eigen::MatrixXd &F,
    const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &H,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    const Eigen::MatrixXd &P0,
    const Eigen::VectorXd &x0)
{
    this->F = F;
    this->B = B;
    this->H = H;
    this->Q = Q;
    this->R = R;
    this->P = P0;
    x_hat = x0; // x_hat.setZero();
    I_n.setIdentity();
    is_initialized = true;
}
void KalmanFilter::SetProcessModel(const Eigen::MatrixXd &F, const Eigen::MatrixXd &B)
{
    this->F = F;
    this->B = B;
}
void KalmanFilter::SetMeasurementModel(const Eigen::MatrixXd &H)
{
    this->H = H;
}
void KalmanFilter::SetCovariance(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
{
    this->Q = Q;
    this->R = R;
}
void KalmanFilter::SetMeasurementNoiseCovariance(const Eigen::MatrixXd &R)
{
    this->R = R;
}
void KalmanFilter::Update(const Eigen::VectorXd &z, const Eigen::VectorXd &u)
{
    if (!is_initialized)
        throw std::runtime_error("!!! KalmanFilter is not initialized !!!");

    //Prediction
    x_hat_new = F * x_hat + B * u; // Predicted state estimate
    P = F * P * F.transpose() + Q; // Predicted error covariance

    //Update
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse(); //Kalman gain
    x_hat_new += K * (z - H * x_hat_new);                          //Updated state estimate
    P = (I_n - K * H) * P;                                         // Updated error covariance
    x_hat = x_hat_new;
}
void KalmanFilter::Update(const Eigen::VectorXd &z)
{
    u.setZero();
    Update(z, u);
}