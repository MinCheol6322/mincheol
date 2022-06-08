/*
 * KalmanFilter.hpp
 *
 *  Created on: 2022. 1. 26.
 *      Author: KimKyungHwan (kkh9764@naver.com)
 *      Seoultech HRRLAB
 *      
 */

#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_
#include <iostream>
#include <stdexcept>
#include "Eigen/Dense"

class KalmanFilter
{
public:
    /*
  *   x_hat_new = F*x_hat + B*u + w
  *   z = H*x_hat + v
  * 
  *   F - System transition matrix
  *   B - Control-input matrix
  *   H - Measurement matrix
  *   Q - Process noise covariance(w)
  *   R - Measurement noise covariance(v)
  *   P - State error covariance
  *   K - Kalman gain
  */
    KalmanFilter();
    KalmanFilter(int state_dimension, int measure_dimesnion, int input_dimesnion = 1);
    virtual ~KalmanFilter();
    void Initialize(
        const Eigen::MatrixXd &F,
        const Eigen::MatrixXd &B,
        const Eigen::MatrixXd &H,
        const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &R,
        const Eigen::MatrixXd &P0,
        const Eigen::VectorXd &x0);
    void SetProcessModel(const Eigen::MatrixXd &F, const Eigen::MatrixXd &B);
    void SetMeasurementModel(const Eigen::MatrixXd &H);
    void SetCovariance(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);
    void SetMeasurementNoiseCovariance(const Eigen::MatrixXd &R);
    //“prediction” and “update”
    void Update(const Eigen::VectorXd &z);
    void Update(const Eigen::VectorXd &z, const Eigen::VectorXd &u);

    Eigen::VectorXd GetState() { return x_hat; };

private:
    Eigen::MatrixXd F, B, H, Q, R, K, P;
    int m; //state dimensions
    int n; //measurement dimensions
    int k; //input dimensions
    bool is_initialized;
    Eigen::MatrixXd I_n;              // n x n identity
    Eigen::VectorXd x_hat, x_hat_new; // State Vector
    Eigen::VectorXd u;
};
#endif