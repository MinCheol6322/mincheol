/*
 * StateEstimation.hpp
 *
 *  Created on: 2022. 1. 26.
 *      Author: KimKyungHwan (kkh9764@naver.com)
 *      Seoultech HRRLAB
 *      
 */

#ifndef STATE_ESTIMATION_HPP_
#define STATE_ESTIMATION_HPP_

#include <iostream>
#include <math.h>
// #include "Eigen/Dense"
#include "KalmanFilter/KalmanFilter.hpp"

class StateEstimation
{
public:
    StateEstimation(double dt=0.001);
    virtual ~StateEstimation();

    void UpdateX(const Eigen::VectorXd &z, const Eigen::VectorXd &u, const Eigen::MatrixXd &R);
    void UpdateX(const Eigen::VectorXd &z, const Eigen::VectorXd &u);
    void UpdateY(const Eigen::VectorXd &z, const Eigen::VectorXd &u, const Eigen::MatrixXd &R);
    void UpdateY(const Eigen::VectorXd &z, const Eigen::VectorXd &u);
    void UpdateZ(const Eigen::VectorXd &z, const Eigen::VectorXd &u, const Eigen::MatrixXd &R);
    void UpdateZ(const Eigen::VectorXd &z, const Eigen::VectorXd &u);
    Eigen::VectorXd GetXState();
    Eigen::VectorXd GetYState();
    Eigen::VectorXd GetZState();
    // Eigen::VectorXd GetFootState();

private:
    KalmanFilter x_state_kalmanfilter;
    KalmanFilter y_state_kalmanfilter;
    KalmanFilter z_state_kalmanfilter;
    Eigen::MatrixXd R_;//measurement covariance
    double dt;//time step
};
#endif