/*
 * StateEstimation.cpp
 *
 *  Created on: 2022. 1. 26.
 *      Author: KimKyungHwan (kkh9764@naver.com)
 *      Seoultech HRRLAB
 *      
 */
#include "StateEstimation.hpp"
StateEstimation::~StateEstimation() {}
StateEstimation::StateEstimation(double dt)
    : x_state_kalmanfilter(6, 8, 1),
      y_state_kalmanfilter(6, 8, 1),
      z_state_kalmanfilter(6, 8, 1),
      R_(8, 8)
{
    this->dt=dt;
    //F(6,6), B(6, 1), H(8,6),Q(6, 6), R(8, 8), P(6, 6)
    //x_hat(6), z(8), u(1)
    Eigen::MatrixXd F(6,6), B(6, 1),Q(6, 6), H(8,6), P0(6, 6);
    Eigen::VectorXd x_state0(6),y_state0(6),z_state0(6);
    double L1=0.35;
    double L2=0.11;
    F<<1, dt, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
    B<<0.5*dt*dt, dt, 0, 0, 0, 0;
    H<<-1, 0, 1, 0, 0, 0,
       -1, 0, 0, 1, 0, 0,
       -1, 0, 0, 0, 1, 0,
       -1, 0, 0, 0, 0, 1,
       0, -1, 0, 0, 0, 0,
       0, -1, 0, 0, 0, 0,
       0, -1, 0, 0, 0, 0,
       0, -1, 0, 0, 0, 0;
       //Q down->filter up
    // Q<<0.25*0.01*dt*dt*dt*dt, 0, 0, 0, 0, 0,
    //    0, 1.*dt*dt*0.0001, 0, 0, 0, 0,
    //    0, 0, 0.001, 0, 0, 0,
    //    0, 0, 0, 0.001, 0, 0,
    //    0, 0, 0, 0, 0.001, 0,
    //    0, 0, 0, 0, 0, 0.001;
    Q << 0.001, 0, 0, 0, 0, 0,
        0, 0.001, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;
    R_ << 1, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 100, 0, 0, 0,
        0, 0, 0, 0, 0, 100, 0, 0,
        0, 0, 0, 0, 0, 0, 100, 0,
        0, 0, 0, 0, 0, 0, 0, 100;
    P0 << 10000, 0, 0, 0, 0, 0,
        0, 10000, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;
    x_state0<<0, 0, L1, L1, -L1, -L1;
    y_state0<<0, 0, L2, -L2, L2, -L2;
    z_state0<<0.45, 0, 0, 0, 0, 0;
    x_state_kalmanfilter.Initialize(F,B,H,Q,R_,P0,x_state0);
    y_state_kalmanfilter.Initialize(F,B,H,Q,R_,P0,y_state0);
    z_state_kalmanfilter.Initialize(F,B,H,Q,R_,P0,z_state0);
}


void StateEstimation::UpdateX(const Eigen::VectorXd &z, const Eigen::VectorXd &u, const Eigen::MatrixXd &R){
    R_=R;
    x_state_kalmanfilter.SetMeasurementNoiseCovariance(R_);
    x_state_kalmanfilter.Update(z,u);
}
void StateEstimation::UpdateX(const Eigen::VectorXd &z, const Eigen::VectorXd &u){
    x_state_kalmanfilter.SetMeasurementNoiseCovariance(R_);
    x_state_kalmanfilter.Update(z,u);
}
void StateEstimation::UpdateY(const Eigen::VectorXd &z, const Eigen::VectorXd &u, const Eigen::MatrixXd &R){
    R_=R;
    y_state_kalmanfilter.SetMeasurementNoiseCovariance(R_);
    y_state_kalmanfilter.Update(z,u);
}
void StateEstimation::UpdateY(const Eigen::VectorXd &z, const Eigen::VectorXd &u){
    y_state_kalmanfilter.SetMeasurementNoiseCovariance(R_);
    y_state_kalmanfilter.Update(z,u);
}
void StateEstimation::UpdateZ(const Eigen::VectorXd &z, const Eigen::VectorXd &u, const Eigen::MatrixXd &R){
    R_=R;
    z_state_kalmanfilter.SetMeasurementNoiseCovariance(R_);
    z_state_kalmanfilter.Update(z,u);
}
void StateEstimation::UpdateZ(const Eigen::VectorXd &z, const Eigen::VectorXd &u){
    z_state_kalmanfilter.SetMeasurementNoiseCovariance(R_);
    z_state_kalmanfilter.Update(z,u);
}
Eigen::VectorXd StateEstimation::GetXState(){
    return x_state_kalmanfilter.GetState();
}
Eigen::VectorXd StateEstimation::GetYState(){
    return y_state_kalmanfilter.GetState();
}
Eigen::VectorXd StateEstimation::GetZState(){
    return z_state_kalmanfilter.GetState();
}

 
