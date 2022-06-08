/*
 * TrajectoryGenerator.h
 *
 *  Created on: 2021. 12. 23.
 *      Author: KimKyungHwan , HRRLAB , SeoulTech
 * 
 *  unit: meter[m], second[s]
 * 
 * ==========================================
 * 
        trajectory single-axis example
  
  [m]   3-	        _          _  
        2-	     _/  \_      /  \
        1-	___/       \____/    \  
        0-	                      \_______
   time[s]: 0 1 2 3 4 5 6 7 8 9 10 11 12...   
   state:   0  -1  1 -1 2   -1 3 -1 4 4 4....
  
#include <iostream>
#include "TrajectoryGenerator.h"
int main(int, char **)
{
    TrajectoryGenerator trajectory;
    trajectory.SetTimeInterval(0.01);
    int iter = 0;
    while (iter <= 1200)
    {
        if (trajectory.GetState() == 0)
        {
            trajectory.SetInitialPosition(0.5);
            trajectory.SetTrajectoryType(TrajectoryGenerator::SINUSOIDAL);
            trajectory.SetFinalPosition(3);
            trajectory.SetDuration(4);
        }
        else if (trajectory.GetState() == 1)
        {
            trajectory.SetTrajectoryType(TrajectoryGenerator::LSPB);
            trajectory.SetFinalPosition(1);
            trajectory.SetDuration(2, 1);
            trajectory.SetConstantVelocity(1.5);
        }
        else if (trajectory.GetState() == 2)
        {
            trajectory.SetTrajectoryType(TrajectoryGenerator::QUINTIC);
            trajectory.SetFinalPosition(3);
            trajectory.SetFinalVelocity(1);
            trajectory.SetFinalAcceleration(0.5);
            trajectory.SetDuration(2);
        }
        else if (trajectory.GetState() == 3)
        {
            trajectory.SetTrajectoryType(TrajectoryGenerator::QUINTIC);
            trajectory.SetBasicParameters(0., 2);
        }

        trajectory.ComputeTrajectory();

        std::cout << iter << "    " << trajectory.GetPosition()<<"    " <<trajectory.GetVelocity()<<"    " <<trajectory.GetAcceleration() << std::endl;
        iter++;
    }
    return 0;
}
 * 
 * 
 * ============================================
 */
#ifndef TRAJECTORYGENERATOR_H_
#define TRAJECTORYGENERATOR_H_

#include <cmath>
#include <iostream>

class TrajectoryGenerator
{
public:
    enum TrajectoryType
    {
        SINUSOIDAL = 0,
        LSPB = 1,
        QUINTIC = 2,
        BEZIER = 3,
    };

private:
    enum TrajectoryMode
    {
        POINT_SINGLE = 1,
        POINT_XYZ = 3,
    };
    enum Axis
    {
        SINGLE_AXIS = 0,
        X = 0,
        Y = 1,
        Z = 2,
        NUM_AXIS = 3,
    };
    const double kEpsilon;
    const double kPi;
    double dt_; //delta time, default 0.001s
    double t_;  // trajectory time -> 0~duration
    double gt_; // global time -> 0~ Inf  !!Unimplemented!!
    unsigned int trajectory_type_;
    unsigned int trajectory_mode_;
    double pi_[NUM_AXIS];      //initial position
    double pf_[NUM_AXIS];      //final position
    double vi_[NUM_AXIS];      //initial velocity
    double vf_[NUM_AXIS];      //final velocity
    double ai_[NUM_AXIS];      //initial acceleration
    double af_[NUM_AXIS];      //final acceleration
    double tf_;                //final time(=duration)
    double v_const_[NUM_AXIS]; //LSPB constant velocity
    double tb_[NUM_AXIS];      //LSPB trajectory blend time
    double a_const_[NUM_AXIS]; //LSPB constant acceleration
    double inv_tf_;            // =1/tf_
    double waiting_time_;
    double end_time_;
    double pm_[NUM_AXIS]; // for bezier trajectory(middle params)
    int state_;           //arrived(0,1,2...) or moving(-1)
    int state_old_;
    double quintic_coefficient_[NUM_AXIS][6];

    //time-varying
    double p_[NUM_AXIS]; // position
    double v_[NUM_AXIS]; // velocity
    double a_[NUM_AXIS]; // acceleration

    //boolean
    bool pause_;
    bool bezier_init_[NUM_AXIS];
    bool has_final_position;
    bool has_final_velocity;
    bool has_final_acceleration;
    bool has_bezier_middle_point;
    bool has_duration;
    bool is_computed_once;
    bool is_initialize_point;

public:
    TrajectoryGenerator();
    virtual ~TrajectoryGenerator();
    void SetTimeInterval(double dt) { dt_ = dt; }
    void SetTrajectoryType(int type) { trajectory_type_ = type; }
    void SetInitialPosition(double initial_position)
    {
        pi_[SINGLE_AXIS] = initial_position;
        is_initialize_point = true;
    }
    void SetInitialPosition(double initial_x, double initial_y, double initial_z)
    {
        pi_[X] = initial_x;
        pi_[Y] = initial_y;
        pi_[Z] = initial_z;
        trajectory_mode_ = POINT_XYZ;
        is_initialize_point = true;
    }

    void SetBasicParameters(double final_position, double duration_t, double wait_t = 0)
    {
        pf_[SINGLE_AXIS] = final_position;
        tf_ = duration_t;
        waiting_time_ = wait_t;
        has_final_position = true;
        has_duration = true;
        is_computed_once = false;
    }
    void SetBasicParameters(double final_x, double final_y, double final_z, double duration_t, double wait_t = 0)
    {
        pf_[X] = final_x;
        pf_[Y] = final_y;
        pf_[Z] = final_z;
        tf_ = duration_t;
        waiting_time_ = wait_t;
        has_final_position = true;
        has_duration = true;
        is_computed_once = false;
        trajectory_mode_ = POINT_XYZ;
    }
    void SetFinalPosition(double final_position)
    {
        pf_[SINGLE_AXIS] = final_position;
        has_final_position = true;
        is_computed_once = false;
    }
    void SetFinalPosition(double final_x, double final_y, double final_z)
    {
        pf_[X] = final_x;
        pf_[Y] = final_y;
        pf_[Z] = final_z;
        has_final_position = true;
        is_computed_once = false;
        trajectory_mode_ = POINT_XYZ;
    }
    void SetFinalVelocity(double final_velocity)
    {
        vf_[SINGLE_AXIS] = final_velocity;
        has_final_velocity = true;
    }
    void SetFinalVelocity(double final_x_dot, double final_y_dot, double final_z_dot)
    {
        vf_[X] = final_x_dot;
        vf_[Y] = final_y_dot;
        vf_[Z] = final_z_dot;
        has_final_velocity = true;
    }
    void SetFinalAcceleration(double final_acceleration)
    {
        af_[SINGLE_AXIS] = final_acceleration;
        has_final_acceleration = true;
    }
    void SetFinalAcceleration(double final_x_ddot, double final_y_ddot, double final_z_ddot)
    {
        af_[X] = final_x_ddot;
        af_[Y] = final_y_ddot;
        af_[Z] = final_z_ddot;
        has_final_acceleration = true;
    }

    void SetConstantVelocity(double const_velocity)
    {
        v_const_[SINGLE_AXIS] = const_velocity;
    }
    void SetConstantVelocity(double const_x_dot, double const_y_dot, double const_z_dot)
    {
        v_const_[X] = const_x_dot;
        v_const_[Y] = const_y_dot;
        v_const_[Z] = const_z_dot;
    }
    void SetDuration(double duration_t, double wait_t = 0)
    {
        tf_ = duration_t;
        waiting_time_ = wait_t;
        has_duration = true;
        is_computed_once = false;
    }
    void SetBezierMiddlePosition(double bezier_middle_position)
    {
        pm_[SINGLE_AXIS] = bezier_middle_position;
        has_bezier_middle_point = true;
    }
    void SetBezierMiddlePosition(double bezier_mid_x, double bezier_mid_y, double bezier_mid_z)
    {
        pm_[X] = bezier_mid_x;
        pm_[Y] = bezier_mid_y;
        pm_[Z] = bezier_mid_z;
        has_bezier_middle_point = true;
    }

    void ComputeTrajectory();

    int GetState() { return state_; }
    double GetPosition(int axis = SINGLE_AXIS) { return p_[axis]; }
    double GetVelocity(int axis = SINGLE_AXIS) { return v_[axis]; }
    double GetAcceleration(int axis = SINGLE_AXIS) { return a_[axis]; }
    double *GetXYZ() { return p_; }
    double *GetXYZDot() { return v_; }
    double *GetXYZDdot() { return a_; }
    double GetFinalPosition(int axis = SINGLE_AXIS) { return pf_[axis]; }
    double GetTime(){return t_;}
    void Resume() { pause_ = false; }
    void Pause() { pause_ = true; }
    void Reset() { state_ = 0; }
    void example();
    void BreakingUp();

private:
    void Initialize();
    void IncreaseTimeAndStateUpdate();
    void FinalValueToInitialValue(int axis = SINGLE_AXIS);
    int Sgn(double val) { return (kEpsilon < val) - (val < -kEpsilon); }
    void SetTempQuinticTraj(const double pi, const double pf, const double vi, const double vf, const double ai, const double af, const double tf, size_t axis);
    void ComputeTrajectoryUsingTime(double time, size_t axis);
};
#endif /* TRAJECTORYGENERATOR_H_ */
