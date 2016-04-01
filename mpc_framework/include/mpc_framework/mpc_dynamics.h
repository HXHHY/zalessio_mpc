/*
 */

#ifndef MPC_DYNAMICS_H
#define MPC_DYNAMICS_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <ros/ros.h>

//odeint
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <functional>

//msgs
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

namespace rpg_mpc {

  class ModelPredictiveControlDynamics
  {
   public:
    ModelPredictiveControlDynamics(ros::NodeHandle& nh_);
    ~ModelPredictiveControlDynamics();

    // Functions
    std::vector<nav_msgs::Odometry> symulateUAV(double initial_time,
      double final_time, double dt, nav_msgs::Odometry & initial_condition);
    std::vector<nav_msgs::Odometry> symulateUGV(double initial_time,
      double final_time, double dt, nav_msgs::Odometry & initial_condition);

   private:
    //  Variables
    //Quad constants
    float m;
    float Ixx;
    float Iyy;
    float Izz;
    float d;
    //Physical constants
    float g;
    float k;
    float b;
    //State
    typedef boost::array< double , 13 > state_type;
    double t0;
    double tf;
    double tm;
    double n_step;
    //Final containers
    std::vector<state_type> final_solution_;
    std::vector<double> final_times_;
    std::vector<nav_msgs::Odometry> final_solution_odometry_;
    //Stepper
    typedef boost::numeric::odeint::runge_kutta4< state_type > stepper_type;

    // Functions
    double * calculateInputControl(double *x, double *x_final);
    void UGVDynamics( const state_type &x , state_type &xprime , double t );
    void UAVDynamics( const state_type &x , state_type &xprime , double t );
    void saveDynamics( const state_type &x , const double t );
    state_type initialConditionSystem(double initial_time, double final_time,
       double dt, nav_msgs::Odometry & initial_condition);

  };

}

#endif
