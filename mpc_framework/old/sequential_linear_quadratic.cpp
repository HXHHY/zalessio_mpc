/*!
 * Discrete-Time Finite-Horizon Linear Quadratic Optimal Control Problem
 *
 * Copyright 2016 by Alessio Zanchettin
 *
 */

 #ifndef SLQ_ALGORITHM_CPP
 #define SLQ_ALGORITHM_CPP

#include "rpg_slq_mpc/sequential_linear_quadratic.h"
#include "uav_dynamics.c"

namespace sequential_linear_quadratic
{
  SLQAlgorithm::SLQAlgorithm(double dt, int n_state, int n_input, double *x_final)
  {
   SLQAlgorithm::initSLQ( dt, n_state, n_input, x_final);
  }

  SLQAlgorithm::~SLQAlgorithm()
  {
  }

  void SLQAlgorithm::initSLQ(double dt, int n_state, int n_input, double *x_final){

    dt_ = dt;
    n_state_ = n_state;
    n_input_ = n_input;
    x_final_ = x_final;

  }

  void SLQAlgorithm::simulateSystem(double* contorls, double* x0, double T){

    tf_ = T;
    n_step_ = ceil(tf_/dt_);

    controls_ = ( double * ) malloc ( n_step_ * sizeof ( double ) );
    for(int i = 0; i < n_step_; i++)
      controls_[i] = contorls[i];

    x0_ = ( double * ) malloc ( n_state_ * sizeof ( double ) );
    for(int i = 0; i < n_state_; i++)
      x0_[i] = x0[i];

    x_predict_ = simulateUavDynamics( 0.0,  tf_,  dt_, x0_, controls_ , n_state_);
  }

  void SLQAlgorithm::solveLQProblem(){

    /*double t = 0;
    std::vector<double *> A;
    std::vector<double *> B;
    for(int i = 0; i < n_step_; i++ , t+=dt_){
      A[i] = uavDynamicsLinearizeA(t, n_state_,  &x_predict_[i*n_state_], &controls_[i*n_input_] );
      B[i] = uavDynamicsLinearizeB(t, n_state_,  n_input_,  &x_predict_[i*n_state_], &controls_[i*n_input_] );
    }

    for(int i = n_step_ ; i >= 0;i-- , t+=dt_){
      H = R + Bt[i]*P_tp1_*B[i];
      G = Bt[i]*P_tp1_*A[i];
      g = r + Bt[i]*p_tp1;
      K[i] = -Hinv*G;
      l[i] = -Hinv*g;
      P_t = Q + At[i]*P_tp1*A[i] + Kt[i]*H*K[i]*G + Gt*K[i];
      p_t = q + At[i]*p_tp1      + Kt[i]*H*l[i] + lt[i]*g + Gt*l[i];
      P_tp1 = P_t;
      p_tp1 = p_t;
    }*/

  }

}

#endif
