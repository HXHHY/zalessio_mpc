/*!
 * Discrete-Time Finite-Horizon Linear Quadratic Optimal Control Problem
 *
 * Copyright 2016 by Alessio Zanchettin
 *
 */

 #ifndef SLQ_ALGORITHM_H
 #define SLQ_ALGORITHM_H

 #include "ros/ros.h"

namespace sequential_linear_quadratic
{

  class SLQAlgorithm
  {
  public:

    SLQAlgorithm(double dt, int n_state, int n_input, double *x_final);

    ~SLQAlgorithm();

    //Parameters

    //Functions


  private:
    //Parameters
    double tf_;
    double dt_;
    int n_step_;
    int n_state_;
    int n_input_;
    double *x0_;
    double *x_final_;
    double *controls_;
    double *x_predict_;

    // Node handlers
    //ros::NodeHandle nh_;

    // Subscribers
    //ros::Subscriber quad_state_subscriber_;

    // Publishers
    //ros::Publisher window_flight_msg_pub_;

    //Functions
    void initSLQ(double dt, int n_state, int n_input, double *x_final);
    void simulateSystem(double* contorls, double* x0, double T);
    void solveLQProblem();

  };

}

#endif
