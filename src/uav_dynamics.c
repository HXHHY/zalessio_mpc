/*!
 * UAV DYNAMICS
 *
 * Copyright 2016 by Alessio Zanchettin
 *
 */

#ifndef UAV_DINAMICS_C
#define UAV_DINAMICS_C

#include "rk4_ode_solver.c"

//  Quad constants
float m = 1.56779;
float Ixx = 3.47563e-2;
float Iyy = 3.47563e-2;
float Izz = 9.77e-2;
float d = 0.215;
//  Physical constants
float g = 9.81;
float k = 1.6e-2;
float b = 1;

/*
  TODO should add some noise? model the noise?
  TODO all this calculation can be done in parallel they do not depend on each other!
*/

/******************************************************************************/
/*
  uavDynamics evaluates the right hand side of a vector ODE describing the uav
  dynamics.

  Parameters:
    Input:  - double t, the current time.
            - int n_state, the dimension of the system.
            - double x[], the current solution value.
                   0  1  2   3   4   5   6   7   8   9  10 11 12
                  [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r]
            - double u[], the current input value.
                    0   1   2   3
                  [f1, f2, f3, f4]
    Output: - double xprime[], the value of the derivative, dx/dt.
*/
double *uavDynamics( double t, int n_state, double x[], double u[] )
{
  double c = (u[0] + u[1] + u[2] + u[3])/m;
  double halfsqrt2 = sqrt(2)/2;

  double *xprime;
  xprime = ( double * ) malloc ( n_state * sizeof ( double ) );

  xprime[0]  = x[3];
  xprime[1]  = x[4];
  xprime[2]  = x[5];
  xprime[3]  =  2*(x[6]*x[8] + x[7]*x[9])*c;
  xprime[4]  =  2*(x[8]*x[9] - x[6]*x[7])*c;
  xprime[5]  = -g   + (x[6]*x[6] - x[7]*x[7] - x[8]*x[8] - x[9]*x[9])*c;
  xprime[6]  = 0.5*(            - x[10]*x[7] - x[11]*x[8] - x[12]*x[9]);
  xprime[7]  = 0.5*( x[10]*x[6]              + x[12]*x[8] - x[11]*x[9]);
  xprime[8]  = 0.5*( x[11]*x[6] - x[12]*x[7]              - x[10]*x[9]);
  xprime[9]  = 0.5*( x[12]*x[6] + x[11]*x[7] - x[10]*x[8]             );
  xprime[10] = (halfsqrt2*d*b*( u[0] - u[1] - u[2] + u[3]) - x[11]*x[12]*(Izz - Iyy))/Ixx;
  xprime[11] = (halfsqrt2*d*b*(-u[0] - u[1] + u[2] + u[3]) - x[10]*x[12]*(Ixx - Izz))/Iyy;
  xprime[12] =             (k*( u[0] - u[1] + u[2] - u[3]) - x[10]*x[11]*(Iyy - Ixx))/Izz;

  return xprime;
}

/******************************************************************************/
/*
  uavDynamicsLinearizeA evaluates the derivative with respect of x of the
  right hand side of a vector ODE describing the uav dynamics.

  Parameters:
    Input:  - double t, the current time.
            - int n_state, the dimension of the system.
            - double x[], the current solution value.
                   0  1  2   3   4   5   6   7   8   9  10 11 12
                  [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r]
            - double u[], the current input value.
                    0   1   2   3
                  [f1, f2, f3, f4]
    Output: - double A[], the value of the derivative, df/dx.
*/
double *uavDynamicsLinearizeA( double t, int n_state, double x[], double u[] )
{
  double c = (u[0] + u[1] + u[2] + u[3])/m;

  double *A;
  A = ( double * ) malloc ( n_state *n_state* sizeof ( double ) );

  for(int i = 0; i < n_state; i ++){
    A[n_state*i    ] = 0;
    A[n_state*i + 1] = 0;
    A[n_state*i + 2] = 0;
  }

  for(int i = 0; i < 3; i ++){
    A[n_state*i + 6] = 0;
    A[n_state*i + 7] = 0;
    A[n_state*i + 8] = 0;
    A[n_state*i + 9] = 0;
    A[n_state*i + 10] = 0;
    A[n_state*i + 11] = 0;
    A[n_state*i + 12] = 0;
  }

  for(int i = 3; i < n_state; i ++){
    A[n_state*i + 3] = 0;
    A[n_state*i + 4] = 0;
    A[n_state*i + 5] = 0;
  }

  for(int i = 3; i < 6; i ++){
    A[n_state*i + 10] = 0;
    A[n_state*i + 11] = 0;
    A[n_state*i + 12] = 0;
  }

  for(int i = 10; i < n_state; i ++){
    A[n_state*i + 6] = 0;
    A[n_state*i + 7] = 0;
    A[n_state*i + 8] = 0;
    A[n_state*i + 9] = 0;
  }

  A[3]   = 1;
  A[4]   = 0;
  A[5]   = 0;

  A[16]  = 0;
  A[17]  = 1;
  A[18]  = 0;

  A[29]  = 0;
  A[30]  = 0;
  A[31]  = 1;

  A[45]  = 2*x[8]*c;
  A[46]  = 2*x[9]*c;
  A[47]  = 2*x[6]*c;
  A[48]  = 2*x[7]*c;

  A[58]  = -2*x[7]*c;
  A[59]  = -2*x[6]*c;
  A[60]  = 2*x[9]*c;
  A[61]  = 2*x[8]*c;

  A[71]  =  2*x[6]*c;
  A[72]  = -2*x[7]*c;
  A[73]  = -2*x[8]*c;
  A[74]  = -2*x[9]*c;

  A[84]  = 0;
  A[85]  = -0.5*x[10];
  A[86]  = -0.5*x[11];
  A[87]  = -0.5*x[12];
  A[88]  = -0.5*x[7];
  A[89]  = -0.5*x[8];
  A[90]  = -0.5*x[9];

  A[97]  = 0.5*x[10];
  A[98]  = 0;
  A[99]  = 0.5*x[12];
  A[100] = -0.5*x[11];
  A[101] = 0.5*x[6];
  A[102] = -0.5*x[9];
  A[103] = 0.5*x[8];

  A[110] = 0.5*x[11];
  A[111] = -0.5*x[12];
  A[112] = 0;
  A[113] = -0.5*x[10];
  A[114] = -0.5*x[9];
  A[115] = 0.5*x[6];
  A[116] = -0.5*x[7];

  A[123] = 0.5*x[12];
  A[124] = 0.5*x[11];
  A[125] = -0.5*x[10];
  A[126] = 0;
  A[127] = -0.5*x[8];
  A[128] = 0.5*x[7];
  A[129] = 0.5*x[6];

  A[140] = 0;
  A[141] = -x[12]*(Izz - Iyy)/Ixx;
  A[142] = -x[11]*(Izz - Iyy)/Ixx;

  A[153] = -x[12]*(Ixx - Izz)/Iyy;
  A[154] = 0;
  A[155] = -x[10]*(Ixx - Izz)/Iyy;

  A[166] = -x[11]*(Iyy - Ixx)/Izz;
  A[167] = -x[10]*(Iyy - Ixx)/Izz;
  A[168] = 0;

  return A;
}

/******************************************************************************/
/*
  uavDynamicsLinearizeB evaluates the derivative with respect of u of the
  right hand side of a vector ODE describing the uav dynamics.

  Parameters:
    Input:  - double t, the current time.
            - int n_state, the dimension of the system.
            - double x[], the current solution value.
                   0  1  2   3   4   5   6   7   8   9  10 11 12
                  [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r]
            - double u[], the current input value.
                    0   1   2   3
                  [f1, f2, f3, f4]
    Output: - double B[], the value of the derivative, df/du.
*/
double *uavDynamicsLinearizeB( double t, int n_state, int n_input,  double x[], double u[] )
{
  double halfsqrt2 = sqrt(2)/2;

  double *B;
  B = ( double * ) malloc ( n_state*n_input * sizeof ( double ) );

  for(int i = 0; i < 3*n_input; i ++)
    B[i] = 0;

  for(int i = 6*n_input; i < 10*n_input; i ++)
    B[i] = 0;

  B[12]   = 2*(x[6]*x[8] + x[7]*x[9])/m;
  B[13]   = 2*(x[6]*x[8] + x[7]*x[9])/m;
  B[14]   = 2*(x[6]*x[8] + x[7]*x[9])/m;
  B[15]   = 2*(x[6]*x[8] + x[7]*x[9])/m;

  B[16]   = 2*(x[8]*x[9] - x[6]*x[7])/m;
  B[17]   = 2*(x[8]*x[9] - x[6]*x[7])/m;
  B[18]   = 2*(x[8]*x[9] - x[6]*x[7])/m;
  B[19]   = 2*(x[8]*x[9] - x[6]*x[7])/m;

  B[20]   = (x[6]*x[6] - x[7]*x[7] - x[8]*x[8] - x[9]*x[9])/m;
  B[21]   = (x[6]*x[6] - x[7]*x[7] - x[8]*x[8] - x[9]*x[9])/m;
  B[22]   = (x[6]*x[6] - x[7]*x[7] - x[8]*x[8] - x[9]*x[9])/m;
  B[23]   = (x[6]*x[6] - x[7]*x[7] - x[8]*x[8] - x[9]*x[9])/m;

  B[40]   = halfsqrt2*d*b/Ixx;
  B[41]   = -halfsqrt2*d*b/Ixx;
  B[42]   = -halfsqrt2*d*b/Ixx;
  B[43]   = halfsqrt2*d*b/Ixx;

  B[44]   = -halfsqrt2*d*b/Iyy;
  B[45]   = -halfsqrt2*d*b/Iyy;
  B[46]   = halfsqrt2*d*b/Iyy;
  B[47]   = halfsqrt2*d*b/Iyy;

  B[48]   = k/Izz;
  B[49]   = -k/Izz;
  B[50]   = k/Izz;
  B[51]   = -k/Izz;

  return B;
}

/******************************************************************************/
/*
  simulateUavDynamics simulate the ODE system describing the uav dynamics for
  a determinate time range and time step.

  Parameters:
    Input:  - double t0 initial time.
            - double tf final time.
            - double dt time step.
            - double u0[4*(tf-dt-t0)/dt] the input vector.
                    0      1      2      3
                [f1(t0), f2t0), f3t0), f4t0), f1(t0+dt), ... , f4(tf-dt)]
            - int n_state, the dimension of the system.
    Output: - double xprime[], the value of the derivative, dx/dt.
*/
double * simulateUavDynamics(double t0, double tf, double dt, double *x0, double *u0 , int n_state)
{
  int n_step = ceil((tf-t0)/dt + 1);
  int pointer_final_state = 0;

  double *X;
  X = ( double * ) malloc ( n_state *n_step * sizeof ( double ) );

  for ( ; pointer_final_state < n_state; pointer_final_state++ )
    X[pointer_final_state] = x0[pointer_final_state];


  double *x1;
  //input: [f1, f2, f3, f4]
  double *u;
  u = ( double * ) malloc ( 4 * sizeof ( double ) );

  for(int j = 0; j < n_step-1; j++, t0 += dt)
  {
    u[0] = u0[j*4 + 0];
    u[1] = u0[j*4 + 1];
    u[2] = u0[j*4 + 2];
    u[3] = u0[j*4 + 3];

    x1 = rk4 ( t0, n_state, &X[pointer_final_state-n_state], u, dt, uavDynamics );

    for ( int i = 0; i < n_state; i++, pointer_final_state++ )
      X[pointer_final_state] = x1[i];

    free( x1 );
  }

  return X;
}
















/******************************************************************************/
/*
  calculateInpuControl calculate the PD controller to apply from state x to reach
  state x_final

  Parameters:
    Input:  - double x[n_state], the current solution value.
                   0  1  2   3   4   5   6   7   8   9  10 11 12
                  [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r]
            - double x_final[], the final state.
                   0  1  2   3   4   5  6  7  8
                  [x, y, z, vx, vy, vz, p, q, r]
    Output: - double f[], the input value to apply to the dynamics.
                   0   1   2   3
                 [f1, f2, f3, f4]
*/
double * calculateInpuControl(double *x, double *x_final)
{
  // PID gains
  float kxp = 0.3;
  float kxd = 0.9;
  float kyp = 0.3;
  float kyd = 0.9;
  float kzp = 4;
  float kzd = 5;
  float kphip = 10;
  float kphid = 5;
  float kthetap = 10;
  float kthetad = 5;
  float kpsip = 3;
  float kpsid = 1.7;

  // Error with target position
  float e_x = x_final[0] - x[0];
  float e_y = x_final[1] - x[1];
  float e_z = x_final[2] - x[2];
  // Error with target velocity
  float e_velx = x_final[3] - x[3];
  float e_vely = x_final[4] - x[4];
  float e_velz = x_final[5] - x[5];
  // Error with target angular velocity
  float e_velphi   = x_final[6] - x[10];
  float e_veltheta = x_final[7] - x[11];
  float e_velpsi   = x_final[8] - x[12];
  // Quaternion
  float qw = x[6];
  float qx = x[7];
  float qy = x[8];
  float qz = x[9];

  //PD for position  x,y,z
  float dx = kxp*e_x + kxd*e_velx;
  float dy = kyp*e_y + kyd*e_vely;
  float dz = kzp*e_z + kzd*e_velz;

  //Set target for the angles:
  //     based on the x,y PD controller we set the target quaternion that we want in order to go in that position
  //     when we are on the position the target angles are [1,0,0,0]
  float qd_w = sqrt(0.5+(dz + g)/(2*sqrt(dx*dx + dy*dy + (dz + g)*(dz + g))) - qz*qz);
  float qd_x = (dx*qz - dy*qw)*(2*(qw*qw+qz*qz)-1)/(2*(dz + g)*(qw*qw+qz*qz));
  float qd_y = (dx*qw + dy*qz)*(2*(qw*qw+qz*qz)-1)/(2*(dz + g)*(qw*qw+qz*qz));
  float qd_z = 0.0;
  // Error with target angles
  float qe_1 = -qd_w*qx - qd_z*qy + qd_y*qz + qd_x*qw;
  float qe_2 =  qd_z*qx - qd_w*qy - qd_x*qz + qd_y*qw;
  float qe_3 = -qd_y*qx + qd_x*qy - qd_w*qz + qd_z*qw;
  float qe_4 =  qd_x*qx + qd_y*qy + qd_z*qz + qd_w*qw;
  //PD for attitude
  float dphi   = kphip*qe_1*qe_4   + kphid*(e_velphi);
  float dtheta = kthetap*qe_2*qe_4 + kthetad*(e_veltheta);
  float dpsi   = kpsip*qe_3*qe_4   + kpsid*(e_velpsi);

  //Set the controllers:
  //     Thrust T aligned with the quad's z axis
  //     Torque Tauphi   along the x axis to move the quad along its y axis
  //     Torque Tautheta along the y axis to move the quad along its x axis
  //     Torque Taupsi   along the z axis to move the quad along its z axis
  float T        = m*(dz + g)/(2*(qw*qw+qz*qz)-1);
  float Tauphi   = dphi*Ixx;
  float Tautheta = dtheta*Iyy;
  float Taupsi   = dpsi*Izz;

  //For each rotor we have a portion of the controllers depending on the position of the rotor under consideration:
  float cT        = T/(b*4);
  float cTauphi   = sqrt(2)*Tauphi/(b*d*4);
  float cTautheta = sqrt(2)*Tautheta/(b*d*4);
  float cTaupsi   = Taupsi/(k*4);

  double * f;
  f = ( double * ) malloc ( 4 * sizeof ( double ) );
  f[0] = cT + cTaupsi - cTautheta + cTauphi;
  f[1] = cT - cTaupsi - cTautheta - cTauphi;
  f[2] = cT + cTaupsi + cTautheta - cTauphi;
  f[3] = cT - cTaupsi + cTautheta + cTauphi;

 return f;
}

/******************************************************************************/
/*
  uav_dynamics evaluates the right hand side of a vector ODE describing the uav
  dynamics.

  Parameters:
    Input:  - double t, the current time.
            - int n_state, the dimension of the system.
            - double x[n_state], the current solution value.
                   0  1  2   3   4   5   6   7   8   9  10 11 12
                  [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r]
            - double x_final[], the final state.
                   0  1  2   3   4   5  6  7  8
                  [x, y, z, vx, vy, vz, p, q, r]
    Output: - double xprime[n_state], the value of the derivative, dx/dt.
*/
double *uavDynamicsPDController( double t, int n_state, double x[], double x_final[] )
{
  double *u = calculateInpuControl(x, x_final);

  double c = (u[0] + u[1] + u[2] + u[3])/m;
  double halfsqrt2 = sqrt(2)/2;

  double *xprime;
  xprime = ( double * ) malloc ( n_state * sizeof ( double ) );

  xprime[0]  = x[3];
  xprime[1]  = x[4];
  xprime[2]  = x[5];
  xprime[3]  =  2*(x[6]*x[8] + x[7]*x[9])*c;
  xprime[4]  =  2*(x[8]*x[9] - x[6]*x[7])*c;
  xprime[5]  = -g   + (x[6]*x[6] - x[7]*x[7] - x[8]*x[8] - x[9]*x[9])*c;
  xprime[6]  = 0.5*(            - x[10]*x[7] - x[11]*x[8] - x[12]*x[9]);
  xprime[7]  = 0.5*( x[10]*x[6]              + x[12]*x[8] - x[11]*x[9]);
  xprime[8]  = 0.5*( x[11]*x[6] - x[12]*x[7]              - x[10]*x[9]);
  xprime[9]  = 0.5*( x[12]*x[6] + x[11]*x[7] - x[10]*x[8]             );
  xprime[10] = (halfsqrt2*d*b*( u[0] - u[1] - u[2] + u[3]) - x[11]*x[12]*(Izz - Iyy))/Ixx;
  xprime[11] = (halfsqrt2*d*b*(-u[0] - u[1] + u[2] + u[3]) - x[10]*x[12]*(Ixx - Izz))/Iyy;
  xprime[12] =             (k*( u[0] - u[1] + u[2] - u[3]) - x[10]*x[11]*(Iyy - Ixx))/Izz;

  return xprime;
}

/*
  simulate_uav_dynamics simulate the ODE system describing the uav dynamics for
  a determinate time range and time step.

  Parameters:
    Input:  - double t0 initial time.
            - double tf final time.
            - double dt time step.
            - double xfinal[n_state] the final state vector.
                  0  1  2   3   4   5  6  7  8
                 [x, y, z, vx, vy, vz, p, q, r]
            - int n_state, the dimension of the system.
    Output: - double X[], the value of the derivative, dx/dt for all t.
*/
double * simulateUavDynamicsPDController(double t0, double tf, double dt, double *x0, double *xfinal, int n_state )
{
  int n_step = ceil((tf-t0)/dt + 1);
  int pointer_final_state = 0;

  double *X;
  X = ( double * ) malloc ( n_state *n_step * sizeof ( double ) );
  for ( ; pointer_final_state < n_state; pointer_final_state++ )
    X[pointer_final_state] = x0[pointer_final_state];

  double *x1;
  for(int j = 0; j < n_step-1; j++, t0 += dt)
  {
    x1 = rk4 ( t0, n_state, &X[pointer_final_state-n_state], xfinal, dt, uavDynamicsPDController );

    for ( int i = 0; i < n_state; i++, pointer_final_state++ )
      X[pointer_final_state] = x1[i];

    free( x1 );
  }

  return X;
}

#endif
