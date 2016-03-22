# include <math.h>

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

double * calculate_input(double *x, double *x_final)
{
  //  PID gains
  float kxp = 0.25;
  float kxd = 0.95;
  float kyp = 0.25;
  float kyd = 0.95;
  float kzp = 4;
  float kzd = 5;
  float kphip = 8;
  float kphid = 4;
  float kthetap = 8;
  float kthetad = 4;
  float kpsip = 3;
  float kpsid = 1.7;

  //Take euler angles and calulate quaternion
  double cPhi = cos(x[6]/2);
  double sPhi = sin(x[6]/2);
  double cTheta = cos(x[7]/2);
  double sTheta = sin(x[7]/2);
  double cPsi = cos(x[8]/2);
  double sPsi = sin(x[8]/2);
  float qw = cPhi*cTheta*cPsi + sPhi*sTheta*sPsi;
  float qx = sPhi*cTheta*cPsi - cPhi*sTheta*sPsi;
  float qy = cPhi*sTheta*cPsi + sPhi*cTheta*sPsi;
  float qz = cPhi*cTheta*sPsi - sPhi*sTheta*cPsi;

  // Error with target position
  float e_x = x_final[0] - x[0];
  float e_y = x_final[1] - x[1];
  float e_z = x_final[2] - x[2];
  // Error with target velocity
  float e_velx = x_final[3] - x[3];
  float e_vely = x_final[4] - x[4];
  float e_velz = x_final[5] - x[5];
  // Error with target angular velocity
  float e_velphi   = 0.0 - x[9];
  float e_veltheta = 0.0 - x[10];
  float e_velpsi   = 0.0 - x[11];

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

/*
  TODO should add some noise? model the noise?
  TODO all this calculation can be done in parallel they do not depend on each other!
  TODO polinomial approximation of cos and sin?
*/

/******************************************************************************/
/*
  uav_dynamics evaluates the right hand side of a vector ODE describing the uav
  dynamics.

  Parameters:
    Input:  - double t, the current time.
            - int n, the dimension of the system.
            - double x[], the current solution value.
                   0  1  2   3   4   5   6     7     8    9 10 11
                  [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
            - double u[], the current input value.
                    0   1   2   3  4  5    6    7    8    9
                  [f1, f2, f3, f4, l, k, mass, Ixx, Iyy, Izz]
    Output: - double xprime[], the value of the derivative, dx/dt.
*/
double *uav_dynamics( double t, int n_state, double x[], double u[] )
{
  double c = (u[0] + u[1] + u[2] + u[3])/m;
  double halfsqrt2 = sqrt(2)/2;
  double cPhi = cos(x[6]);
  double sPhi = sin(x[6]);
  double cPsi = cos(x[8]);
  double sPsi = sin(x[8]);
  double cTheta = cos(x[7]);
  double sTheta = sin(x[7]);
  double tTheta = tan(x[7]);

  double *xprime;
  xprime = ( double * ) malloc ( n_state * sizeof ( double ) );

  xprime[0]  = x[3];
  xprime[1]  = x[4];
  xprime[2]  = x[5];
  xprime[3]  =         (cPsi*sTheta*cPhi + sPsi*sPhi)*c;
  xprime[4]  =         (sPsi*sTheta*cPhi - cPsi*sPhi)*c;
  xprime[5]  = -g   +                  (cTheta*cPhi)*c; ;
  xprime[6]  = x[9] + tTheta*(sPhi*x[10] + cPhi*x[11]);
  xprime[7]  =                cPhi*x[10] - sPhi*x[11];
  xprime[8]  =               (sPhi*x[10] + cPhi*x[11])/cTheta;
  xprime[9]  = (halfsqrt2*d*b*( u[0] - u[1] - u[2] + u[3]) - x[10]*x[11]*(Izz - Iyy))/Ixx;
  xprime[10] = (halfsqrt2*d*b*(-u[0] - u[1] + u[2] + u[3]) -  x[9]*x[11]*(Ixx - Izz))/Iyy;
  xprime[11] =             (k*( u[0] - u[1] + u[2] - u[3]) -  x[9]*x[10]*(Iyy - Ixx))/Izz;

  return xprime;
}

/******************************************************************************/
/*
  uav_dynamics evaluates the right hand side of a vector ODE describing the uav
  dynamics.

  Parameters:
    Input:  - double t, the current time.
            - int n, the dimension of the system.
            - double x[], the current solution value.
                   0  1  2   3   4   5   6     7     8   9  10 11
                  [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
            - double u[], the current input value / final state.
                    0   1   2   3  4  5    6    7    8    9   10 11 12  13  14  15
                  [f1, f2, f3, f4, l, k, mass, Ixx, Iyy, Izz, x, y, z, vx, vy, vz]
    Output: - double xprime[], the value of the derivative, dx/dt.
*/
double *uav_dynamics_pd_controller( double t, int n_state, double x[], double u[] )
{

  double *f = calculate_input(x, &u[10]);
  u[0] = f[0];
  u[1] = f[1];
  u[2] = f[2];
  u[3] = f[3];

  double c = (u[0] + u[1] + u[2] + u[3])/m;

  double halfsqrt2 = sqrt(2)/2;
  double cPhi = cos(x[6]);
  double sPhi = sin(x[6]);
  double cPsi = cos(x[8]);
  double sPsi = sin(x[8]);
  double cTheta = cos(x[7]);
  double sTheta = sin(x[7]);
  double tTheta = tan(x[7]);

  double *xprime;
  xprime = ( double * ) malloc ( n_state * sizeof ( double ) );
  xprime[0]  = x[3];
  xprime[1]  = x[4];
  xprime[2]  = x[5];
  xprime[3]  =         (cPsi*sTheta*cPhi + sPsi*sPhi)*c;
  xprime[4]  =         (sPsi*sTheta*cPhi - cPsi*sPhi)*c;
  xprime[5]  = -g   +                  (cTheta*cPhi)*c; ;
  xprime[6]  = x[9] + tTheta*(sPhi*x[10] + cPhi*x[11]);
  xprime[7]  =                cPhi*x[10] - sPhi*x[11];
  xprime[8]  =               (sPhi*x[10] + cPhi*x[11])/cTheta;
  xprime[9]  = (halfsqrt2*d*b*( u[0] - u[1] - u[2] + u[3]) - x[10]*x[11]*(Izz - Iyy))/Ixx;
  xprime[10] = (halfsqrt2*d*b*(-u[0] - u[1] + u[2] + u[3]) -  x[9]*x[11]*(Ixx - Izz))/Iyy;
  xprime[11] =             (k*( u[0] - u[1] + u[2] - u[3]) -  x[9]*x[10]*(Iyy - Ixx))/Izz;

  return xprime;
}

/******************************************************************************/
/*
  simulate_uav_dynamics simulate the ODE system describing the uav dynamics for
  a determinate time range and time step.

  Parameters:
    Input:  - double t0 initial time.
            - double tf final time.
            - double dt time step.
            - double u0[4*(tf-dt-t0)/dt] the input vector.
                    0      1      2      3
                [f1(t0), f2t0), f3t0), f4t0), f1(t0+dt), ... , f4(tf-dt)]
    Output: - double xprime[], the value of the derivative, dx/dt.
*/

double * simulate_uav_dynamics(double t0, double tf, double dt, double *x0, double *u0 )
{

  int n_state = 12;
  int n_step = ceil((tf-t0)/dt + 1);
  int pointer_final_state = 0;

  double *X;
  X = ( double * ) malloc ( n_state *n_step * sizeof ( double ) );

  for ( ; pointer_final_state < n_state; pointer_final_state++ )
  {
    X[pointer_final_state] = x0[pointer_final_state];
  }

  double *x1;
  //input: [f1, f2, f3, f4]
  double *u;
  u = ( double * ) malloc ( 4 * sizeof ( double ) );

  for(int j = 0; j < n_step-1; j++)
  {
    u[0] = u0[j*4 + 0];
    u[1] = u0[j*4 + 1];
    u[2] = u0[j*4 + 2];
    u[3] = u0[j*4 + 3];

    x1 = rk4 ( t0, n_state, &X[pointer_final_state-n_state], u, dt, uav_dynamics );

    for ( int i = 0; i < n_state; i++, pointer_final_state++ )
    {
      X[pointer_final_state] = x1[i];
    }

    free( x1 );

    t0 += dt;
  }

  return X;
}

/*
  simulate_uav_dynamics simulate the ODE system describing the uav dynamics for
  a determinate time range and time step.

  Parameters:
    Input:  - double t0 initial time.
            - double tf final time.
            - double dt time step.
            - double xfinal[n] the final state vector.
                     0  1  2   3   4   5   6     7     8    9 10 11
                    [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    Output: - double X[], the value of the derivative, dx/dt for all t.
*/
double * simulate_uav_dynamics_pd_controller(double t0, double tf, double dt, double *x0, double *xfinal )
{
  int n_state = 12;
  int n_step = ceil((tf-t0)/dt + 1);
  int pointer_final_state = 0;

  //input: [f1, f2, f3, f4, -, -, -, -, -, -, x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
  double *u;
  u = ( double * ) malloc ( 22 * sizeof ( double ) );
  u[10] = xfinal[0];
  u[11] = xfinal[1];
  u[12] = xfinal[2];
  u[13] = xfinal[3];
  u[14] = xfinal[4];
  u[15] = xfinal[5];
  u[18] = xfinal[8];
  u[19] = xfinal[9];
  u[20] = xfinal[10];
  u[21] = xfinal[11];

  double *X;
  X = ( double * ) malloc ( n_state *n_step * sizeof ( double ) );
  for ( ; pointer_final_state < n_state; pointer_final_state++ )
    X[pointer_final_state] = x0[pointer_final_state];

  double *x1;
  for(int j = 0; j < n_step-1; j++, t0 += dt)
  {
    x1 = rk4 ( t0, n_state, &X[pointer_final_state-n_state], u, dt, uav_dynamics_pd_controller );

    for ( int i = 0; i < n_state; i++, pointer_final_state++ )
      X[pointer_final_state] = x1[i];

    free( x1 );
  }

  return X;
}
