#ifndef RK4_TEST
#define RK4_TEST

# include <stdlib.h>
# include <stdio.h>
# include <math.h>
#include <ctime>



//#include "rpg_slq_mpc/sequential_linear_quadratic.h"
#include "uav_dynamics.c"

void test_uav_with_given_input()
{
  double dt = 0.01;
  double t0 = 0.0;
  double tf = 10.0;
  double n_step = ceil((tf-t0)/dt + 1);

  int i;

  // initial conditions: [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r]
  int n_state = 13;
  double *x0;
  x0 = ( double * ) malloc ( n_state * sizeof ( double ) );
  for(i = 0; i < n_state; ++i){
    x0[i] = 0;
  }
  x0[6] = 1.0;

  //input: [f1, f2, f3, f4]
  //to hover f = 1.56779*9.81/4;
  double *u;
  u = ( double * ) malloc ( 4 *(n_step-1)* sizeof ( double ) );
  for(i = 0; i < 4*(n_step-1); i+=4){
    u[i]   = 1.56779*9.81/4.0;
    u[i+1] = 1.56779*9.81/4.0;
    u[i+2] = 1.56779*9.81/4.0;
    u[i+3] = 1.56779*9.81/4.0;
  }

  double *X;
  //X = simulateUavDynamics( t0,  tf,  dt, x0, u , n_state);

  printf ("rk4 takes a Runge Kutta step for a vector ODE.\n" );
  printf ( "\n" );
  printf ( "               t               x               y              z\n" );
  printf ( "\n" );
  for ( i = 0; i < n_step; i++, t0 += dt)
    printf ( "  %14.8g  %14.8g  %14.8g %14.8g\n", t0, X[i*n_state+0], X[i*n_state+1], X[i*n_state+2] );

  return;
}

void test_uav_with_given_final_state ( )
{
  double dt = 0.01;
  double t0 = 0.0;
  double tf = 100.0;
  double n_step = ceil((tf-t0)/dt + 1);

  int i;

  // initial conditions: [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r]
  int n_state = 13;
  double *x0;
  x0 = ( double * ) malloc ( n_state * sizeof ( double ) );
  for(i = 0; i < n_state; ++i){
    x0[i] = 0;
  }
  x0[6] = 1.0;

  // final conditions: [x, y, z, vx, vy, vz, p, q, r]
  double *xfinal;
  xfinal = ( double * ) malloc ( 9* sizeof ( double ) );
  xfinal[0] = 1.0; xfinal[1] = 0.2; xfinal[2] = 2.0;
  xfinal[3] = 0.0; xfinal[4] = 0.0; xfinal[5] = 0.0;
  xfinal[6] = 0.0; xfinal[7] = 0.0; xfinal[8] = 0.0;

  double *X;
  clock_t begin = clock();
  X = simulateUavDynamicsPDController( t0,  tf,  dt, x0, xfinal, n_state);
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  printf("%g seconds\n", elapsed_secs);

  /*printf ("rk4 takes a Runge Kutta step for a vector ODE.\n" );
  printf ( "\n" );
  printf ( "               t               x               y              z\n" );
  printf ( "\n" );
  for ( i = 0; i < n_step; i++, t0 += dt)
    printf ( "  %14.8g  %14.8g  %14.8g %14.8g\n", t0, X[i*n_state+0], X[i*n_state+1], X[i*n_state+2] );*/

  return;
}


/******************************************************************************/
int main()
{
  //test_uav_with_given_input();
  test_uav_with_given_final_state();

  return 0;
}

#endif
