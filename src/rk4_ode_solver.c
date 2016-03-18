/*!
 * RUNGE KUTTA 4TH ORDER SOLVER FOR A SYSTEM OF FIRST ORDER ODE
 *
 * Copyright 2016 by Alessio Zanchettin
 *
 */

#ifndef RK4_ODE_SOLVER
#define RK4_ODE_SOLVER

# include <stdlib.h>
# include <stdio.h>

/*
  rk4 takes one Runge-Kutta step for a vector ODE.
  It wants as input the initial value problem:
      dx/dt = f ( t, x, u )
      x(t0) = x0
			u(t) given
  It outputs the fourth-order Runge Kutta estimate to the solution at time t+dt.

  Parameters:
    Input: - double t0, the current time.
           - int n_state, the state dimension.
           - double x0[n_state], the solution estimate at the current time.
           - double dt, the time step.
           - double *f ( double t, int n_state, double x[n_state], double u[] )
					    right hand side of the problem.

    Output: - double rk4[n_state], the fourth-order Runge-Kutta solution
    				  estimate at time t0+dt.
*/

double * rk4 ( double t0, int n_state, double x0[], double u[], double dt, double *f ( double t, int n_state, double x[], double u[] ) )
{
  double *f0; double *f1; double *f2; double *f3;
  double t1;  double t2;  double t3;
  double *x;  double *x1; double *x2; double *x3;
	int i;

  // Get four sample values of the derivative.
	// 0
  f0 = f ( t0, n_state, x0, u );

	// 1
  t1 = t0 + dt / 2.0;
  x1 = ( double * ) malloc ( n_state * sizeof ( double ) );
  for ( i = 0; i < n_state; i++ )
    x1[i] = x0[i] + dt * f0[i] / 2.0;
  f1 = f ( t1, n_state, x1, u );

	// 2
  t2 = t0 + dt / 2.0;
  x2 = ( double * ) malloc ( n_state * sizeof ( double ) );
  for ( i = 0; i < n_state; i++ )
    x2[i] = x0[i] + dt * f1[i] / 2.0;
  f2 = f ( t2, n_state, x2, u );

  // 3
  t3 = t0 + dt;
  x3 = ( double * ) malloc ( n_state * sizeof ( double ) );
  for ( i = 0; i < n_state; i++ )
     x3[i] = x0[i] + dt * f2[i];
  f3 = f ( t3, n_state, x3, u );

  //Combine them to estimate the solution.
  x = ( double * ) malloc ( n_state * sizeof ( double ) );
  for ( i = 0; i < n_state; i++ )
     x[i] = x0[i] + dt * ( f0[i] + 2.0 * f1[i] + 2.0 * f2[i] + f3[i] ) / 6.0;

  //Free memory.
  free ( f0 ); free ( f1 ); free ( f2 ); free ( f3 );
	free ( x1 ); free ( x2 ); free ( x3 );
  return x;
}

#endif
