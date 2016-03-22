#include <iostream>
#include <ctime>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace boost::numeric::odeint;

//  Quad constants
const float m = 1.56779;
const float Ixx = 3.47563e-2;
const float Iyy = 3.47563e-2;
const float Izz = 9.77e-2;
const float d = 0.215;
//  Physical constants
const float g = 9.81;
const float k = 1.6e-2;
const float b = 1;

typedef boost::array< double , 13 > state_type;
//final containers
vector<state_type> final_solution;
vector<double> final_times;
//stepper
typedef runge_kutta4< state_type > stepper_type;

struct push_back_state_and_time
{
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
    : m_states( states ) , m_times( times ) { }

    void operator()( const state_type &x , double t )
    {
        m_states.push_back( x );
        m_times.push_back( t );
    }
};


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


void uavDynamics( const state_type &x , state_type &xprime , double t ) //double t, int n_state, double x[], double u[] )
{
  // final conditions: [x, y, z, vx, vy, vz, p, q, r]
  double *xfinal;
  xfinal = ( double * ) malloc ( 9* sizeof ( double ) );
  xfinal[0] = 1.0; xfinal[1] = 0.2; xfinal[2] = 2.0;
  xfinal[3] = 0.0; xfinal[4] = 0.0; xfinal[5] = 0.0;
  xfinal[6] = 0.0; xfinal[7] = 0.0; xfinal[8] = 0.0;

  double *xx;
  xx = ( double * ) malloc ( 13* sizeof ( double ) );
  xx[0] = x[0]; xx[1] = x[1]; xx[2] = x[2];
  xx[3] = x[3]; xx[4] = x[4]; xx[5] = x[5];
  xx[6] = x[6]; xx[7] = x[7]; xx[8] = x[8]; xx[9] = x[9];
  xx[10] = x[10]; xx[11] = x[11]; xx[12] = x[12];

  double *u = calculateInpuControl(xx, xfinal);
  double c = (u[0] + u[1] + u[2] + u[3])/m;
  double halfsqrt2 = sqrt(2)/2;
  double ground_effect = 0;
  double G = ground_effect + g;

  xprime[0]  = x[3];
  xprime[1]  = x[4];
  xprime[2]  = x[5];
  xprime[3]  =  2*(x[6]*x[8] + x[7]*x[9])*c;
  xprime[4]  =  2*(x[8]*x[9] - x[6]*x[7])*c;
  xprime[5]  = -G   + (x[6]*x[6] - x[7]*x[7] - x[8]*x[8] - x[9]*x[9])*c;
  xprime[6]  = 0.5*(            - x[10]*x[7] - x[11]*x[8] - x[12]*x[9]);
  xprime[7]  = 0.5*( x[10]*x[6]              + x[12]*x[8] - x[11]*x[9]);
  xprime[8]  = 0.5*( x[11]*x[6] - x[12]*x[7]              - x[10]*x[9]);
  xprime[9]  = 0.5*( x[12]*x[6] + x[11]*x[7] - x[10]*x[8]             );
  xprime[10] = (halfsqrt2*d*b*( u[0] - u[1] - u[2] + u[3]) - x[11]*x[12]*(Izz - Iyy))/Ixx;
  xprime[11] = (halfsqrt2*d*b*(-u[0] - u[1] + u[2] + u[3]) - x[10]*x[12]*(Ixx - Izz))/Iyy;
  xprime[12] =             (k*( u[0] - u[1] + u[2] - u[3]) - x[10]*x[11]*(Iyy - Ixx))/Izz;

  free(xfinal);
  free(xx);
}

void write_output( const state_type &x , const double t )
{
  //printf ( "  %14.8g  %14.8g  %14.8g %14.8g\n", t, x[0], x[1], x[2] );
  final_times.push_back(t);
  final_solution.push_back(x);
}


int main(int argc, char **argv)
{
    // initial conditions
    double t0 = 0.0;
    double tf = 20.0;
    double dt = 0.01;
    double n_step = ceil((tf - t0)/dt);
    state_type x = {{ 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0, 1.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0, 0.0 }};
    //final containers
    final_solution.reserve(n_step+1);
    final_times.reserve(n_step+1);

    clock_t begin = clock();
    integrate_n_steps(stepper_type(), uavDynamics , x , t0 , dt , n_step, write_output );
    //integrate( uavDynamics , x , t0 , tf , dt, push_back_state_and_time( final_solution , final_times )  );
    clock_t end = clock();

    /*for(int i = 0; i < final_solution.size(); i++){
      printf ( "  %14.8g  %14.8g  %14.8g %14.8g\n", final_times[i], final_solution[i][0], final_solution[i][1], final_solution[i][2] );
    }*/
    //cout << final_solution.size()<< std::endl;
    //std::cout << n_step << std::endl;
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << elapsed_secs << " seconds \n";
}
