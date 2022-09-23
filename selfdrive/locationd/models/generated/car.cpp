#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3213497222826560325) {
   out_3213497222826560325[0] = delta_x[0] + nom_x[0];
   out_3213497222826560325[1] = delta_x[1] + nom_x[1];
   out_3213497222826560325[2] = delta_x[2] + nom_x[2];
   out_3213497222826560325[3] = delta_x[3] + nom_x[3];
   out_3213497222826560325[4] = delta_x[4] + nom_x[4];
   out_3213497222826560325[5] = delta_x[5] + nom_x[5];
   out_3213497222826560325[6] = delta_x[6] + nom_x[6];
   out_3213497222826560325[7] = delta_x[7] + nom_x[7];
   out_3213497222826560325[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5342840856144633296) {
   out_5342840856144633296[0] = -nom_x[0] + true_x[0];
   out_5342840856144633296[1] = -nom_x[1] + true_x[1];
   out_5342840856144633296[2] = -nom_x[2] + true_x[2];
   out_5342840856144633296[3] = -nom_x[3] + true_x[3];
   out_5342840856144633296[4] = -nom_x[4] + true_x[4];
   out_5342840856144633296[5] = -nom_x[5] + true_x[5];
   out_5342840856144633296[6] = -nom_x[6] + true_x[6];
   out_5342840856144633296[7] = -nom_x[7] + true_x[7];
   out_5342840856144633296[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5608686398213682926) {
   out_5608686398213682926[0] = 1.0;
   out_5608686398213682926[1] = 0;
   out_5608686398213682926[2] = 0;
   out_5608686398213682926[3] = 0;
   out_5608686398213682926[4] = 0;
   out_5608686398213682926[5] = 0;
   out_5608686398213682926[6] = 0;
   out_5608686398213682926[7] = 0;
   out_5608686398213682926[8] = 0;
   out_5608686398213682926[9] = 0;
   out_5608686398213682926[10] = 1.0;
   out_5608686398213682926[11] = 0;
   out_5608686398213682926[12] = 0;
   out_5608686398213682926[13] = 0;
   out_5608686398213682926[14] = 0;
   out_5608686398213682926[15] = 0;
   out_5608686398213682926[16] = 0;
   out_5608686398213682926[17] = 0;
   out_5608686398213682926[18] = 0;
   out_5608686398213682926[19] = 0;
   out_5608686398213682926[20] = 1.0;
   out_5608686398213682926[21] = 0;
   out_5608686398213682926[22] = 0;
   out_5608686398213682926[23] = 0;
   out_5608686398213682926[24] = 0;
   out_5608686398213682926[25] = 0;
   out_5608686398213682926[26] = 0;
   out_5608686398213682926[27] = 0;
   out_5608686398213682926[28] = 0;
   out_5608686398213682926[29] = 0;
   out_5608686398213682926[30] = 1.0;
   out_5608686398213682926[31] = 0;
   out_5608686398213682926[32] = 0;
   out_5608686398213682926[33] = 0;
   out_5608686398213682926[34] = 0;
   out_5608686398213682926[35] = 0;
   out_5608686398213682926[36] = 0;
   out_5608686398213682926[37] = 0;
   out_5608686398213682926[38] = 0;
   out_5608686398213682926[39] = 0;
   out_5608686398213682926[40] = 1.0;
   out_5608686398213682926[41] = 0;
   out_5608686398213682926[42] = 0;
   out_5608686398213682926[43] = 0;
   out_5608686398213682926[44] = 0;
   out_5608686398213682926[45] = 0;
   out_5608686398213682926[46] = 0;
   out_5608686398213682926[47] = 0;
   out_5608686398213682926[48] = 0;
   out_5608686398213682926[49] = 0;
   out_5608686398213682926[50] = 1.0;
   out_5608686398213682926[51] = 0;
   out_5608686398213682926[52] = 0;
   out_5608686398213682926[53] = 0;
   out_5608686398213682926[54] = 0;
   out_5608686398213682926[55] = 0;
   out_5608686398213682926[56] = 0;
   out_5608686398213682926[57] = 0;
   out_5608686398213682926[58] = 0;
   out_5608686398213682926[59] = 0;
   out_5608686398213682926[60] = 1.0;
   out_5608686398213682926[61] = 0;
   out_5608686398213682926[62] = 0;
   out_5608686398213682926[63] = 0;
   out_5608686398213682926[64] = 0;
   out_5608686398213682926[65] = 0;
   out_5608686398213682926[66] = 0;
   out_5608686398213682926[67] = 0;
   out_5608686398213682926[68] = 0;
   out_5608686398213682926[69] = 0;
   out_5608686398213682926[70] = 1.0;
   out_5608686398213682926[71] = 0;
   out_5608686398213682926[72] = 0;
   out_5608686398213682926[73] = 0;
   out_5608686398213682926[74] = 0;
   out_5608686398213682926[75] = 0;
   out_5608686398213682926[76] = 0;
   out_5608686398213682926[77] = 0;
   out_5608686398213682926[78] = 0;
   out_5608686398213682926[79] = 0;
   out_5608686398213682926[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3659170848706843666) {
   out_3659170848706843666[0] = state[0];
   out_3659170848706843666[1] = state[1];
   out_3659170848706843666[2] = state[2];
   out_3659170848706843666[3] = state[3];
   out_3659170848706843666[4] = state[4];
   out_3659170848706843666[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3659170848706843666[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3659170848706843666[7] = state[7];
   out_3659170848706843666[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3854287443842015086) {
   out_3854287443842015086[0] = 1;
   out_3854287443842015086[1] = 0;
   out_3854287443842015086[2] = 0;
   out_3854287443842015086[3] = 0;
   out_3854287443842015086[4] = 0;
   out_3854287443842015086[5] = 0;
   out_3854287443842015086[6] = 0;
   out_3854287443842015086[7] = 0;
   out_3854287443842015086[8] = 0;
   out_3854287443842015086[9] = 0;
   out_3854287443842015086[10] = 1;
   out_3854287443842015086[11] = 0;
   out_3854287443842015086[12] = 0;
   out_3854287443842015086[13] = 0;
   out_3854287443842015086[14] = 0;
   out_3854287443842015086[15] = 0;
   out_3854287443842015086[16] = 0;
   out_3854287443842015086[17] = 0;
   out_3854287443842015086[18] = 0;
   out_3854287443842015086[19] = 0;
   out_3854287443842015086[20] = 1;
   out_3854287443842015086[21] = 0;
   out_3854287443842015086[22] = 0;
   out_3854287443842015086[23] = 0;
   out_3854287443842015086[24] = 0;
   out_3854287443842015086[25] = 0;
   out_3854287443842015086[26] = 0;
   out_3854287443842015086[27] = 0;
   out_3854287443842015086[28] = 0;
   out_3854287443842015086[29] = 0;
   out_3854287443842015086[30] = 1;
   out_3854287443842015086[31] = 0;
   out_3854287443842015086[32] = 0;
   out_3854287443842015086[33] = 0;
   out_3854287443842015086[34] = 0;
   out_3854287443842015086[35] = 0;
   out_3854287443842015086[36] = 0;
   out_3854287443842015086[37] = 0;
   out_3854287443842015086[38] = 0;
   out_3854287443842015086[39] = 0;
   out_3854287443842015086[40] = 1;
   out_3854287443842015086[41] = 0;
   out_3854287443842015086[42] = 0;
   out_3854287443842015086[43] = 0;
   out_3854287443842015086[44] = 0;
   out_3854287443842015086[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3854287443842015086[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3854287443842015086[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3854287443842015086[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3854287443842015086[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3854287443842015086[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3854287443842015086[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3854287443842015086[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3854287443842015086[53] = -9.8000000000000007*dt;
   out_3854287443842015086[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3854287443842015086[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3854287443842015086[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3854287443842015086[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3854287443842015086[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3854287443842015086[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3854287443842015086[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3854287443842015086[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3854287443842015086[62] = 0;
   out_3854287443842015086[63] = 0;
   out_3854287443842015086[64] = 0;
   out_3854287443842015086[65] = 0;
   out_3854287443842015086[66] = 0;
   out_3854287443842015086[67] = 0;
   out_3854287443842015086[68] = 0;
   out_3854287443842015086[69] = 0;
   out_3854287443842015086[70] = 1;
   out_3854287443842015086[71] = 0;
   out_3854287443842015086[72] = 0;
   out_3854287443842015086[73] = 0;
   out_3854287443842015086[74] = 0;
   out_3854287443842015086[75] = 0;
   out_3854287443842015086[76] = 0;
   out_3854287443842015086[77] = 0;
   out_3854287443842015086[78] = 0;
   out_3854287443842015086[79] = 0;
   out_3854287443842015086[80] = 1;
}
void h_25(double *state, double *unused, double *out_4182785460120350154) {
   out_4182785460120350154[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4672028984688361642) {
   out_4672028984688361642[0] = 0;
   out_4672028984688361642[1] = 0;
   out_4672028984688361642[2] = 0;
   out_4672028984688361642[3] = 0;
   out_4672028984688361642[4] = 0;
   out_4672028984688361642[5] = 0;
   out_4672028984688361642[6] = 1;
   out_4672028984688361642[7] = 0;
   out_4672028984688361642[8] = 0;
}
void h_24(double *state, double *unused, double *out_575991834050178471) {
   out_575991834050178471[0] = state[4];
   out_575991834050178471[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8229954583490684778) {
   out_8229954583490684778[0] = 0;
   out_8229954583490684778[1] = 0;
   out_8229954583490684778[2] = 0;
   out_8229954583490684778[3] = 0;
   out_8229954583490684778[4] = 1;
   out_8229954583490684778[5] = 0;
   out_8229954583490684778[6] = 0;
   out_8229954583490684778[7] = 0;
   out_8229954583490684778[8] = 0;
   out_8229954583490684778[9] = 0;
   out_8229954583490684778[10] = 0;
   out_8229954583490684778[11] = 0;
   out_8229954583490684778[12] = 0;
   out_8229954583490684778[13] = 0;
   out_8229954583490684778[14] = 1;
   out_8229954583490684778[15] = 0;
   out_8229954583490684778[16] = 0;
   out_8229954583490684778[17] = 0;
}
void h_30(double *state, double *unused, double *out_8096844573618067159) {
   out_8096844573618067159[0] = state[4];
}
void H_30(double *state, double *unused, double *out_144332654560753444) {
   out_144332654560753444[0] = 0;
   out_144332654560753444[1] = 0;
   out_144332654560753444[2] = 0;
   out_144332654560753444[3] = 0;
   out_144332654560753444[4] = 1;
   out_144332654560753444[5] = 0;
   out_144332654560753444[6] = 0;
   out_144332654560753444[7] = 0;
   out_144332654560753444[8] = 0;
}
void h_26(double *state, double *unused, double *out_3095171305957221514) {
   out_3095171305957221514[0] = state[7];
}
void H_26(double *state, double *unused, double *out_930525665814305418) {
   out_930525665814305418[0] = 0;
   out_930525665814305418[1] = 0;
   out_930525665814305418[2] = 0;
   out_930525665814305418[3] = 0;
   out_930525665814305418[4] = 0;
   out_930525665814305418[5] = 0;
   out_930525665814305418[6] = 0;
   out_930525665814305418[7] = 1;
   out_930525665814305418[8] = 0;
}
void h_27(double *state, double *unused, double *out_818887807540495491) {
   out_818887807540495491[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2030430657239671467) {
   out_2030430657239671467[0] = 0;
   out_2030430657239671467[1] = 0;
   out_2030430657239671467[2] = 0;
   out_2030430657239671467[3] = 1;
   out_2030430657239671467[4] = 0;
   out_2030430657239671467[5] = 0;
   out_2030430657239671467[6] = 0;
   out_2030430657239671467[7] = 0;
   out_2030430657239671467[8] = 0;
}
void h_29(double *state, double *unused, double *out_1094081869825001380) {
   out_1094081869825001380[0] = state[1];
}
void H_29(double *state, double *unused, double *out_654563998875145628) {
   out_654563998875145628[0] = 0;
   out_654563998875145628[1] = 1;
   out_654563998875145628[2] = 0;
   out_654563998875145628[3] = 0;
   out_654563998875145628[4] = 0;
   out_654563998875145628[5] = 0;
   out_654563998875145628[6] = 0;
   out_654563998875145628[7] = 0;
   out_654563998875145628[8] = 0;
}
void h_28(double *state, double *unused, double *out_5820561075201341136) {
   out_5820561075201341136[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2618194270440471879) {
   out_2618194270440471879[0] = 1;
   out_2618194270440471879[1] = 0;
   out_2618194270440471879[2] = 0;
   out_2618194270440471879[3] = 0;
   out_2618194270440471879[4] = 0;
   out_2618194270440471879[5] = 0;
   out_2618194270440471879[6] = 0;
   out_2618194270440471879[7] = 0;
   out_2618194270440471879[8] = 0;
}
void h_31(double *state, double *unused, double *out_7820234988374372504) {
   out_7820234988374372504[0] = state[8];
}
void H_31(double *state, double *unused, double *out_304317563580953942) {
   out_304317563580953942[0] = 0;
   out_304317563580953942[1] = 0;
   out_304317563580953942[2] = 0;
   out_304317563580953942[3] = 0;
   out_304317563580953942[4] = 0;
   out_304317563580953942[5] = 0;
   out_304317563580953942[6] = 0;
   out_304317563580953942[7] = 0;
   out_304317563580953942[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3213497222826560325) {
  err_fun(nom_x, delta_x, out_3213497222826560325);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5342840856144633296) {
  inv_err_fun(nom_x, true_x, out_5342840856144633296);
}
void car_H_mod_fun(double *state, double *out_5608686398213682926) {
  H_mod_fun(state, out_5608686398213682926);
}
void car_f_fun(double *state, double dt, double *out_3659170848706843666) {
  f_fun(state,  dt, out_3659170848706843666);
}
void car_F_fun(double *state, double dt, double *out_3854287443842015086) {
  F_fun(state,  dt, out_3854287443842015086);
}
void car_h_25(double *state, double *unused, double *out_4182785460120350154) {
  h_25(state, unused, out_4182785460120350154);
}
void car_H_25(double *state, double *unused, double *out_4672028984688361642) {
  H_25(state, unused, out_4672028984688361642);
}
void car_h_24(double *state, double *unused, double *out_575991834050178471) {
  h_24(state, unused, out_575991834050178471);
}
void car_H_24(double *state, double *unused, double *out_8229954583490684778) {
  H_24(state, unused, out_8229954583490684778);
}
void car_h_30(double *state, double *unused, double *out_8096844573618067159) {
  h_30(state, unused, out_8096844573618067159);
}
void car_H_30(double *state, double *unused, double *out_144332654560753444) {
  H_30(state, unused, out_144332654560753444);
}
void car_h_26(double *state, double *unused, double *out_3095171305957221514) {
  h_26(state, unused, out_3095171305957221514);
}
void car_H_26(double *state, double *unused, double *out_930525665814305418) {
  H_26(state, unused, out_930525665814305418);
}
void car_h_27(double *state, double *unused, double *out_818887807540495491) {
  h_27(state, unused, out_818887807540495491);
}
void car_H_27(double *state, double *unused, double *out_2030430657239671467) {
  H_27(state, unused, out_2030430657239671467);
}
void car_h_29(double *state, double *unused, double *out_1094081869825001380) {
  h_29(state, unused, out_1094081869825001380);
}
void car_H_29(double *state, double *unused, double *out_654563998875145628) {
  H_29(state, unused, out_654563998875145628);
}
void car_h_28(double *state, double *unused, double *out_5820561075201341136) {
  h_28(state, unused, out_5820561075201341136);
}
void car_H_28(double *state, double *unused, double *out_2618194270440471879) {
  H_28(state, unused, out_2618194270440471879);
}
void car_h_31(double *state, double *unused, double *out_7820234988374372504) {
  h_31(state, unused, out_7820234988374372504);
}
void car_H_31(double *state, double *unused, double *out_304317563580953942) {
  H_31(state, unused, out_304317563580953942);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
