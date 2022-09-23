#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6837797094004033531) {
   out_6837797094004033531[0] = delta_x[0] + nom_x[0];
   out_6837797094004033531[1] = delta_x[1] + nom_x[1];
   out_6837797094004033531[2] = delta_x[2] + nom_x[2];
   out_6837797094004033531[3] = delta_x[3] + nom_x[3];
   out_6837797094004033531[4] = delta_x[4] + nom_x[4];
   out_6837797094004033531[5] = delta_x[5] + nom_x[5];
   out_6837797094004033531[6] = delta_x[6] + nom_x[6];
   out_6837797094004033531[7] = delta_x[7] + nom_x[7];
   out_6837797094004033531[8] = delta_x[8] + nom_x[8];
   out_6837797094004033531[9] = delta_x[9] + nom_x[9];
   out_6837797094004033531[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2473313046454877375) {
   out_2473313046454877375[0] = -nom_x[0] + true_x[0];
   out_2473313046454877375[1] = -nom_x[1] + true_x[1];
   out_2473313046454877375[2] = -nom_x[2] + true_x[2];
   out_2473313046454877375[3] = -nom_x[3] + true_x[3];
   out_2473313046454877375[4] = -nom_x[4] + true_x[4];
   out_2473313046454877375[5] = -nom_x[5] + true_x[5];
   out_2473313046454877375[6] = -nom_x[6] + true_x[6];
   out_2473313046454877375[7] = -nom_x[7] + true_x[7];
   out_2473313046454877375[8] = -nom_x[8] + true_x[8];
   out_2473313046454877375[9] = -nom_x[9] + true_x[9];
   out_2473313046454877375[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_8072351116438945665) {
   out_8072351116438945665[0] = 1.0;
   out_8072351116438945665[1] = 0;
   out_8072351116438945665[2] = 0;
   out_8072351116438945665[3] = 0;
   out_8072351116438945665[4] = 0;
   out_8072351116438945665[5] = 0;
   out_8072351116438945665[6] = 0;
   out_8072351116438945665[7] = 0;
   out_8072351116438945665[8] = 0;
   out_8072351116438945665[9] = 0;
   out_8072351116438945665[10] = 0;
   out_8072351116438945665[11] = 0;
   out_8072351116438945665[12] = 1.0;
   out_8072351116438945665[13] = 0;
   out_8072351116438945665[14] = 0;
   out_8072351116438945665[15] = 0;
   out_8072351116438945665[16] = 0;
   out_8072351116438945665[17] = 0;
   out_8072351116438945665[18] = 0;
   out_8072351116438945665[19] = 0;
   out_8072351116438945665[20] = 0;
   out_8072351116438945665[21] = 0;
   out_8072351116438945665[22] = 0;
   out_8072351116438945665[23] = 0;
   out_8072351116438945665[24] = 1.0;
   out_8072351116438945665[25] = 0;
   out_8072351116438945665[26] = 0;
   out_8072351116438945665[27] = 0;
   out_8072351116438945665[28] = 0;
   out_8072351116438945665[29] = 0;
   out_8072351116438945665[30] = 0;
   out_8072351116438945665[31] = 0;
   out_8072351116438945665[32] = 0;
   out_8072351116438945665[33] = 0;
   out_8072351116438945665[34] = 0;
   out_8072351116438945665[35] = 0;
   out_8072351116438945665[36] = 1.0;
   out_8072351116438945665[37] = 0;
   out_8072351116438945665[38] = 0;
   out_8072351116438945665[39] = 0;
   out_8072351116438945665[40] = 0;
   out_8072351116438945665[41] = 0;
   out_8072351116438945665[42] = 0;
   out_8072351116438945665[43] = 0;
   out_8072351116438945665[44] = 0;
   out_8072351116438945665[45] = 0;
   out_8072351116438945665[46] = 0;
   out_8072351116438945665[47] = 0;
   out_8072351116438945665[48] = 1.0;
   out_8072351116438945665[49] = 0;
   out_8072351116438945665[50] = 0;
   out_8072351116438945665[51] = 0;
   out_8072351116438945665[52] = 0;
   out_8072351116438945665[53] = 0;
   out_8072351116438945665[54] = 0;
   out_8072351116438945665[55] = 0;
   out_8072351116438945665[56] = 0;
   out_8072351116438945665[57] = 0;
   out_8072351116438945665[58] = 0;
   out_8072351116438945665[59] = 0;
   out_8072351116438945665[60] = 1.0;
   out_8072351116438945665[61] = 0;
   out_8072351116438945665[62] = 0;
   out_8072351116438945665[63] = 0;
   out_8072351116438945665[64] = 0;
   out_8072351116438945665[65] = 0;
   out_8072351116438945665[66] = 0;
   out_8072351116438945665[67] = 0;
   out_8072351116438945665[68] = 0;
   out_8072351116438945665[69] = 0;
   out_8072351116438945665[70] = 0;
   out_8072351116438945665[71] = 0;
   out_8072351116438945665[72] = 1.0;
   out_8072351116438945665[73] = 0;
   out_8072351116438945665[74] = 0;
   out_8072351116438945665[75] = 0;
   out_8072351116438945665[76] = 0;
   out_8072351116438945665[77] = 0;
   out_8072351116438945665[78] = 0;
   out_8072351116438945665[79] = 0;
   out_8072351116438945665[80] = 0;
   out_8072351116438945665[81] = 0;
   out_8072351116438945665[82] = 0;
   out_8072351116438945665[83] = 0;
   out_8072351116438945665[84] = 1.0;
   out_8072351116438945665[85] = 0;
   out_8072351116438945665[86] = 0;
   out_8072351116438945665[87] = 0;
   out_8072351116438945665[88] = 0;
   out_8072351116438945665[89] = 0;
   out_8072351116438945665[90] = 0;
   out_8072351116438945665[91] = 0;
   out_8072351116438945665[92] = 0;
   out_8072351116438945665[93] = 0;
   out_8072351116438945665[94] = 0;
   out_8072351116438945665[95] = 0;
   out_8072351116438945665[96] = 1.0;
   out_8072351116438945665[97] = 0;
   out_8072351116438945665[98] = 0;
   out_8072351116438945665[99] = 0;
   out_8072351116438945665[100] = 0;
   out_8072351116438945665[101] = 0;
   out_8072351116438945665[102] = 0;
   out_8072351116438945665[103] = 0;
   out_8072351116438945665[104] = 0;
   out_8072351116438945665[105] = 0;
   out_8072351116438945665[106] = 0;
   out_8072351116438945665[107] = 0;
   out_8072351116438945665[108] = 1.0;
   out_8072351116438945665[109] = 0;
   out_8072351116438945665[110] = 0;
   out_8072351116438945665[111] = 0;
   out_8072351116438945665[112] = 0;
   out_8072351116438945665[113] = 0;
   out_8072351116438945665[114] = 0;
   out_8072351116438945665[115] = 0;
   out_8072351116438945665[116] = 0;
   out_8072351116438945665[117] = 0;
   out_8072351116438945665[118] = 0;
   out_8072351116438945665[119] = 0;
   out_8072351116438945665[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_9069411754979803373) {
   out_9069411754979803373[0] = dt*state[3] + state[0];
   out_9069411754979803373[1] = dt*state[4] + state[1];
   out_9069411754979803373[2] = dt*state[5] + state[2];
   out_9069411754979803373[3] = state[3];
   out_9069411754979803373[4] = state[4];
   out_9069411754979803373[5] = state[5];
   out_9069411754979803373[6] = dt*state[7] + state[6];
   out_9069411754979803373[7] = dt*state[8] + state[7];
   out_9069411754979803373[8] = state[8];
   out_9069411754979803373[9] = state[9];
   out_9069411754979803373[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6250175446585140500) {
   out_6250175446585140500[0] = 1;
   out_6250175446585140500[1] = 0;
   out_6250175446585140500[2] = 0;
   out_6250175446585140500[3] = dt;
   out_6250175446585140500[4] = 0;
   out_6250175446585140500[5] = 0;
   out_6250175446585140500[6] = 0;
   out_6250175446585140500[7] = 0;
   out_6250175446585140500[8] = 0;
   out_6250175446585140500[9] = 0;
   out_6250175446585140500[10] = 0;
   out_6250175446585140500[11] = 0;
   out_6250175446585140500[12] = 1;
   out_6250175446585140500[13] = 0;
   out_6250175446585140500[14] = 0;
   out_6250175446585140500[15] = dt;
   out_6250175446585140500[16] = 0;
   out_6250175446585140500[17] = 0;
   out_6250175446585140500[18] = 0;
   out_6250175446585140500[19] = 0;
   out_6250175446585140500[20] = 0;
   out_6250175446585140500[21] = 0;
   out_6250175446585140500[22] = 0;
   out_6250175446585140500[23] = 0;
   out_6250175446585140500[24] = 1;
   out_6250175446585140500[25] = 0;
   out_6250175446585140500[26] = 0;
   out_6250175446585140500[27] = dt;
   out_6250175446585140500[28] = 0;
   out_6250175446585140500[29] = 0;
   out_6250175446585140500[30] = 0;
   out_6250175446585140500[31] = 0;
   out_6250175446585140500[32] = 0;
   out_6250175446585140500[33] = 0;
   out_6250175446585140500[34] = 0;
   out_6250175446585140500[35] = 0;
   out_6250175446585140500[36] = 1;
   out_6250175446585140500[37] = 0;
   out_6250175446585140500[38] = 0;
   out_6250175446585140500[39] = 0;
   out_6250175446585140500[40] = 0;
   out_6250175446585140500[41] = 0;
   out_6250175446585140500[42] = 0;
   out_6250175446585140500[43] = 0;
   out_6250175446585140500[44] = 0;
   out_6250175446585140500[45] = 0;
   out_6250175446585140500[46] = 0;
   out_6250175446585140500[47] = 0;
   out_6250175446585140500[48] = 1;
   out_6250175446585140500[49] = 0;
   out_6250175446585140500[50] = 0;
   out_6250175446585140500[51] = 0;
   out_6250175446585140500[52] = 0;
   out_6250175446585140500[53] = 0;
   out_6250175446585140500[54] = 0;
   out_6250175446585140500[55] = 0;
   out_6250175446585140500[56] = 0;
   out_6250175446585140500[57] = 0;
   out_6250175446585140500[58] = 0;
   out_6250175446585140500[59] = 0;
   out_6250175446585140500[60] = 1;
   out_6250175446585140500[61] = 0;
   out_6250175446585140500[62] = 0;
   out_6250175446585140500[63] = 0;
   out_6250175446585140500[64] = 0;
   out_6250175446585140500[65] = 0;
   out_6250175446585140500[66] = 0;
   out_6250175446585140500[67] = 0;
   out_6250175446585140500[68] = 0;
   out_6250175446585140500[69] = 0;
   out_6250175446585140500[70] = 0;
   out_6250175446585140500[71] = 0;
   out_6250175446585140500[72] = 1;
   out_6250175446585140500[73] = dt;
   out_6250175446585140500[74] = 0;
   out_6250175446585140500[75] = 0;
   out_6250175446585140500[76] = 0;
   out_6250175446585140500[77] = 0;
   out_6250175446585140500[78] = 0;
   out_6250175446585140500[79] = 0;
   out_6250175446585140500[80] = 0;
   out_6250175446585140500[81] = 0;
   out_6250175446585140500[82] = 0;
   out_6250175446585140500[83] = 0;
   out_6250175446585140500[84] = 1;
   out_6250175446585140500[85] = dt;
   out_6250175446585140500[86] = 0;
   out_6250175446585140500[87] = 0;
   out_6250175446585140500[88] = 0;
   out_6250175446585140500[89] = 0;
   out_6250175446585140500[90] = 0;
   out_6250175446585140500[91] = 0;
   out_6250175446585140500[92] = 0;
   out_6250175446585140500[93] = 0;
   out_6250175446585140500[94] = 0;
   out_6250175446585140500[95] = 0;
   out_6250175446585140500[96] = 1;
   out_6250175446585140500[97] = 0;
   out_6250175446585140500[98] = 0;
   out_6250175446585140500[99] = 0;
   out_6250175446585140500[100] = 0;
   out_6250175446585140500[101] = 0;
   out_6250175446585140500[102] = 0;
   out_6250175446585140500[103] = 0;
   out_6250175446585140500[104] = 0;
   out_6250175446585140500[105] = 0;
   out_6250175446585140500[106] = 0;
   out_6250175446585140500[107] = 0;
   out_6250175446585140500[108] = 1;
   out_6250175446585140500[109] = 0;
   out_6250175446585140500[110] = 0;
   out_6250175446585140500[111] = 0;
   out_6250175446585140500[112] = 0;
   out_6250175446585140500[113] = 0;
   out_6250175446585140500[114] = 0;
   out_6250175446585140500[115] = 0;
   out_6250175446585140500[116] = 0;
   out_6250175446585140500[117] = 0;
   out_6250175446585140500[118] = 0;
   out_6250175446585140500[119] = 0;
   out_6250175446585140500[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_5785123329468198314) {
   out_5785123329468198314[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_6027419960807912310) {
   out_6027419960807912310[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6027419960807912310[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6027419960807912310[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6027419960807912310[3] = 0;
   out_6027419960807912310[4] = 0;
   out_6027419960807912310[5] = 0;
   out_6027419960807912310[6] = 1;
   out_6027419960807912310[7] = 0;
   out_6027419960807912310[8] = 0;
   out_6027419960807912310[9] = 0;
   out_6027419960807912310[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_2312599276522945867) {
   out_2312599276522945867[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_3434415847638054501) {
   out_3434415847638054501[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3434415847638054501[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3434415847638054501[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3434415847638054501[3] = 0;
   out_3434415847638054501[4] = 0;
   out_3434415847638054501[5] = 0;
   out_3434415847638054501[6] = 1;
   out_3434415847638054501[7] = 0;
   out_3434415847638054501[8] = 0;
   out_3434415847638054501[9] = 1;
   out_3434415847638054501[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_71670101605741836) {
   out_71670101605741836[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_2577971656064248687) {
   out_2577971656064248687[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[6] = 0;
   out_2577971656064248687[7] = 1;
   out_2577971656064248687[8] = 0;
   out_2577971656064248687[9] = 0;
   out_2577971656064248687[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_71670101605741836) {
   out_71670101605741836[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_2577971656064248687) {
   out_2577971656064248687[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2577971656064248687[6] = 0;
   out_2577971656064248687[7] = 1;
   out_2577971656064248687[8] = 0;
   out_2577971656064248687[9] = 0;
   out_2577971656064248687[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6837797094004033531) {
  err_fun(nom_x, delta_x, out_6837797094004033531);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2473313046454877375) {
  inv_err_fun(nom_x, true_x, out_2473313046454877375);
}
void gnss_H_mod_fun(double *state, double *out_8072351116438945665) {
  H_mod_fun(state, out_8072351116438945665);
}
void gnss_f_fun(double *state, double dt, double *out_9069411754979803373) {
  f_fun(state,  dt, out_9069411754979803373);
}
void gnss_F_fun(double *state, double dt, double *out_6250175446585140500) {
  F_fun(state,  dt, out_6250175446585140500);
}
void gnss_h_6(double *state, double *sat_pos, double *out_5785123329468198314) {
  h_6(state, sat_pos, out_5785123329468198314);
}
void gnss_H_6(double *state, double *sat_pos, double *out_6027419960807912310) {
  H_6(state, sat_pos, out_6027419960807912310);
}
void gnss_h_20(double *state, double *sat_pos, double *out_2312599276522945867) {
  h_20(state, sat_pos, out_2312599276522945867);
}
void gnss_H_20(double *state, double *sat_pos, double *out_3434415847638054501) {
  H_20(state, sat_pos, out_3434415847638054501);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_71670101605741836) {
  h_7(state, sat_pos_vel, out_71670101605741836);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2577971656064248687) {
  H_7(state, sat_pos_vel, out_2577971656064248687);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_71670101605741836) {
  h_21(state, sat_pos_vel, out_71670101605741836);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2577971656064248687) {
  H_21(state, sat_pos_vel, out_2577971656064248687);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
