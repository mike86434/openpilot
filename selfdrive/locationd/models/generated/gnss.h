#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6837797094004033531);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2473313046454877375);
void gnss_H_mod_fun(double *state, double *out_8072351116438945665);
void gnss_f_fun(double *state, double dt, double *out_9069411754979803373);
void gnss_F_fun(double *state, double dt, double *out_6250175446585140500);
void gnss_h_6(double *state, double *sat_pos, double *out_5785123329468198314);
void gnss_H_6(double *state, double *sat_pos, double *out_6027419960807912310);
void gnss_h_20(double *state, double *sat_pos, double *out_2312599276522945867);
void gnss_H_20(double *state, double *sat_pos, double *out_3434415847638054501);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_71670101605741836);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2577971656064248687);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_71670101605741836);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2577971656064248687);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}