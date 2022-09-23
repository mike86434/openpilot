#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3213497222826560325);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5342840856144633296);
void car_H_mod_fun(double *state, double *out_5608686398213682926);
void car_f_fun(double *state, double dt, double *out_3659170848706843666);
void car_F_fun(double *state, double dt, double *out_3854287443842015086);
void car_h_25(double *state, double *unused, double *out_4182785460120350154);
void car_H_25(double *state, double *unused, double *out_4672028984688361642);
void car_h_24(double *state, double *unused, double *out_575991834050178471);
void car_H_24(double *state, double *unused, double *out_8229954583490684778);
void car_h_30(double *state, double *unused, double *out_8096844573618067159);
void car_H_30(double *state, double *unused, double *out_144332654560753444);
void car_h_26(double *state, double *unused, double *out_3095171305957221514);
void car_H_26(double *state, double *unused, double *out_930525665814305418);
void car_h_27(double *state, double *unused, double *out_818887807540495491);
void car_H_27(double *state, double *unused, double *out_2030430657239671467);
void car_h_29(double *state, double *unused, double *out_1094081869825001380);
void car_H_29(double *state, double *unused, double *out_654563998875145628);
void car_h_28(double *state, double *unused, double *out_5820561075201341136);
void car_H_28(double *state, double *unused, double *out_2618194270440471879);
void car_h_31(double *state, double *unused, double *out_7820234988374372504);
void car_H_31(double *state, double *unused, double *out_304317563580953942);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}