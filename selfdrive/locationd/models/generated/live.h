#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_2537912128445620253);
void live_err_fun(double *nom_x, double *delta_x, double *out_1659985015181901835);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8271410697931862202);
void live_H_mod_fun(double *state, double *out_5531808134556081331);
void live_f_fun(double *state, double dt, double *out_313123907282535158);
void live_F_fun(double *state, double dt, double *out_7439547743424380131);
void live_h_4(double *state, double *unused, double *out_5777872899246538591);
void live_H_4(double *state, double *unused, double *out_3167217879617347232);
void live_h_9(double *state, double *unused, double *out_3795054158466794246);
void live_H_9(double *state, double *unused, double *out_3408407526246937877);
void live_h_10(double *state, double *unused, double *out_3387832074626512663);
void live_H_10(double *state, double *unused, double *out_3987256872720503137);
void live_h_12(double *state, double *unused, double *out_7008879033050106886);
void live_H_12(double *state, double *unused, double *out_8186674287649309027);
void live_h_35(double *state, double *unused, double *out_6214778580612386987);
void live_H_35(double *state, double *unused, double *out_7514506753735228880);
void live_h_32(double *state, double *unused, double *out_5249153204612664783);
void live_H_32(double *state, double *unused, double *out_380984566704161748);
void live_h_13(double *state, double *unused, double *out_3644699877240433340);
void live_H_13(double *state, double *unused, double *out_1662231771709590587);
void live_h_14(double *state, double *unused, double *out_3795054158466794246);
void live_H_14(double *state, double *unused, double *out_3408407526246937877);
void live_h_33(double *state, double *unused, double *out_7123440364411686312);
void live_H_33(double *state, double *unused, double *out_4363949749096371276);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}