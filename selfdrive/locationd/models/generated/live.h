#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_2024605755553595659);
void live_err_fun(double *nom_x, double *delta_x, double *out_4260489608260856341);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5378964316105151810);
void live_H_mod_fun(double *state, double *out_480180537204152249);
void live_f_fun(double *state, double dt, double *out_4220746504579219447);
void live_F_fun(double *state, double dt, double *out_7364842153695283302);
void live_h_4(double *state, double *unused, double *out_6639604506101609515);
void live_H_4(double *state, double *unused, double *out_8966242213456634991);
void live_h_9(double *state, double *unused, double *out_3287469242688749972);
void live_H_9(double *state, double *unused, double *out_8725052566827044346);
void live_h_10(double *state, double *unused, double *out_2658485406827076195);
void live_H_10(double *state, double *unused, double *out_8159430252957405731);
void live_h_12(double *state, double *unused, double *out_5708523749317566816);
void live_H_12(double *state, double *unused, double *out_3946785805424673196);
void live_h_31(double *state, double *unused, double *out_1563305614564091925);
void live_H_31(double *state, double *unused, double *out_5599580156084027615);
void live_h_32(double *state, double *unused, double *out_4020076223444387283);
void live_H_32(double *state, double *unused, double *out_8506159141892417113);
void live_h_13(double *state, double *unused, double *out_7360360736870817300);
void live_H_13(double *state, double *unused, double *out_5599881678157580703);
void live_h_14(double *state, double *unused, double *out_3287469242688749972);
void live_H_14(double *state, double *unused, double *out_8725052566827044346);
void live_h_33(double *state, double *unused, double *out_162370700888639778);
void live_H_33(double *state, double *unused, double *out_2449023151445170011);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}