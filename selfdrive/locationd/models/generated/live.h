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
void live_H(double *in_vec, double *out_1072191033157158601);
void live_err_fun(double *nom_x, double *delta_x, double *out_9153287392632638135);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8767774856622703482);
void live_H_mod_fun(double *state, double *out_1721134699022982220);
void live_f_fun(double *state, double dt, double *out_2791114234699295272);
void live_F_fun(double *state, double dt, double *out_3238609712970279026);
void live_h_4(double *state, double *unused, double *out_448328737891450255);
void live_H_4(double *state, double *unused, double *out_2486283194390190191);
void live_h_9(double *state, double *unused, double *out_1526521770634024069);
void live_H_9(double *state, double *unused, double *out_4800935740874257279);
void live_h_10(double *state, double *unused, double *out_1240963690229572142);
void live_H_10(double *state, double *unused, double *out_8362305503185082671);
void live_h_12(double *state, double *unused, double *out_5526052934432677459);
void live_H_12(double *state, double *unused, double *out_8867541571432923187);
void live_h_31(double *state, double *unused, double *out_8226496415171608299);
void live_H_31(double *state, double *unused, double *out_880378862982417185);
void live_h_32(double *state, double *unused, double *out_2713534762175175397);
void live_H_32(double *state, double *unused, double *out_4966114551079340285);
void live_h_13(double *state, double *unused, double *out_6011079237240868127);
void live_H_13(double *state, double *unused, double *out_8607241580012214345);
void live_h_14(double *state, double *unused, double *out_1526521770634024069);
void live_H_14(double *state, double *unused, double *out_4800935740874257279);
void live_h_33(double *state, double *unused, double *out_140589368303733818);
void live_H_33(double *state, double *unused, double *out_7369778917453420002);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}