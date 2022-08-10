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
void car_err_fun(double *nom_x, double *delta_x, double *out_5087377540895903948);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1760528671002568790);
void car_H_mod_fun(double *state, double *out_36576598272594351);
void car_f_fun(double *state, double dt, double *out_4730314693298424207);
void car_F_fun(double *state, double dt, double *out_3551468011127802550);
void car_h_25(double *state, double *unused, double *out_299851405951739482);
void car_H_25(double *state, double *unused, double *out_2659690145638598458);
void car_h_24(double *state, double *unused, double *out_5923104642486504031);
void car_H_24(double *state, double *unused, double *out_9211482121479435057);
void car_h_30(double *state, double *unused, double *out_7463413850868932959);
void car_H_30(double *state, double *unused, double *out_8870363586579336403);
void car_h_26(double *state, double *unused, double *out_4426627799424600274);
void car_H_26(double *state, double *unused, double *out_1081813173235457766);
void car_h_27(double *state, double *unused, double *out_2736934645492593203);
void car_H_27(double *state, double *unused, double *out_7401617175329790302);
void car_h_29(double *state, double *unused, double *out_2461740583208087314);
void car_H_29(double *state, double *unused, double *out_8360132242264944219);
void car_h_28(double *state, double *unused, double *out_5117106018126811552);
void car_H_28(double *state, double *unused, double *out_5004212814375076823);
void car_h_31(double *state, double *unused, double *out_7070686632302090418);
void car_H_31(double *state, double *unused, double *out_2690336107515558886);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}