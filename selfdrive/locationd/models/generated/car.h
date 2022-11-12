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
void car_err_fun(double *nom_x, double *delta_x, double *out_8744870931434381505);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2789839103479686818);
void car_H_mod_fun(double *state, double *out_1493479091139239047);
void car_f_fun(double *state, double dt, double *out_8702949118916194196);
void car_F_fun(double *state, double dt, double *out_791503401684057011);
void car_h_25(double *state, double *unused, double *out_2856288359181493714);
void car_H_25(double *state, double *unused, double *out_2863785685210894639);
void car_h_24(double *state, double *unused, double *out_4313985665054181548);
void car_H_24(double *state, double *unused, double *out_7610295012177403673);
void car_h_30(double *state, double *unused, double *out_4346927597804492256);
void car_H_30(double *state, double *unused, double *out_345452726703646012);
void car_h_26(double *state, double *unused, double *out_6124616607452235092);
void car_H_26(double *state, double *unused, double *out_6605289004084950863);
void car_h_27(double *state, double *unused, double *out_3459604243857971231);
void car_H_27(double *state, double *unused, double *out_2520216038504070923);
void car_h_29(double *state, double *unused, double *out_7097053772111993581);
void car_H_29(double *state, double *unused, double *out_164778617610746172);
void car_h_28(double *state, double *unused, double *out_5735133061471023365);
void car_H_28(double *state, double *unused, double *out_4917620399458784402);
void car_h_31(double *state, double *unused, double *out_91437631173991247);
void car_H_31(double *state, double *unused, double *out_7231497106318302339);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}