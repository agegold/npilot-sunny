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
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5087377540895903948) {
   out_5087377540895903948[0] = delta_x[0] + nom_x[0];
   out_5087377540895903948[1] = delta_x[1] + nom_x[1];
   out_5087377540895903948[2] = delta_x[2] + nom_x[2];
   out_5087377540895903948[3] = delta_x[3] + nom_x[3];
   out_5087377540895903948[4] = delta_x[4] + nom_x[4];
   out_5087377540895903948[5] = delta_x[5] + nom_x[5];
   out_5087377540895903948[6] = delta_x[6] + nom_x[6];
   out_5087377540895903948[7] = delta_x[7] + nom_x[7];
   out_5087377540895903948[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1760528671002568790) {
   out_1760528671002568790[0] = -nom_x[0] + true_x[0];
   out_1760528671002568790[1] = -nom_x[1] + true_x[1];
   out_1760528671002568790[2] = -nom_x[2] + true_x[2];
   out_1760528671002568790[3] = -nom_x[3] + true_x[3];
   out_1760528671002568790[4] = -nom_x[4] + true_x[4];
   out_1760528671002568790[5] = -nom_x[5] + true_x[5];
   out_1760528671002568790[6] = -nom_x[6] + true_x[6];
   out_1760528671002568790[7] = -nom_x[7] + true_x[7];
   out_1760528671002568790[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_36576598272594351) {
   out_36576598272594351[0] = 1.0;
   out_36576598272594351[1] = 0;
   out_36576598272594351[2] = 0;
   out_36576598272594351[3] = 0;
   out_36576598272594351[4] = 0;
   out_36576598272594351[5] = 0;
   out_36576598272594351[6] = 0;
   out_36576598272594351[7] = 0;
   out_36576598272594351[8] = 0;
   out_36576598272594351[9] = 0;
   out_36576598272594351[10] = 1.0;
   out_36576598272594351[11] = 0;
   out_36576598272594351[12] = 0;
   out_36576598272594351[13] = 0;
   out_36576598272594351[14] = 0;
   out_36576598272594351[15] = 0;
   out_36576598272594351[16] = 0;
   out_36576598272594351[17] = 0;
   out_36576598272594351[18] = 0;
   out_36576598272594351[19] = 0;
   out_36576598272594351[20] = 1.0;
   out_36576598272594351[21] = 0;
   out_36576598272594351[22] = 0;
   out_36576598272594351[23] = 0;
   out_36576598272594351[24] = 0;
   out_36576598272594351[25] = 0;
   out_36576598272594351[26] = 0;
   out_36576598272594351[27] = 0;
   out_36576598272594351[28] = 0;
   out_36576598272594351[29] = 0;
   out_36576598272594351[30] = 1.0;
   out_36576598272594351[31] = 0;
   out_36576598272594351[32] = 0;
   out_36576598272594351[33] = 0;
   out_36576598272594351[34] = 0;
   out_36576598272594351[35] = 0;
   out_36576598272594351[36] = 0;
   out_36576598272594351[37] = 0;
   out_36576598272594351[38] = 0;
   out_36576598272594351[39] = 0;
   out_36576598272594351[40] = 1.0;
   out_36576598272594351[41] = 0;
   out_36576598272594351[42] = 0;
   out_36576598272594351[43] = 0;
   out_36576598272594351[44] = 0;
   out_36576598272594351[45] = 0;
   out_36576598272594351[46] = 0;
   out_36576598272594351[47] = 0;
   out_36576598272594351[48] = 0;
   out_36576598272594351[49] = 0;
   out_36576598272594351[50] = 1.0;
   out_36576598272594351[51] = 0;
   out_36576598272594351[52] = 0;
   out_36576598272594351[53] = 0;
   out_36576598272594351[54] = 0;
   out_36576598272594351[55] = 0;
   out_36576598272594351[56] = 0;
   out_36576598272594351[57] = 0;
   out_36576598272594351[58] = 0;
   out_36576598272594351[59] = 0;
   out_36576598272594351[60] = 1.0;
   out_36576598272594351[61] = 0;
   out_36576598272594351[62] = 0;
   out_36576598272594351[63] = 0;
   out_36576598272594351[64] = 0;
   out_36576598272594351[65] = 0;
   out_36576598272594351[66] = 0;
   out_36576598272594351[67] = 0;
   out_36576598272594351[68] = 0;
   out_36576598272594351[69] = 0;
   out_36576598272594351[70] = 1.0;
   out_36576598272594351[71] = 0;
   out_36576598272594351[72] = 0;
   out_36576598272594351[73] = 0;
   out_36576598272594351[74] = 0;
   out_36576598272594351[75] = 0;
   out_36576598272594351[76] = 0;
   out_36576598272594351[77] = 0;
   out_36576598272594351[78] = 0;
   out_36576598272594351[79] = 0;
   out_36576598272594351[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4730314693298424207) {
   out_4730314693298424207[0] = state[0];
   out_4730314693298424207[1] = state[1];
   out_4730314693298424207[2] = state[2];
   out_4730314693298424207[3] = state[3];
   out_4730314693298424207[4] = state[4];
   out_4730314693298424207[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4730314693298424207[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4730314693298424207[7] = state[7];
   out_4730314693298424207[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3551468011127802550) {
   out_3551468011127802550[0] = 1;
   out_3551468011127802550[1] = 0;
   out_3551468011127802550[2] = 0;
   out_3551468011127802550[3] = 0;
   out_3551468011127802550[4] = 0;
   out_3551468011127802550[5] = 0;
   out_3551468011127802550[6] = 0;
   out_3551468011127802550[7] = 0;
   out_3551468011127802550[8] = 0;
   out_3551468011127802550[9] = 0;
   out_3551468011127802550[10] = 1;
   out_3551468011127802550[11] = 0;
   out_3551468011127802550[12] = 0;
   out_3551468011127802550[13] = 0;
   out_3551468011127802550[14] = 0;
   out_3551468011127802550[15] = 0;
   out_3551468011127802550[16] = 0;
   out_3551468011127802550[17] = 0;
   out_3551468011127802550[18] = 0;
   out_3551468011127802550[19] = 0;
   out_3551468011127802550[20] = 1;
   out_3551468011127802550[21] = 0;
   out_3551468011127802550[22] = 0;
   out_3551468011127802550[23] = 0;
   out_3551468011127802550[24] = 0;
   out_3551468011127802550[25] = 0;
   out_3551468011127802550[26] = 0;
   out_3551468011127802550[27] = 0;
   out_3551468011127802550[28] = 0;
   out_3551468011127802550[29] = 0;
   out_3551468011127802550[30] = 1;
   out_3551468011127802550[31] = 0;
   out_3551468011127802550[32] = 0;
   out_3551468011127802550[33] = 0;
   out_3551468011127802550[34] = 0;
   out_3551468011127802550[35] = 0;
   out_3551468011127802550[36] = 0;
   out_3551468011127802550[37] = 0;
   out_3551468011127802550[38] = 0;
   out_3551468011127802550[39] = 0;
   out_3551468011127802550[40] = 1;
   out_3551468011127802550[41] = 0;
   out_3551468011127802550[42] = 0;
   out_3551468011127802550[43] = 0;
   out_3551468011127802550[44] = 0;
   out_3551468011127802550[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3551468011127802550[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3551468011127802550[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3551468011127802550[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3551468011127802550[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3551468011127802550[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3551468011127802550[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3551468011127802550[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3551468011127802550[53] = -9.8000000000000007*dt;
   out_3551468011127802550[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3551468011127802550[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3551468011127802550[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3551468011127802550[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3551468011127802550[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3551468011127802550[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3551468011127802550[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3551468011127802550[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3551468011127802550[62] = 0;
   out_3551468011127802550[63] = 0;
   out_3551468011127802550[64] = 0;
   out_3551468011127802550[65] = 0;
   out_3551468011127802550[66] = 0;
   out_3551468011127802550[67] = 0;
   out_3551468011127802550[68] = 0;
   out_3551468011127802550[69] = 0;
   out_3551468011127802550[70] = 1;
   out_3551468011127802550[71] = 0;
   out_3551468011127802550[72] = 0;
   out_3551468011127802550[73] = 0;
   out_3551468011127802550[74] = 0;
   out_3551468011127802550[75] = 0;
   out_3551468011127802550[76] = 0;
   out_3551468011127802550[77] = 0;
   out_3551468011127802550[78] = 0;
   out_3551468011127802550[79] = 0;
   out_3551468011127802550[80] = 1;
}
void h_25(double *state, double *unused, double *out_299851405951739482) {
   out_299851405951739482[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2659690145638598458) {
   out_2659690145638598458[0] = 0;
   out_2659690145638598458[1] = 0;
   out_2659690145638598458[2] = 0;
   out_2659690145638598458[3] = 0;
   out_2659690145638598458[4] = 0;
   out_2659690145638598458[5] = 0;
   out_2659690145638598458[6] = 1;
   out_2659690145638598458[7] = 0;
   out_2659690145638598458[8] = 0;
}
void h_24(double *state, double *unused, double *out_5923104642486504031) {
   out_5923104642486504031[0] = state[4];
   out_5923104642486504031[1] = state[5];
}
void H_24(double *state, double *unused, double *out_9211482121479435057) {
   out_9211482121479435057[0] = 0;
   out_9211482121479435057[1] = 0;
   out_9211482121479435057[2] = 0;
   out_9211482121479435057[3] = 0;
   out_9211482121479435057[4] = 1;
   out_9211482121479435057[5] = 0;
   out_9211482121479435057[6] = 0;
   out_9211482121479435057[7] = 0;
   out_9211482121479435057[8] = 0;
   out_9211482121479435057[9] = 0;
   out_9211482121479435057[10] = 0;
   out_9211482121479435057[11] = 0;
   out_9211482121479435057[12] = 0;
   out_9211482121479435057[13] = 0;
   out_9211482121479435057[14] = 1;
   out_9211482121479435057[15] = 0;
   out_9211482121479435057[16] = 0;
   out_9211482121479435057[17] = 0;
}
void h_30(double *state, double *unused, double *out_7463413850868932959) {
   out_7463413850868932959[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8870363586579336403) {
   out_8870363586579336403[0] = 0;
   out_8870363586579336403[1] = 0;
   out_8870363586579336403[2] = 0;
   out_8870363586579336403[3] = 0;
   out_8870363586579336403[4] = 1;
   out_8870363586579336403[5] = 0;
   out_8870363586579336403[6] = 0;
   out_8870363586579336403[7] = 0;
   out_8870363586579336403[8] = 0;
}
void h_26(double *state, double *unused, double *out_4426627799424600274) {
   out_4426627799424600274[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1081813173235457766) {
   out_1081813173235457766[0] = 0;
   out_1081813173235457766[1] = 0;
   out_1081813173235457766[2] = 0;
   out_1081813173235457766[3] = 0;
   out_1081813173235457766[4] = 0;
   out_1081813173235457766[5] = 0;
   out_1081813173235457766[6] = 0;
   out_1081813173235457766[7] = 1;
   out_1081813173235457766[8] = 0;
}
void h_27(double *state, double *unused, double *out_2736934645492593203) {
   out_2736934645492593203[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7401617175329790302) {
   out_7401617175329790302[0] = 0;
   out_7401617175329790302[1] = 0;
   out_7401617175329790302[2] = 0;
   out_7401617175329790302[3] = 1;
   out_7401617175329790302[4] = 0;
   out_7401617175329790302[5] = 0;
   out_7401617175329790302[6] = 0;
   out_7401617175329790302[7] = 0;
   out_7401617175329790302[8] = 0;
}
void h_29(double *state, double *unused, double *out_2461740583208087314) {
   out_2461740583208087314[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8360132242264944219) {
   out_8360132242264944219[0] = 0;
   out_8360132242264944219[1] = 1;
   out_8360132242264944219[2] = 0;
   out_8360132242264944219[3] = 0;
   out_8360132242264944219[4] = 0;
   out_8360132242264944219[5] = 0;
   out_8360132242264944219[6] = 0;
   out_8360132242264944219[7] = 0;
   out_8360132242264944219[8] = 0;
}
void h_28(double *state, double *unused, double *out_5117106018126811552) {
   out_5117106018126811552[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5004212814375076823) {
   out_5004212814375076823[0] = 1;
   out_5004212814375076823[1] = 0;
   out_5004212814375076823[2] = 0;
   out_5004212814375076823[3] = 0;
   out_5004212814375076823[4] = 0;
   out_5004212814375076823[5] = 0;
   out_5004212814375076823[6] = 0;
   out_5004212814375076823[7] = 0;
   out_5004212814375076823[8] = 0;
}
void h_31(double *state, double *unused, double *out_7070686632302090418) {
   out_7070686632302090418[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2690336107515558886) {
   out_2690336107515558886[0] = 0;
   out_2690336107515558886[1] = 0;
   out_2690336107515558886[2] = 0;
   out_2690336107515558886[3] = 0;
   out_2690336107515558886[4] = 0;
   out_2690336107515558886[5] = 0;
   out_2690336107515558886[6] = 0;
   out_2690336107515558886[7] = 0;
   out_2690336107515558886[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5087377540895903948) {
  err_fun(nom_x, delta_x, out_5087377540895903948);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1760528671002568790) {
  inv_err_fun(nom_x, true_x, out_1760528671002568790);
}
void car_H_mod_fun(double *state, double *out_36576598272594351) {
  H_mod_fun(state, out_36576598272594351);
}
void car_f_fun(double *state, double dt, double *out_4730314693298424207) {
  f_fun(state,  dt, out_4730314693298424207);
}
void car_F_fun(double *state, double dt, double *out_3551468011127802550) {
  F_fun(state,  dt, out_3551468011127802550);
}
void car_h_25(double *state, double *unused, double *out_299851405951739482) {
  h_25(state, unused, out_299851405951739482);
}
void car_H_25(double *state, double *unused, double *out_2659690145638598458) {
  H_25(state, unused, out_2659690145638598458);
}
void car_h_24(double *state, double *unused, double *out_5923104642486504031) {
  h_24(state, unused, out_5923104642486504031);
}
void car_H_24(double *state, double *unused, double *out_9211482121479435057) {
  H_24(state, unused, out_9211482121479435057);
}
void car_h_30(double *state, double *unused, double *out_7463413850868932959) {
  h_30(state, unused, out_7463413850868932959);
}
void car_H_30(double *state, double *unused, double *out_8870363586579336403) {
  H_30(state, unused, out_8870363586579336403);
}
void car_h_26(double *state, double *unused, double *out_4426627799424600274) {
  h_26(state, unused, out_4426627799424600274);
}
void car_H_26(double *state, double *unused, double *out_1081813173235457766) {
  H_26(state, unused, out_1081813173235457766);
}
void car_h_27(double *state, double *unused, double *out_2736934645492593203) {
  h_27(state, unused, out_2736934645492593203);
}
void car_H_27(double *state, double *unused, double *out_7401617175329790302) {
  H_27(state, unused, out_7401617175329790302);
}
void car_h_29(double *state, double *unused, double *out_2461740583208087314) {
  h_29(state, unused, out_2461740583208087314);
}
void car_H_29(double *state, double *unused, double *out_8360132242264944219) {
  H_29(state, unused, out_8360132242264944219);
}
void car_h_28(double *state, double *unused, double *out_5117106018126811552) {
  h_28(state, unused, out_5117106018126811552);
}
void car_H_28(double *state, double *unused, double *out_5004212814375076823) {
  H_28(state, unused, out_5004212814375076823);
}
void car_h_31(double *state, double *unused, double *out_7070686632302090418) {
  h_31(state, unused, out_7070686632302090418);
}
void car_H_31(double *state, double *unused, double *out_2690336107515558886) {
  H_31(state, unused, out_2690336107515558886);
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
