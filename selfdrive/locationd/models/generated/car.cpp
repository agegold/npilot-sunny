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
void err_fun(double *nom_x, double *delta_x, double *out_8744870931434381505) {
   out_8744870931434381505[0] = delta_x[0] + nom_x[0];
   out_8744870931434381505[1] = delta_x[1] + nom_x[1];
   out_8744870931434381505[2] = delta_x[2] + nom_x[2];
   out_8744870931434381505[3] = delta_x[3] + nom_x[3];
   out_8744870931434381505[4] = delta_x[4] + nom_x[4];
   out_8744870931434381505[5] = delta_x[5] + nom_x[5];
   out_8744870931434381505[6] = delta_x[6] + nom_x[6];
   out_8744870931434381505[7] = delta_x[7] + nom_x[7];
   out_8744870931434381505[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2789839103479686818) {
   out_2789839103479686818[0] = -nom_x[0] + true_x[0];
   out_2789839103479686818[1] = -nom_x[1] + true_x[1];
   out_2789839103479686818[2] = -nom_x[2] + true_x[2];
   out_2789839103479686818[3] = -nom_x[3] + true_x[3];
   out_2789839103479686818[4] = -nom_x[4] + true_x[4];
   out_2789839103479686818[5] = -nom_x[5] + true_x[5];
   out_2789839103479686818[6] = -nom_x[6] + true_x[6];
   out_2789839103479686818[7] = -nom_x[7] + true_x[7];
   out_2789839103479686818[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1493479091139239047) {
   out_1493479091139239047[0] = 1.0;
   out_1493479091139239047[1] = 0;
   out_1493479091139239047[2] = 0;
   out_1493479091139239047[3] = 0;
   out_1493479091139239047[4] = 0;
   out_1493479091139239047[5] = 0;
   out_1493479091139239047[6] = 0;
   out_1493479091139239047[7] = 0;
   out_1493479091139239047[8] = 0;
   out_1493479091139239047[9] = 0;
   out_1493479091139239047[10] = 1.0;
   out_1493479091139239047[11] = 0;
   out_1493479091139239047[12] = 0;
   out_1493479091139239047[13] = 0;
   out_1493479091139239047[14] = 0;
   out_1493479091139239047[15] = 0;
   out_1493479091139239047[16] = 0;
   out_1493479091139239047[17] = 0;
   out_1493479091139239047[18] = 0;
   out_1493479091139239047[19] = 0;
   out_1493479091139239047[20] = 1.0;
   out_1493479091139239047[21] = 0;
   out_1493479091139239047[22] = 0;
   out_1493479091139239047[23] = 0;
   out_1493479091139239047[24] = 0;
   out_1493479091139239047[25] = 0;
   out_1493479091139239047[26] = 0;
   out_1493479091139239047[27] = 0;
   out_1493479091139239047[28] = 0;
   out_1493479091139239047[29] = 0;
   out_1493479091139239047[30] = 1.0;
   out_1493479091139239047[31] = 0;
   out_1493479091139239047[32] = 0;
   out_1493479091139239047[33] = 0;
   out_1493479091139239047[34] = 0;
   out_1493479091139239047[35] = 0;
   out_1493479091139239047[36] = 0;
   out_1493479091139239047[37] = 0;
   out_1493479091139239047[38] = 0;
   out_1493479091139239047[39] = 0;
   out_1493479091139239047[40] = 1.0;
   out_1493479091139239047[41] = 0;
   out_1493479091139239047[42] = 0;
   out_1493479091139239047[43] = 0;
   out_1493479091139239047[44] = 0;
   out_1493479091139239047[45] = 0;
   out_1493479091139239047[46] = 0;
   out_1493479091139239047[47] = 0;
   out_1493479091139239047[48] = 0;
   out_1493479091139239047[49] = 0;
   out_1493479091139239047[50] = 1.0;
   out_1493479091139239047[51] = 0;
   out_1493479091139239047[52] = 0;
   out_1493479091139239047[53] = 0;
   out_1493479091139239047[54] = 0;
   out_1493479091139239047[55] = 0;
   out_1493479091139239047[56] = 0;
   out_1493479091139239047[57] = 0;
   out_1493479091139239047[58] = 0;
   out_1493479091139239047[59] = 0;
   out_1493479091139239047[60] = 1.0;
   out_1493479091139239047[61] = 0;
   out_1493479091139239047[62] = 0;
   out_1493479091139239047[63] = 0;
   out_1493479091139239047[64] = 0;
   out_1493479091139239047[65] = 0;
   out_1493479091139239047[66] = 0;
   out_1493479091139239047[67] = 0;
   out_1493479091139239047[68] = 0;
   out_1493479091139239047[69] = 0;
   out_1493479091139239047[70] = 1.0;
   out_1493479091139239047[71] = 0;
   out_1493479091139239047[72] = 0;
   out_1493479091139239047[73] = 0;
   out_1493479091139239047[74] = 0;
   out_1493479091139239047[75] = 0;
   out_1493479091139239047[76] = 0;
   out_1493479091139239047[77] = 0;
   out_1493479091139239047[78] = 0;
   out_1493479091139239047[79] = 0;
   out_1493479091139239047[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8702949118916194196) {
   out_8702949118916194196[0] = state[0];
   out_8702949118916194196[1] = state[1];
   out_8702949118916194196[2] = state[2];
   out_8702949118916194196[3] = state[3];
   out_8702949118916194196[4] = state[4];
   out_8702949118916194196[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8702949118916194196[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8702949118916194196[7] = state[7];
   out_8702949118916194196[8] = state[8];
}
void F_fun(double *state, double dt, double *out_791503401684057011) {
   out_791503401684057011[0] = 1;
   out_791503401684057011[1] = 0;
   out_791503401684057011[2] = 0;
   out_791503401684057011[3] = 0;
   out_791503401684057011[4] = 0;
   out_791503401684057011[5] = 0;
   out_791503401684057011[6] = 0;
   out_791503401684057011[7] = 0;
   out_791503401684057011[8] = 0;
   out_791503401684057011[9] = 0;
   out_791503401684057011[10] = 1;
   out_791503401684057011[11] = 0;
   out_791503401684057011[12] = 0;
   out_791503401684057011[13] = 0;
   out_791503401684057011[14] = 0;
   out_791503401684057011[15] = 0;
   out_791503401684057011[16] = 0;
   out_791503401684057011[17] = 0;
   out_791503401684057011[18] = 0;
   out_791503401684057011[19] = 0;
   out_791503401684057011[20] = 1;
   out_791503401684057011[21] = 0;
   out_791503401684057011[22] = 0;
   out_791503401684057011[23] = 0;
   out_791503401684057011[24] = 0;
   out_791503401684057011[25] = 0;
   out_791503401684057011[26] = 0;
   out_791503401684057011[27] = 0;
   out_791503401684057011[28] = 0;
   out_791503401684057011[29] = 0;
   out_791503401684057011[30] = 1;
   out_791503401684057011[31] = 0;
   out_791503401684057011[32] = 0;
   out_791503401684057011[33] = 0;
   out_791503401684057011[34] = 0;
   out_791503401684057011[35] = 0;
   out_791503401684057011[36] = 0;
   out_791503401684057011[37] = 0;
   out_791503401684057011[38] = 0;
   out_791503401684057011[39] = 0;
   out_791503401684057011[40] = 1;
   out_791503401684057011[41] = 0;
   out_791503401684057011[42] = 0;
   out_791503401684057011[43] = 0;
   out_791503401684057011[44] = 0;
   out_791503401684057011[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_791503401684057011[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_791503401684057011[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_791503401684057011[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_791503401684057011[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_791503401684057011[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_791503401684057011[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_791503401684057011[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_791503401684057011[53] = -9.8000000000000007*dt;
   out_791503401684057011[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_791503401684057011[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_791503401684057011[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_791503401684057011[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_791503401684057011[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_791503401684057011[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_791503401684057011[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_791503401684057011[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_791503401684057011[62] = 0;
   out_791503401684057011[63] = 0;
   out_791503401684057011[64] = 0;
   out_791503401684057011[65] = 0;
   out_791503401684057011[66] = 0;
   out_791503401684057011[67] = 0;
   out_791503401684057011[68] = 0;
   out_791503401684057011[69] = 0;
   out_791503401684057011[70] = 1;
   out_791503401684057011[71] = 0;
   out_791503401684057011[72] = 0;
   out_791503401684057011[73] = 0;
   out_791503401684057011[74] = 0;
   out_791503401684057011[75] = 0;
   out_791503401684057011[76] = 0;
   out_791503401684057011[77] = 0;
   out_791503401684057011[78] = 0;
   out_791503401684057011[79] = 0;
   out_791503401684057011[80] = 1;
}
void h_25(double *state, double *unused, double *out_2856288359181493714) {
   out_2856288359181493714[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2863785685210894639) {
   out_2863785685210894639[0] = 0;
   out_2863785685210894639[1] = 0;
   out_2863785685210894639[2] = 0;
   out_2863785685210894639[3] = 0;
   out_2863785685210894639[4] = 0;
   out_2863785685210894639[5] = 0;
   out_2863785685210894639[6] = 1;
   out_2863785685210894639[7] = 0;
   out_2863785685210894639[8] = 0;
}
void h_24(double *state, double *unused, double *out_4313985665054181548) {
   out_4313985665054181548[0] = state[4];
   out_4313985665054181548[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7610295012177403673) {
   out_7610295012177403673[0] = 0;
   out_7610295012177403673[1] = 0;
   out_7610295012177403673[2] = 0;
   out_7610295012177403673[3] = 0;
   out_7610295012177403673[4] = 1;
   out_7610295012177403673[5] = 0;
   out_7610295012177403673[6] = 0;
   out_7610295012177403673[7] = 0;
   out_7610295012177403673[8] = 0;
   out_7610295012177403673[9] = 0;
   out_7610295012177403673[10] = 0;
   out_7610295012177403673[11] = 0;
   out_7610295012177403673[12] = 0;
   out_7610295012177403673[13] = 0;
   out_7610295012177403673[14] = 1;
   out_7610295012177403673[15] = 0;
   out_7610295012177403673[16] = 0;
   out_7610295012177403673[17] = 0;
}
void h_30(double *state, double *unused, double *out_4346927597804492256) {
   out_4346927597804492256[0] = state[4];
}
void H_30(double *state, double *unused, double *out_345452726703646012) {
   out_345452726703646012[0] = 0;
   out_345452726703646012[1] = 0;
   out_345452726703646012[2] = 0;
   out_345452726703646012[3] = 0;
   out_345452726703646012[4] = 1;
   out_345452726703646012[5] = 0;
   out_345452726703646012[6] = 0;
   out_345452726703646012[7] = 0;
   out_345452726703646012[8] = 0;
}
void h_26(double *state, double *unused, double *out_6124616607452235092) {
   out_6124616607452235092[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6605289004084950863) {
   out_6605289004084950863[0] = 0;
   out_6605289004084950863[1] = 0;
   out_6605289004084950863[2] = 0;
   out_6605289004084950863[3] = 0;
   out_6605289004084950863[4] = 0;
   out_6605289004084950863[5] = 0;
   out_6605289004084950863[6] = 0;
   out_6605289004084950863[7] = 1;
   out_6605289004084950863[8] = 0;
}
void h_27(double *state, double *unused, double *out_3459604243857971231) {
   out_3459604243857971231[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2520216038504070923) {
   out_2520216038504070923[0] = 0;
   out_2520216038504070923[1] = 0;
   out_2520216038504070923[2] = 0;
   out_2520216038504070923[3] = 1;
   out_2520216038504070923[4] = 0;
   out_2520216038504070923[5] = 0;
   out_2520216038504070923[6] = 0;
   out_2520216038504070923[7] = 0;
   out_2520216038504070923[8] = 0;
}
void h_29(double *state, double *unused, double *out_7097053772111993581) {
   out_7097053772111993581[0] = state[1];
}
void H_29(double *state, double *unused, double *out_164778617610746172) {
   out_164778617610746172[0] = 0;
   out_164778617610746172[1] = 1;
   out_164778617610746172[2] = 0;
   out_164778617610746172[3] = 0;
   out_164778617610746172[4] = 0;
   out_164778617610746172[5] = 0;
   out_164778617610746172[6] = 0;
   out_164778617610746172[7] = 0;
   out_164778617610746172[8] = 0;
}
void h_28(double *state, double *unused, double *out_5735133061471023365) {
   out_5735133061471023365[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4917620399458784402) {
   out_4917620399458784402[0] = 1;
   out_4917620399458784402[1] = 0;
   out_4917620399458784402[2] = 0;
   out_4917620399458784402[3] = 0;
   out_4917620399458784402[4] = 0;
   out_4917620399458784402[5] = 0;
   out_4917620399458784402[6] = 0;
   out_4917620399458784402[7] = 0;
   out_4917620399458784402[8] = 0;
}
void h_31(double *state, double *unused, double *out_91437631173991247) {
   out_91437631173991247[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7231497106318302339) {
   out_7231497106318302339[0] = 0;
   out_7231497106318302339[1] = 0;
   out_7231497106318302339[2] = 0;
   out_7231497106318302339[3] = 0;
   out_7231497106318302339[4] = 0;
   out_7231497106318302339[5] = 0;
   out_7231497106318302339[6] = 0;
   out_7231497106318302339[7] = 0;
   out_7231497106318302339[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8744870931434381505) {
  err_fun(nom_x, delta_x, out_8744870931434381505);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2789839103479686818) {
  inv_err_fun(nom_x, true_x, out_2789839103479686818);
}
void car_H_mod_fun(double *state, double *out_1493479091139239047) {
  H_mod_fun(state, out_1493479091139239047);
}
void car_f_fun(double *state, double dt, double *out_8702949118916194196) {
  f_fun(state,  dt, out_8702949118916194196);
}
void car_F_fun(double *state, double dt, double *out_791503401684057011) {
  F_fun(state,  dt, out_791503401684057011);
}
void car_h_25(double *state, double *unused, double *out_2856288359181493714) {
  h_25(state, unused, out_2856288359181493714);
}
void car_H_25(double *state, double *unused, double *out_2863785685210894639) {
  H_25(state, unused, out_2863785685210894639);
}
void car_h_24(double *state, double *unused, double *out_4313985665054181548) {
  h_24(state, unused, out_4313985665054181548);
}
void car_H_24(double *state, double *unused, double *out_7610295012177403673) {
  H_24(state, unused, out_7610295012177403673);
}
void car_h_30(double *state, double *unused, double *out_4346927597804492256) {
  h_30(state, unused, out_4346927597804492256);
}
void car_H_30(double *state, double *unused, double *out_345452726703646012) {
  H_30(state, unused, out_345452726703646012);
}
void car_h_26(double *state, double *unused, double *out_6124616607452235092) {
  h_26(state, unused, out_6124616607452235092);
}
void car_H_26(double *state, double *unused, double *out_6605289004084950863) {
  H_26(state, unused, out_6605289004084950863);
}
void car_h_27(double *state, double *unused, double *out_3459604243857971231) {
  h_27(state, unused, out_3459604243857971231);
}
void car_H_27(double *state, double *unused, double *out_2520216038504070923) {
  H_27(state, unused, out_2520216038504070923);
}
void car_h_29(double *state, double *unused, double *out_7097053772111993581) {
  h_29(state, unused, out_7097053772111993581);
}
void car_H_29(double *state, double *unused, double *out_164778617610746172) {
  H_29(state, unused, out_164778617610746172);
}
void car_h_28(double *state, double *unused, double *out_5735133061471023365) {
  h_28(state, unused, out_5735133061471023365);
}
void car_H_28(double *state, double *unused, double *out_4917620399458784402) {
  H_28(state, unused, out_4917620399458784402);
}
void car_h_31(double *state, double *unused, double *out_91437631173991247) {
  h_31(state, unused, out_91437631173991247);
}
void car_H_31(double *state, double *unused, double *out_7231497106318302339) {
  H_31(state, unused, out_7231497106318302339);
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
