//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: BEV_image.cu
//
// GPU Coder version                    : 2.1
// CUDA/C/C++ source code generated on  : 31-Aug-2021 16:30:47
//

// Include Files
#include "BEV_image.h"
#include "BEV_image_data.h"
#include "BEV_image_initialize.h"
#include "rt_nonfinite.h"
#include "MWCUSOLVERUtils.hpp"
#include "MWCudaDimUtility.hpp"
#include "MWLaunchParametersUtilities.hpp"
#include "math_constants.h"
#include <cmath>

// Variable Definitions
static double State[2520];
static double out_Prob_ctrv;
static bool out_Prob_ctrv_not_empty;
static double out_Prob_cv;
static double out_P_ctrv[25];
static double out_P_cv[25];
static double flag[15];
static bool just_one_check_not_empty;

// Function Declarations
static __global__ void BEV_image_kernel1(double b_flag[15], double old_flag[15]);
static __global__ void BEV_image_kernel10(const double Vr[25], const double
  *Class_B_idx_9, const double L[25], const double a, double P_ctrv_out[25]);
static __global__ void BEV_image_kernel100(const double P_zz[9], double A[9]);
static __global__ void BEV_image_kernel101(const double P_zz[9], double A[9]);
static __global__ void BEV_image_kernel102(const double P_zz[9], double A[9]);
static __global__ void BEV_image_kernel103(double A[9]);
static __global__ void BEV_image_kernel104(double A[9]);
static __global__ void BEV_image_kernel105(const double *Class_B_idx_9, double
  A[9], double *Y_AB);
static __global__ void BEV_image_kernel106(const double v[3], const double
  y_out[3], double Y_k_hat[3]);
static __global__ void BEV_image_kernel107(const double y[9], const double
  Y_k_hat[3], double dv1[3]);
static __global__ void BEV_image_kernel108(const double v[3], const double
  y_out[3], const double dv1[3], double *d);
static __global__ void BEV_image_kernel109(double x_ctrv[5]);
static __global__ void BEV_image_kernel11(const double x_cv_out[5], const double
  x_ini[5], double tmp_ego_y[5], double work[5]);
static __global__ void BEV_image_kernel110(const double v[3], const double
  y_out[3], double W[3]);
static __global__ void BEV_image_kernel111(const double x_k_hat[5], const double
  W[3], const double K[15], double x_ctrv[5]);
static __global__ void BEV_image_kernel112(const double P_zz[9], const double K
  [15], double P_xz[15]);
static __global__ void BEV_image_kernel113(const double L[25], const int
  sample_ts, const double K[15], const double P_xz[15], double P_ctrv_tmp[250]);
static __global__ void BEV_image_kernel114(const double x_ini[5], double x_ctrv
  [5]);
static __global__ void BEV_image_kernel115(double x_ctrv[5]);
static __global__ void BEV_image_kernel116(const double P_cv[25], const int
  sample_ts, double P_cv_tmp[250]);
static __global__ void BEV_image_kernel117(const double P_cv_out[25], double L
  [25]);
static __global__ void BEV_image_kernel118(double work[5]);
static __global__ void BEV_image_kernel119(const double L[25], const int i,
  double *Y_AB);
static __global__ void BEV_image_kernel12(const double tmp_ego_y[5], const
  double work[5], double b_out_P_ctrv[25], double L[25]);
static __global__ void BEV_image_kernel120(const int knt, double *X_AB);
static __global__ void BEV_image_kernel121(const int i, double L[25]);
static __global__ void BEV_image_kernel122(const int lastc, double work[5]);
static __global__ void BEV_image_kernel123(const int ix, const double L[25],
  const int *b_L, const int jA, double work[5]);
static __global__ void BEV_image_kernel124(const int lastc, double work[5]);
static __global__ void BEV_image_kernel125(const int iaii, const double L[25],
  const int lastv, const int knt, const int DEC_param, double work[5]);
static __global__ void BEV_image_kernel126(const double *Y_AB, const int i,
  double L[25]);
static __global__ void BEV_image_kernel127(const double L[25], double Vr[25]);
static __global__ void BEV_image_kernel128(double Vr[25]);
static __global__ void BEV_image_kernel129(double Vr[25]);
static __global__ void BEV_image_kernel13(const double x_cv_out[5], const double
  x_ini[5], double tmp_ego_y[5], double work[5]);
static __global__ void BEV_image_kernel130(double work[5]);
static __global__ void BEV_image_kernel131(const int iaii, double Vr[25]);
static __global__ void BEV_image_kernel132(const int lastc, double work[5]);
static __global__ void BEV_image_kernel133(const double Vr[25], const int lastv,
  const int iaii, const int DEC_param, double work[5]);
static __global__ void BEV_image_kernel134(double v[3]);
static __global__ void BEV_image_kernel135(double L[25]);
static __global__ void BEV_image_kernel136(const double L[25], const int lastc,
  double *Y_AB);
static __global__ void BEV_image_kernel137(const double L[25], const int lastc,
  double *Y_AB);
static __global__ void BEV_image_kernel138(const double L[25], const int lastc,
  double *Y_AB);
static __global__ void BEV_image_kernel139(const int lastc, double L[25]);
static __global__ void BEV_image_kernel14(const double tmp_ego_y[5], const
  double work[5], double b_out_P_cv[25], double Vr[25]);
static __global__ void BEV_image_kernel140(const double L[25], const int
  DEC_param, const int *nr, double v[3]);
static __global__ void BEV_image_kernel141(const double v[3], const double *X_AB,
  double *Class_B_idx_10);
static __global__ void BEV_image_kernel142(const double a, const int *nr, double
  v[3]);
static __global__ void BEV_image_kernel143(const int *nr, double v[3]);
static __global__ void BEV_image_kernel144(const double v[3], double *X_AB);
static __global__ void BEV_image_kernel145(const double a, const int *nr, double
  v[3]);
static __global__ void BEV_image_kernel146(const int knt, double *X_AB);
static __global__ void BEV_image_kernel147(const double *Y_AB, double v[3]);
static __global__ void BEV_image_kernel148(const double v[3], const double
  *Class_B_idx_10, double *Y_AB);
static __global__ void BEV_image_kernel149(const double v[3], const double
  *Class_B_idx_10, double *Class_B_idx_9);
static __global__ void BEV_image_kernel15(const double Vr[25], const double
  *Class_B_idx_9, const double L[25], const double a, double P_cv_out[25]);
static __global__ void BEV_image_kernel150(const double Vr[25], const double L
  [25], creal_T U[25], creal_T L_tmp[25]);
static __global__ void BEV_image_kernel151(const double *X_AB, const double
  *Y_AB, const double *Class_B_idx_9, creal_T *c);
static __global__ void BEV_image_kernel152(const double *Y_AB, const double
  *Class_B_idx_9, creal_T *c);
static __global__ void BEV_image_kernel153(const double *scale, const creal_T *c,
  const int ix, creal_T L_tmp[25]);
static __global__ void BEV_image_kernel154(const double *scale, const creal_T *c,
  const int ix, creal_T U[25]);
static __global__ void BEV_image_kernel155(const int ix, creal_T L_tmp[25]);
static __global__ void BEV_image_kernel156(creal_T U[25]);
static __global__ void BEV_image_kernel157(creal_T L_tmp[25]);
static __global__ void BEV_image_kernel158(creal_T R[25]);
static __global__ void BEV_image_kernel159(const creal_T L_tmp[25], creal_T R[25]);
static __global__ void BEV_image_kernel16(const double P_ctrv[25], const int
  sample_ts, double P_ctrv_tmp[250]);
static __global__ void BEV_image_kernel160(const creal_T R[25], const creal_T U
  [25], const int *k, creal_T b_U[25]);
static __global__ void BEV_image_kernel161(const creal_T U[25], const creal_T
  b_U[25], const int *k, creal_T L_tmp[25]);
static __global__ void BEV_image_kernel162(const creal_T L_tmp[25], double L[25]);
static __global__ void BEV_image_kernel163(const double L[25], const int j,
  double *scale);
static __global__ void BEV_image_kernel164(const creal_T L_tmp[25], const int j,
  double *scale);
static __global__ void BEV_image_kernel165(creal_T L_tmp[25]);
static __global__ void BEV_image_kernel166(const creal_T L_tmp[25], double L[25]);
static __global__ void BEV_image_kernel167(double X_k[55]);
static __global__ void BEV_image_kernel168(double W[11]);
static __global__ void BEV_image_kernel169(const double L[25], const int i,
  const double x_cv_out[5], const int b_i, double X_k[55]);
static __global__ void BEV_image_kernel17(const double P_ctrv_out[25], double L
  [25]);
static __global__ void BEV_image_kernel170(const int i, double W[11]);
static __global__ void BEV_image_kernel171(const double L[25], const int i,
  const double x_cv_out[5], const int b_i, double X_k[55]);
static __global__ void BEV_image_kernel172(const int i, double W[11]);
static __global__ void BEV_image_kernel173(const double x_cv_out[5], double X_k
  [55]);
static __global__ void BEV_image_kernel174(double W[11]);
static __global__ void BEV_image_kernel175(double x_k_hat[5]);
static __global__ void BEV_image_kernel176(double L[25]);
static __global__ void BEV_image_kernel177(double work[5]);
static __global__ void BEV_image_kernel178(const double *scale, const double ts,
  const double *d, const double X_k[55], const int i, double work[5]);
static __global__ void BEV_image_kernel179(const double *X_AB, const int i,
  const double work[5], double x_k_hat[5], double X_k_hat[55]);
static __global__ void BEV_image_kernel18(double work[5]);
static __global__ void BEV_image_kernel180(const double *X_AB, const double
  x_k_hat[5], const double X_k_hat[55], const int i, double tmp_ego_y[5], double
  work[5]);
static __global__ void BEV_image_kernel181(const double tmp_ego_y[5], const
  double work[5], double L[25]);
static __global__ void BEV_image_kernel182(const double P_cv[25], double L[25]);
static __global__ void BEV_image_kernel183(double v[3]);
static __global__ void BEV_image_kernel184(double P_zz[9]);
static __global__ void BEV_image_kernel185(double P_xz[15]);
static __global__ void BEV_image_kernel186(const double X_k[55], const signed
  char a[15], const int i, double Y_k_hat[33]);
static __global__ void BEV_image_kernel187(const double Y_k_hat[33], const int i,
  const double *X_AB, double v[3]);
static __global__ void BEV_image_kernel188(const double *X_AB, const double v[3],
  const double Y_k_hat[33], const int i, double b_Y_k_hat[3], double W[3]);
static __global__ void BEV_image_kernel189(const double Y_k_hat[3], const double
  W[3], double P_zz[9]);
static __global__ void BEV_image_kernel19(const int knt, double *X_AB);
static __global__ void BEV_image_kernel190(const double x_k_hat[5], const double
  X_k_hat[55], const int i, const double *X_AB, double work[5]);
static __global__ void BEV_image_kernel191(const double v[3], const double
  Y_k_hat[33], const int i, double b_Y_k_hat[3]);
static __global__ void BEV_image_kernel192(const double Y_k_hat[3], const double
  work[5], double P_xz[15]);
static __global__ void BEV_image_kernel193(const double R_CTRV_IMM[9], double A
  [9], double P_zz[9]);
static __global__ void BEV_image_kernel194(const int jA, const double P_zz[9],
  const int ix, double A[9]);
static __global__ void BEV_image_kernel195(const int jA, const int *nr, double
  A[9]);
static __global__ void BEV_image_kernel196(const int jA, const int ix, double A
  [9]);
static __global__ void BEV_image_kernel197(const int jA, const int *nr, double
  A[9]);
static __global__ void BEV_image_kernel198(const int jA, const int ix, double A
  [9]);
static __global__ void BEV_image_kernel199(const int jA, const int *nr, double
  A[9]);
static __global__ void BEV_image_kernel2(const int *k, double b_State[2520],
  double dv[2492]);
static __global__ void BEV_image_kernel20(const int lastc, double work[5]);
static __global__ void BEV_image_kernel200(const double P_zz[9], double A[9]);
static __global__ void BEV_image_kernel201(int ipiv_t[3], int ipiv[3]);
static __global__ void BEV_image_kernel202(double A[9]);
static __global__ void BEV_image_kernel203(int ipiv[3]);
static __global__ void BEV_image_kernel204(double A[9], double *temp);
static __global__ void BEV_image_kernel205(const double P_zz[9], double A[9]);
static __global__ void BEV_image_kernel206(const double P_zz[9], double A[9]);
static __global__ void BEV_image_kernel207(const double P_zz[9], double A[9]);
static __global__ void BEV_image_kernel208(double A[9]);
static __global__ void BEV_image_kernel209(double A[9]);
static __global__ void BEV_image_kernel21(const int ix, const double L[25],
  const int *b_L, const int *nr, double work[5]);
static __global__ void BEV_image_kernel210(const double *Y_AB, const int ix,
  double y[9]);
static __global__ void BEV_image_kernel211(const double *Class_B_idx_9, const
  int ix, double y[9]);
static __global__ void BEV_image_kernel212(const double v[3], const double
  y_out[3], double Y_k_hat[3]);
static __global__ void BEV_image_kernel213(const double y[9], const double
  Y_k_hat[3], double dv1[3]);
static __global__ void BEV_image_kernel214(const double v[3], const double
  y_out[3], const double dv1[3], double *d);
static __global__ void BEV_image_kernel215(double work[5]);
static __global__ void BEV_image_kernel216(const double v[3], const double
  y_out[3], double W[3]);
static __global__ void BEV_image_kernel217(const double x_k_hat[5], const double
  W[3], const double K[15], double work[5]);
static __global__ void BEV_image_kernel218(const double P_zz[9], const double K
  [15], double P_xz[15]);
static __global__ void BEV_image_kernel219(const double L[25], const int
  sample_ts, const double K[15], const double P_xz[15], double P_cv_tmp[250]);
static __global__ void BEV_image_kernel22(const int lastc, double work[5]);
static __global__ void BEV_image_kernel220(const double x_ini[5], double work[5]);
static __global__ void BEV_image_kernel221(double work[5]);
static __global__ void BEV_image_kernel222(const double work[5], const double
  *X_AB, const double x_ctrv[5], const double *Y_AB, const int sample_ts, double
  X_pred[50]);
static __global__ void BEV_image_kernel223(const double P_cv_tmp[250], const
  double P_ctrv_tmp[250], double b_out_P_cv[25], double b_out_P_ctrv[25]);
static __global__ void BEV_image_kernel224(const double *X_AB, const double
  *Y_AB, double Training_data_data[28]);
static __global__ void BEV_image_kernel225(const double *X_AB, double
  Training_data_data[28]);
static __global__ void BEV_image_kernel226(const double *Y_AB, double
  Training_data_data[28]);
static __global__ void BEV_image_kernel227(double Training_data_data[28]);
static __global__ void BEV_image_kernel228(double Training_data_data[28]);
static __global__ void BEV_image_kernel229(double Training_data_data[28]);
static __global__ void BEV_image_kernel23(const int iaii, const double L[25],
  const int lastv, const int knt, const int DEC_param, double work[5]);
static __global__ void BEV_image_kernel230(double Training_data_data[28]);
static __global__ void BEV_image_kernel231(const double Training_data_data[28],
  double b_State[2520]);
static __global__ void BEV_image_kernel232(const double dv2[61], const double
  dv1[251], double Ry[15311], double Rx[15311]);
static __global__ void BEV_image_kernel233(const double dv[255], const double
  *scale, double x[255]);
static __global__ void BEV_image_kernel234(unsigned char b_BEV_image[367464]);
static __global__ void BEV_image_kernel235(unsigned char b_BEV_image[367464]);
static __global__ void BEV_image_kernel236(unsigned char b_BEV_image[367464]);
static __global__ void BEV_image_kernel237(unsigned char b_BEV_image[367464]);
static __global__ void BEV_image_kernel238(const double Chassis[12], double
  tmp_ego_y[5]);
static __global__ void BEV_image_kernel239(bool in_tmp[15311]);
static __global__ void BEV_image_kernel24(const double L[25], double Vr[25]);
static __global__ void BEV_image_kernel240(int last[5], signed char first[5]);
static __global__ void BEV_image_kernel241(double work[5]);
static __global__ void BEV_image_kernel242(const double tmp_ego_y[5], const
  double tmp_ego_x[5], const int DEC_param, const int jA, double work[5]);
static __global__ void BEV_image_kernel243(const double a, const int j, const
  int last[5], double work[5]);
static __global__ void BEV_image_kernel244(const double work[5], const int last
  [5], const double tmp_ego_y[5], const double tmp_ego_x[5], const signed char
  first[5], const int *nr, const double *Class_B_idx_10, const double
  *Class_B_idx_9, const double *Y_AB, const double *X_AB, const double Ry[15311],
  const double Rx[15311], bool in_tmp[15311]);
static __global__ void BEV_image_kernel245(const bool in_tmp[15311], bool x
  [15311]);
static __global__ void BEV_image_kernel246(const int i_data[15311], const
  unsigned char j_data[15311], const int ipiv_t, const int b_ipiv_t, unsigned
  char b_BEV_image[367464]);
static __global__ void BEV_image_kernel247(double tmp_ego_x[5]);
static __global__ void BEV_image_kernel248(const double *scale, const double
  *Y_AB, const double *X_AB, const double *d, const double x, const double b_x,
  const double tmp_ego_y[5], double target_y[5], double tmp_ego_x[5]);
static __global__ void BEV_image_kernel249(bool in_tmp[15311]);
static __global__ void BEV_image_kernel25(double Vr[25]);
static __global__ void BEV_image_kernel250(int last[5], signed char first[5]);
static __global__ void BEV_image_kernel251(double work[5]);
static __global__ void BEV_image_kernel252(const double target_y[5], const
  double tmp_ego_x[5], const int DEC_param, const int jA, double work[5]);
static __global__ void BEV_image_kernel253(const double a, const int j, const
  int last[5], double work[5]);
static __global__ void BEV_image_kernel254(const double work[5], const int last
  [5], const double target_y[5], const double tmp_ego_x[5], const signed char
  first[5], const int *nr, const double *Class_B_idx_10, const double
  *Class_B_idx_9, const double *Y_AB, const double *X_AB, const double Ry[15311],
  const double Rx[15311], bool in_tmp[15311]);
static __global__ void BEV_image_kernel255(const bool in_tmp[15311], bool x
  [15311]);
static __global__ void BEV_image_kernel256(const double dv[255], const double *d,
  double x[255]);
static __global__ void BEV_image_kernel257(const double dv[255], const double *d,
  double x[255]);
static __global__ void BEV_image_kernel258(const double a, const double d2,
  double TV_range_y[10]);
static __global__ void BEV_image_kernel259(const double *X_AB, const double a,
  double TV_range_y[10]);
static __global__ void BEV_image_kernel26(double Vr[25]);
static __global__ void BEV_image_kernel260(const double *X_AB, const double
  *Y_AB, const double a, double TV_range_y[10]);
static __global__ void BEV_image_kernel261(const double *X_AB, double
  TV_range_y[10]);
static __global__ void BEV_image_kernel262(double TV_range_x[10]);
static __global__ void BEV_image_kernel263(const double *X_AB, double
  TV_range_x[10]);
static __global__ void BEV_image_kernel264(const double *Y_AB, double
  TV_range_x[10]);
static __global__ void BEV_image_kernel265(const double *X_AB, double
  TV_range_x[10]);
static __global__ void BEV_image_kernel266(const double TV_range_x[10], const
  double TV_range_y[10], const double X_pred[50], double yp4[100], double xp4
  [100], double yp3[100], double xp3[100], double yp2[100], double xp2[100],
  double yp1[100], double xp1[100]);
static __global__ void BEV_image_kernel267(double yp4[100], double xp4[100],
  double yp3[100], double xp3[100], double yp2[100], double xp2[100], double
  yp1[100], double xp1[100]);
static __global__ void BEV_image_kernel268(const double a, const double d2,
  double TV_range_y[10]);
static __global__ void BEV_image_kernel269(const double *X_AB, const double a,
  double TV_range_y[10]);
static __global__ void BEV_image_kernel27(double work[5]);
static __global__ void BEV_image_kernel270(const double *X_AB, const double
  *Y_AB, const double a, double TV_range_y[10]);
static __global__ void BEV_image_kernel271(const double *X_AB, double
  TV_range_y[10]);
static __global__ void BEV_image_kernel272(const double Chassis[12], double
  TV_range_x[10]);
static __global__ void BEV_image_kernel273(const double Chassis[12], double
  *X_AB);
static __global__ void BEV_image_kernel274(const double *X_AB, const double
  old_Prob_cv, double TV_range_x[10]);
static __global__ void BEV_image_kernel275(const double Chassis[12], double
  *X_AB);
static __global__ void BEV_image_kernel276(const double *X_AB, const double
  old_Prob_cv, double TV_range_x[10]);
static __global__ void BEV_image_kernel277(double TV_range_x[10]);
static __global__ void BEV_image_kernel278(const double TV_range_y[10], const
  double TV_range_x[10], const double TJ_X[70], const double TJ_Y[70], const int
  lastc, double yp4[100], double xp4[100], double yp3[100], double xp3[100],
  double yp2[100], double xp2[100], double yp1[100], double xp1[100]);
static __global__ void BEV_image_kernel279(double yp4[100], double xp4[100],
  double yp3[100], double xp3[100], double yp2[100], double xp2[100], double
  yp1[100], double xp1[100]);
static __global__ void BEV_image_kernel28(const int iaii, double Vr[25]);
static __global__ void BEV_image_kernel280(const double yp2[100], const int j,
  double *X_AB);
static __global__ void BEV_image_kernel281(const int j, double yp2[100]);
static __global__ void BEV_image_kernel282(const int j, double yp2[100]);
static __global__ void BEV_image_kernel283(const double xp3[100], const int j,
  double *Y_AB);
static __global__ void BEV_image_kernel284(const int j, double xp3[100]);
static __global__ void BEV_image_kernel285(const int j, double xp3[100]);
static __global__ void BEV_image_kernel286(const int m, const int *nr, const int
  DEC_param, unsigned char b_BEV_image[367464]);
static __global__ void BEV_image_kernel287(const int jA, const int *nr, const
  int DEC_param, unsigned char b_BEV_image[367464]);
static __global__ void BEV_image_kernel288(const int ix, const int *nr, const
  int DEC_param, unsigned char b_BEV_image[367464]);
static __global__ void BEV_image_kernel289(const int knt, const int *nr, const
  int DEC_param, unsigned char b_BEV_image[367464]);
static __global__ void BEV_image_kernel29(const int lastc, double work[5]);
static __global__ void BEV_image_kernel290(const int lastc, const int *nr,
  signed char first[5]);
static __global__ void BEV_image_kernel291(const int lastc, const int *nr, int
  last[5]);
static __global__ void BEV_image_kernel292(const int lastc, const int *nr, int
  last[5]);
static __global__ void BEV_image_kernel293(const int lastc, const int *nr,
  signed char first[5]);
static __global__ void BEV_image_kernel294(const int lastc, const int *nr, int
  last[5]);
static __global__ void BEV_image_kernel295(const int lastc, const int *nr, int
  last[5]);
static __global__ void BEV_image_kernel296(const double *X_AB, const double
  laneInfoR_idx_3, const double varargin_1[251], const double laneInfoR_idx_2,
  const double b[251], const double laneInfoR_idx_1, const double b_b[251],
  const double laneInfoR_idx_0, double tmp_lane_y[251]);
static __global__ void BEV_image_kernel297(const double varargin_1[251], const
  double *X_AB, double y[251]);
static __global__ void BEV_image_kernel298(const double varargin_1[61], const
  double *X_AB, double x[61]);
static __global__ void BEV_image_kernel299(const double *X_AB, const double
  laneInfoL_idx_3, const double varargin_1[251], const double laneInfoL_idx_2,
  const double b[251], const double ts, const double b_b[251], const double
  *temp, double tmp_lane_y[251]);
static __global__ void BEV_image_kernel3(const double dv[2492], const int *k,
  double b_State[2520]);
static __global__ void BEV_image_kernel30(const double Vr[25], const int lastv,
  const int iaii, const int DEC_param, double work[5]);
static __global__ void BEV_image_kernel300(const double varargin_1[251], const
  double *X_AB, double y[251]);
static __global__ void BEV_image_kernel301(const double varargin_1[61], const
  double *X_AB, double x[61]);
static __global__ void BEV_image_kernel31(double v[3]);
static __global__ void BEV_image_kernel32(const double L[25], const int lastc,
  double *Y_AB);
static __global__ void BEV_image_kernel33(const double L[25], const int lastc,
  double *Y_AB);
static __global__ void BEV_image_kernel34(const double L[25], const int lastc,
  double *Y_AB);
static __global__ void BEV_image_kernel35(const double L[25], const int
  DEC_param, const int *nr, double v[3]);
static __global__ void BEV_image_kernel36(const double v[3], const double *X_AB,
  double *Class_B_idx_10);
static __global__ void BEV_image_kernel37(const double a, const int *nr, double
  v[3]);
static __global__ void BEV_image_kernel38(const int *nr, double v[3]);
static __global__ void BEV_image_kernel39(const double v[3], double *X_AB);
static __global__ void BEV_image_kernel4(const int DEC_param, const double *X_AB,
  double TJ_X[70], double TJ_Y[70]);
static __global__ void BEV_image_kernel40(const double a, const int *nr, double
  v[3]);
static __global__ void BEV_image_kernel41(const int knt, double *X_AB);
static __global__ void BEV_image_kernel42(const double *Y_AB, double v[3]);
static __global__ void BEV_image_kernel43(const double v[3], const double
  *Class_B_idx_10, double *Y_AB);
static __global__ void BEV_image_kernel44(const double v[3], const double
  *Class_B_idx_10, double *Class_B_idx_9);
static __global__ void BEV_image_kernel45(const double Vr[25], const double L[25],
  creal_T U[25], creal_T L_tmp[25]);
static __global__ void BEV_image_kernel46(const double *X_AB, const double *Y_AB,
  const double *Class_B_idx_9, creal_T *c);
static __global__ void BEV_image_kernel47(const double *Y_AB, const double
  *Class_B_idx_9, creal_T *c);
static __global__ void BEV_image_kernel48(const double *scale, const creal_T *c,
  const int ix, creal_T L_tmp[25]);
static __global__ void BEV_image_kernel49(const double *scale, const creal_T *c,
  const int ix, creal_T U[25]);
static __global__ void BEV_image_kernel5(const double *temp, double x_ini[5]);
static __global__ void BEV_image_kernel50(const int ix, creal_T L_tmp[25]);
static __global__ void BEV_image_kernel51(creal_T U[25]);
static __global__ void BEV_image_kernel52(creal_T L_tmp[25]);
static __global__ void BEV_image_kernel53(creal_T R[25]);
static __global__ void BEV_image_kernel54(const creal_T L_tmp[25], creal_T R[25]);
static __global__ void BEV_image_kernel55(const creal_T R[25], const creal_T U
  [25], const int *k, creal_T b_U[25]);
static __global__ void BEV_image_kernel56(const creal_T U[25], const creal_T
  b_U[25], const int *k, creal_T L_tmp[25]);
static __global__ void BEV_image_kernel57(const creal_T L_tmp[25], double L[25]);
static __global__ void BEV_image_kernel58(const double L[25], const int j,
  double *scale);
static __global__ void BEV_image_kernel59(const creal_T L_tmp[25], const int j,
  double *scale);
static __global__ void BEV_image_kernel6(const double laneInfoL_idx_4, const
  double old_Prob_cv, const double old_Prob_ctrv, const double laneInfoL_idx_3,
  const double x_ini[5], double tmp_ego_y[5], double work[5], double x_cv_out[5],
  double x_ctrv_out[5]);
static __global__ void BEV_image_kernel60(creal_T L_tmp[25]);
static __global__ void BEV_image_kernel61(const creal_T L_tmp[25], double L[25]);
static __global__ void BEV_image_kernel62(double X_k[55]);
static __global__ void BEV_image_kernel63(double W[11]);
static __global__ void BEV_image_kernel64(const double L[25], const int i, const
  double x_ctrv_out[5], const int b_i, double X_k[55]);
static __global__ void BEV_image_kernel65(const int i, double W[11]);
static __global__ void BEV_image_kernel66(const double L[25], const int i, const
  double x_ctrv_out[5], const int b_i, double X_k[55]);
static __global__ void BEV_image_kernel67(const int i, double W[11]);
static __global__ void BEV_image_kernel68(const double x_ctrv_out[5], double
  X_k[55]);
static __global__ void BEV_image_kernel69(double W[11]);
static __global__ void BEV_image_kernel7(const double tmp_ego_y[5], const double
  work[5], double b_out_P_ctrv[25], double L[25]);
static __global__ void BEV_image_kernel70(double x_k_hat[5]);
static __global__ void BEV_image_kernel71(double L[25]);
static __global__ void BEV_image_kernel72(const double *d, const double X_k[55],
  const int i, double X_hat[5]);
static __global__ void BEV_image_kernel73(const double *scale, const double ts,
  const double *d, const double old_Prob_ctrv, const double X_k[55], const int i,
  double X_hat[5]);
static __global__ void BEV_image_kernel74(const double *X_AB, const int i, const
  double X_hat[5], double x_k_hat[5], double X_k_hat[55]);
static __global__ void BEV_image_kernel75(const double *X_AB, const double
  x_k_hat[5], const double X_k_hat[55], const int i, double tmp_ego_y[5], double
  work[5]);
static __global__ void BEV_image_kernel76(const double tmp_ego_y[5], const
  double work[5], double L[25]);
static __global__ void BEV_image_kernel77(const double P_ctrv[25], double L[25]);
static __global__ void BEV_image_kernel78(double v[3]);
static __global__ void BEV_image_kernel79(double P_zz[9]);
static __global__ void BEV_image_kernel8(const double x_ctrv_out[5], const
  double x_ini[5], double tmp_ego_y[5], double work[5]);
static __global__ void BEV_image_kernel80(double P_xz[15]);
static __global__ void BEV_image_kernel81(const double X_k[55], const signed
  char a[15], const int i, double Y_k_hat[33]);
static __global__ void BEV_image_kernel82(const double Y_k_hat[33], const int i,
  const double *X_AB, double v[3]);
static __global__ void BEV_image_kernel83(const double *X_AB, const double v[3],
  const double Y_k_hat[33], const int i, double b_Y_k_hat[3], double W[3]);
static __global__ void BEV_image_kernel84(const double Y_k_hat[3], const double
  W[3], double P_zz[9]);
static __global__ void BEV_image_kernel85(const double x_k_hat[5], const double
  X_k_hat[55], const int i, const double *X_AB, double work[5]);
static __global__ void BEV_image_kernel86(const double v[3], const double
  Y_k_hat[33], const int i, double b_Y_k_hat[3]);
static __global__ void BEV_image_kernel87(const double Y_k_hat[3], const double
  work[5], double P_xz[15]);
static __global__ void BEV_image_kernel88(const double R_CTRV_IMM[9], double A[9],
  double P_zz[9]);
static __global__ void BEV_image_kernel89(const int jA, const double P_zz[9],
  const int ix, double A[9]);
static __global__ void BEV_image_kernel9(const double tmp_ego_y[5], const double
  work[5], double b_out_P_cv[25], double Vr[25]);
static __global__ void BEV_image_kernel90(const int jA, const int *nr, double A
  [9]);
static __global__ void BEV_image_kernel91(const int jA, const int ix, double A[9]);
static __global__ void BEV_image_kernel92(const int jA, const int *nr, double A
  [9]);
static __global__ void BEV_image_kernel93(const int jA, const int ix, double A[9]);
static __global__ void BEV_image_kernel94(const int jA, const int *nr, double A
  [9]);
static __global__ void BEV_image_kernel95(const double P_zz[9], double A[9]);
static __global__ void BEV_image_kernel96(int ipiv_t[3], int ipiv[3]);
static __global__ void BEV_image_kernel97(double A[9]);
static __global__ void BEV_image_kernel98(int ipiv[3]);
static __global__ void BEV_image_kernel99(double A[9], double *temp);
static __device__ double atomicOpreal_T(double *address, double value);
static __device__ double b_atomicOpreal_T(double *address, double value);
static __device__ double b_threadGroupReduction(double val, unsigned int lane,
  unsigned int mask);
static __device__ double b_workGroupReduction(double val, unsigned int mask,
  unsigned int numActiveWarps);
static double rt_hypotd_snf(double u0, double u1);
static __device__ double shflDown2(double in1, unsigned int offset, unsigned int
  mask);
static __device__ double threadGroupReduction(double val, unsigned int lane,
  unsigned int mask);
static __device__ double workGroupReduction(double val, unsigned int mask,
  unsigned int numActiveWarps);

// Function Definitions
//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double b_flag[15]
//                double old_flag[15]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel1(double b_flag
  [15], double old_flag[15])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 15) {
    old_flag[k] = b_flag[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Vr[25]
//                const double *Class_B_idx_9
//                const double L[25]
//                const double a
//                double P_ctrv_out[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel10(const double
  Vr[25], const double *Class_B_idx_9, const double L[25], const double a,
  double P_ctrv_out[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    P_ctrv_out[k] = a * L[k] + *Class_B_idx_9 * Vr[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_zz[9]
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel100(const double
  P_zz[9], double A[9])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 9) {
    A[k] = P_zz[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_zz[9]
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel101(const double
  P_zz[9], double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[0] = P_zz[2];
    A[2] = P_zz[0];
    A[3] = P_zz[5];
    A[5] = P_zz[3];
    A[6] = P_zz[8];
    A[8] = P_zz[6];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_zz[9]
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel102(const double
  P_zz[9], double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[0] = P_zz[1];
    A[1] = P_zz[0];
    A[3] = P_zz[4];
    A[4] = P_zz[3];
    A[6] = P_zz[7];
    A[7] = P_zz[6];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel103(double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[1] /= A[0];
    A[2] /= A[0];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel104(double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[4] -= A[1] * A[3];
    A[5] -= A[2] * A[3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *Class_B_idx_9
//                double A[9]
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel105(const double
  *Class_B_idx_9, double A[9], double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB = -A[7] * *Class_B_idx_9 / A[4];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double y_out[3]
//                double Y_k_hat[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel106(const double
  v[3], const double y_out[3], double Y_k_hat[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    Y_k_hat[k] = -0.5 * (y_out[k] - v[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y[9]
//                const double Y_k_hat[3]
//                double dv1[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel107(const double
  y[9], const double Y_k_hat[3], double dv1[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    dv1[k] = (Y_k_hat[0] * y[3 * k] + Y_k_hat[1] * y[3 * k + 1]) + Y_k_hat[2] *
      y[3 * k + 2];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double y_out[3]
//                const double dv1[3]
//                double *d
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel108(const double
  v[3], const double y_out[3], const double dv1[3], double *d)
{
  double tmpRed0;
  unsigned int blockStride;
  unsigned int mask;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0.0;
  if (mwIsLastBlock()) {
    int m;
    m = static_cast<int>(3U % blockStride);
    if (static_cast<unsigned int>(m) > 0U) {
      blockStride = static_cast<unsigned int>(m);
    }
  }

  blockStride = ((blockStride + warpSize) - 1U) / warpSize;
  if (threadId <= 2U) {
    tmpRed0 = dv1[threadId] * (y_out[threadId] - v[threadId]);
  }

  mask = __ballot_sync(MAX_uint32_T, threadId <= 2U);
  for (unsigned int idx{threadId + threadStride}; idx <= 2U; idx += threadStride)
  {
    tmpRed0 += dv1[idx] * (y_out[idx] - v[idx]);
  }

  tmpRed0 = b_workGroupReduction(tmpRed0, mask, blockStride);
  if (thBlkId == 0U) {
    b_atomicOpreal_T(&d[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double x_ctrv[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel109(double
  x_ctrv[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    x_ctrv[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_cv_out[5]
//                const double x_ini[5]
//                double tmp_ego_y[5]
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel11(const double
  x_cv_out[5], const double x_ini[5], double tmp_ego_y[5], double work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    double xvFirst;
    double yvFirst;
    xvFirst = x_ini[k];
    yvFirst = x_cv_out[k];
    work[k] = xvFirst - yvFirst;
    tmp_ego_y[k] = xvFirst - yvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double y_out[3]
//                double W[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel110(const double
  v[3], const double y_out[3], double W[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    W[k] = y_out[k] - v[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_k_hat[5]
//                const double W[3]
//                const double K[15]
//                double x_ctrv[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel111(const double
  x_k_hat[5], const double W[3], const double K[15], double x_ctrv[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    x_ctrv[k] = x_k_hat[k] + ((K[k] * W[0] + K[k + 5] * W[1]) + K[k + 10] * W[2]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_zz[9]
//                const double K[15]
//                double P_xz[15]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel112(const double
  P_zz[9], const double K[15], double P_xz[15])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 3UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 3UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 3))) {
    P_xz[k + 5 * i] = (K[k] * P_zz[3 * i] + K[k + 5] * P_zz[3 * i + 1]) + K[k +
      10] * P_zz[3 * i + 2];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int sample_ts
//                const double K[15]
//                const double P_xz[15]
//                double P_ctrv_tmp[250]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel113(const double
  L[25], const int sample_ts, const double K[15], const double P_xz[15], double
  P_ctrv_tmp[250])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    P_ctrv_tmp[(k + 5 * i) + 25 * sample_ts] = L[k + 5 * i] - ((P_xz[k] * K[i] +
      P_xz[k + 5] * K[i + 5]) + P_xz[k + 10] * K[i + 10]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_ini[5]
//                double x_ctrv[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel114(const double
  x_ini[5], double x_ctrv[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    x_ctrv[k] = x_ini[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double x_ctrv[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel115(double
  x_ctrv[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    x_ctrv[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_cv[25]
//                const int sample_ts
//                double P_cv_tmp[250]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel116(const double
  P_cv[25], const int sample_ts, double P_cv_tmp[250])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    P_cv_tmp[(i + 5 * k) + 25 * sample_ts] = P_cv[i + 5 * k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_cv_out[25]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel117(const double
  P_cv_out[25], double L[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    //  if time >=18.49
    //      time = time;
    //  end
    //  x_k = [y x yaw v yawrate]';
    // -------------------------------------------------------------------------
    //  Parameter
    //  L = real(sqrtm((n_x+KAPPA)*P_cv_k));
    L[k] = 10.0 * P_cv_out[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel118(double work
  [5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int i
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel119(const double
  L[25], const int i, double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB = L[(i + 5 * i) + 1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double tmp_ego_y[5]
//                const double work[5]
//                double b_out_P_ctrv[25]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel12(const double
  tmp_ego_y[5], const double work[5], double b_out_P_ctrv[25], double L[25])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    L[i + 5 * k] = b_out_P_ctrv[i + 5 * k] + work[i] * tmp_ego_y[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int knt
//                double *X_AB
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel120(const int
  knt, double *X_AB)
{
  double tmpRed0;
  long loopEnd;
  unsigned int blockStride;
  unsigned int m;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 1.0;
  loopEnd = static_cast<long>(knt);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(knt) + 1L) % static_cast<long>(blockStride);
    if (m > 0U) {
      blockStride = m;
    }
  }

  blockStride = (unsigned int)(blockStride + (static_cast<long>(warpSize) - 1L))
    / warpSize;
  if (static_cast<long>(threadId) <= loopEnd) {
    tmpRed0 = 1.0020841800044864E-292;
  }

  m = __ballot_sync(MAX_uint32_T, static_cast<long>(threadId) <= loopEnd);
  for (unsigned int idx{threadId + threadStride}; idx <= static_cast<unsigned
       int>(loopEnd); idx += threadStride) {
    tmpRed0 *= 1.0020841800044864E-292;
  }

  tmpRed0 = workGroupReduction(tmpRed0, m, blockStride);
  if (thBlkId == 0U) {
    atomicOpreal_T(&X_AB[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int i
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel121(const int i,
  double L[25])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    L[(i + 5 * i) + 1] = 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel122(const int
  lastc, double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(lastc - 1);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int i;
    i = static_cast<int>(idx);
    work[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int ix
//                const double L[25]
//                const int *b_L
//                const int jA
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel123(const int
  ix, const double L[25], const int *b_L, const int jA, double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(jA - *b_L);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int k;
    k = static_cast<int>(idx);
    work[k] += L[*b_L + k] * L[ix];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel124(const int
  lastc, double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(lastc - 1);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int i;
    i = static_cast<int>(idx);
    work[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int iaii
//                const double L[25]
//                const int lastv
//                const int knt
//                const int DEC_param
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel125(const int
  iaii, const double L[25], const int lastv, const int knt, const int DEC_param,
  double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>((DEC_param - knt) / 5);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    double Class_B_idx_10;
    int b_L;
    int m;
    int nr;
    m = static_cast<int>(idx);
    b_L = knt + m * 5;
    Class_B_idx_10 = 0.0;
    nr = (b_L + lastv) - 1;
    for (int k{0}; k <= nr - b_L; k++) {
      Class_B_idx_10 += L[(b_L + k) - 1] * L[iaii + k];
    }

    work[m] += Class_B_idx_10;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *Y_AB
//                const int i
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel126(const double
  *Y_AB, const int i, double L[25])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    L[(i + 5 * i) + 1] = *Y_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                double Vr[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel127(const double
  L[25], double Vr[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    Vr[k] = L[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Vr[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel128(double Vr[25])
{
  unsigned long threadId;
  int i;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId);
  if (i < 5) {
    Vr[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Vr[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel129(double Vr[25])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Vr[0] = 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_cv_out[5]
//                const double x_ini[5]
//                double tmp_ego_y[5]
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel13(const double
  x_cv_out[5], const double x_ini[5], double tmp_ego_y[5], double work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    double xvFirst;
    double yvFirst;
    xvFirst = x_ini[k];
    yvFirst = x_cv_out[k];
    work[k] = xvFirst - yvFirst;
    tmp_ego_y[k] = xvFirst - yvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel130(double work
  [5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int iaii
//                double Vr[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel131(const int
  iaii, double Vr[25])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Vr[iaii - 6] = 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel132(const int
  lastc, double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(lastc - 1);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int i;
    i = static_cast<int>(idx);
    work[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Vr[25]
//                const int lastv
//                const int iaii
//                const int DEC_param
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel133(const
  double Vr[25], const int lastv, const int iaii, const int DEC_param, double
  work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>((DEC_param - iaii) / 5);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    double Class_B_idx_10;
    int L;
    int m;
    int nr;
    m = static_cast<int>(idx);
    L = iaii + m * 5;
    Class_B_idx_10 = 0.0;
    nr = (L + lastv) - 1;
    for (int k{0}; k <= nr - L; k++) {
      Class_B_idx_10 += Vr[(L + k) - 1] * Vr[(iaii + k) - 6];
    }

    work[m] += Class_B_idx_10;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel134(double v[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    v[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel135(double L[25])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    L[2] = 0.0;
    L[3] = 0.0;
    L[8] = 0.0;
    L[9] = 0.0;
    L[14] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int lastc
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel136(const double
  L[25], const int lastc, double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB = fabs(L[(lastc + 5 * (lastc - 1)) - 1]) + fabs(L[lastc + 5 * lastc]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int lastc
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel137(const double
  L[25], const int lastc, double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB = fabs(L[(lastc + 5 * (lastc - 2)) - 1]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int lastc
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel138(const double
  L[25], const int lastc, double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB += fabs(L[(lastc + 5 * lastc) + 1]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel139(const int
  lastc, double L[25])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    L[lastc + 5 * (lastc - 1)] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double tmp_ego_y[5]
//                const double work[5]
//                double b_out_P_cv[25]
//                double Vr[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel14(const double
  tmp_ego_y[5], const double work[5], double b_out_P_cv[25], double Vr[25])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    Vr[i + 5 * k] = b_out_P_cv[i + 5 * k] + work[i] * tmp_ego_y[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int DEC_param
//                const int *nr
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel140(const
  double L[25], const int DEC_param, const int *nr, double v[3])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(*nr - 1);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int j;
    j = static_cast<int>(idx);
    v[j] = L[j + DEC_param];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double *X_AB
//                double *Class_B_idx_10
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel141(const double
  v[3], const double *X_AB, double *Class_B_idx_10)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Class_B_idx_10 = (*X_AB - v[0]) / *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double a
//                const int *nr
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel142(const
  double a, const int *nr, double v[3])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(*nr - 2);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int k;
    k = static_cast<int>(idx);
    v[k + 1] *= a;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int *nr
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel143(const int *
  nr, double v[3])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(*nr - 2);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int k;
    k = static_cast<int>(idx);
    v[k + 1] *= 9.9792015476736E+291;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                double *X_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel144(const double
  v[3], double *X_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *X_AB = fabs(v[1]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double a
//                const int *nr
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel145(const
  double a, const int *nr, double v[3])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(*nr - 2);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int k;
    k = static_cast<int>(idx);
    v[k + 1] *= a;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int knt
//                double *X_AB
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel146(const int
  knt, double *X_AB)
{
  double tmpRed0;
  long loopEnd;
  unsigned int blockStride;
  unsigned int m;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 1.0;
  loopEnd = static_cast<long>(knt);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(knt) + 1L) % static_cast<long>(blockStride);
    if (m > 0U) {
      blockStride = m;
    }
  }

  blockStride = (unsigned int)(blockStride + (static_cast<long>(warpSize) - 1L))
    / warpSize;
  if (static_cast<long>(threadId) <= loopEnd) {
    tmpRed0 = 1.0020841800044864E-292;
  }

  m = __ballot_sync(MAX_uint32_T, static_cast<long>(threadId) <= loopEnd);
  for (unsigned int idx{threadId + threadStride}; idx <= static_cast<unsigned
       int>(loopEnd); idx += threadStride) {
    tmpRed0 *= 1.0020841800044864E-292;
  }

  tmpRed0 = workGroupReduction(tmpRed0, m, blockStride);
  if (thBlkId == 0U) {
    atomicOpreal_T(&X_AB[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *Y_AB
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel147(const double
  *Y_AB, double v[3])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    v[0] = *Y_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double *Class_B_idx_10
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel148(const double
  v[3], const double *Class_B_idx_10, double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB = *Class_B_idx_10 * v[1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double *Class_B_idx_10
//                double *Class_B_idx_9
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel149(const double
  v[3], const double *Class_B_idx_10, double *Class_B_idx_9)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Class_B_idx_9 = *Class_B_idx_10 * v[2];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Vr[25]
//                const double *Class_B_idx_9
//                const double L[25]
//                const double a
//                double P_cv_out[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel15(const double
  Vr[25], const double *Class_B_idx_9, const double L[25], const double a,
  double P_cv_out[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    P_cv_out[k] = a * L[k] + *Class_B_idx_9 * Vr[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Vr[25]
//                const double L[25]
//                creal_T U[25]
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel150(const double
  Vr[25], const double L[25], creal_T U[25], creal_T L_tmp[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L_tmp[k].re = L[k];
    L_tmp[k].im = 0.0;
    U[k].re = Vr[k];
    U[k].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double *Y_AB
//                const double *Class_B_idx_9
//                creal_T *c
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel151(const double
  *X_AB, const double *Y_AB, const double *Class_B_idx_9, creal_T *c)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    c->re = *Class_B_idx_9 / *Y_AB;
    c->im = *X_AB / *Y_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *Y_AB
//                const double *Class_B_idx_9
//                creal_T *c
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel152(const double
  *Y_AB, const double *Class_B_idx_9, creal_T *c)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    c->re = *Class_B_idx_9 / *Y_AB;
    c->im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *scale
//                const creal_T *c
//                const int ix
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel153(const
  double *scale, const creal_T *c, const int ix, creal_T L_tmp[25])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(ix - 1);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    creal_T t1;
    double cp;
    double yv2;
    int i;
    i = static_cast<int>(idx);
    t1 = L_tmp[i + 5 * (ix - 2)];
    cp = c->re * L_tmp[i + 5 * (ix - 2)].im + c->im * L_tmp[i + 5 * (ix - 2)].re;
    L_tmp[i + 5 * (ix - 2)].re = (c->re * L_tmp[i + 5 * (ix - 2)].re - c->im *
      L_tmp[i + 5 * (ix - 2)].im) + *scale * L_tmp[i + 5 * (ix - 1)].re;
    L_tmp[i + 5 * (ix - 2)].im = cp + *scale * L_tmp[i + 5 * (ix - 1)].im;
    cp = L_tmp[i + 5 * (ix - 1)].re;
    yv2 = L_tmp[i + 5 * (ix - 1)].im;
    L_tmp[i + 5 * (ix - 1)].re = (c->re * cp + c->im * yv2) - *scale * t1.re;
    L_tmp[i + 5 * (ix - 1)].im = (c->re * yv2 - c->im * cp) - *scale * t1.im;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *scale
//                const creal_T *c
//                const int ix
//                creal_T U[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel154(const double
  *scale, const creal_T *c, const int ix, creal_T U[25])
{
  unsigned long threadId;
  int i;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId);
  if (i < 5) {
    creal_T t1;
    double cp;
    double yv2;
    t1 = U[i + 5 * (ix - 2)];
    cp = c->re * U[i + 5 * (ix - 2)].im + c->im * U[i + 5 * (ix - 2)].re;
    U[i + 5 * (ix - 2)].re = (c->re * U[i + 5 * (ix - 2)].re - c->im * U[i + 5 *
      (ix - 2)].im) + *scale * U[i + 5 * (ix - 1)].re;
    U[i + 5 * (ix - 2)].im = cp + *scale * U[i + 5 * (ix - 1)].im;
    cp = U[i + 5 * (ix - 1)].re;
    yv2 = U[i + 5 * (ix - 1)].im;
    U[i + 5 * (ix - 1)].re = (c->re * cp + c->im * yv2) - *scale * t1.re;
    U[i + 5 * (ix - 1)].im = (c->re * yv2 - c->im * cp) - *scale * t1.im;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int ix
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel155(const int ix,
  creal_T L_tmp[25])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    L_tmp[(ix + 5 * (ix - 2)) - 1].re = 0.0;
    L_tmp[(ix + 5 * (ix - 2)) - 1].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                creal_T U[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel156(creal_T U[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    U[k].re = CUDART_NAN;
    U[k].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel157(creal_T
  L_tmp[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L_tmp[k].re = CUDART_NAN;
    L_tmp[k].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                creal_T R[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel158(creal_T R[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    R[k].re = 0.0;
    R[k].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T L_tmp[25]
//                creal_T R[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel159(const
  creal_T L_tmp[25], creal_T R[25])
{
  unsigned long threadId;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  j = static_cast<int>(threadId);
  if (j < 5) {
    double Class_B_idx_10;
    double Class_B_idx_9;
    double X_AB;
    Class_B_idx_9 = L_tmp[j + 5 * j].re;
    Class_B_idx_10 = L_tmp[j + 5 * j].im;
    if (Class_B_idx_10 == 0.0) {
      if (Class_B_idx_9 < 0.0) {
        X_AB = 0.0;
        Class_B_idx_9 = sqrt(-Class_B_idx_9);
      } else {
        X_AB = sqrt(Class_B_idx_9);
        Class_B_idx_9 = 0.0;
      }
    } else if (Class_B_idx_9 == 0.0) {
      if (Class_B_idx_10 < 0.0) {
        X_AB = sqrt(-Class_B_idx_10 / 2.0);
        Class_B_idx_9 = -X_AB;
      } else {
        X_AB = sqrt(Class_B_idx_10 / 2.0);
        Class_B_idx_9 = X_AB;
      }
    } else if (isnan(Class_B_idx_9)) {
      X_AB = Class_B_idx_9;
    } else if (isnan(Class_B_idx_10)) {
      X_AB = Class_B_idx_10;
      Class_B_idx_9 = Class_B_idx_10;
    } else if (isinf(Class_B_idx_10)) {
      X_AB = fabs(Class_B_idx_10);
      Class_B_idx_9 = Class_B_idx_10;
    } else if (isinf(Class_B_idx_9)) {
      if (Class_B_idx_9 < 0.0) {
        X_AB = 0.0;
        Class_B_idx_9 = Class_B_idx_10 * -Class_B_idx_9;
      } else {
        X_AB = Class_B_idx_9;
        Class_B_idx_9 = 0.0;
      }
    } else {
      double Y_AB;
      Y_AB = fabs(Class_B_idx_9);
      X_AB = fabs(Class_B_idx_10);
      if ((static_cast<int>(Y_AB > 4.4942328371557893E+307)) || (static_cast<int>
           (X_AB > 4.4942328371557893E+307))) {
        Y_AB *= 0.5;
        X_AB = hypot(Y_AB, X_AB * 0.5);
        if (X_AB > Y_AB) {
          X_AB = sqrt(X_AB) * sqrt(Y_AB / X_AB + 1.0);
        } else {
          X_AB = sqrt(X_AB) * 1.4142135623730951;
        }
      } else {
        X_AB = sqrt((hypot(Y_AB, X_AB) + Y_AB) * 0.5);
      }

      if (Class_B_idx_9 > 0.0) {
        Class_B_idx_9 = 0.5 * (Class_B_idx_10 / X_AB);
      } else {
        if (Class_B_idx_10 < 0.0) {
          Class_B_idx_9 = -X_AB;
        } else {
          Class_B_idx_9 = X_AB;
        }

        X_AB = 0.5 * (Class_B_idx_10 / Class_B_idx_9);
      }
    }

    R[j + 5 * j].re = X_AB;
    R[j + 5 * j].im = Class_B_idx_9;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_ctrv[25]
//                const int sample_ts
//                double P_ctrv_tmp[250]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel16(const double
  P_ctrv[25], const int sample_ts, double P_ctrv_tmp[250])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    P_ctrv_tmp[(i + 5 * k) + 25 * sample_ts] = P_ctrv[i + 5 * k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T R[25]
//                const creal_T U[25]
//                const int *k
//                creal_T b_U[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel160(const
  creal_T R[25], const creal_T U[25], const int *k, creal_T b_U[25])
{
  unsigned long threadId;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  j = static_cast<int>(threadId);
  if (j < 5) {
    double re;
    double yv2;
    re = 0.0;
    yv2 = 0.0;
    for (int i{0}; i < 5; i++) {
      double U_im;
      double xv2;
      double xvFirst;
      double yvFirst;
      xvFirst = U[*k + 5 * i].re;
      yvFirst = U[*k + 5 * i].im;
      U_im = R[i + 5 * j].re;
      xv2 = R[i + 5 * j].im;
      re += xvFirst * U_im - yvFirst * xv2;
      yv2 += xvFirst * xv2 + yvFirst * U_im;
    }

    b_U[*k + 5 * j].re = re;
    b_U[*k + 5 * j].im = yv2;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T U[25]
//                const creal_T b_U[25]
//                const int *k
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel161(const
  creal_T U[25], const creal_T b_U[25], const int *k, creal_T L_tmp[25])
{
  unsigned long threadId;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  j = static_cast<int>(threadId);
  if (j < 5) {
    double re;
    double yv2;
    re = 0.0;
    yv2 = 0.0;
    for (int i{0}; i < 5; i++) {
      double U_im;
      double cp;
      double xvFirst;
      double yvFirst;
      cp = b_U[j + 5 * i].re;
      U_im = -b_U[j + 5 * i].im;
      xvFirst = U[*k + 5 * i].re;
      yvFirst = U[*k + 5 * i].im;
      re += xvFirst * cp - yvFirst * U_im;
      yv2 += xvFirst * U_im + yvFirst * cp;
    }

    L_tmp[*k + 5 * j].re = re;
    L_tmp[*k + 5 * j].im = yv2;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T L_tmp[25]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel162(const
  creal_T L_tmp[25], double L[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L[k] = L_tmp[k].im;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int j
//                double *scale
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel163(const double
  L[25], const int j, double *scale)
{
  double tmpRed0;
  unsigned int blockStride;
  unsigned int mask;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0.0;
  if (mwIsLastBlock()) {
    int m;
    m = static_cast<int>(5U % blockStride);
    if (static_cast<unsigned int>(m) > 0U) {
      blockStride = static_cast<unsigned int>(m);
    }
  }

  blockStride = ((blockStride + warpSize) - 1U) / warpSize;
  if (threadId <= 4U) {
    tmpRed0 = fabs(L[static_cast<int>(threadId) + 5 * j]);
  }

  mask = __ballot_sync(MAX_uint32_T, threadId <= 4U);
  for (unsigned int idx{threadId + threadStride}; idx <= 4U; idx += threadStride)
  {
    tmpRed0 += fabs(L[static_cast<int>(idx) + 5 * j]);
  }

  tmpRed0 = b_workGroupReduction(tmpRed0, mask, blockStride);
  if (thBlkId == 0U) {
    b_atomicOpreal_T(&scale[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T L_tmp[25]
//                const int j
//                double *scale
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel164(const
  creal_T L_tmp[25], const int j, double *scale)
{
  double tmpRed0;
  unsigned int blockStride;
  unsigned int mask;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0.0;
  if (mwIsLastBlock()) {
    int m;
    m = static_cast<int>(5U % blockStride);
    if (static_cast<unsigned int>(m) > 0U) {
      blockStride = static_cast<unsigned int>(m);
    }
  }

  blockStride = ((blockStride + warpSize) - 1U) / warpSize;
  if (threadId <= 4U) {
    tmpRed0 = hypot(L_tmp[static_cast<int>(threadId) + 5 * j].re, L_tmp[
                    static_cast<int>(threadId) + 5 * j].im);
  }

  mask = __ballot_sync(MAX_uint32_T, threadId <= 4U);
  for (unsigned int idx{threadId + threadStride}; idx <= 4U; idx += threadStride)
  {
    tmpRed0 += hypot(L_tmp[static_cast<int>(idx) + 5 * j].re, L_tmp[static_cast<
                     int>(idx) + 5 * j].im);
  }

  tmpRed0 = b_workGroupReduction(tmpRed0, mask, blockStride);
  if (thBlkId == 0U) {
    b_atomicOpreal_T(&scale[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel165(creal_T
  L_tmp[25])
{
  unsigned long threadId;
  int i;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  j = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(j < 5)) && (static_cast<int>(i < 5))) {
    L_tmp[i + 5 * j].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T L_tmp[25]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel166(const
  creal_T L_tmp[25], double L[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L[k] = L_tmp[k].re;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double X_k[55]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void BEV_image_kernel167(double X_k
  [55])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 55) {
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Sampling
    X_k[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double W[11]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel168(double W[11])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 11) {
    W[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int i
//                const double x_cv_out[5]
//                const int b_i
//                double X_k[55]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel169(const double
  L[25], const int i, const double x_cv_out[5], const int b_i, double X_k[55])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    X_k[k + 5 * b_i] = x_cv_out[k] - L[i + 5 * k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_ctrv_out[25]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel17(const double
  P_ctrv_out[25], double L[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    //  if time >=18.49
    //      time = time;
    //  end
    //  x_k = [y x yaw v yawrate]';
    // -------------------------------------------------------------------------
    //  Parameter
    //  ts = 0.01;
    L[k] = 10.0 * P_ctrv_out[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int i
//                double W[11]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel170(const int i,
  double W[11])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    W[i] = 0.05;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int i
//                const double x_cv_out[5]
//                const int b_i
//                double X_k[55]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel171(const double
  L[25], const int i, const double x_cv_out[5], const int b_i, double X_k[55])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    X_k[k + 5 * b_i] = x_cv_out[k] + L[i + 5 * k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int i
//                double W[11]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel172(const int i,
  double W[11])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    W[i] = 0.05;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_cv_out[5]
//                double X_k[55]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel173(const double
  x_cv_out[5], double X_k[55])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    X_k[k] = x_cv_out[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double W[11]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel174(double W[11])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    W[0] = 0.5;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double x_k_hat[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel175(double
  x_k_hat[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Model prediction
    x_k_hat[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel176(double L[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel177(double work
  [5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    work[4] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *scale
//                const double ts
//                const double *d
//                const double X_k[55]
//                const int i
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel178(const double
  *scale, const double ts, const double *d, const double X_k[55], const int i,
  double work[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    work[0] = X_k[5 * i] + *d * ts * sin(*scale);
    work[1] = X_k[5 * i + 1] + *d * ts * cos(*scale);
    work[2] = *scale;
    work[3] = *d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const int i
//                const double work[5]
//                double x_k_hat[5]
//                double X_k_hat[55]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel179(const double
  *X_AB, const int i, const double work[5], double x_k_hat[5], double X_k_hat[55])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    double xvFirst;
    xvFirst = work[k];
    X_k_hat[k + 5 * i] = xvFirst;
    x_k_hat[k] += *X_AB * xvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel18(double work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double x_k_hat[5]
//                const double X_k_hat[55]
//                const int i
//                double tmp_ego_y[5]
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel180(const double
  *X_AB, const double x_k_hat[5], const double X_k_hat[55], const int i, double
  tmp_ego_y[5], double work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    double xvFirst;
    double yvFirst;
    xvFirst = X_k_hat[k + 5 * i];
    yvFirst = x_k_hat[k];
    work[k] = *X_AB * (xvFirst - yvFirst);
    tmp_ego_y[k] = xvFirst - yvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double tmp_ego_y[5]
//                const double work[5]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel181(const double
  tmp_ego_y[5], const double work[5], double L[25])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    L[i + 5 * k] += work[i] * tmp_ego_y[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_cv[25]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel182(const double
  P_cv[25], double L[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L[k] += P_cv[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel183(double v[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Measurement update
    v[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double P_zz[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel184(double P_zz
  [9])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 9) {
    P_zz[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double P_xz[15]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel185(double P_xz
  [15])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 15) {
    P_xz[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double X_k[55]
//                const signed char a[15]
//                const int i
//                double Y_k_hat[33]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel186(const double
  X_k[55], const signed char a[15], const int i, double Y_k_hat[33])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    double xvFirst;
    xvFirst = 0.0;
    for (int b_i{0}; b_i < 5; b_i++) {
      xvFirst += static_cast<double>(a[k + 3 * b_i]) * X_k[b_i + 5 * i];
    }

    Y_k_hat[k + 3 * i] = xvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Y_k_hat[33]
//                const int i
//                const double *X_AB
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel187(const double
  Y_k_hat[33], const int i, const double *X_AB, double v[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    v[k] += *X_AB * Y_k_hat[k + 3 * i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double v[3]
//                const double Y_k_hat[33]
//                const int i
//                double b_Y_k_hat[3]
//                double W[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel188(const double
  *X_AB, const double v[3], const double Y_k_hat[33], const int i, double
  b_Y_k_hat[3], double W[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    double xvFirst;
    double yvFirst;
    xvFirst = Y_k_hat[k + 3 * i];
    yvFirst = v[k];
    W[k] = *X_AB * (xvFirst - yvFirst);
    b_Y_k_hat[k] = xvFirst - yvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Y_k_hat[3]
//                const double W[3]
//                double P_zz[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel189(const double
  Y_k_hat[3], const double W[3], double P_zz[9])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 3UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 3UL);
  if ((static_cast<int>(k < 3)) && (static_cast<int>(i < 3))) {
    P_zz[i + 3 * k] += W[i] * Y_k_hat[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int knt
//                double *X_AB
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel19(const int
  knt, double *X_AB)
{
  double tmpRed0;
  long loopEnd;
  unsigned int blockStride;
  unsigned int m;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 1.0;
  loopEnd = static_cast<long>(knt);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(knt) + 1L) % static_cast<long>(blockStride);
    if (m > 0U) {
      blockStride = m;
    }
  }

  blockStride = (unsigned int)(blockStride + (static_cast<long>(warpSize) - 1L))
    / warpSize;
  if (static_cast<long>(threadId) <= loopEnd) {
    tmpRed0 = 1.0020841800044864E-292;
  }

  m = __ballot_sync(MAX_uint32_T, static_cast<long>(threadId) <= loopEnd);
  for (unsigned int idx{threadId + threadStride}; idx <= static_cast<unsigned
       int>(loopEnd); idx += threadStride) {
    tmpRed0 *= 1.0020841800044864E-292;
  }

  tmpRed0 = workGroupReduction(tmpRed0, m, blockStride);
  if (thBlkId == 0U) {
    atomicOpreal_T(&X_AB[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_k_hat[5]
//                const double X_k_hat[55]
//                const int i
//                const double *X_AB
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel190(const double
  x_k_hat[5], const double X_k_hat[55], const int i, const double *X_AB, double
  work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = *X_AB * (X_k_hat[k + 5 * i] - x_k_hat[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double Y_k_hat[33]
//                const int i
//                double b_Y_k_hat[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel191(const double
  v[3], const double Y_k_hat[33], const int i, double b_Y_k_hat[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    b_Y_k_hat[k] = Y_k_hat[k + 3 * i] - v[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Y_k_hat[3]
//                const double work[5]
//                double P_xz[15]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel192(const double
  Y_k_hat[3], const double work[5], double P_xz[15])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 3)) && (static_cast<int>(i < 5))) {
    P_xz[i + 5 * k] += work[i] * Y_k_hat[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double R_CTRV_IMM[9]
//                double A[9]
//                double P_zz[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel193(const double
  R_CTRV_IMM[9], double A[9], double P_zz[9])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 9) {
    double xvFirst;
    xvFirst = P_zz[k] + R_CTRV_IMM[k];
    P_zz[k] = xvFirst;
    A[k] = xvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const double P_zz[9]
//                const int ix
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel194(const int jA,
  const double P_zz[9], const int ix, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[ix] = P_zz[ix] / P_zz[jA];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int *nr
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel195(const int jA,
  const int *nr, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[*nr] /= A[jA];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int ix
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel196(const int jA,
  const int ix, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[ix + 3] -= A[ix] * A[jA + 3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int *nr
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel197(const int jA,
  const int *nr, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[*nr + 3] -= A[*nr] * A[jA + 3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int ix
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel198(const int jA,
  const int ix, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[ix + 6] -= A[ix] * A[jA + 6];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int *nr
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel199(const int jA,
  const int *nr, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[*nr + 6] -= A[*nr] * A[jA + 6];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int *k
//                double b_State[2520]
//                double dv[2492]
// Return Type  : void
//
static __global__ __launch_bounds__(96, 1) void BEV_image_kernel2(const int *k,
  double b_State[2520], double dv[2492])
{
  unsigned long threadId;
  int i;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId);
  if (i < 89) {
    dv[i + 89 * *k] = b_State[i + 90 * *k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel20(const int
  lastc, double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(lastc - 1);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int i;
    i = static_cast<int>(idx);
    work[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_zz[9]
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel200(const double
  P_zz[9], double A[9])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 9) {
    A[k] = P_zz[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                int ipiv_t[3]
//                int ipiv[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel201(int ipiv_t[3],
  int ipiv[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    ipiv[k] = ipiv_t[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel202(double A[9])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 9) {
    A[k] = CUDART_NAN;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                int ipiv[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel203(int ipiv[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    ipiv[k] = k + 1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double A[9]
//                double *temp
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel204(double A[9],
  double *temp)
{
  double tmpRed0;
  unsigned int blockStride;
  unsigned int mask;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 1.0;
  if (mwIsLastBlock()) {
    int m;
    m = static_cast<int>(3U % blockStride);
    if (static_cast<unsigned int>(m) > 0U) {
      blockStride = static_cast<unsigned int>(m);
    }
  }

  blockStride = ((blockStride + warpSize) - 1U) / warpSize;
  if (threadId <= 2U) {
    tmpRed0 = A[static_cast<int>(threadId) + 3 * static_cast<int>(threadId)];
  }

  mask = __ballot_sync(MAX_uint32_T, threadId <= 2U);
  for (unsigned int idx{threadId + threadStride}; idx <= 2U; idx += threadStride)
  {
    tmpRed0 *= A[static_cast<int>(idx) + 3 * static_cast<int>(idx)];
  }

  tmpRed0 = workGroupReduction(tmpRed0, mask, blockStride);
  if (thBlkId == 0U) {
    atomicOpreal_T(&temp[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_zz[9]
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel205(const double
  P_zz[9], double A[9])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 9) {
    A[k] = P_zz[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_zz[9]
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel206(const double
  P_zz[9], double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[0] = P_zz[2];
    A[2] = P_zz[0];
    A[3] = P_zz[5];
    A[5] = P_zz[3];
    A[6] = P_zz[8];
    A[8] = P_zz[6];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_zz[9]
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel207(const double
  P_zz[9], double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[0] = P_zz[1];
    A[1] = P_zz[0];
    A[3] = P_zz[4];
    A[4] = P_zz[3];
    A[6] = P_zz[7];
    A[7] = P_zz[6];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel208(double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[1] /= A[0];
    A[2] /= A[0];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel209(double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[4] -= A[1] * A[3];
    A[5] -= A[2] * A[3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int ix
//                const double L[25]
//                const int *b_L
//                const int *nr
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel21(const int
  ix, const double L[25], const int *b_L, const int *nr, double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(*nr - *b_L);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int k;
    k = static_cast<int>(idx);
    work[k] += L[*b_L + k] * L[ix];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *Y_AB
//                const int ix
//                double y[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel210(const double
  *Y_AB, const int ix, double y[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    y[ix + 1] = *Y_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *Class_B_idx_9
//                const int ix
//                double y[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel211(const double
  *Class_B_idx_9, const int ix, double y[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    y[ix + 2] = *Class_B_idx_9;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double y_out[3]
//                double Y_k_hat[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel212(const double
  v[3], const double y_out[3], double Y_k_hat[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    Y_k_hat[k] = -0.5 * (y_out[k] - v[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y[9]
//                const double Y_k_hat[3]
//                double dv1[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel213(const double
  y[9], const double Y_k_hat[3], double dv1[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    dv1[k] = (Y_k_hat[0] * y[3 * k] + Y_k_hat[1] * y[3 * k + 1]) + Y_k_hat[2] *
      y[3 * k + 2];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double y_out[3]
//                const double dv1[3]
//                double *d
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel214(const double
  v[3], const double y_out[3], const double dv1[3], double *d)
{
  double tmpRed0;
  unsigned int blockStride;
  unsigned int mask;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0.0;
  if (mwIsLastBlock()) {
    int m;
    m = static_cast<int>(3U % blockStride);
    if (static_cast<unsigned int>(m) > 0U) {
      blockStride = static_cast<unsigned int>(m);
    }
  }

  blockStride = ((blockStride + warpSize) - 1U) / warpSize;
  if (threadId <= 2U) {
    tmpRed0 = dv1[threadId] * (y_out[threadId] - v[threadId]);
  }

  mask = __ballot_sync(MAX_uint32_T, threadId <= 2U);
  for (unsigned int idx{threadId + threadStride}; idx <= 2U; idx += threadStride)
  {
    tmpRed0 += dv1[idx] * (y_out[idx] - v[idx]);
  }

  tmpRed0 = b_workGroupReduction(tmpRed0, mask, blockStride);
  if (thBlkId == 0U) {
    b_atomicOpreal_T(&d[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel215(double work
  [5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double y_out[3]
//                double W[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel216(const double
  v[3], const double y_out[3], double W[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    W[k] = y_out[k] - v[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_k_hat[5]
//                const double W[3]
//                const double K[15]
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel217(const double
  x_k_hat[5], const double W[3], const double K[15], double work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = x_k_hat[k] + ((K[k] * W[0] + K[k + 5] * W[1]) + K[k + 10] * W[2]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_zz[9]
//                const double K[15]
//                double P_xz[15]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel218(const double
  P_zz[9], const double K[15], double P_xz[15])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 3UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 3UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 3))) {
    P_xz[k + 5 * i] = (K[k] * P_zz[3 * i] + K[k + 5] * P_zz[3 * i + 1]) + K[k +
      10] * P_zz[3 * i + 2];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int sample_ts
//                const double K[15]
//                const double P_xz[15]
//                double P_cv_tmp[250]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel219(const double
  L[25], const int sample_ts, const double K[15], const double P_xz[15], double
  P_cv_tmp[250])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    P_cv_tmp[(k + 5 * i) + 25 * sample_ts] = L[k + 5 * i] - ((P_xz[k] * K[i] +
      P_xz[k + 5] * K[i + 5]) + P_xz[k + 10] * K[i + 10]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel22(const int
  lastc, double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(lastc - 1);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int i;
    i = static_cast<int>(idx);
    work[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_ini[5]
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel220(const double
  x_ini[5], double work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = x_ini[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel221(double work
  [5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double work[5]
//                const double *X_AB
//                const double x_ctrv[5]
//                const double *Y_AB
//                const int sample_ts
//                double X_pred[50]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel222(const double
  work[5], const double *X_AB, const double x_ctrv[5], const double *Y_AB, const
  int sample_ts, double X_pred[50])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    X_pred[k + 5 * sample_ts] = *Y_AB * x_ctrv[k] + *X_AB * work[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_cv_tmp[250]
//                const double P_ctrv_tmp[250]
//                double b_out_P_cv[25]
//                double b_out_P_ctrv[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel223(const double
  P_cv_tmp[250], const double P_ctrv_tmp[250], double b_out_P_cv[25], double
  b_out_P_ctrv[25])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    b_out_P_ctrv[i + 5 * k] = P_ctrv_tmp[i + 5 * k];
    b_out_P_cv[i + 5 * k] = P_cv_tmp[i + 5 * k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double *Y_AB
//                double Training_data_data[28]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel224(const double
  *X_AB, const double *Y_AB, double Training_data_data[28])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Training_data_data[21] = (Training_data_data[5] - *Y_AB) / (*X_AB - *Y_AB);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                double Training_data_data[28]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel225(const double
  *X_AB, double Training_data_data[28])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Training_data_data[19] = *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *Y_AB
//                double Training_data_data[28]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel226(const double
  *Y_AB, double Training_data_data[28])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Training_data_data[20] = *Y_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Training_data_data[28]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel227(double
  Training_data_data[28])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Training_data_data[22] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Training_data_data[28]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel228(double
  Training_data_data[28])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Training_data_data[23] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Training_data_data[28]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel229(double
  Training_data_data[28])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Training_data_data[24] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int iaii
//                const double L[25]
//                const int lastv
//                const int knt
//                const int DEC_param
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel23(const int
  iaii, const double L[25], const int lastv, const int knt, const int DEC_param,
  double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>((DEC_param - knt) / 5);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    double Class_B_idx_10;
    int b_L;
    int m;
    int nr;
    m = static_cast<int>(idx);
    b_L = knt + m * 5;
    Class_B_idx_10 = 0.0;
    nr = (b_L + lastv) - 1;
    for (int k{0}; k <= nr - b_L; k++) {
      Class_B_idx_10 += L[(b_L + k) - 1] * L[iaii + k];
    }

    work[m] += Class_B_idx_10;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Training_data_data[28]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel230(double
  Training_data_data[28])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Training_data_data[25] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Training_data_data[28]
//                double b_State[2520]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel231(const double
  Training_data_data[28], double b_State[2520])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 28) {
    b_State[90 * k] = Training_data_data[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double dv2[61]
//                const double dv1[251]
//                double Ry[15311]
//                double Rx[15311]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel232(const
  double dv2[61], const double dv1[251], double Ry[15311], double Rx[15311])
{
  unsigned long threadId;
  int i;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 61UL);
  j = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 61UL);
  if ((static_cast<int>(j < 251)) && (static_cast<int>(i < 61))) {
    Rx[i + 61 * j] = dv1[j];
    Ry[i + 61 * j] = dv2[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double dv[255]
//                const double *scale
//                double x[255]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void BEV_image_kernel233(const
  double dv[255], const double *scale, double x[255])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 255) {
    x[k] = fabs(*scale - dv[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char b_BEV_image[367464]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel234(unsigned
  char b_BEV_image[367464])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 251UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 251UL);
  if ((static_cast<int>(k < 61)) && (static_cast<int>(i < 251))) {
    unsigned char u;
    u = b_BEV_image[(i + 251 * k) + 15311];
    b_BEV_image[(i + 251 * k) + 61244] = u;
    b_BEV_image[(i + 251 * k) + 107177] = u;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char b_BEV_image[367464]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel235(unsigned
  char b_BEV_image[367464])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 251UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 251UL);
  if ((static_cast<int>(k < 61)) && (static_cast<int>(i < 251))) {
    unsigned char u;
    u = b_BEV_image[(i + 251 * k) + 15311];
    b_BEV_image[(i + 251 * k) + 153110] = u;
    b_BEV_image[(i + 251 * k) + 199043] = u;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char b_BEV_image[367464]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel236(unsigned
  char b_BEV_image[367464])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 251UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 251UL);
  if ((static_cast<int>(k < 61)) && (static_cast<int>(i < 251))) {
    unsigned char u;
    u = b_BEV_image[(i + 251 * k) + 15311];
    b_BEV_image[(i + 251 * k) + 244976] = u;
    b_BEV_image[(i + 251 * k) + 290909] = u;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                unsigned char b_BEV_image[367464]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel237(unsigned
  char b_BEV_image[367464])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 251UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 251UL);
  if ((static_cast<int>(k < 61)) && (static_cast<int>(i < 251))) {
    b_BEV_image[(i + 251 * k) + 336842] = b_BEV_image[(i + 251 * k) + 15311];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Chassis[12]
//                double tmp_ego_y[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel238(const double
  Chassis[12], double tmp_ego_y[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    // && track_number == Traffic_Number
    tmp_ego_y[0] = -Chassis[11] / 2.0;
    tmp_ego_y[1] = -Chassis[11] / 2.0;
    tmp_ego_y[2] = Chassis[11] / 2.0;
    tmp_ego_y[3] = Chassis[11] / 2.0;
    tmp_ego_y[4] = -Chassis[11] / 2.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                bool in_tmp[15311]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel239(bool
  in_tmp[15311])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 15311) {
    in_tmp[k] = false;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                double Vr[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel24(const double
  L[25], double Vr[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    Vr[k] = L[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                int last[5]
//                signed char first[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel240(int last[5],
  signed char first[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    first[k] = static_cast<signed char>(0);
    last[k] = 0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel241(double work
  [5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double tmp_ego_y[5]
//                const double tmp_ego_x[5]
//                const int DEC_param
//                const int jA
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel242(const
  double tmp_ego_y[5], const double tmp_ego_x[5], const int DEC_param, const int
  jA, double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(jA - DEC_param);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    double U_im;
    double cp;
    int i;
    i = static_cast<int>(idx);
    i += DEC_param;
    U_im = fabs(0.5 * (tmp_ego_x[i - 1] + tmp_ego_x[i]));
    cp = fabs(0.5 * (tmp_ego_y[i - 1] + tmp_ego_y[i]));
    if ((static_cast<int>(U_im > 1.0)) && (static_cast<int>(cp > 1.0))) {
      U_im *= cp;
    } else if ((static_cast<int>(cp > U_im)) || (static_cast<int>(isnan(U_im))))
    {
      U_im = cp;
    }

    work[i - 1] = U_im * 6.6613381477509392E-16;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double a
//                const int j
//                const int last[5]
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel243(const double
  a, const int j, const int last[5], double work[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    work[last[j] - 1] = a * 6.6613381477509392E-16;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double work[5]
//                const int last[5]
//                const double tmp_ego_y[5]
//                const double tmp_ego_x[5]
//                const signed char first[5]
//                const int *nr
//                const double *Class_B_idx_10
//                const double *Class_B_idx_9
//                const double *Y_AB
//                const double *X_AB
//                const double Ry[15311]
//                const double Rx[15311]
//                bool in_tmp[15311]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel244(const
  double work[5], const int last[5], const double tmp_ego_y[5], const double
  tmp_ego_x[5], const signed char first[5], const int *nr, const double
  *Class_B_idx_10, const double *Class_B_idx_9, const double *Y_AB, const double
  *X_AB, const double Ry[15311], const double Rx[15311], bool in_tmp[15311])
{
  unsigned long threadId;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  j = static_cast<int>(threadId);
  if (j < 15311) {
    double xj;
    double yj;
    xj = Rx[j];
    yj = Ry[j];
    in_tmp[j] = false;
    if ((static_cast<int>((static_cast<int>((static_cast<int>(xj >= *X_AB)) && (
             static_cast<int>(xj <= *Y_AB)))) && (static_cast<int>(yj >=
            *Class_B_idx_9)))) && (static_cast<int>(yj <= *Class_B_idx_10))) {
      int k;
      signed char sdq;
      sdq = static_cast<signed char>(0);
      k = 1;
      int exitg2;
      do {
        exitg2 = 0;
        if (k <= *nr) {
          double xv2;
          double xvFirst;
          double yv2;
          double yvFirst;
          int exitg1;
          int i;
          signed char dquad;
          signed char quad2;
          signed char quadFirst;
          bool onj;
          xvFirst = tmp_ego_x[static_cast<int>(first[k - 1]) - 1] - xj;
          yvFirst = tmp_ego_y[static_cast<int>(first[k - 1]) - 1] - yj;
          if (xvFirst > 0.0) {
            if (yvFirst > 0.0) {
              quadFirst = static_cast<signed char>(0);
            } else {
              quadFirst = static_cast<signed char>(3);
            }
          } else if (yvFirst > 0.0) {
            quadFirst = static_cast<signed char>(1);
          } else {
            quadFirst = static_cast<signed char>(2);
          }

          xv2 = xvFirst;
          yv2 = yvFirst;
          quad2 = quadFirst;
          i = static_cast<int>(first[k - 1]);
          do {
            exitg1 = 0;
            if (i <= last[k - 1] - 1) {
              double U_im;
              double cp;
              double re;
              re = xv2;
              U_im = yv2;
              xv2 = tmp_ego_x[i] - xj;
              yv2 = tmp_ego_y[i] - yj;
              dquad = quad2;
              if (xv2 > 0.0) {
                if (yv2 > 0.0) {
                  quad2 = static_cast<signed char>(0);
                } else {
                  quad2 = static_cast<signed char>(3);
                }
              } else if (yv2 > 0.0) {
                quad2 = static_cast<signed char>(1);
              } else {
                quad2 = static_cast<signed char>(2);
              }

              onj = false;
              dquad = static_cast<signed char>(static_cast<int>(quad2) -
                static_cast<int>(dquad));
              cp = re * yv2 - xv2 * U_im;
              if (fabs(cp) < work[i - 1]) {
                onj = (re * xv2 + U_im * yv2 <= 0.0);
                if ((static_cast<int>(static_cast<int>(dquad) == 2)) || (
                     static_cast<int>(static_cast<int>(dquad) == -2))) {
                  dquad = static_cast<signed char>(0);
                } else if (static_cast<int>(dquad) == -3) {
                  dquad = static_cast<signed char>(1);
                } else if (static_cast<int>(dquad) == 3) {
                  dquad = static_cast<signed char>(-1);
                }
              } else if (cp < 0.0) {
                if (static_cast<int>(dquad) == 2) {
                  dquad = static_cast<signed char>(-2);
                } else if (static_cast<int>(dquad) == -3) {
                  dquad = static_cast<signed char>(1);
                } else if (static_cast<int>(dquad) == 3) {
                  dquad = static_cast<signed char>(-1);
                }
              } else if (static_cast<int>(dquad) == -2) {
                dquad = static_cast<signed char>(2);
              } else if (static_cast<int>(dquad) == -3) {
                dquad = static_cast<signed char>(1);
              } else if (static_cast<int>(dquad) == 3) {
                dquad = static_cast<signed char>(-1);
              }

              if (onj) {
                in_tmp[j] = true;
                exitg1 = 1;
              } else {
                sdq = static_cast<signed char>(static_cast<int>(sdq) +
                  static_cast<int>(dquad));
                i++;
              }
            } else {
              double cp;
              onj = false;
              dquad = static_cast<signed char>(static_cast<int>(quadFirst) -
                static_cast<int>(quad2));
              cp = xv2 * yvFirst - xvFirst * yv2;
              if (fabs(cp) < work[last[k - 1] - 1]) {
                onj = (xv2 * xvFirst + yv2 * yvFirst <= 0.0);
                if ((static_cast<int>(static_cast<int>(dquad) == 2)) || (
                     static_cast<int>(static_cast<int>(dquad) == -2))) {
                  dquad = static_cast<signed char>(0);
                } else if (static_cast<int>(dquad) == -3) {
                  dquad = static_cast<signed char>(1);
                } else if (static_cast<int>(dquad) == 3) {
                  dquad = static_cast<signed char>(-1);
                }
              } else if (cp < 0.0) {
                if (static_cast<int>(dquad) == 2) {
                  dquad = static_cast<signed char>(-2);
                } else if (static_cast<int>(dquad) == -3) {
                  dquad = static_cast<signed char>(1);
                } else if (static_cast<int>(dquad) == 3) {
                  dquad = static_cast<signed char>(-1);
                }
              } else if (static_cast<int>(dquad) == -2) {
                dquad = static_cast<signed char>(2);
              } else if (static_cast<int>(dquad) == -3) {
                dquad = static_cast<signed char>(1);
              } else if (static_cast<int>(dquad) == 3) {
                dquad = static_cast<signed char>(-1);
              }

              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = 1;
          } else if (onj) {
            in_tmp[j] = true;
            exitg2 = 1;
          } else {
            sdq = static_cast<signed char>(static_cast<int>(sdq) + static_cast<
              int>(dquad));
            k++;
          }
        } else {
          in_tmp[j] = (static_cast<int>(sdq) != 0);
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool in_tmp[15311]
//                bool x[15311]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel245(const bool
  in_tmp[15311], bool x[15311])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 15311) {
    x[k] = in_tmp[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int i_data[15311]
//                const unsigned char j_data[15311]
//                const int ipiv_t
//                const int b_ipiv_t
//                unsigned char b_BEV_image[367464]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel246(const int
  i_data[15311], const unsigned char j_data[15311], const int ipiv_t, const int
  b_ipiv_t, unsigned char b_BEV_image[367464])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = (static_cast<unsigned long>(ipiv_t) + 1UL) * ((static_cast<unsigned
    long>(b_ipiv_t) + 1UL) * 24UL) - 1UL;
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    unsigned long tmpIndex;
    int i;
    int j;
    int k;
    i = static_cast<int>(idx % (static_cast<unsigned long>(ipiv_t) + 1UL));
    tmpIndex = (idx - static_cast<unsigned long>(i)) / (static_cast<unsigned
      long>(ipiv_t) + 1UL);
    j = static_cast<int>(tmpIndex % (static_cast<unsigned long>(b_ipiv_t) + 1UL));
    tmpIndex = (tmpIndex - static_cast<unsigned long>(j)) / (static_cast<
      unsigned long>(b_ipiv_t) + 1UL);
    k = static_cast<int>(tmpIndex);
    b_BEV_image[((static_cast<int>(j_data[i]) + 251 * (i_data[j] - 1)) + 15311 *
                 k) - 1] = MAX_uint8_T;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double tmp_ego_x[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel247(double
  tmp_ego_x[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    tmp_ego_x[3] = 0.0;
    tmp_ego_x[4] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *scale
//                const double *Y_AB
//                const double *X_AB
//                const double *d
//                const double x
//                const double b_x
//                const double tmp_ego_y[5]
//                double target_y[5]
//                double tmp_ego_x[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel248(const double
  *scale, const double *Y_AB, const double *X_AB, const double *d, const double
  x, const double b_x, const double tmp_ego_y[5], double target_y[5], double
  tmp_ego_x[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    double xvFirst;
    double yvFirst;
    xvFirst = tmp_ego_x[k];
    yvFirst = tmp_ego_y[k];
    target_y[k] = (xvFirst * b_x + yvFirst * x) + *d;
    xvFirst = (xvFirst * *X_AB - yvFirst * *Y_AB) + *scale;
    tmp_ego_x[k] = xvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                bool in_tmp[15311]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel249(bool
  in_tmp[15311])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 15311) {
    in_tmp[k] = false;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Vr[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel25(double Vr[25])
{
  unsigned long threadId;
  int i;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId);
  if (i < 5) {
    Vr[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                int last[5]
//                signed char first[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel250(int last[5],
  signed char first[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    first[k] = static_cast<signed char>(0);
    last[k] = 0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel251(double work
  [5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double target_y[5]
//                const double tmp_ego_x[5]
//                const int DEC_param
//                const int jA
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel252(const
  double target_y[5], const double tmp_ego_x[5], const int DEC_param, const int
  jA, double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(jA - DEC_param);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    double U_im;
    double cp;
    int i;
    i = static_cast<int>(idx);
    i += DEC_param;
    U_im = fabs(0.5 * (tmp_ego_x[i - 1] + tmp_ego_x[i]));
    cp = fabs(0.5 * (target_y[i - 1] + target_y[i]));
    if ((static_cast<int>(U_im > 1.0)) && (static_cast<int>(cp > 1.0))) {
      U_im *= cp;
    } else if ((static_cast<int>(cp > U_im)) || (static_cast<int>(isnan(U_im))))
    {
      U_im = cp;
    }

    work[i - 1] = U_im * 6.6613381477509392E-16;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double a
//                const int j
//                const int last[5]
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel253(const double
  a, const int j, const int last[5], double work[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    work[last[j] - 1] = a * 6.6613381477509392E-16;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double work[5]
//                const int last[5]
//                const double target_y[5]
//                const double tmp_ego_x[5]
//                const signed char first[5]
//                const int *nr
//                const double *Class_B_idx_10
//                const double *Class_B_idx_9
//                const double *Y_AB
//                const double *X_AB
//                const double Ry[15311]
//                const double Rx[15311]
//                bool in_tmp[15311]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel254(const
  double work[5], const int last[5], const double target_y[5], const double
  tmp_ego_x[5], const signed char first[5], const int *nr, const double
  *Class_B_idx_10, const double *Class_B_idx_9, const double *Y_AB, const double
  *X_AB, const double Ry[15311], const double Rx[15311], bool in_tmp[15311])
{
  unsigned long threadId;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  j = static_cast<int>(threadId);
  if (j < 15311) {
    double xj;
    double yj;
    xj = Rx[j];
    yj = Ry[j];
    in_tmp[j] = false;
    if ((static_cast<int>((static_cast<int>((static_cast<int>(xj >= *X_AB)) && (
             static_cast<int>(xj <= *Y_AB)))) && (static_cast<int>(yj >=
            *Class_B_idx_9)))) && (static_cast<int>(yj <= *Class_B_idx_10))) {
      int k;
      signed char sdq;
      sdq = static_cast<signed char>(0);
      k = 1;
      int exitg2;
      do {
        exitg2 = 0;
        if (k <= *nr) {
          double xv2;
          double xvFirst;
          double yv2;
          double yvFirst;
          int exitg1;
          int i;
          signed char dquad;
          signed char quad2;
          signed char quadFirst;
          bool onj;
          xvFirst = tmp_ego_x[static_cast<int>(first[k - 1]) - 1] - xj;
          yvFirst = target_y[static_cast<int>(first[k - 1]) - 1] - yj;
          if (xvFirst > 0.0) {
            if (yvFirst > 0.0) {
              quadFirst = static_cast<signed char>(0);
            } else {
              quadFirst = static_cast<signed char>(3);
            }
          } else if (yvFirst > 0.0) {
            quadFirst = static_cast<signed char>(1);
          } else {
            quadFirst = static_cast<signed char>(2);
          }

          xv2 = xvFirst;
          yv2 = yvFirst;
          quad2 = quadFirst;
          i = static_cast<int>(first[k - 1]);
          do {
            exitg1 = 0;
            if (i <= last[k - 1] - 1) {
              double U_im;
              double cp;
              double re;
              re = xv2;
              U_im = yv2;
              xv2 = tmp_ego_x[i] - xj;
              yv2 = target_y[i] - yj;
              dquad = quad2;
              if (xv2 > 0.0) {
                if (yv2 > 0.0) {
                  quad2 = static_cast<signed char>(0);
                } else {
                  quad2 = static_cast<signed char>(3);
                }
              } else if (yv2 > 0.0) {
                quad2 = static_cast<signed char>(1);
              } else {
                quad2 = static_cast<signed char>(2);
              }

              onj = false;
              dquad = static_cast<signed char>(static_cast<int>(quad2) -
                static_cast<int>(dquad));
              cp = re * yv2 - xv2 * U_im;
              if (fabs(cp) < work[i - 1]) {
                onj = (re * xv2 + U_im * yv2 <= 0.0);
                if ((static_cast<int>(static_cast<int>(dquad) == 2)) || (
                     static_cast<int>(static_cast<int>(dquad) == -2))) {
                  dquad = static_cast<signed char>(0);
                } else if (static_cast<int>(dquad) == -3) {
                  dquad = static_cast<signed char>(1);
                } else if (static_cast<int>(dquad) == 3) {
                  dquad = static_cast<signed char>(-1);
                }
              } else if (cp < 0.0) {
                if (static_cast<int>(dquad) == 2) {
                  dquad = static_cast<signed char>(-2);
                } else if (static_cast<int>(dquad) == -3) {
                  dquad = static_cast<signed char>(1);
                } else if (static_cast<int>(dquad) == 3) {
                  dquad = static_cast<signed char>(-1);
                }
              } else if (static_cast<int>(dquad) == -2) {
                dquad = static_cast<signed char>(2);
              } else if (static_cast<int>(dquad) == -3) {
                dquad = static_cast<signed char>(1);
              } else if (static_cast<int>(dquad) == 3) {
                dquad = static_cast<signed char>(-1);
              }

              if (onj) {
                in_tmp[j] = true;
                exitg1 = 1;
              } else {
                sdq = static_cast<signed char>(static_cast<int>(sdq) +
                  static_cast<int>(dquad));
                i++;
              }
            } else {
              double cp;
              onj = false;
              dquad = static_cast<signed char>(static_cast<int>(quadFirst) -
                static_cast<int>(quad2));
              cp = xv2 * yvFirst - xvFirst * yv2;
              if (fabs(cp) < work[last[k - 1] - 1]) {
                onj = (xv2 * xvFirst + yv2 * yvFirst <= 0.0);
                if ((static_cast<int>(static_cast<int>(dquad) == 2)) || (
                     static_cast<int>(static_cast<int>(dquad) == -2))) {
                  dquad = static_cast<signed char>(0);
                } else if (static_cast<int>(dquad) == -3) {
                  dquad = static_cast<signed char>(1);
                } else if (static_cast<int>(dquad) == 3) {
                  dquad = static_cast<signed char>(-1);
                }
              } else if (cp < 0.0) {
                if (static_cast<int>(dquad) == 2) {
                  dquad = static_cast<signed char>(-2);
                } else if (static_cast<int>(dquad) == -3) {
                  dquad = static_cast<signed char>(1);
                } else if (static_cast<int>(dquad) == 3) {
                  dquad = static_cast<signed char>(-1);
                }
              } else if (static_cast<int>(dquad) == -2) {
                dquad = static_cast<signed char>(2);
              } else if (static_cast<int>(dquad) == -3) {
                dquad = static_cast<signed char>(1);
              } else if (static_cast<int>(dquad) == 3) {
                dquad = static_cast<signed char>(-1);
              }

              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = 1;
          } else if (onj) {
            in_tmp[j] = true;
            exitg2 = 1;
          } else {
            sdq = static_cast<signed char>(static_cast<int>(sdq) + static_cast<
              int>(dquad));
            k++;
          }
        } else {
          in_tmp[j] = (static_cast<int>(sdq) != 0);
          exitg2 = 1;
        }
      } while (exitg2 == 0);
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const bool in_tmp[15311]
//                bool x[15311]
// Return Type  : void
//
static __global__ __launch_bounds__(512, 1) void BEV_image_kernel255(const bool
  in_tmp[15311], bool x[15311])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 15311) {
    x[k] = in_tmp[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double dv[255]
//                const double *d
//                double x[255]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void BEV_image_kernel256(const
  double dv[255], const double *d, double x[255])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 255) {
    x[k] = fabs(*d - dv[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double dv[255]
//                const double *d
//                double x[255]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void BEV_image_kernel257(const
  double dv[255], const double *d, double x[255])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 255) {
    x[k] = fabs(*d - dv[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double a
//                const double d2
//                double TV_range_y[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel258(const double
  a, const double d2, double TV_range_y[10])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    TV_range_y[9] = d2;
    TV_range_y[0] = a;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double a
//                double TV_range_y[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel259(const double
  *X_AB, const double a, double TV_range_y[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_y[k + 1] = a + (static_cast<double>(k) + 1.0) * *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Vr[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel26(double Vr[25])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Vr[0] = 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double *Y_AB
//                const double a
//                double TV_range_y[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel260(const double
  *X_AB, const double *Y_AB, const double a, double TV_range_y[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_y[k + 1] = (a + *Y_AB * (static_cast<double>(k) + 1.0)) - *X_AB * (
      static_cast<double>(k) + 1.0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                double TV_range_y[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel261(const double
  *X_AB, double TV_range_y[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_y[k + 1] = (2.0 * (static_cast<double>(k) + 2.0) - 11.0) * *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double TV_range_x[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel262(double
  TV_range_x[10])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    TV_range_x[0] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                double TV_range_x[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel263(const double
  *X_AB, double TV_range_x[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_x[k + 1] = (static_cast<double>(k) + 1.0) * *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *Y_AB
//                double TV_range_x[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel264(const double
  *Y_AB, double TV_range_x[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_x[k + 1] = *Y_AB * (static_cast<double>(k) + 1.0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                double TV_range_x[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel265(const double
  *X_AB, double TV_range_x[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_x[k + 1] = (2.0 * (static_cast<double>(k) + 2.0) - 11.0) * *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double TV_range_x[10]
//                const double TV_range_y[10]
//                const double X_pred[50]
//                double yp4[100]
//                double xp4[100]
//                double yp3[100]
//                double xp3[100]
//                double yp2[100]
//                double xp2[100]
//                double yp1[100]
//                double xp1[100]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void BEV_image_kernel266(const
  double TV_range_x[10], const double TV_range_y[10], const double X_pred[50],
  double yp4[100], double xp4[100], double yp3[100], double xp3[100], double
  yp2[100], double xp2[100], double yp1[100], double xp1[100])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId % 10UL);
  i = static_cast<int>((threadId - static_cast<unsigned long>(k)) / 10UL);
  if ((static_cast<int>(i < 10)) && (static_cast<int>(k < 10))) {
    double X_AB;
    double Y_AB;
    double xvFirst;
    double yvFirst;
    X_AB = X_pred[5 * i];
    Y_AB = X_pred[5 * i + 1];
    xvFirst = X_pred[5 * i + 1];
    xp1[k + 10 * i] = xvFirst;
    yvFirst = TV_range_y[k];
    yp1[k + 10 * i] = yvFirst + X_AB;
    xp2[k + 10 * i] = TV_range_x[9] + xvFirst;
    yp2[k + 10 * i] = yvFirst + X_AB;
    xp3[k + 10 * i] = TV_range_x[k] + Y_AB;
    xvFirst = X_pred[5 * i];
    yp3[k + 10 * i] = TV_range_y[0] + xvFirst;
    xp4[k + 10 * i] = TV_range_x[k] + Y_AB;
    yp4[k + 10 * i] = TV_range_y[9] + xvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double yp4[100]
//                double xp4[100]
//                double yp3[100]
//                double xp3[100]
//                double yp2[100]
//                double xp2[100]
//                double yp1[100]
//                double xp1[100]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void BEV_image_kernel267(double yp4
  [100], double xp4[100], double yp3[100], double xp3[100], double yp2[100],
  double xp2[100], double yp1[100], double xp1[100])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 100) {
    xp1[k] = round(xp1[k] * -5.0 + 126.0);
    yp1[k] = round(yp1[k] * -5.0 + 31.0);
    xp2[k] = round(xp2[k] * -5.0 + 126.0);
    yp2[k] = round(yp2[k] * -5.0 + 31.0);
    xp3[k] = round(xp3[k] * -5.0 + 126.0);
    yp3[k] = round(yp3[k] * -5.0 + 31.0);
    xp4[k] = round(xp4[k] * -5.0 + 126.0);
    yp4[k] = round(yp4[k] * -5.0 + 31.0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double a
//                const double d2
//                double TV_range_y[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel268(const double
  a, const double d2, double TV_range_y[10])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
    TV_range_y[9] = d2;
    TV_range_y[0] = a;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double a
//                double TV_range_y[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel269(const double
  *X_AB, const double a, double TV_range_y[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_y[k + 1] = a + (static_cast<double>(k) + 1.0) * *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel27(double work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double *Y_AB
//                const double a
//                double TV_range_y[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel270(const double
  *X_AB, const double *Y_AB, const double a, double TV_range_y[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_y[k + 1] = (a + *Y_AB * (static_cast<double>(k) + 1.0)) - *X_AB * (
      static_cast<double>(k) + 1.0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                double TV_range_y[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel271(const double
  *X_AB, double TV_range_y[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_y[k + 1] = (2.0 * (static_cast<double>(k) + 2.0) - 11.0) * *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Chassis[12]
//                double TV_range_x[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel272(const double
  Chassis[12], double TV_range_x[10])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    TV_range_x[9] = 0.0;
    TV_range_x[0] = -Chassis[10];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Chassis[12]
//                double *X_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel273(const double
  Chassis[12], double *X_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *X_AB = (0.0 - (-Chassis[10])) / 9.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double old_Prob_cv
//                double TV_range_x[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel274(const double
  *X_AB, const double old_Prob_cv, double TV_range_x[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_x[k + 1] = old_Prob_cv + (static_cast<double>(k) + 1.0) * *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Chassis[12]
//                double *X_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel275(const double
  Chassis[12], double *X_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *X_AB = -Chassis[10] / 9.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double old_Prob_cv
//                double TV_range_x[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel276(const double
  *X_AB, const double old_Prob_cv, double TV_range_x[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_x[k + 1] = old_Prob_cv - *X_AB * (static_cast<double>(k) + 1.0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double TV_range_x[10]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel277(double
  TV_range_x[10])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 8) {
    TV_range_x[k + 1] = (2.0 * (static_cast<double>(k) + 2.0) - 11.0) * 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double TV_range_y[10]
//                const double TV_range_x[10]
//                const double TJ_X[70]
//                const double TJ_Y[70]
//                const int lastc
//                double yp4[100]
//                double xp4[100]
//                double yp3[100]
//                double xp3[100]
//                double yp2[100]
//                double xp2[100]
//                double yp1[100]
//                double xp1[100]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void BEV_image_kernel278(const
  double TV_range_y[10], const double TV_range_x[10], const double TJ_X[70],
  const double TJ_Y[70], const int lastc, double yp4[100], double xp4[100],
  double yp3[100], double xp3[100], double yp2[100], double xp2[100], double
  yp1[100], double xp1[100])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId % 10UL);
  i = static_cast<int>((threadId - static_cast<unsigned long>(k)) / 10UL);
  if ((static_cast<int>(i < 10)) && (static_cast<int>(k < 10))) {
    double X_AB;
    double Y_AB;
    double xvFirst;
    double yvFirst;
    X_AB = TJ_Y[i + 10 * lastc];
    Y_AB = TJ_X[i + 10 * lastc];
    xvFirst = TJ_X[i + 10 * lastc];
    xp1[k + 10 * i] = TV_range_x[0] + xvFirst;
    yvFirst = TV_range_y[k];
    yp1[k + 10 * i] = yvFirst + X_AB;
    xp2[k + 10 * i] = xvFirst;
    yp2[k + 10 * i] = yvFirst + X_AB;
    xp3[k + 10 * i] = TV_range_x[k] + Y_AB;
    xvFirst = TJ_Y[i + 10 * lastc];
    yp3[k + 10 * i] = TV_range_y[0] + xvFirst;
    xp4[k + 10 * i] = TV_range_x[k] + Y_AB;
    yp4[k + 10 * i] = TV_range_y[9] + xvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double yp4[100]
//                double xp4[100]
//                double yp3[100]
//                double xp3[100]
//                double yp2[100]
//                double xp2[100]
//                double yp1[100]
//                double xp1[100]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void BEV_image_kernel279(double yp4
  [100], double xp4[100], double yp3[100], double xp3[100], double yp2[100],
  double xp2[100], double yp1[100], double xp1[100])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 100) {
    xp1[k] = round(xp1[k] * -5.0 + 126.0);
    yp1[k] = round(yp1[k] * -5.0 + 31.0);
    xp2[k] = round(xp2[k] * -5.0 + 126.0);
    yp2[k] = round(yp2[k] * -5.0 + 31.0);
    xp3[k] = round(xp3[k] * -5.0 + 126.0);
    yp3[k] = round(yp3[k] * -5.0 + 31.0);
    xp4[k] = round(xp4[k] * -5.0 + 126.0);
    yp4[k] = round(yp4[k] * -5.0 + 31.0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int iaii
//                double Vr[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel28(const int
  iaii, double Vr[25])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Vr[iaii - 6] = 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double yp2[100]
//                const int j
//                double *X_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel280(const double
  yp2[100], const int j, double *X_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *X_AB = yp2[j];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int j
//                double yp2[100]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel281(const int j,
  double yp2[100])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    yp2[j] = 61.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int j
//                double yp2[100]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel282(const int j,
  double yp2[100])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    yp2[j] = 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double xp3[100]
//                const int j
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel283(const double
  xp3[100], const int j, double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB = xp3[j];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int j
//                double xp3[100]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel284(const int j,
  double xp3[100])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    xp3[j] = 251.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int j
//                double xp3[100]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel285(const int j,
  double xp3[100])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    xp3[j] = 1.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int m
//                const int *nr
//                const int DEC_param
//                unsigned char b_BEV_image[367464]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel286(const int m,
  const int *nr, const int DEC_param, unsigned char b_BEV_image[367464])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    b_BEV_image[(DEC_param + 251 * *nr) + 15311 * (m + k)] = MAX_uint8_T;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int *nr
//                const int DEC_param
//                unsigned char b_BEV_image[367464]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel287(const int jA,
  const int *nr, const int DEC_param, unsigned char b_BEV_image[367464])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    b_BEV_image[(DEC_param + 251 * *nr) + 15311 * (jA + k)] = MAX_uint8_T;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int ix
//                const int *nr
//                const int DEC_param
//                unsigned char b_BEV_image[367464]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel288(const int ix,
  const int *nr, const int DEC_param, unsigned char b_BEV_image[367464])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    b_BEV_image[(DEC_param + 251 * *nr) + 15311 * (ix + k)] = MAX_uint8_T;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int knt
//                const int *nr
//                const int DEC_param
//                unsigned char b_BEV_image[367464]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel289(const int
  knt, const int *nr, const int DEC_param, unsigned char b_BEV_image[367464])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    b_BEV_image[(DEC_param + 251 * *nr) + 15311 * (knt + k)] = MAX_uint8_T;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel29(const int
  lastc, double work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(lastc - 1);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int i;
    i = static_cast<int>(idx);
    work[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                const int *nr
//                signed char first[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel290(const int
  lastc, const int *nr, signed char first[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    first[*nr - 1] = static_cast<signed char>(lastc + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                const int *nr
//                int last[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel291(const int
  lastc, const int *nr, int last[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    last[*nr - 1] = lastc + 1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                const int *nr
//                int last[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel292(const int
  lastc, const int *nr, int last[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    last[*nr - 1] = lastc;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                const int *nr
//                signed char first[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel293(const int
  lastc, const int *nr, signed char first[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    first[*nr - 1] = static_cast<signed char>(lastc + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                const int *nr
//                int last[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel294(const int
  lastc, const int *nr, int last[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    last[*nr - 1] = lastc + 1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int lastc
//                const int *nr
//                int last[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel295(const int
  lastc, const int *nr, int last[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    last[*nr - 1] = lastc;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double laneInfoR_idx_3
//                const double varargin_1[251]
//                const double laneInfoR_idx_2
//                const double b[251]
//                const double laneInfoR_idx_1
//                const double b_b[251]
//                const double laneInfoR_idx_0
//                double tmp_lane_y[251]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void BEV_image_kernel296(const
  double *X_AB, const double laneInfoR_idx_3, const double varargin_1[251],
  const double laneInfoR_idx_2, const double b[251], const double
  laneInfoR_idx_1, const double b_b[251], const double laneInfoR_idx_0, double
  tmp_lane_y[251])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 251) {
    tmp_lane_y[k] = (((laneInfoR_idx_0 * b_b[k] + laneInfoR_idx_1 * b[k]) +
                      laneInfoR_idx_2 * varargin_1[k]) + laneInfoR_idx_3) -
      *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double varargin_1[251]
//                const double *X_AB
//                double y[251]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void BEV_image_kernel297(const
  double varargin_1[251], const double *X_AB, double y[251])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 251) {
    y[k] = fabs(*X_AB - varargin_1[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double varargin_1[61]
//                const double *X_AB
//                double x[61]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void BEV_image_kernel298(const double
  varargin_1[61], const double *X_AB, double x[61])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 61) {
    x[k] = fabs(*X_AB - varargin_1[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double laneInfoL_idx_3
//                const double varargin_1[251]
//                const double laneInfoL_idx_2
//                const double b[251]
//                const double ts
//                const double b_b[251]
//                const double *temp
//                double tmp_lane_y[251]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void BEV_image_kernel299(const
  double *X_AB, const double laneInfoL_idx_3, const double varargin_1[251],
  const double laneInfoL_idx_2, const double b[251], const double ts, const
  double b_b[251], const double *temp, double tmp_lane_y[251])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 251) {
    tmp_lane_y[k] = (((*temp * b_b[k] + ts * b[k]) + laneInfoL_idx_2 *
                      varargin_1[k]) + laneInfoL_idx_3) + *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double dv[2492]
//                const int *k
//                double b_State[2520]
// Return Type  : void
//
static __global__ __launch_bounds__(96, 1) void BEV_image_kernel3(const double
  dv[2492], const int *k, double b_State[2520])
{
  unsigned long threadId;
  int i;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId);
  if (i < 89) {
    b_State[(i + 90 * *k) + 1] = dv[i + 89 * *k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Vr[25]
//                const int lastv
//                const int iaii
//                const int DEC_param
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel30(const
  double Vr[25], const int lastv, const int iaii, const int DEC_param, double
  work[5])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>((DEC_param - iaii) / 5);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    double Class_B_idx_10;
    int L;
    int m;
    int nr;
    m = static_cast<int>(idx);
    L = iaii + m * 5;
    Class_B_idx_10 = 0.0;
    nr = (L + lastv) - 1;
    for (int k{0}; k <= nr - L; k++) {
      Class_B_idx_10 += Vr[(L + k) - 1] * Vr[(iaii + k) - 6];
    }

    work[m] += Class_B_idx_10;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double varargin_1[251]
//                const double *X_AB
//                double y[251]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void BEV_image_kernel300(const
  double varargin_1[251], const double *X_AB, double y[251])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 251) {
    y[k] = fabs(*X_AB - varargin_1[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double varargin_1[61]
//                const double *X_AB
//                double x[61]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void BEV_image_kernel301(const double
  varargin_1[61], const double *X_AB, double x[61])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 61) {
    x[k] = fabs(*X_AB - varargin_1[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel31(double v[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    v[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int lastc
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel32(const double
  L[25], const int lastc, double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB = fabs(L[(lastc + 5 * (lastc - 1)) - 1]) + fabs(L[lastc + 5 * lastc]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int lastc
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel33(const double
  L[25], const int lastc, double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB = fabs(L[(lastc + 5 * (lastc - 2)) - 1]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int lastc
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel34(const double
  L[25], const int lastc, double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB += fabs(L[(lastc + 5 * lastc) + 1]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int DEC_param
//                const int *nr
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel35(const
  double L[25], const int DEC_param, const int *nr, double v[3])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(*nr - 1);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int j;
    j = static_cast<int>(idx);
    v[j] = L[j + DEC_param];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double *X_AB
//                double *Class_B_idx_10
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel36(const double
  v[3], const double *X_AB, double *Class_B_idx_10)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Class_B_idx_10 = (*X_AB - v[0]) / *X_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double a
//                const int *nr
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel37(const
  double a, const int *nr, double v[3])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(*nr - 2);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int k;
    k = static_cast<int>(idx);
    v[k + 1] *= a;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int *nr
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel38(const int
  *nr, double v[3])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(*nr - 2);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int k;
    k = static_cast<int>(idx);
    v[k + 1] *= 9.9792015476736E+291;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                double *X_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel39(const double
  v[3], double *X_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *X_AB = fabs(v[1]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int DEC_param
//                const double *X_AB
//                double TJ_X[70]
//                double TJ_Y[70]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel4(const int
  DEC_param, const double *X_AB, double TJ_X[70], double TJ_Y[70])
{
  unsigned long threadId;
  int i;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId);
  if (i < 10) {
    double U_im;
    double cp;
    double xv2;
    double xvFirst;
    double yvFirst;

    //  ACC Y
    //  DEC Y
    //  ESL X
    //  ESL Y
    //  ELCL X
    //  ELCL Y
    //  ESS X
    //  ESS Y
    //  ACC Y
    cp = (static_cast<double>(i) + 1.0) * 0.2;

    // ESL
    xvFirst = 0.14323944878270578 * sin(3.2360431875928319 * cp) -
      0.46352904242782755 * cp;
    TJ_Y[i + 20] = xvFirst;
    yvFirst = *X_AB * cp + 0.5 * static_cast<double>(DEC_param) * (cp * cp);
    TJ_X[i + 20] = yvFirst;

    // ESR
    U_im = 0.55704230082163364 * sin(1.6409734010875878 * cp) -
      0.91409159892893144 * cp;
    TJ_Y[i + 40] = U_im;
    xv2 = *X_AB * cp + 0.5 * static_cast<double>(DEC_param) * (cp * cp);
    TJ_X[i + 40] = xv2;
    TJ_X[i] = *X_AB * cp + 2.0 * (cp * cp);
    TJ_X[i + 10] = *X_AB * cp + -5.0 * (cp * cp);
    TJ_X[i + 30] = yvFirst;
    TJ_Y[i + 30] = -xvFirst;
    TJ_X[i + 50] = xv2;
    TJ_Y[i + 50] = -U_im;
    TJ_X[i + 60] = xv2;
    TJ_Y[i + 60] = U_im;
    TJ_Y[i] = 0.0;
    TJ_Y[i + 10] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double a
//                const int *nr
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel40(const
  double a, const int *nr, double v[3])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(*nr - 2);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    int k;
    k = static_cast<int>(idx);
    v[k + 1] *= a;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int knt
//                double *X_AB
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel41(const int
  knt, double *X_AB)
{
  double tmpRed0;
  long loopEnd;
  unsigned int blockStride;
  unsigned int m;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 1.0;
  loopEnd = static_cast<long>(knt);
  if (mwIsLastBlock()) {
    m = (static_cast<long>(knt) + 1L) % static_cast<long>(blockStride);
    if (m > 0U) {
      blockStride = m;
    }
  }

  blockStride = (unsigned int)(blockStride + (static_cast<long>(warpSize) - 1L))
    / warpSize;
  if (static_cast<long>(threadId) <= loopEnd) {
    tmpRed0 = 1.0020841800044864E-292;
  }

  m = __ballot_sync(MAX_uint32_T, static_cast<long>(threadId) <= loopEnd);
  for (unsigned int idx{threadId + threadStride}; idx <= static_cast<unsigned
       int>(loopEnd); idx += threadStride) {
    tmpRed0 *= 1.0020841800044864E-292;
  }

  tmpRed0 = workGroupReduction(tmpRed0, m, blockStride);
  if (thBlkId == 0U) {
    atomicOpreal_T(&X_AB[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *Y_AB
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel42(const double *
  Y_AB, double v[3])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    v[0] = *Y_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double *Class_B_idx_10
//                double *Y_AB
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel43(const double
  v[3], const double *Class_B_idx_10, double *Y_AB)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Y_AB = *Class_B_idx_10 * v[1];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double *Class_B_idx_10
//                double *Class_B_idx_9
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel44(const double
  v[3], const double *Class_B_idx_10, double *Class_B_idx_9)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    *Class_B_idx_9 = *Class_B_idx_10 * v[2];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Vr[25]
//                const double L[25]
//                creal_T U[25]
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel45(const double
  Vr[25], const double L[25], creal_T U[25], creal_T L_tmp[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L_tmp[k].re = L[k];
    L_tmp[k].im = 0.0;
    U[k].re = Vr[k];
    U[k].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double *Y_AB
//                const double *Class_B_idx_9
//                creal_T *c
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel46(const double *
  X_AB, const double *Y_AB, const double *Class_B_idx_9, creal_T *c)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    c->re = *Class_B_idx_9 / *Y_AB;
    c->im = *X_AB / *Y_AB;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *Y_AB
//                const double *Class_B_idx_9
//                creal_T *c
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel47(const double *
  Y_AB, const double *Class_B_idx_9, creal_T *c)
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    c->re = *Class_B_idx_9 / *Y_AB;
    c->im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *scale
//                const creal_T *c
//                const int ix
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(1024, 1) void BEV_image_kernel48(const
  double *scale, const creal_T *c, const int ix, creal_T L_tmp[25])
{
  unsigned long loopEnd;
  unsigned long threadId;
  unsigned long threadStride;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  threadStride = mwGetTotalThreadsLaunched();
  loopEnd = static_cast<unsigned long>(ix - 1);
  for (unsigned long idx{threadId}; idx <= loopEnd; idx += threadStride) {
    creal_T t1;
    double cp;
    double yv2;
    int i;
    i = static_cast<int>(idx);
    t1 = L_tmp[i + 5 * (ix - 2)];
    cp = c->re * L_tmp[i + 5 * (ix - 2)].im + c->im * L_tmp[i + 5 * (ix - 2)].re;
    L_tmp[i + 5 * (ix - 2)].re = (c->re * L_tmp[i + 5 * (ix - 2)].re - c->im *
      L_tmp[i + 5 * (ix - 2)].im) + *scale * L_tmp[i + 5 * (ix - 1)].re;
    L_tmp[i + 5 * (ix - 2)].im = cp + *scale * L_tmp[i + 5 * (ix - 1)].im;
    cp = L_tmp[i + 5 * (ix - 1)].re;
    yv2 = L_tmp[i + 5 * (ix - 1)].im;
    L_tmp[i + 5 * (ix - 1)].re = (c->re * cp + c->im * yv2) - *scale * t1.re;
    L_tmp[i + 5 * (ix - 1)].im = (c->re * yv2 - c->im * cp) - *scale * t1.im;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *scale
//                const creal_T *c
//                const int ix
//                creal_T U[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel49(const double *
  scale, const creal_T *c, const int ix, creal_T U[25])
{
  unsigned long threadId;
  int i;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId);
  if (i < 5) {
    creal_T t1;
    double cp;
    double yv2;
    t1 = U[i + 5 * (ix - 2)];
    cp = c->re * U[i + 5 * (ix - 2)].im + c->im * U[i + 5 * (ix - 2)].re;
    U[i + 5 * (ix - 2)].re = (c->re * U[i + 5 * (ix - 2)].re - c->im * U[i + 5 *
      (ix - 2)].im) + *scale * U[i + 5 * (ix - 1)].re;
    U[i + 5 * (ix - 2)].im = cp + *scale * U[i + 5 * (ix - 1)].im;
    cp = U[i + 5 * (ix - 1)].re;
    yv2 = U[i + 5 * (ix - 1)].im;
    U[i + 5 * (ix - 1)].re = (c->re * cp + c->im * yv2) - *scale * t1.re;
    U[i + 5 * (ix - 1)].im = (c->re * yv2 - c->im * cp) - *scale * t1.im;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *temp
//                double x_ini[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel5(const double
  *temp, double x_ini[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    x_ini[3] = -*temp;
    x_ini[4] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int ix
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel50(const int ix,
  creal_T L_tmp[25])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    L_tmp[(ix + 5 * (ix - 2)) - 1].re = 0.0;
    L_tmp[(ix + 5 * (ix - 2)) - 1].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                creal_T U[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel51(creal_T U[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    U[k].re = CUDART_NAN;
    U[k].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel52(creal_T
  L_tmp[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L_tmp[k].re = CUDART_NAN;
    L_tmp[k].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                creal_T R[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel53(creal_T R[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    R[k].re = 0.0;
    R[k].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T L_tmp[25]
//                creal_T R[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel54(const creal_T
  L_tmp[25], creal_T R[25])
{
  unsigned long threadId;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  j = static_cast<int>(threadId);
  if (j < 5) {
    double Class_B_idx_10;
    double Class_B_idx_9;
    double X_AB;
    Class_B_idx_9 = L_tmp[j + 5 * j].re;
    Class_B_idx_10 = L_tmp[j + 5 * j].im;
    if (Class_B_idx_10 == 0.0) {
      if (Class_B_idx_9 < 0.0) {
        X_AB = 0.0;
        Class_B_idx_9 = sqrt(-Class_B_idx_9);
      } else {
        X_AB = sqrt(Class_B_idx_9);
        Class_B_idx_9 = 0.0;
      }
    } else if (Class_B_idx_9 == 0.0) {
      if (Class_B_idx_10 < 0.0) {
        X_AB = sqrt(-Class_B_idx_10 / 2.0);
        Class_B_idx_9 = -X_AB;
      } else {
        X_AB = sqrt(Class_B_idx_10 / 2.0);
        Class_B_idx_9 = X_AB;
      }
    } else if (isnan(Class_B_idx_9)) {
      X_AB = Class_B_idx_9;
    } else if (isnan(Class_B_idx_10)) {
      X_AB = Class_B_idx_10;
      Class_B_idx_9 = Class_B_idx_10;
    } else if (isinf(Class_B_idx_10)) {
      X_AB = fabs(Class_B_idx_10);
      Class_B_idx_9 = Class_B_idx_10;
    } else if (isinf(Class_B_idx_9)) {
      if (Class_B_idx_9 < 0.0) {
        X_AB = 0.0;
        Class_B_idx_9 = Class_B_idx_10 * -Class_B_idx_9;
      } else {
        X_AB = Class_B_idx_9;
        Class_B_idx_9 = 0.0;
      }
    } else {
      double Y_AB;
      Y_AB = fabs(Class_B_idx_9);
      X_AB = fabs(Class_B_idx_10);
      if ((static_cast<int>(Y_AB > 4.4942328371557893E+307)) || (static_cast<int>
           (X_AB > 4.4942328371557893E+307))) {
        Y_AB *= 0.5;
        X_AB = hypot(Y_AB, X_AB * 0.5);
        if (X_AB > Y_AB) {
          X_AB = sqrt(X_AB) * sqrt(Y_AB / X_AB + 1.0);
        } else {
          X_AB = sqrt(X_AB) * 1.4142135623730951;
        }
      } else {
        X_AB = sqrt((hypot(Y_AB, X_AB) + Y_AB) * 0.5);
      }

      if (Class_B_idx_9 > 0.0) {
        Class_B_idx_9 = 0.5 * (Class_B_idx_10 / X_AB);
      } else {
        if (Class_B_idx_10 < 0.0) {
          Class_B_idx_9 = -X_AB;
        } else {
          Class_B_idx_9 = X_AB;
        }

        X_AB = 0.5 * (Class_B_idx_10 / Class_B_idx_9);
      }
    }

    R[j + 5 * j].re = X_AB;
    R[j + 5 * j].im = Class_B_idx_9;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T R[25]
//                const creal_T U[25]
//                const int *k
//                creal_T b_U[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel55(const creal_T
  R[25], const creal_T U[25], const int *k, creal_T b_U[25])
{
  unsigned long threadId;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  j = static_cast<int>(threadId);
  if (j < 5) {
    double re;
    double yv2;
    re = 0.0;
    yv2 = 0.0;
    for (int i{0}; i < 5; i++) {
      double U_im;
      double xv2;
      double xvFirst;
      double yvFirst;
      xvFirst = U[*k + 5 * i].re;
      yvFirst = U[*k + 5 * i].im;
      U_im = R[i + 5 * j].re;
      xv2 = R[i + 5 * j].im;
      re += xvFirst * U_im - yvFirst * xv2;
      yv2 += xvFirst * xv2 + yvFirst * U_im;
    }

    b_U[*k + 5 * j].re = re;
    b_U[*k + 5 * j].im = yv2;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T U[25]
//                const creal_T b_U[25]
//                const int *k
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel56(const creal_T
  U[25], const creal_T b_U[25], const int *k, creal_T L_tmp[25])
{
  unsigned long threadId;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  j = static_cast<int>(threadId);
  if (j < 5) {
    double re;
    double yv2;
    re = 0.0;
    yv2 = 0.0;
    for (int i{0}; i < 5; i++) {
      double U_im;
      double cp;
      double xvFirst;
      double yvFirst;
      cp = b_U[j + 5 * i].re;
      U_im = -b_U[j + 5 * i].im;
      xvFirst = U[*k + 5 * i].re;
      yvFirst = U[*k + 5 * i].im;
      re += xvFirst * cp - yvFirst * U_im;
      yv2 += xvFirst * U_im + yvFirst * cp;
    }

    L_tmp[*k + 5 * j].re = re;
    L_tmp[*k + 5 * j].im = yv2;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T L_tmp[25]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel57(const creal_T
  L_tmp[25], double L[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L[k] = L_tmp[k].im;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int j
//                double *scale
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel58(const double
  L[25], const int j, double *scale)
{
  double tmpRed0;
  unsigned int blockStride;
  unsigned int mask;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0.0;
  if (mwIsLastBlock()) {
    int m;
    m = static_cast<int>(5U % blockStride);
    if (static_cast<unsigned int>(m) > 0U) {
      blockStride = static_cast<unsigned int>(m);
    }
  }

  blockStride = ((blockStride + warpSize) - 1U) / warpSize;
  if (threadId <= 4U) {
    tmpRed0 = fabs(L[static_cast<int>(threadId) + 5 * j]);
  }

  mask = __ballot_sync(MAX_uint32_T, threadId <= 4U);
  for (unsigned int idx{threadId + threadStride}; idx <= 4U; idx += threadStride)
  {
    tmpRed0 += fabs(L[static_cast<int>(idx) + 5 * j]);
  }

  tmpRed0 = b_workGroupReduction(tmpRed0, mask, blockStride);
  if (thBlkId == 0U) {
    b_atomicOpreal_T(&scale[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T L_tmp[25]
//                const int j
//                double *scale
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel59(const creal_T
  L_tmp[25], const int j, double *scale)
{
  double tmpRed0;
  unsigned int blockStride;
  unsigned int mask;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 0.0;
  if (mwIsLastBlock()) {
    int m;
    m = static_cast<int>(5U % blockStride);
    if (static_cast<unsigned int>(m) > 0U) {
      blockStride = static_cast<unsigned int>(m);
    }
  }

  blockStride = ((blockStride + warpSize) - 1U) / warpSize;
  if (threadId <= 4U) {
    tmpRed0 = hypot(L_tmp[static_cast<int>(threadId) + 5 * j].re, L_tmp[
                    static_cast<int>(threadId) + 5 * j].im);
  }

  mask = __ballot_sync(MAX_uint32_T, threadId <= 4U);
  for (unsigned int idx{threadId + threadStride}; idx <= 4U; idx += threadStride)
  {
    tmpRed0 += hypot(L_tmp[static_cast<int>(idx) + 5 * j].re, L_tmp[static_cast<
                     int>(idx) + 5 * j].im);
  }

  tmpRed0 = b_workGroupReduction(tmpRed0, mask, blockStride);
  if (thBlkId == 0U) {
    b_atomicOpreal_T(&scale[0], tmpRed0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double laneInfoL_idx_4
//                const double old_Prob_cv
//                const double old_Prob_ctrv
//                const double laneInfoL_idx_3
//                const double x_ini[5]
//                double tmp_ego_y[5]
//                double work[5]
//                double x_cv_out[5]
//                double x_ctrv_out[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel6(const double
  laneInfoL_idx_4, const double old_Prob_cv, const double old_Prob_ctrv, const
  double laneInfoL_idx_3, const double x_ini[5], double tmp_ego_y[5], double
  work[5], double x_cv_out[5], double x_ctrv_out[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    double xvFirst;
    double yvFirst;
    xvFirst = x_ini[k];
    yvFirst = xvFirst / laneInfoL_idx_3 * 0.9 * old_Prob_ctrv + xvFirst /
      laneInfoL_idx_3 * 0.1 * old_Prob_cv;
    x_ctrv_out[k] = yvFirst;
    x_cv_out[k] = xvFirst / laneInfoL_idx_4 * 0.1 * old_Prob_ctrv + xvFirst /
      laneInfoL_idx_4 * 0.9 * old_Prob_cv;
    work[k] = xvFirst - yvFirst;
    tmp_ego_y[k] = xvFirst - yvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                creal_T L_tmp[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel60(creal_T
  L_tmp[25])
{
  unsigned long threadId;
  int i;
  int j;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  j = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(j < 5)) && (static_cast<int>(i < 5))) {
    L_tmp[i + 5 * j].im = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const creal_T L_tmp[25]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel61(const creal_T
  L_tmp[25], double L[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L[k] = L_tmp[k].re;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double X_k[55]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void BEV_image_kernel62(double X_k[55])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 55) {
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Sampling
    X_k[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double W[11]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel63(double W[11])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 11) {
    W[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int i
//                const double x_ctrv_out[5]
//                const int b_i
//                double X_k[55]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel64(const double
  L[25], const int i, const double x_ctrv_out[5], const int b_i, double X_k[55])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    X_k[k + 5 * b_i] = x_ctrv_out[k] - L[i + 5 * k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int i
//                double W[11]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel65(const int i,
  double W[11])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    W[i] = 0.05;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double L[25]
//                const int i
//                const double x_ctrv_out[5]
//                const int b_i
//                double X_k[55]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel66(const double
  L[25], const int i, const double x_ctrv_out[5], const int b_i, double X_k[55])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    X_k[k + 5 * b_i] = x_ctrv_out[k] + L[i + 5 * k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int i
//                double W[11]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel67(const int i,
  double W[11])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    W[i] = 0.05;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_ctrv_out[5]
//                double X_k[55]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel68(const double
  x_ctrv_out[5], double X_k[55])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    X_k[k] = x_ctrv_out[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double W[11]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel69(double W[11])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    W[0] = 0.5;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double tmp_ego_y[5]
//                const double work[5]
//                double b_out_P_ctrv[25]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel7(const double
  tmp_ego_y[5], const double work[5], double b_out_P_ctrv[25], double L[25])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    L[i + 5 * k] = b_out_P_ctrv[i + 5 * k] + work[i] * tmp_ego_y[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double x_k_hat[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel70(double
  x_k_hat[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Model prediction
    x_k_hat[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel71(double L[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *d
//                const double X_k[55]
//                const int i
//                double X_hat[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel72(const double *
  d, const double X_k[55], const int i, double X_hat[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    //  x_kk(1) = x_k(1) + x_k(4)/x_k(5)*(-cos(x_k(5)*ts+x_k(3)) + cos(x_k(3)));
    //  x_kk(2) = x_k(2) + x_k(4)/x_k(5)*(-sin(x_k(3)) + sin(x_k(5)*ts+x_k(3)));
    //  x_kk(3) = x_k(3) + x_k(5)*ts;
    //  x_kk(4) = x_k(4);
    //  x_kk(5) = -sign(x_k(3))*10^(-1);
    X_hat[0] = X_k[5 * i];
    X_hat[1] = X_k[5 * i + 1];
    X_hat[2] = X_k[5 * i + 2];
    X_hat[3] = X_k[5 * i + 3];
    X_hat[4] = *d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *scale
//                const double ts
//                const double *d
//                const double old_Prob_ctrv
//                const double X_k[55]
//                const int i
//                double X_hat[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel73(const double *
  scale, const double ts, const double *d, const double old_Prob_ctrv, const
  double X_k[55], const int i, double X_hat[5])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    X_hat[0] = X_k[5 * i] + old_Prob_ctrv / *d * (-cos(*d * ts + *scale) + cos
      (*scale));
    X_hat[1] = X_k[5 * i + 1] + old_Prob_ctrv / *d * (-sin(*scale) + sin(*d * ts
      + *scale));
    X_hat[2] = *scale + *d * ts;
    X_hat[3] = old_Prob_ctrv;
    X_hat[4] = *d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const int i
//                const double X_hat[5]
//                double x_k_hat[5]
//                double X_k_hat[55]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel74(const double *
  X_AB, const int i, const double X_hat[5], double x_k_hat[5], double X_k_hat[55])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    double xvFirst;
    xvFirst = X_hat[k];
    X_k_hat[k + 5 * i] = xvFirst;
    x_k_hat[k] += *X_AB * xvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double x_k_hat[5]
//                const double X_k_hat[55]
//                const int i
//                double tmp_ego_y[5]
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel75(const double *
  X_AB, const double x_k_hat[5], const double X_k_hat[55], const int i, double
  tmp_ego_y[5], double work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    double xvFirst;
    double yvFirst;
    xvFirst = X_k_hat[k + 5 * i];
    yvFirst = x_k_hat[k];
    work[k] = *X_AB * (xvFirst - yvFirst);
    tmp_ego_y[k] = xvFirst - yvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double tmp_ego_y[5]
//                const double work[5]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel76(const double
  tmp_ego_y[5], const double work[5], double L[25])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    L[i + 5 * k] += work[i] * tmp_ego_y[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_ctrv[25]
//                double L[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel77(const double
  P_ctrv[25], double L[25])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 25) {
    L[k] += P_ctrv[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel78(double v[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Measurement update
    v[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double P_zz[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel79(double P_zz[9])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 9) {
    P_zz[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_ctrv_out[5]
//                const double x_ini[5]
//                double tmp_ego_y[5]
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel8(const double
  x_ctrv_out[5], const double x_ini[5], double tmp_ego_y[5], double work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    double xvFirst;
    double yvFirst;
    xvFirst = x_ini[k];
    yvFirst = x_ctrv_out[k];
    work[k] = xvFirst - yvFirst;
    tmp_ego_y[k] = xvFirst - yvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double P_xz[15]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel80(double P_xz
  [15])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 15) {
    P_xz[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double X_k[55]
//                const signed char a[15]
//                const int i
//                double Y_k_hat[33]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel81(const double
  X_k[55], const signed char a[15], const int i, double Y_k_hat[33])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    double xvFirst;
    xvFirst = 0.0;
    for (int b_i{0}; b_i < 5; b_i++) {
      xvFirst += static_cast<double>(a[k + 3 * b_i]) * X_k[b_i + 5 * i];
    }

    Y_k_hat[k + 3 * i] = xvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Y_k_hat[33]
//                const int i
//                const double *X_AB
//                double v[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel82(const double
  Y_k_hat[33], const int i, const double *X_AB, double v[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    v[k] += *X_AB * Y_k_hat[k + 3 * i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double *X_AB
//                const double v[3]
//                const double Y_k_hat[33]
//                const int i
//                double b_Y_k_hat[3]
//                double W[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel83(const double *
  X_AB, const double v[3], const double Y_k_hat[33], const int i, double
  b_Y_k_hat[3], double W[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    double xvFirst;
    double yvFirst;
    xvFirst = Y_k_hat[k + 3 * i];
    yvFirst = v[k];
    W[k] = *X_AB * (xvFirst - yvFirst);
    b_Y_k_hat[k] = xvFirst - yvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Y_k_hat[3]
//                const double W[3]
//                double P_zz[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel84(const double
  Y_k_hat[3], const double W[3], double P_zz[9])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 3UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 3UL);
  if ((static_cast<int>(k < 3)) && (static_cast<int>(i < 3))) {
    P_zz[i + 3 * k] += W[i] * Y_k_hat[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double x_k_hat[5]
//                const double X_k_hat[55]
//                const int i
//                const double *X_AB
//                double work[5]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel85(const double
  x_k_hat[5], const double X_k_hat[55], const int i, const double *X_AB, double
  work[5])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 5) {
    work[k] = *X_AB * (X_k_hat[k + 5 * i] - x_k_hat[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double v[3]
//                const double Y_k_hat[33]
//                const int i
//                double b_Y_k_hat[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel86(const double
  v[3], const double Y_k_hat[33], const int i, double b_Y_k_hat[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    b_Y_k_hat[k] = Y_k_hat[k + 3 * i] - v[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Y_k_hat[3]
//                const double work[5]
//                double P_xz[15]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel87(const double
  Y_k_hat[3], const double work[5], double P_xz[15])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 3)) && (static_cast<int>(i < 5))) {
    P_xz[i + 5 * k] += work[i] * Y_k_hat[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double R_CTRV_IMM[9]
//                double A[9]
//                double P_zz[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel88(const double
  R_CTRV_IMM[9], double A[9], double P_zz[9])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 9) {
    double xvFirst;
    xvFirst = P_zz[k] + R_CTRV_IMM[k];
    P_zz[k] = xvFirst;
    A[k] = xvFirst;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const double P_zz[9]
//                const int ix
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel89(const int jA,
  const double P_zz[9], const int ix, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[ix] = P_zz[ix] / P_zz[jA];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double tmp_ego_y[5]
//                const double work[5]
//                double b_out_P_cv[25]
//                double Vr[25]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel9(const double
  tmp_ego_y[5], const double work[5], double b_out_P_cv[25], double Vr[25])
{
  unsigned long threadId;
  int i;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  i = static_cast<int>(threadId % 5UL);
  k = static_cast<int>((threadId - static_cast<unsigned long>(i)) / 5UL);
  if ((static_cast<int>(k < 5)) && (static_cast<int>(i < 5))) {
    Vr[i + 5 * k] = b_out_P_cv[i + 5 * k] + work[i] * tmp_ego_y[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int *nr
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel90(const int jA,
  const int *nr, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[*nr] /= A[jA];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int ix
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel91(const int jA,
  const int ix, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[ix + 3] -= A[ix] * A[jA + 3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int *nr
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel92(const int jA,
  const int *nr, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[*nr + 3] -= A[*nr] * A[jA + 3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int ix
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel93(const int jA,
  const int ix, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[ix + 6] -= A[ix] * A[jA + 6];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const int jA
//                const int *nr
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel94(const int jA,
  const int *nr, double A[9])
{
  unsigned long threadId;
  int tmpIdx;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    A[*nr + 6] -= A[*nr] * A[jA + 6];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double P_zz[9]
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel95(const double
  P_zz[9], double A[9])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 9) {
    A[k] = P_zz[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                int ipiv_t[3]
//                int ipiv[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel96(int ipiv_t[3],
  int ipiv[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    ipiv[k] = ipiv_t[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double A[9]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel97(double A[9])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 9) {
    A[k] = CUDART_NAN;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                int ipiv[3]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel98(int ipiv[3])
{
  unsigned long threadId;
  int k;
  threadId = static_cast<unsigned long>(mwGetGlobalThreadIndexInXDimension());
  k = static_cast<int>(threadId);
  if (k < 3) {
    ipiv[k] = k + 1;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double A[9]
//                double *temp
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void BEV_image_kernel99(double A[9],
  double *temp)
{
  double tmpRed0;
  unsigned int blockStride;
  unsigned int mask;
  unsigned int thBlkId;
  unsigned int threadId;
  unsigned int threadStride;
  threadStride = static_cast<unsigned int>(mwGetTotalThreadsLaunched());
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  blockStride = static_cast<unsigned int>(mwGetThreadsPerBlock());
  tmpRed0 = 1.0;
  if (mwIsLastBlock()) {
    int m;
    m = static_cast<int>(3U % blockStride);
    if (static_cast<unsigned int>(m) > 0U) {
      blockStride = static_cast<unsigned int>(m);
    }
  }

  blockStride = ((blockStride + warpSize) - 1U) / warpSize;
  if (threadId <= 2U) {
    tmpRed0 = A[static_cast<int>(threadId) + 3 * static_cast<int>(threadId)];
  }

  mask = __ballot_sync(MAX_uint32_T, threadId <= 2U);
  for (unsigned int idx{threadId + threadStride}; idx <= 2U; idx += threadStride)
  {
    tmpRed0 *= A[static_cast<int>(idx) + 3 * static_cast<int>(idx)];
  }

  tmpRed0 = workGroupReduction(tmpRed0, mask, blockStride);
  if (thBlkId == 0U) {
    atomicOpreal_T(&temp[0], tmpRed0);
  }
}

//
// Arguments    : double *address
//                double value
// Return Type  : double
//
static __device__ double atomicOpreal_T(double *address, double value)
{
  unsigned long long int old;
  unsigned long long int *address_as_up;
  address_as_up = (unsigned long long int *)address;
  old = *address_as_up;
  unsigned long long int assumed;
  do {
    assumed = old;
    old = atomicCAS(address_as_up, old, __double_as_longlong(value *
      __longlong_as_double(old)));
  } while (assumed != old);

  return __longlong_as_double(old);
}

//
// Arguments    : double *address
//                double value
// Return Type  : double
//
static __device__ double b_atomicOpreal_T(double *address, double value)
{
  unsigned long long int old;
  unsigned long long int *address_as_up;
  address_as_up = (unsigned long long int *)address;
  old = *address_as_up;
  unsigned long long int assumed;
  do {
    assumed = old;
    old = atomicCAS(address_as_up, old, __double_as_longlong(value +
      __longlong_as_double(old)));
  } while (assumed != old);

  return __longlong_as_double(old);
}

//
// Arguments    : double val
//                unsigned int lane
//                unsigned int mask
// Return Type  : double
//
static __device__ double b_threadGroupReduction(double val, unsigned int lane,
  unsigned int mask)
{
  unsigned int activeSize;
  unsigned int offset;
  activeSize = __popc(mask);
  offset = (activeSize + 1U) / 2U;
  while (activeSize > 1U) {
    double other;
    other = shflDown2(val, offset, mask);
    if (lane + offset < activeSize) {
      val += other;
    }

    activeSize = offset;
    offset = (offset + 1U) / 2U;
  }

  return val;
}

//
// Arguments    : double val
//                unsigned int mask
//                unsigned int numActiveWarps
// Return Type  : double
//
static __device__ double b_workGroupReduction(double val, unsigned int mask,
  unsigned int numActiveWarps)
{
  __shared__ double shared[32];
  unsigned int lane;
  unsigned int thBlkId;
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  lane = thBlkId % warpSize;
  thBlkId /= warpSize;
  val = b_threadGroupReduction(val, lane, mask);
  if (lane == 0U) {
    shared[thBlkId] = val;
  }

  __syncthreads();
  mask = __ballot_sync(MAX_uint32_T, lane < numActiveWarps);
  val = shared[lane];
  if (thBlkId == 0U) {
    val = b_threadGroupReduction(val, lane, mask);
  }

  return val;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * std::sqrt(y * y + 1.0);
  } else if (!std::isnan(y)) {
    y = a * 1.4142135623730951;
  }

  return y;
}

//
// Arguments    : double in1
//                unsigned int offset
//                unsigned int mask
// Return Type  : double
//
static __device__ double shflDown2(double in1, unsigned int offset, unsigned int
  mask)
{
  int2 tmp;
  tmp = *(int2 *)&in1;
  tmp.x = __shfl_down_sync(mask, tmp.x, offset);
  tmp.y = __shfl_down_sync(mask, tmp.y, offset);
  return *(double *)&tmp;
}

//
// Arguments    : double val
//                unsigned int lane
//                unsigned int mask
// Return Type  : double
//
static __device__ double threadGroupReduction(double val, unsigned int lane,
  unsigned int mask)
{
  unsigned int activeSize;
  unsigned int offset;
  activeSize = __popc(mask);
  offset = (activeSize + 1U) / 2U;
  while (activeSize > 1U) {
    double other;
    other = shflDown2(val, offset, mask);
    if (lane + offset < activeSize) {
      val *= other;
    }

    activeSize = offset;
    offset = (offset + 1U) / 2U;
  }

  return val;
}

//
// Arguments    : double val
//                unsigned int mask
//                unsigned int numActiveWarps
// Return Type  : double
//
static __device__ double workGroupReduction(double val, unsigned int mask,
  unsigned int numActiveWarps)
{
  __shared__ double shared[32];
  unsigned int lane;
  unsigned int thBlkId;
  thBlkId = static_cast<unsigned int>(mwGetThreadIndexWithinBlock());
  lane = thBlkId % warpSize;
  thBlkId /= warpSize;
  val = threadGroupReduction(val, lane, mask);
  if (lane == 0U) {
    shared[thBlkId] = val;
  }

  __syncthreads();
  mask = __ballot_sync(MAX_uint32_T, lane < numActiveWarps);
  val = shared[lane];
  if (thBlkId == 0U) {
    val = threadGroupReduction(val, lane, mask);
  }

  return val;
}

//
// Arguments    : const double Chassis[12]
//                const double Sensor[11]
//                const double Traffic[10]
//                const double Lane[11]
//                const double AEB_in[2]
//                unsigned char b_BEV_image[367464]
// Return Type  : void
//
void BEV_image(const double Chassis[12], const double [11], const double
               Traffic[10], const double Lane[11], const double AEB_in[2],
               unsigned char b_BEV_image[367464])
{
  static const double dv[255]{ -0.2, -0.1952755905511811, -0.19055118110236222,
    -0.18582677165354333, -0.18110236220472442, -0.17637795275590551,
    -0.17165354330708663, -0.16692913385826774, -0.16220472440944883,
    -0.15748031496062992, -0.15275590551181104, -0.14803149606299215,
    -0.14330708661417324, -0.13858267716535433, -0.13385826771653545,
    -0.12913385826771656, -0.12440944881889765, -0.11968503937007875,
    -0.11496062992125985, -0.11023622047244096, -0.10551181102362206,
    -0.10078740157480316, -0.096062992125984265, -0.091338582677165367,
    -0.086614173228346469, -0.081889763779527572, -0.077165354330708674,
    -0.072440944881889791, -0.067716535433070879, -0.062992125984251968,
    -0.058267716535433084, -0.0535433070866142, -0.048818897637795289,
    -0.044094488188976377, -0.039370078740157494, -0.03464566929133861,
    -0.0299212598425197, -0.025196850393700787, -0.020472440944881903,
    -0.01574803149606302, -0.011023622047244108, -0.0062992125984251968,
    -0.0015748031496063131, 0.0031496062992125706, 0.0078740157480314821,
    0.012598425196850394, 0.017322834645669277, 0.022047244094488161,
    0.026771653543307072, 0.031496062992125984, 0.036220472440944868,
    0.040944881889763751, 0.045669291338582663, 0.050393700787401574,
    0.05511811023622043, 0.059842519685039341, 0.064566929133858253,
    0.069291338582677164, 0.074015748031496076, 0.078740157480314932,
    0.083464566929133843, 0.088188976377952755, 0.092913385826771611,
    0.097637795275590522, 0.10236220472440943, 0.10708661417322834,
    0.11181102362204726, 0.11653543307086611, 0.12125984251968502,
    0.12598425196850394, 0.13070866141732279, 0.1354330708661417,
    0.14015748031496061, 0.14488188976377953, 0.14960629921259844,
    0.15433070866141729, 0.1590551181102362, 0.16377952755905512,
    0.16850393700787397, 0.17322834645669288, 0.17795275590551179,
    0.18267716535433071, 0.18740157480314962, 0.19212598425196847,
    0.19685039370078738, 0.2015748031496063, 0.20629921259842515,
    0.21102362204724406, 0.21574803149606298, 0.22047244094488189,
    0.2251968503937008, 0.22992125984251965, 0.23464566929133857,
    0.23937007874015748, 0.24409448818897633, 0.24881889763779524,
    0.25354330708661416, 0.25826771653543307, 0.262992125984252,
    0.26771653543307083, 0.27244094488188975, 0.27716535433070866,
    0.28188976377952751, 0.28661417322834642, 0.29133858267716534,
    0.29606299212598425, 0.30078740157480316, 0.30551181102362207,
    0.31023622047244087, 0.31496062992125978, 0.31968503937007869,
    0.32440944881889761, 0.32913385826771652, 0.33385826771653543,
    0.33858267716535434, 0.34330708661417325, 0.34803149606299216,
    0.35275590551181096, 0.35748031496062987, 0.36220472440944879,
    0.3669291338582677, 0.37165354330708661, 0.37637795275590552,
    0.38110236220472443, 0.38582677165354323, 0.39055118110236214,
    0.39527559055118106, 0.39999999999999997, 0.40472440944881888,
    0.40944881889763779, 0.4141732283464567, 0.41889763779527561,
    0.42362204724409452, 0.42834645669291332, 0.43307086614173224,
    0.43779527559055115, 0.44251968503937006, 0.44724409448818897,
    0.45196850393700788, 0.45669291338582679, 0.46141732283464559,
    0.4661417322834645, 0.47086614173228342, 0.47559055118110233,
    0.48031496062992124, 0.48503937007874015, 0.48976377952755906,
    0.494488188976378, 0.49921259842519689, 0.50393700787401574,
    0.50866141732283454, 0.51338582677165356, 0.51811023622047236,
    0.52283464566929139, 0.52755905511811019, 0.53228346456692921,
    0.537007874015748, 0.54173228346456681, 0.54645669291338583,
    0.55118110236220463, 0.55590551181102366, 0.56062992125984246,
    0.56535433070866148, 0.57007874015748028, 0.5748031496062993,
    0.5795275590551181, 0.5842519685039369, 0.58897637795275593,
    0.59370078740157473, 0.59842519685039375, 0.60314960629921255,
    0.60787401574803157, 0.61259842519685037, 0.61732283464566917,
    0.62204724409448819, 0.626771653543307, 0.631496062992126,
    0.63622047244094482, 0.64094488188976384, 0.64566929133858264,
    0.65039370078740166, 0.65511811023622046, 0.65984251968503926,
    0.66456692913385829, 0.66929133858267709, 0.67401574803149611,
    0.67874015748031491, 0.68346456692913393, 0.68818897637795273,
    0.69291338582677153, 0.69763779527559056, 0.70236220472440936,
    0.70708661417322838, 0.71181102362204718, 0.7165354330708662,
    0.721259842519685, 0.725984251968504, 0.73070866141732282,
    0.73543307086614162, 0.74015748031496065, 0.74488188976377945,
    0.74960629921259847, 0.75433070866141727, 0.75905511811023629,
    0.76377952755905509, 0.76850393700787389, 0.77322834645669292,
    0.77795275590551172, 0.78267716535433074, 0.78740157480314954,
    0.79212598425196856, 0.79685039370078736, 0.80157480314960639,
    0.80629921259842519, 0.81102362204724421, 0.815748031496063,
    0.82047244094488181, 0.82519685039370083, 0.82992125984251963,
    0.83464566929133865, 0.83937007874015745, 0.84409448818897648,
    0.84881889763779528, 0.8535433070866143, 0.8582677165354331,
    0.8629921259842519, 0.86771653543307092, 0.87244094488188972,
    0.87716535433070875, 0.88188976377952755, 0.88661417322834657,
    0.89133858267716537, 0.89606299212598439, 0.90078740157480319,
    0.905511811023622, 0.910236220472441, 0.91496062992125982,
    0.91968503937007884, 0.92440944881889764, 0.92913385826771666,
    0.93385826771653546, 0.93858267716535426, 0.94330708661417328,
    0.94803149606299209, 0.95275590551181111, 0.95748031496062991,
    0.96220472440944893, 0.96692913385826773, 0.97165354330708653,
    0.97637795275590555, 0.98110236220472435, 0.98582677165354338,
    0.99055118110236218, 0.9952755905511812, 1.0 };

  static const double b[251]{ 625.0, 615.04000000000008, 605.16000000000008,
    595.36000000000013, 585.6400000000001, 576.0, 566.44, 556.96,
    547.56000000000006, 538.24000000000012, 529.0, 519.84, 510.76000000000005,
    501.7600000000001, 492.84000000000015, 484.0, 475.24, 466.56000000000006,
    457.96000000000009, 449.44000000000011, 441.0, 432.64000000000004,
    424.36000000000007, 416.16000000000008, 408.04000000000013, 400.0, 392.04,
    384.16000000000008, 376.36000000000007, 368.6400000000001, 361.0,
    353.44000000000005, 345.96000000000004, 338.56000000000006,
    331.23999999999995, 324.0, 316.84000000000003, 309.76000000000005,
    302.76000000000005, 295.84, 289.0, 282.24, 275.56000000000006,
    268.96000000000009, 262.44, 256.0, 249.64000000000001, 243.36000000000004,
    237.16000000000003, 231.04000000000002, 225.0, 219.04000000000002,
    213.16000000000005, 207.36, 201.64000000000004, 196.0, 190.44000000000003,
    184.96000000000004, 179.56, 174.24000000000004, 169.0, 163.84000000000003,
    158.76000000000005, 153.76000000000002, 148.84000000000003, 144.0, 139.24,
    134.56000000000003, 129.96, 125.44000000000003, 121.0, 116.64000000000001,
    112.36000000000003, 108.16000000000001, 104.04000000000002, 100.0,
    96.04000000000002, 92.160000000000025, 88.360000000000014,
    84.640000000000015, 81.0, 77.440000000000012, 73.96, 70.56,
    67.240000000000023, 64.0, 60.840000000000011, 57.760000000000005,
    54.760000000000005, 51.84, 49.0, 46.240000000000009, 43.560000000000009,
    40.960000000000008, 38.440000000000005, 36.0, 33.640000000000008,
    31.360000000000007, 29.160000000000004, 27.040000000000003, 25.0,
    23.040000000000006, 21.160000000000004, 19.360000000000003, 17.64, 16.0,
    14.440000000000001, 12.96, 11.560000000000002, 10.240000000000002, 9.0,
    7.8400000000000016, 6.7600000000000007, 5.7600000000000016,
    4.8400000000000007, 4.0, 3.24, 2.5600000000000005, 1.9600000000000004,
    1.4400000000000004, 1.0, 0.64000000000000012, 0.3600000000000001,
    0.16000000000000003, 0.040000000000000008, 0.0, 0.040000000000000008,
    0.16000000000000003, 0.3600000000000001, 0.64000000000000012, 1.0,
    1.4400000000000004, 1.9600000000000004, 2.5600000000000005, 3.24, 4.0,
    4.8400000000000007, 5.7600000000000016, 6.7600000000000007,
    7.8400000000000016, 9.0, 10.240000000000002, 11.560000000000002, 12.96,
    14.440000000000001, 16.0, 17.64, 19.360000000000003, 21.160000000000004,
    23.040000000000006, 25.0, 27.040000000000003, 29.160000000000004,
    31.360000000000007, 33.640000000000008, 36.0, 38.440000000000005,
    40.960000000000008, 43.560000000000009, 46.240000000000009, 49.0, 51.84,
    54.760000000000005, 57.760000000000005, 60.840000000000011, 64.0,
    67.240000000000023, 70.56, 73.96, 77.440000000000012, 81.0,
    84.640000000000015, 88.360000000000014, 92.160000000000025,
    96.04000000000002, 100.0, 104.04000000000002, 108.16000000000001,
    112.36000000000003, 116.64000000000001, 121.0, 125.44000000000003, 129.96,
    134.56000000000003, 139.24, 144.0, 148.84000000000003, 153.76000000000002,
    158.76000000000005, 163.84000000000003, 169.0, 174.24000000000004, 179.56,
    184.96000000000004, 190.44000000000003, 196.0, 201.64000000000004, 207.36,
    213.16000000000005, 219.04000000000002, 225.0, 231.04000000000002,
    237.16000000000003, 243.36000000000004, 249.64000000000001, 256.0, 262.44,
    268.96000000000009, 275.56000000000006, 282.24, 289.0, 295.84,
    302.76000000000005, 309.76000000000005, 316.84000000000003, 324.0,
    331.23999999999995, 338.56000000000006, 345.96000000000004,
    353.44000000000005, 361.0, 368.6400000000001, 376.36000000000007,
    384.16000000000008, 392.04, 400.0, 408.04000000000013, 416.16000000000008,
    424.36000000000007, 432.64000000000004, 441.0, 449.44000000000011,
    457.96000000000009, 466.56000000000006, 475.24, 484.0, 492.84000000000015,
    501.7600000000001, 510.76000000000005, 519.84, 529.0, 538.24000000000012,
    547.56000000000006, 556.96, 566.44, 576.0, 585.6400000000001,
    595.36000000000013, 605.16000000000008, 615.04000000000008, 625.0 };

  static const double b_b[251]{ 15625.0, 15252.992000000002, 14886.936000000003,
    14526.784000000003, 14172.488000000005, 13824.0, 13481.272,
    13144.256000000001, 12812.904000000004, 12487.168000000005, 12167.0,
    11852.352, 11543.176000000003, 11239.424000000003, 10941.048000000004,
    10648.0, 10360.232, 10077.696000000002, 9800.3440000000028,
    9528.1280000000042, 9261.0, 8998.912, 8741.8160000000025, 8489.6640000000025,
    8242.4080000000031, 8000.0, 7762.3920000000007, 7529.5360000000019,
    7301.3840000000027, 7077.8880000000026, 6859.0, 6644.6720000000005,
    6434.8560000000016, 6229.5040000000026, 6028.5679999999993, 5832.0, 5639.752,
    5451.7760000000017, 5268.0240000000022, 5088.4479999999994, 4913.0,
    4741.6320000000005, 4574.2960000000012, 4410.9440000000013,
    4251.5279999999993, 4096.0, 3944.3120000000004, 3796.4160000000011, 3652.264,
    3511.8080000000009, 3375.0, 3241.7920000000004, 3112.1360000000009,
    2985.9840000000004, 2863.2880000000005, 2744.0, 2628.072, 2515.456000000001,
    2406.1040000000003, 2299.9680000000008, 2197.0, 2097.1520000000005,
    2000.3760000000007, 1906.6240000000003, 1815.8480000000004, 1728.0,
    1643.0320000000002, 1560.8960000000006, 1481.544, 1404.9280000000003, 1331.0,
    1259.7120000000002, 1191.0160000000005, 1124.864, 1061.2080000000003, 1000.0,
    941.19200000000023, 884.73600000000033, 830.58400000000006,
    778.68800000000033, 729.0, 681.47200000000021, 636.05599999999993,
    592.70400000000006, 551.36800000000017, 512.0, 474.55200000000013,
    438.97600000000011, 405.22400000000005, 373.24800000000005, 343.0,
    314.43200000000013, 287.49600000000009, 262.14400000000006,
    238.32800000000003, 216.0, 195.11200000000008, 175.61600000000004,
    157.46400000000003, 140.608, 125.0, 110.59200000000004, 97.336000000000041,
    85.184000000000026, 74.088000000000008, 64.0, 54.872000000000014,
    46.656000000000006, 39.304000000000016, 32.768000000000008, 27.0,
    21.952000000000005, 17.576, 13.824000000000005, 10.648000000000003, 8.0,
    5.8320000000000007, 4.096000000000001, 2.7440000000000007,
    1.7280000000000006, 1.0, 0.51200000000000012, 0.21600000000000008,
    0.064000000000000015, 0.0080000000000000019, 0.0, -0.0080000000000000019,
    -0.064000000000000015, -0.21600000000000008, -0.51200000000000012, -1.0,
    -1.7280000000000006, -2.7440000000000007, -4.096000000000001,
    -5.8320000000000007, -8.0, -10.648000000000003, -13.824000000000005, -17.576,
    -21.952000000000005, -27.0, -32.768000000000008, -39.304000000000016,
    -46.656000000000006, -54.872000000000014, -64.0, -74.088000000000008,
    -85.184000000000026, -97.336000000000041, -110.59200000000004, -125.0,
    -140.608, -157.46400000000003, -175.61600000000004, -195.11200000000008,
    -216.0, -238.32800000000003, -262.14400000000006, -287.49600000000009,
    -314.43200000000013, -343.0, -373.24800000000005, -405.22400000000005,
    -438.97600000000011, -474.55200000000013, -512.0, -551.36800000000017,
    -592.70400000000006, -636.05599999999993, -681.47200000000021, -729.0,
    -778.68800000000033, -830.58400000000006, -884.73600000000033,
    -941.19200000000023, -1000.0, -1061.2080000000003, -1124.864,
    -1191.0160000000005, -1259.7120000000002, -1331.0, -1404.9280000000003,
    -1481.544, -1560.8960000000006, -1643.0320000000002, -1728.0,
    -1815.8480000000004, -1906.6240000000003, -2000.3760000000007,
    -2097.1520000000005, -2197.0, -2299.9680000000008, -2406.1040000000003,
    -2515.456000000001, -2628.072, -2744.0, -2863.2880000000005,
    -2985.9840000000004, -3112.1360000000009, -3241.7920000000004, -3375.0,
    -3511.8080000000009, -3652.264, -3796.4160000000011, -3944.3120000000004,
    -4096.0, -4251.5279999999993, -4410.9440000000013, -4574.2960000000012,
    -4741.6320000000005, -4913.0, -5088.4479999999994, -5268.0240000000022,
    -5451.7760000000017, -5639.752, -5832.0, -6028.5679999999993,
    -6229.5040000000026, -6434.8560000000016, -6644.6720000000005, -6859.0,
    -7077.8880000000026, -7301.3840000000027, -7529.5360000000019,
    -7762.3920000000007, -8000.0, -8242.4080000000031, -8489.6640000000025,
    -8741.8160000000025, -8998.912, -9261.0, -9528.1280000000042,
    -9800.3440000000028, -10077.696000000002, -10360.232, -10648.0,
    -10941.048000000004, -11239.424000000003, -11543.176000000003, -11852.352,
    -12167.0, -12487.168000000005, -12812.904000000004, -13144.256000000001,
    -13481.272, -13824.0, -14172.488000000005, -14526.784000000003,
    -14886.936000000003, -15252.992000000002, -15625.0 };

  static const double dv1[251]{ 25.0, 24.8, 24.6, 24.400000000000002,
    24.200000000000003, 24.0, 23.8, 23.6, 23.400000000000002, 23.200000000000003,
    23.0, 22.8, 22.6, 22.400000000000002, 22.200000000000003, 22.0, 21.8, 21.6,
    21.400000000000002, 21.200000000000003, 21.0, 20.8, 20.6, 20.400000000000002,
    20.200000000000003, 20.0, 19.8, 19.6, 19.400000000000002, 19.200000000000003,
    19.0, 18.8, 18.6, 18.400000000000002, 18.2, 18.0, 17.8, 17.6,
    17.400000000000002, 17.2, 17.0, 16.8, 16.6, 16.400000000000002, 16.2, 16.0,
    15.8, 15.600000000000001, 15.4, 15.200000000000001, 15.0, 14.8,
    14.600000000000001, 14.4, 14.200000000000001, 14.0, 13.8, 13.600000000000001,
    13.4, 13.200000000000001, 13.0, 12.8, 12.600000000000001, 12.4,
    12.200000000000001, 12.0, 11.8, 11.600000000000001, 11.4, 11.200000000000001,
    11.0, 10.8, 10.600000000000001, 10.4, 10.200000000000001, 10.0, 9.8,
    9.6000000000000014, 9.4, 9.2000000000000011, 9.0, 8.8, 8.6, 8.4,
    8.2000000000000011, 8.0, 7.8000000000000007, 7.6000000000000005, 7.4, 7.2,
    7.0, 6.8000000000000007, 6.6000000000000005, 6.4, 6.2, 6.0,
    5.8000000000000007, 5.6000000000000005, 5.4, 5.2, 5.0, 4.8000000000000007,
    4.6000000000000005, 4.4, 4.2, 4.0, 3.8000000000000003, 3.6,
    3.4000000000000004, 3.2, 3.0, 2.8000000000000003, 2.6, 2.4000000000000004,
    2.2, 2.0, 1.8, 1.6, 1.4000000000000001, 1.2000000000000002, 1.0, 0.8,
    0.60000000000000009, 0.4, 0.2, 0.0, -0.2, -0.4, -0.60000000000000009, -0.8,
    -1.0, -1.2000000000000002, -1.4000000000000001, -1.6, -1.8, -2.0, -2.2,
    -2.4000000000000004, -2.6, -2.8000000000000003, -3.0, -3.2,
    -3.4000000000000004, -3.6, -3.8000000000000003, -4.0, -4.2, -4.4,
    -4.6000000000000005, -4.8000000000000007, -5.0, -5.2, -5.4,
    -5.6000000000000005, -5.8000000000000007, -6.0, -6.2, -6.4,
    -6.6000000000000005, -6.8000000000000007, -7.0, -7.2, -7.4,
    -7.6000000000000005, -7.8000000000000007, -8.0, -8.2000000000000011, -8.4,
    -8.6, -8.8, -9.0, -9.2000000000000011, -9.4, -9.6000000000000014, -9.8,
    -10.0, -10.200000000000001, -10.4, -10.600000000000001, -10.8, -11.0,
    -11.200000000000001, -11.4, -11.600000000000001, -11.8, -12.0,
    -12.200000000000001, -12.4, -12.600000000000001, -12.8, -13.0,
    -13.200000000000001, -13.4, -13.600000000000001, -13.8, -14.0,
    -14.200000000000001, -14.4, -14.600000000000001, -14.8, -15.0,
    -15.200000000000001, -15.4, -15.600000000000001, -15.8, -16.0, -16.2,
    -16.400000000000002, -16.6, -16.8, -17.0, -17.2, -17.400000000000002, -17.6,
    -17.8, -18.0, -18.2, -18.400000000000002, -18.6, -18.8, -19.0,
    -19.200000000000003, -19.400000000000002, -19.6, -19.8, -20.0,
    -20.200000000000003, -20.400000000000002, -20.6, -20.8, -21.0,
    -21.200000000000003, -21.400000000000002, -21.6, -21.8, -22.0,
    -22.200000000000003, -22.400000000000002, -22.6, -22.8, -23.0,
    -23.200000000000003, -23.400000000000002, -23.6, -23.8, -24.0,
    -24.200000000000003, -24.400000000000002, -24.6, -24.8, -25.0 };

  static const double varargin_1[251]{ 25.0, 24.8, 24.6, 24.400000000000002,
    24.200000000000003, 24.0, 23.8, 23.6, 23.400000000000002, 23.200000000000003,
    23.0, 22.8, 22.6, 22.400000000000002, 22.200000000000003, 22.0, 21.8, 21.6,
    21.400000000000002, 21.200000000000003, 21.0, 20.8, 20.6, 20.400000000000002,
    20.200000000000003, 20.0, 19.8, 19.6, 19.400000000000002, 19.200000000000003,
    19.0, 18.8, 18.6, 18.400000000000002, 18.2, 18.0, 17.8, 17.6,
    17.400000000000002, 17.2, 17.0, 16.8, 16.6, 16.400000000000002, 16.2, 16.0,
    15.8, 15.600000000000001, 15.4, 15.200000000000001, 15.0, 14.8,
    14.600000000000001, 14.4, 14.200000000000001, 14.0, 13.8, 13.600000000000001,
    13.4, 13.200000000000001, 13.0, 12.8, 12.600000000000001, 12.4,
    12.200000000000001, 12.0, 11.8, 11.600000000000001, 11.4, 11.200000000000001,
    11.0, 10.8, 10.600000000000001, 10.4, 10.200000000000001, 10.0, 9.8,
    9.6000000000000014, 9.4, 9.2000000000000011, 9.0, 8.8, 8.6, 8.4,
    8.2000000000000011, 8.0, 7.8000000000000007, 7.6000000000000005, 7.4, 7.2,
    7.0, 6.8000000000000007, 6.6000000000000005, 6.4, 6.2, 6.0,
    5.8000000000000007, 5.6000000000000005, 5.4, 5.2, 5.0, 4.8000000000000007,
    4.6000000000000005, 4.4, 4.2, 4.0, 3.8000000000000003, 3.6,
    3.4000000000000004, 3.2, 3.0, 2.8000000000000003, 2.6, 2.4000000000000004,
    2.2, 2.0, 1.8, 1.6, 1.4000000000000001, 1.2000000000000002, 1.0, 0.8,
    0.60000000000000009, 0.4, 0.2, 0.0, -0.2, -0.4, -0.60000000000000009, -0.8,
    -1.0, -1.2000000000000002, -1.4000000000000001, -1.6, -1.8, -2.0, -2.2,
    -2.4000000000000004, -2.6, -2.8000000000000003, -3.0, -3.2,
    -3.4000000000000004, -3.6, -3.8000000000000003, -4.0, -4.2, -4.4,
    -4.6000000000000005, -4.8000000000000007, -5.0, -5.2, -5.4,
    -5.6000000000000005, -5.8000000000000007, -6.0, -6.2, -6.4,
    -6.6000000000000005, -6.8000000000000007, -7.0, -7.2, -7.4,
    -7.6000000000000005, -7.8000000000000007, -8.0, -8.2000000000000011, -8.4,
    -8.6, -8.8, -9.0, -9.2000000000000011, -9.4, -9.6000000000000014, -9.8,
    -10.0, -10.200000000000001, -10.4, -10.600000000000001, -10.8, -11.0,
    -11.200000000000001, -11.4, -11.600000000000001, -11.8, -12.0,
    -12.200000000000001, -12.4, -12.600000000000001, -12.8, -13.0,
    -13.200000000000001, -13.4, -13.600000000000001, -13.8, -14.0,
    -14.200000000000001, -14.4, -14.600000000000001, -14.8, -15.0,
    -15.200000000000001, -15.4, -15.600000000000001, -15.8, -16.0, -16.2,
    -16.400000000000002, -16.6, -16.8, -17.0, -17.2, -17.400000000000002, -17.6,
    -17.8, -18.0, -18.2, -18.400000000000002, -18.6, -18.8, -19.0,
    -19.200000000000003, -19.400000000000002, -19.6, -19.8, -20.0,
    -20.200000000000003, -20.400000000000002, -20.6, -20.8, -21.0,
    -21.200000000000003, -21.400000000000002, -21.6, -21.8, -22.0,
    -22.200000000000003, -22.400000000000002, -22.6, -22.8, -23.0,
    -23.200000000000003, -23.400000000000002, -23.6, -23.8, -24.0,
    -24.200000000000003, -24.400000000000002, -24.6, -24.8, -25.0 };

  static const double b_varargin_1[61]{ 6.0, 5.8000000000000007,
    5.6000000000000005, 5.4, 5.2, 5.0, 4.8000000000000007, 4.6000000000000005,
    4.4, 4.2, 4.0, 3.8000000000000003, 3.6, 3.4000000000000004, 3.2, 3.0,
    2.8000000000000003, 2.6, 2.4000000000000004, 2.2, 2.0, 1.8, 1.6,
    1.4000000000000001, 1.2000000000000002, 1.0, 0.8, 0.60000000000000009, 0.4,
    0.2, 0.0, -0.2, -0.4, -0.60000000000000009, -0.8, -1.0, -1.2000000000000002,
    -1.4000000000000001, -1.6, -1.8, -2.0, -2.2, -2.4000000000000004, -2.6,
    -2.8000000000000003, -3.0, -3.2, -3.4000000000000004, -3.6,
    -3.8000000000000003, -4.0, -4.2, -4.4, -4.6000000000000005,
    -4.8000000000000007, -5.0, -5.2, -5.4, -5.6000000000000005,
    -5.8000000000000007, -6.0 };

  static const double dv2[61]{ 6.0, 5.8000000000000007, 5.6000000000000005, 5.4,
    5.2, 5.0, 4.8000000000000007, 4.6000000000000005, 4.4, 4.2, 4.0,
    3.8000000000000003, 3.6, 3.4000000000000004, 3.2, 3.0, 2.8000000000000003,
    2.6, 2.4000000000000004, 2.2, 2.0, 1.8, 1.6, 1.4000000000000001,
    1.2000000000000002, 1.0, 0.8, 0.60000000000000009, 0.4, 0.2, 0.0, -0.2, -0.4,
    -0.60000000000000009, -0.8, -1.0, -1.2000000000000002, -1.4000000000000001,
    -1.6, -1.8, -2.0, -2.2, -2.4000000000000004, -2.6, -2.8000000000000003, -3.0,
    -3.2, -3.4000000000000004, -3.6, -3.8000000000000003, -4.0, -4.2, -4.4,
    -4.6000000000000005, -4.8000000000000007, -5.0, -5.2, -5.4,
    -5.6000000000000005, -5.8000000000000007, -6.0 };

  static const double P_ctrv[25]{ 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
    0.0, 0.0, 0.0, 0.0, 3.0461741978670866E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0004,
    0.0, 0.0, 0.0, 0.0, 0.0, 6.8538919452009421E-6 };

  static const double P_cv[25]{ 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.8538919452009418E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0004,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0004 };

  static const double R_CTRV_IMM[9]{ 25.0, 0.0, 0.0, 0.0, 25.0, 0.0, 0.0, 0.0,
    0.27415567780803768 };

  static double empty_black_image[367464];
  static const signed char b_a[15]{ 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0
  };

  dim3 block;
  dim3 grid;
  creal_T L_tmp[25];
  creal_T R[25];
  creal_T U[25];
  creal_T (*b_gpu_U)[25];
  creal_T (*gpu_L_tmp)[25];
  creal_T (*gpu_R)[25];
  creal_T (*gpu_U)[25];
  creal_T c;
  creal_T *gpu_c;
  double (*gpu_Rx)[15311];
  double (*gpu_Ry)[15311];
  double (*gpu_State)[2520];
  double (*gpu_dv)[2492];
  double c_x[255];
  double (*b_gpu_dv)[255];
  double (*gpu_x)[255];
  double b_y[251];
  double tmp_lane_y[251];
  double (*b_gpu_b)[251];
  double (*b_gpu_dv1)[251];
  double (*b_gpu_y)[251];
  double (*gpu_b)[251];
  double (*gpu_tmp_lane_y)[251];
  double (*gpu_varargin_1)[251];
  double (*gpu_P_ctrv_tmp)[250];
  double (*gpu_P_cv_tmp)[250];
  double xp1[100];
  double xp2[100];
  double xp3[100];
  double xp4[100];
  double yp1[100];
  double yp2[100];
  double yp3[100];
  double yp4[100];
  double (*gpu_xp1)[100];
  double (*gpu_xp2)[100];
  double (*gpu_xp3)[100];
  double (*gpu_xp4)[100];
  double (*gpu_yp1)[100];
  double (*gpu_yp2)[100];
  double (*gpu_yp3)[100];
  double (*gpu_yp4)[100];
  double (*gpu_TJ_X)[70];
  double (*gpu_TJ_Y)[70];
  double d_x[61];
  double (*b_gpu_varargin_1)[61];
  double (*b_gpu_x)[61];
  double (*gpu_dv2)[61];
  double X_k[55];
  double (*gpu_X_k)[55];
  double (*gpu_X_k_hat)[55];
  double (*gpu_X_pred)[50];
  double (*gpu_Y_k_hat)[33];
  double Training_data_data[28];
  double (*gpu_Training_data_data)[28];
  double L[25];
  double Vr[25];
  double (*gpu_L)[25];
  double (*gpu_P_ctrv)[25];
  double (*gpu_P_ctrv_out)[25];
  double (*gpu_P_cv)[25];
  double (*gpu_P_cv_out)[25];
  double (*gpu_Vr)[25];
  double (*gpu_out_P_ctrv)[25];
  double (*gpu_out_P_cv)[25];
  double K[15];
  double P_xz[15];
  double old_flag[15];
  double (*gpu_K)[15];
  double (*gpu_P_xz)[15];
  double (*gpu_flag)[15];
  double (*gpu_old_flag)[15];
  double (*gpu_Chassis)[12];
  double W[11];
  double (*gpu_W)[11];
  double TV_range_x[10];
  double (*gpu_TV_range_x)[10];
  double (*gpu_TV_range_y)[10];
  double A[9];
  double P_zz[9];
  double y[9];
  double (*gpu_A)[9];
  double (*gpu_P_zz)[9];
  double (*gpu_R_CTRV_IMM)[9];
  double (*gpu_y)[9];
  double target_y[5];
  double tmp_ego_x[5];
  double tmp_ego_y[5];
  double work[5];
  double x_ini[5];
  double (*gpu_X_hat)[5];
  double (*gpu_target_y)[5];
  double (*gpu_tmp_ego_x)[5];
  double (*gpu_tmp_ego_y)[5];
  double (*gpu_work)[5];
  double (*gpu_x_ctrv)[5];
  double (*gpu_x_ctrv_out)[5];
  double (*gpu_x_cv_out)[5];
  double (*gpu_x_ini)[5];
  double (*gpu_x_k_hat)[5];
  double tau[4];
  double v[3];
  double y_out[3];
  double (*b_gpu_W)[3];
  double (*b_gpu_Y_k_hat)[3];
  double (*gpu_dv1)[3];
  double (*gpu_v)[3];
  double (*gpu_y_out)[3];
  double Class_B_idx_10;
  double Class_B_idx_9;
  double X_AB;
  double Y_AB;
  double a;
  double b_x;
  double d;
  double d2;
  double laneInfoL_idx_2;
  double laneInfoL_idx_3;
  double laneInfoL_idx_4;
  double laneInfoR_idx_0;
  double laneInfoR_idx_1;
  double laneInfoR_idx_2;
  double laneInfoR_idx_3;
  double laneInfoR_idx_4;
  double old_Prob_ctrv;
  double old_Prob_cv;
  double range_X_min;
  double range_Y_min;
  double scale;
  double t;
  double temp;
  double ts;
  double x;
  double *gpu_Class_B_idx_10;
  double *gpu_Class_B_idx_9;
  double *gpu_X_AB;
  double *gpu_Y_AB;
  double *gpu_d;
  double *gpu_scale;
  double *gpu_temp;
  int i_data[15311];
  int (*gpu_i_data)[15311];
  int last[5];
  int (*gpu_last)[5];
  int ipiv[3];
  int ipiv_t[3];
  int (*gpu_ipiv)[3];
  int (*gpu_ipiv_t)[3];
  int DEC_param;
  int b_L;
  int b_i;
  int b_info;
  int i;
  int info;
  int ix;
  int j;
  int jA;
  int k;
  int knt;
  int lastc;
  int m;
  int nr;
  int *b_gpu_L;
  int *b_gpu_info;
  int *gpu_info;
  int *gpu_k;
  int *gpu_nr;
  unsigned char (*gpu_BEV_image)[367464];
  unsigned char j_data[15311];
  unsigned char (*gpu_j_data)[15311];
  signed char (*gpu_a)[15];
  signed char first[5];
  signed char (*gpu_first)[5];
  bool e_x[15311];
  bool (*c_gpu_x)[15311];
  bool (*gpu_in_tmp)[15311];
  bool A_dirtyOnCpu;
  bool BEV_image_dirtyOnCpu;
  bool BEV_image_dirtyOnGpu;
  bool Chassis_dirtyOnCpu;
  bool Class_B_idx_10_dirtyOnGpu;
  bool Class_B_idx_9_dirtyOnCpu;
  bool K_dirtyOnCpu;
  bool L_dirtyOnGpu;
  bool P_ctrv_dirtyOnCpu;
  bool P_cv_dirtyOnCpu;
  bool R_CTRV_IMM_dirtyOnCpu;
  bool R_dirtyOnCpu;
  bool TV_range_x_dirtyOnGpu;
  bool U_dirtyOnCpu;
  bool Vr_dirtyOnCpu;
  bool Vr_dirtyOnGpu;
  bool Y_AB_dirtyOnCpu;
  bool Y_AB_dirtyOnGpu;
  bool a_dirtyOnCpu;
  bool b_b_dirtyOnCpu;
  bool b_dirtyOnCpu;
  bool b_varargin_1_dirtyOnCpu;
  bool c_dirtyOnCpu;
  bool c_dirtyOnGpu;
  bool dv_dirtyOnCpu;
  bool exitg1;
  bool goto150;
  bool i_data_dirtyOnCpu;
  bool ipiv_t_dirtyOnGpu;
  bool j_data_dirtyOnCpu;
  bool nr_dirtyOnCpu;
  bool old_flag_dirtyOnGpu;
  bool tmp_ego_x_dirtyOnGpu;
  bool tmp_ego_y_dirtyOnGpu;
  bool v_dirtyOnCpu;
  bool v_dirtyOnGpu;
  bool validLaunchParams;
  bool varargin_1_dirtyOnCpu;
  bool xp1_dirtyOnCpu;
  bool xp2_dirtyOnCpu;
  bool xp3_dirtyOnCpu;
  bool xp4_dirtyOnCpu;
  bool y_dirtyOnGpu;
  bool y_out_dirtyOnCpu;
  bool yp1_dirtyOnCpu;
  bool yp2_dirtyOnCpu;
  bool yp3_dirtyOnCpu;
  bool yp4_dirtyOnCpu;
  if (!isInitialized_BEV_image) {
    BEV_image_initialize();
  }

  cudaMalloc(&gpu_xp1, 800UL);
  cudaMalloc(&gpu_yp1, 800UL);
  cudaMalloc(&gpu_xp2, 800UL);
  cudaMalloc(&gpu_yp2, 800UL);
  cudaMalloc(&gpu_xp3, 800UL);
  cudaMalloc(&gpu_yp3, 800UL);
  cudaMalloc(&gpu_xp4, 800UL);
  cudaMalloc(&gpu_yp4, 800UL);
  cudaMalloc(&gpu_TV_range_x, 80UL);
  cudaMalloc(&gpu_TV_range_y, 80UL);
  cudaMalloc(&gpu_j_data, 15311UL);
  cudaMalloc(&gpu_i_data, 61244UL);
  cudaMalloc(&gpu_target_y, 40UL);
  cudaMalloc(&c_gpu_x, 15311UL);
  cudaMalloc(&gpu_tmp_ego_x, 40UL);
  cudaMalloc(&gpu_first, 5UL);
  cudaMalloc(&gpu_last, 20UL);
  cudaMalloc(&b_gpu_x, 488UL);
  cudaMalloc(&b_gpu_varargin_1, 488UL);
  cudaMalloc(&gpu_in_tmp, 15311UL);
  cudaMalloc(&b_gpu_y, 2008UL);
  cudaMalloc(&gpu_Chassis, 96UL);
  cudaMalloc(&gpu_BEV_image, 367464UL);
  cudaMalloc(&gpu_tmp_lane_y, 2008UL);
  cudaMalloc(&b_gpu_b, 2008UL);
  cudaMalloc(&gpu_b, 2008UL);
  cudaMalloc(&gpu_varargin_1, 2008UL);
  cudaMalloc(&gpu_x, 2040UL);
  cudaMalloc(&b_gpu_dv, 2040UL);
  cudaMalloc(&gpu_Rx, 122488UL);
  cudaMalloc(&gpu_Ry, 122488UL);
  cudaMalloc(&b_gpu_dv1, 2008UL);
  cudaMalloc(&gpu_dv2, 488UL);
  cudaMalloc(&gpu_Training_data_data, 224UL);
  cudaMalloc(&gpu_X_pred, 400UL);
  cudaMalloc(&b_gpu_info, 4UL);
  cudaMalloc(&gpu_P_cv, 200UL);
  cudaMalloc(&gpu_K, 120UL);
  cudaMalloc(&gpu_x_ctrv, 40UL);
  cudaMalloc(&gpu_dv1, 24UL);
  cudaMalloc(&gpu_y, 72UL);
  cudaMalloc(&gpu_y_out, 24UL);
  cudaMalloc(&gpu_ipiv, 12UL);
  cudaMalloc(&gpu_info, 4UL);
  cudaMalloc(&gpu_ipiv_t, 12UL);
  cudaMalloc(&gpu_Class_B_idx_10, 8UL);
  cudaMalloc(&b_gpu_W, 24UL);
  cudaMalloc(&b_gpu_Y_k_hat, 24UL);
  cudaMalloc(&gpu_A, 72UL);
  cudaMalloc(&gpu_R_CTRV_IMM, 72UL);
  cudaMalloc(&gpu_Y_k_hat, 264UL);
  cudaMalloc(&gpu_a, 15UL);
  cudaMalloc(&gpu_P_xz, 120UL);
  cudaMalloc(&gpu_P_zz, 72UL);
  cudaMalloc(&gpu_X_hat, 40UL);
  cudaMalloc(&gpu_d, 8UL);
  cudaMalloc(&gpu_X_k_hat, 440UL);
  cudaMalloc(&gpu_c, 16UL);
  cudaMalloc(&gpu_x_k_hat, 40UL);
  cudaMalloc(&gpu_nr, 4UL);
  cudaMalloc(&b_gpu_L, 4UL);
  cudaMalloc(&gpu_W, 88UL);
  cudaMalloc(&gpu_X_k, 440UL);
  cudaMalloc(&gpu_Y_AB, 8UL);
  cudaMalloc(&gpu_scale, 8UL);
  cudaMalloc(&b_gpu_U, 400UL);
  cudaMalloc(&gpu_v, 24UL);
  cudaMalloc(&gpu_R, 400UL);
  cudaMalloc(&gpu_L_tmp, 400UL);
  cudaMalloc(&gpu_U, 400UL);
  cudaMalloc(&gpu_P_ctrv, 200UL);
  cudaMalloc(&gpu_P_ctrv_tmp, 2000UL);
  cudaMalloc(&gpu_P_cv_tmp, 2000UL);
  cudaMalloc(&gpu_P_cv_out, 200UL);
  cudaMalloc(&gpu_P_ctrv_out, 200UL);
  cudaMalloc(&gpu_Class_B_idx_9, 8UL);
  cudaMalloc(&gpu_Vr, 200UL);
  cudaMalloc(&gpu_out_P_cv, 200UL);
  cudaMalloc(&gpu_L, 200UL);
  cudaMalloc(&gpu_out_P_ctrv, 200UL);
  cudaMalloc(&gpu_x_ctrv_out, 40UL);
  cudaMalloc(&gpu_x_cv_out, 40UL);
  cudaMalloc(&gpu_work, 40UL);
  cudaMalloc(&gpu_tmp_ego_y, 40UL);
  cudaMalloc(&gpu_x_ini, 40UL);
  cudaMalloc(&gpu_temp, 8UL);
  cudaMalloc(&gpu_TJ_Y, 560UL);
  cudaMalloc(&gpu_TJ_X, 560UL);
  cudaMalloc(&gpu_X_AB, 8UL);
  cudaMalloc(&gpu_dv, 19936UL);
  cudaMalloc(&gpu_State, 20160UL);
  cudaMalloc(&gpu_k, 4UL);
  cudaMalloc(&gpu_old_flag, 120UL);
  cudaMalloc(&gpu_flag, 120UL);
  TV_range_x_dirtyOnGpu = false;
  tmp_ego_x_dirtyOnGpu = false;
  BEV_image_dirtyOnGpu = false;
  y_dirtyOnGpu = false;
  ipiv_t_dirtyOnGpu = false;
  c_dirtyOnGpu = false;
  old_flag_dirtyOnGpu = false;
  xp1_dirtyOnCpu = false;
  yp1_dirtyOnCpu = false;
  xp2_dirtyOnCpu = false;
  yp2_dirtyOnCpu = false;
  xp3_dirtyOnCpu = false;
  yp3_dirtyOnCpu = false;
  xp4_dirtyOnCpu = false;
  yp4_dirtyOnCpu = false;
  j_data_dirtyOnCpu = false;
  i_data_dirtyOnCpu = false;
  BEV_image_dirtyOnCpu = false;
  K_dirtyOnCpu = false;
  A_dirtyOnCpu = false;
  c_dirtyOnCpu = false;
  v_dirtyOnCpu = false;
  R_dirtyOnCpu = false;
  U_dirtyOnCpu = false;
  b_varargin_1_dirtyOnCpu = true;
  Chassis_dirtyOnCpu = true;
  b_b_dirtyOnCpu = true;
  b_dirtyOnCpu = true;
  varargin_1_dirtyOnCpu = true;
  dv_dirtyOnCpu = true;
  P_cv_dirtyOnCpu = true;
  R_CTRV_IMM_dirtyOnCpu = true;
  a_dirtyOnCpu = true;
  P_ctrv_dirtyOnCpu = true;

  // State,out_Prob_ctrv,out_Prob_cv,out_P_ctrv,out_P_cv,flag]
  // old_State,old_Prob_ctrv,old_Prob_cv,old_P_ctrv,old_P_cv,old_flag)
  //  p.145
  //  p.620
  //  p.621
  //  p.622
  //  p.623
  if (!out_Prob_ctrv_not_empty) {
    old_Prob_ctrv = 0.8;
    old_Prob_cv = 0.2;
    out_Prob_ctrv_not_empty = true;
  } else {
    old_Prob_ctrv = out_Prob_ctrv;
    old_Prob_cv = out_Prob_cv;
    cudaMemcpy(*gpu_flag, flag, 120UL, cudaMemcpyHostToDevice);
    BEV_image_kernel1<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_flag,
      *gpu_old_flag);
    cudaMemcpy(flag, *gpu_flag, 120UL, cudaMemcpyDeviceToHost);
    old_flag_dirtyOnGpu = true;
  }

  //  CarMaker data의 sample time
  //  Input Parameter
  //  Chassis
  // Sensor
  //  Traffic
  //  Lane
  //
  //
  //  Switch
  //  CarMaker에서 생성한 data에 threat metric을 추가하여 BEV Window 생성, 저장
  //  저장된 BEV Window를 학습이 가능하도록 annotation 별로 분류
  //  저장된 BEV Window를 이용하여 Image Augmentation 하여 학습데이터 추가
  //  학습
  //  0 = 전방위 인지, 1 = FOV 적용
  //  BEV Window 생성 Parameter
  //  누적하는 궤적의 길이 (s)
  //  impact 전 첫 BEV window를 생성할 시점 (s)
  //  흑백 image
  //  컬러 image -> State (I_Lat, Velocity, Heading)
  //  Image_Channel_1일 경우 아래 2개중 하나 선택
  //  Image X 축의 크기 (pixel)
  //  Image Y 축의 크기 (pixel)
  //  m
  //  m
  //  X_MIN = -10; % m
  //  X_MAX = 40; % m
  //  m
  //  m
  if (!just_one_check_not_empty) {
    just_one_check_not_empty = true;
  } else {
    for (k = 0; k < 28; k++) {
      cudaMemcpy(gpu_k, &k, 4UL, cudaMemcpyHostToDevice);
      cudaMemcpy(*gpu_State, State, 20160UL, cudaMemcpyHostToDevice);
      BEV_image_kernel2<<<dim3(1U, 1U, 1U), dim3(96U, 1U, 1U)>>>(gpu_k,
        *gpu_State, *gpu_dv);
      cudaMemcpy(State, *gpu_State, 20160UL, cudaMemcpyDeviceToHost);
      cudaMemcpy(*gpu_State, State, 20160UL, cudaMemcpyHostToDevice);
      BEV_image_kernel3<<<dim3(1U, 1U, 1U), dim3(96U, 1U, 1U)>>>(*gpu_dv, gpu_k,
        *gpu_State);
      cudaMemcpy(State, *gpu_State, 20160UL, cudaMemcpyDeviceToHost);
    }
  }

  //
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //  이 line 이후로는 코드 실행 전 수정할 필요 없음
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //  CarMaker data의 sample time
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //  BEV Window Parameter
  //  if Image_Channel_1 == 0
  //      Only_Position_for_Channel = 0;
  //      Only_I_Lat_for_Channel = 0;
  //  end
  //  Crash data 중 lateral collision index(I_lat) > threshold 조건을 만족하면 training set으로 저장
  //  I_Lat_Threshold = 0.514687775463802; % SVM 학습을 통해 구한 I_Lat threshold
  //  SVM 학습을 통해 구한 I_Lat threshold
  //  Unsafe data 의 BEV Window가 저장되는 sample time
  //  Safe data 의 BEV Window가 저장되는 sample time
  //  elseif Image_Channel_12 % 1:3(AEB),4:6(DEC),7:9(ES),10:12(ELC)
  //    IMAGE_Z = 3;
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //  BEV Window Normalization Parameter
  //  m
  //  m
  //  rad
  //  rad
  //  s
  //  s
  //  s
  //  s
  //  s
  //  s
  //  empty_black_image = uint8(zeros(IMAGE_X, IMAGE_Y, IMAGE_Z));
  //
  //  Threat Parameter
  // 상대차 최대감속도
  // 자차 최소감속도
  // 자차 최대가속도
  // 상대차 제동시간 (Vo -> zero)
  // 횡방향 최대가속도
  // 횡방향 최소감속도
  // 상대차 제동시간 (Vo -> zero)
  // 횡방향 마진
  // 무게중심에서 전륜까지의 거리
  // 차량 너비
  //  AddThreatMetricMultiTarget_Predict_Trajectory.m 의 Parameter
  //       [rad]              Global heading angle
  //       [m]                Global longitudinal position
  //       [m]                Global lateral position
  //       [m/s]              absolute velocity
  //       [m/s^2]
  //      [m/s]
  //      [m/s]
  //  CLASS B
  //       [rad]                                            global 좌표계에서의 heading angle
  //       [m]                                             position = 뒷범퍼 중심
  //       [m]
  //       [m/s]
  //       [m/s]
  //       [m]
  //       [m]
  //      [m]
  //      [m]
  //      [m/s]
  //      [m/s]
  //      [rad]
  //  Description
  //                       Class_B 에서 출력되는 최대 state 개수
  //                        Preprocessing 에서 추가될 state 개수
  //  Road
  //      [m]
  //      [1/m]
  //      [rad]
  //      [m]
  //      [m]
  //  ROAD.PREPROCESSING.STATE_NUMBER                    = length(fieldnames(ROAD.PREPROCESSING));
  //  Line
  //      [1/m^2]
  //      [1/m]
  //      [rad]
  //      [m]
  //      [1/m^2]
  //      [1/m]
  //  Initialization
  //  Traffic Coodinate Transform
  //   this part will be changed
  //  Fr0(global)
  //  Fr0(global)
  //  wheel velocity
  //  Fr1(body fixed)
  //  Fr1(body fixed)
  //  Fr1(body fixed)
  X_AB = Traffic[2] - (Chassis[9] * std::cos(Chassis[1]) + Chassis[3]);
  Y_AB = Traffic[1] - (Chassis[9] * std::sin(Chassis[1]) + Chassis[2]);
  Y_AB_dirtyOnCpu = true;
  Class_B_idx_10 = (Traffic[4] * std::cos(Chassis[1]) + Traffic[5] * std::sin
                    (Chassis[1])) - Chassis[6];
  Class_B_idx_9 = (-Traffic[4] * std::sin(Chassis[1]) + Traffic[5] * std::cos
                   (Chassis[1])) - Chassis[7];

  //  Lane distance
  //  Generation Training data
  Training_data_data[1] = Chassis[11];
  Training_data_data[2] = Chassis[10];
  Training_data_data[5] = X_AB * std::cos(Chassis[1]) + Y_AB * std::sin(Chassis
    [1]);
  Training_data_data[6] = -X_AB * std::sin(Chassis[1]) + Y_AB * std::cos
    (Chassis[1]);
  Training_data_data[7] = Class_B_idx_10;
  Training_data_data[8] = Class_B_idx_9;
  Training_data_data[9] = std::sqrt(Class_B_idx_10 * Class_B_idx_10 +
    Class_B_idx_9 * Class_B_idx_9);
  Training_data_data[10] = Traffic[3] - Chassis[1];
  Training_data_data[3] = Traffic[9];
  Training_data_data[4] = Traffic[8];

  //  IMM-UKF
  //  [y x yaw v yawrate]
  out_Prob_cv = 0.2;
  out_Prob_ctrv = 0.8;

  //  ACC DEC ESL ESR ELCL ELCR ESS
  //  TJ_ELCL=zeros(length(index_time),10);
  //  TJ_ELCR=zeros(length(index_time),10);
  //  TJ_DEC=zeros(length(index_time),10);
  //  TJ_ACC=zeros(length(index_time),10);
  //  for index_time = 1:length(sim_time)
  scale = 3.3121686421112381E-170;
  Class_B_idx_9 = std::abs(Class_B_idx_9);
  if (Class_B_idx_9 > 3.3121686421112381E-170) {
    X_AB = 1.0;
    scale = Class_B_idx_9;
  } else {
    t = Class_B_idx_9 / 3.3121686421112381E-170;
    X_AB = t * t;
  }

  Class_B_idx_9 = std::abs(Class_B_idx_10);
  if (Class_B_idx_9 > scale) {
    t = scale / Class_B_idx_9;
    X_AB = X_AB * t * t + 1.0;
    scale = Class_B_idx_9;
  } else {
    t = Class_B_idx_9 / scale;
    X_AB += t * t;
  }

  X_AB = scale * std::sqrt(X_AB);

  //  Fr1(body fixed)
  goto150 = true;
  lastc = 1;
  exitg1 = false;
  while ((!exitg1) && (lastc < 3)) {
    if (!(AEB_in[lastc - 1] == 0.0)) {
      goto150 = false;
      exitg1 = true;
    } else {
      lastc++;
    }
  }

  //  ACC Y
  //  DEC Y
  //  ESL X
  //  ESL Y
  //  ELCL X
  //  ELCL Y
  //  ESS X
  //  ESS Y
  //  ACC Y
  cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
  if (goto150) {
    b_i = 0;
  } else {
    b_i = -6;
  }

  BEV_image_kernel4<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(b_i, gpu_X_AB,
    *gpu_TJ_X, *gpu_TJ_Y);

  //  DEC Y
  if ((Training_data_data[5] >= -25.0) && (Training_data_data[5] <= 25.0) &&
      (Training_data_data[6] >= -6.0) && (Training_data_data[6] <= 6.0)) {
    flag[0] = 1.0;
  } else {
    flag[0] = 0.0;
  }

  scale = 3.3121686421112381E-170;
  Class_B_idx_9 = std::abs(Training_data_data[8]);
  if (Class_B_idx_9 > 3.3121686421112381E-170) {
    temp = 1.0;
    scale = Class_B_idx_9;
  } else {
    t = Class_B_idx_9 / 3.3121686421112381E-170;
    temp = t * t;
  }

  Class_B_idx_9 = std::abs(Class_B_idx_10);
  if (Class_B_idx_9 > scale) {
    t = scale / Class_B_idx_9;
    temp = temp * t * t + 1.0;
    scale = Class_B_idx_9;
  } else {
    t = Class_B_idx_9 / scale;
    temp += t * t;
  }

  temp = scale * std::sqrt(temp);
  y_out[0] = Training_data_data[6];
  y_out[1] = Training_data_data[5];
  y_out[2] = -Training_data_data[10];
  y_out_dirtyOnCpu = true;
  x_ini[0] = Training_data_data[6];
  x_ini[1] = Training_data_data[5];
  x_ini[2] = -Training_data_data[10];
  cudaMemcpy(gpu_temp, &temp, 8UL, cudaMemcpyHostToDevice);
  cudaMemcpy(*gpu_x_ini, x_ini, 40UL, cudaMemcpyHostToDevice);
  BEV_image_kernel5<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_temp,
    *gpu_x_ini);

  //  hansaem
  if (old_flag_dirtyOnGpu) {
    cudaMemcpy(old_flag, *gpu_old_flag, 120UL, cudaMemcpyDeviceToHost);
  }

  if ((old_flag[0] == 0.0) && (flag[0] == 1.0)) {
    old_Prob_ctrv = 0.8;
    old_Prob_cv = 0.2;
  }

  // zeros(5,1);
  // zeros(5,1);
  //  ts=0.2;
  // -------------------------------------------------------------------------
  // -------------------------------------------------------------------------
  //  Prob_ctrv_old=;
  //  Prob_cv_old=;
  //  x_ctrv_old=;
  //  x_cv_old=;
  laneInfoL_idx_3 = 0.9 * old_Prob_ctrv + 0.1 * old_Prob_cv;
  laneInfoL_idx_4 = 0.1 * old_Prob_ctrv + 0.9 * old_Prob_cv;
  Class_B_idx_9 = 0.1 * old_Prob_cv / laneInfoL_idx_3;
  BEV_image_kernel6<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(laneInfoL_idx_4,
    old_Prob_cv, old_Prob_ctrv, laneInfoL_idx_3, *gpu_x_ini, *gpu_tmp_ego_y,
    *gpu_work, *gpu_x_cv_out, *gpu_x_ctrv_out);
  cudaMemcpy(*gpu_out_P_ctrv, out_P_ctrv, 200UL, cudaMemcpyHostToDevice);
  BEV_image_kernel7<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_tmp_ego_y,
    *gpu_work, *gpu_out_P_ctrv, *gpu_L);
  cudaMemcpy(out_P_ctrv, *gpu_out_P_ctrv, 200UL, cudaMemcpyDeviceToHost);
  BEV_image_kernel8<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_ctrv_out,
    *gpu_x_ini, *gpu_tmp_ego_y, *gpu_work);
  cudaMemcpy(*gpu_out_P_cv, out_P_cv, 200UL, cudaMemcpyHostToDevice);
  BEV_image_kernel9<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_tmp_ego_y,
    *gpu_work, *gpu_out_P_cv, *gpu_Vr);
  cudaMemcpy(out_P_cv, *gpu_out_P_cv, 200UL, cudaMemcpyDeviceToHost);
  cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL, cudaMemcpyHostToDevice);
  BEV_image_kernel10<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_Vr,
    gpu_Class_B_idx_9, *gpu_L, 0.9 * old_Prob_ctrv / laneInfoL_idx_3,
    *gpu_P_ctrv_out);
  Class_B_idx_9 = 0.9 * old_Prob_cv / laneInfoL_idx_4;
  BEV_image_kernel11<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_cv_out,
    *gpu_x_ini, *gpu_tmp_ego_y, *gpu_work);
  cudaMemcpy(*gpu_out_P_ctrv, out_P_ctrv, 200UL, cudaMemcpyHostToDevice);
  BEV_image_kernel12<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_tmp_ego_y,
    *gpu_work, *gpu_out_P_ctrv, *gpu_L);
  cudaMemcpy(out_P_ctrv, *gpu_out_P_ctrv, 200UL, cudaMemcpyDeviceToHost);
  BEV_image_kernel13<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_cv_out,
    *gpu_x_ini, *gpu_tmp_ego_y, *gpu_work);
  tmp_ego_y_dirtyOnGpu = true;
  cudaMemcpy(*gpu_out_P_cv, out_P_cv, 200UL, cudaMemcpyHostToDevice);
  BEV_image_kernel14<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_tmp_ego_y,
    *gpu_work, *gpu_out_P_cv, *gpu_Vr);
  cudaMemcpy(out_P_cv, *gpu_out_P_cv, 200UL, cudaMemcpyDeviceToHost);
  Vr_dirtyOnCpu = false;
  cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL, cudaMemcpyHostToDevice);
  BEV_image_kernel15<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_Vr,
    gpu_Class_B_idx_9, *gpu_L, 0.1 * old_Prob_ctrv / laneInfoL_idx_4,
    *gpu_P_cv_out);
  for (int sample_ts{0}; sample_ts < 10; sample_ts++) {
    creal_T t1;
    int exitg2;
    int exitg5;
    int iaii;
    int im1n;
    int in;
    int lastv;
    bool exitg3;
    bool exitg4;
    ts = (static_cast<double>(sample_ts) + 1.0) * 0.2;
    if (P_ctrv_dirtyOnCpu) {
      cudaMemcpy(*gpu_P_ctrv, P_ctrv, 200UL, cudaMemcpyHostToDevice);
    }

    BEV_image_kernel16<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_ctrv,
      sample_ts, *gpu_P_ctrv_tmp);

    //  if time >=18.49
    //      time = time;
    //  end
    //  x_k = [y x yaw v yawrate]';
    // -------------------------------------------------------------------------
    //  Parameter
    //  ts = 0.01;
    BEV_image_kernel17<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_ctrv_out,
      *gpu_L);
    ipiv_t_dirtyOnGpu = false;
    L_dirtyOnGpu = true;
    goto150 = true;
    for (lastc = 0; lastc < 25; lastc++) {
      if (goto150) {
        if (L_dirtyOnGpu) {
          cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
        }

        L_dirtyOnGpu = false;
        if (std::isinf(L[lastc]) || std::isnan(L[lastc])) {
          goto150 = false;
        }
      } else {
        goto150 = false;
      }
    }

    if (!goto150) {
      if (U_dirtyOnCpu) {
        cudaMemcpy(*gpu_U, U, 400UL, cudaMemcpyHostToDevice);
      }

      BEV_image_kernel51<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_U);
      U_dirtyOnCpu = false;
      Y_AB_dirtyOnGpu = true;
      for (j = 0; j < 3; j++) {
        DEC_param = j + 2;
        for (i = 0; i <= 4 - DEC_param; i++) {
          if (Y_AB_dirtyOnGpu) {
            cudaMemcpy(U, *gpu_U, 400UL, cudaMemcpyDeviceToHost);
          }

          U[(DEC_param + i) + 5 * j].re = 0.0;
          U[(DEC_param + i) + 5 * j].im = 0.0;
          Y_AB_dirtyOnGpu = false;
          U_dirtyOnCpu = true;
        }
      }

      BEV_image_kernel52<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp);
      validLaunchParams = true;
    } else {
      BEV_image_kernel18<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_work);
      old_flag_dirtyOnGpu = true;
      for (i = 0; i < 4; i++) {
        im1n = i * 5 + 2;
        in = (i + 1) * 5;
        if (L_dirtyOnGpu) {
          cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
        }

        L_dirtyOnGpu = false;
        Y_AB = L[(i + 5 * i) + 1];
        Y_AB_dirtyOnCpu = true;
        DEC_param = i + 3;
        if (i + 3 >= 5) {
          DEC_param = 5;
        }

        jA = (DEC_param + i * 5) - 1;
        tau[i] = 0.0;
        X_AB = 0.0;
        if (3 - i >= 1) {
          if (3 - i == 1) {
            X_AB = std::abs(L[jA]);
          } else {
            scale = 3.3121686421112381E-170;
            DEC_param = (jA - i) + 2;
            for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
              Class_B_idx_9 = std::abs(L[jA + lastc]);
              if (Class_B_idx_9 > scale) {
                t = scale / Class_B_idx_9;
                X_AB = X_AB * t * t + 1.0;
                scale = Class_B_idx_9;
              } else {
                t = Class_B_idx_9 / scale;
                X_AB += t * t;
              }
            }

            X_AB = scale * std::sqrt(X_AB);
          }
        }

        if (X_AB != 0.0) {
          X_AB = rt_hypotd_snf(L[(i + 5 * i) + 1], X_AB);
          if (L[(i + 5 * i) + 1] >= 0.0) {
            X_AB = -X_AB;
          }

          if (std::abs(X_AB) < 1.0020841800044864E-292) {
            knt = -1;
            DEC_param = (jA - i) + 2;
            do {
              knt++;
              for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
                k = jA + lastc;
                L[k] *= 9.9792015476736E+291;
              }

              X_AB *= 9.9792015476736E+291;
              Y_AB *= 9.9792015476736E+291;
            } while (!(std::abs(X_AB) >= 1.0020841800044864E-292));

            X_AB = 0.0;
            if (3 - i >= 1) {
              if (3 - i == 1) {
                X_AB = std::abs(L[jA]);
              } else {
                scale = 3.3121686421112381E-170;
                DEC_param = (jA - i) + 2;
                for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
                  Class_B_idx_9 = std::abs(L[jA + lastc]);
                  if (Class_B_idx_9 > scale) {
                    t = scale / Class_B_idx_9;
                    X_AB = X_AB * t * t + 1.0;
                    scale = Class_B_idx_9;
                  } else {
                    t = Class_B_idx_9 / scale;
                    X_AB += t * t;
                  }
                }

                X_AB = scale * std::sqrt(X_AB);
              }
            }

            X_AB = rt_hypotd_snf(Y_AB, X_AB);
            if (Y_AB >= 0.0) {
              X_AB = -X_AB;
            }

            tau[i] = (X_AB - Y_AB) / X_AB;
            a = 1.0 / (Y_AB - X_AB);
            DEC_param = (jA - i) + 2;
            for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
              k = jA + lastc;
              L[k] *= a;
            }

            validLaunchParams = mwGetLaunchParameters(static_cast<double>(knt +
              1L), &grid, &block, 1024U, 65535U);
            if (validLaunchParams) {
              cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
              BEV_image_kernel19<<<grid, block>>>(knt, gpu_X_AB);
              cudaMemcpy(&X_AB, gpu_X_AB, 8UL, cudaMemcpyDeviceToHost);
            }

            Y_AB = X_AB;
          } else {
            tau[i] = (X_AB - L[(i + 5 * i) + 1]) / X_AB;
            a = 1.0 / (L[(i + 5 * i) + 1] - X_AB);
            DEC_param = (jA - i) + 2;
            for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
              k = jA + lastc;
              L[k] *= a;
            }

            Y_AB = X_AB;
          }
        }

        L[(i + 5 * i) + 1] = 1.0;
        ipiv_t_dirtyOnGpu = true;
        iaii = (i + im1n) - 1;
        knt = in + 1;
        if (tau[i] != 0.0) {
          lastv = 3 - i;
          b_i = (iaii - i) + 3;
          while ((lastv + 1 > 0) && (L[b_i] == 0.0)) {
            lastv--;
            b_i--;
          }

          lastc = 5;
          exitg1 = false;
          while ((!exitg1) && (lastc > 0)) {
            DEC_param = in + lastc;
            k = DEC_param;
            do {
              exitg2 = 0;
              if (k <= DEC_param + lastv * 5) {
                if (L[k - 1] != 0.0) {
                  exitg2 = 1;
                } else {
                  k += 5;
                }
              } else {
                lastc--;
                exitg2 = 2;
              }
            } while (exitg2 == 0);

            if (exitg2 == 1) {
              exitg1 = true;
            }
          }
        } else {
          lastv = -1;
          lastc = 0;
        }

        if (lastv + 1 > 0) {
          if (lastc != 0) {
            validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
              ((lastc - 1) + 1L), &grid, &block, 1024U, 65535U);
            if (validLaunchParams) {
              BEV_image_kernel20<<<grid, block>>>(lastc, *gpu_work);
              old_flag_dirtyOnGpu = true;
            }

            ix = iaii;
            DEC_param = (in + 5 * lastv) + 1;
            for (m = 0; m <= (DEC_param - knt) / 5; m++) {
              b_L = in + m * 5;
              nr = (b_L + lastc) - 1;
              validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
                ((nr - b_L) + 1L), &grid, &block, 1024U, 65535U);
              if (validLaunchParams) {
                if (ipiv_t_dirtyOnGpu) {
                  cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
                }

                ipiv_t_dirtyOnGpu = false;
                cudaMemcpy(b_gpu_L, &b_L, 4UL, cudaMemcpyHostToDevice);
                cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
                BEV_image_kernel21<<<grid, block>>>(ix, *gpu_L, b_gpu_L, gpu_nr,
                  *gpu_work);
                old_flag_dirtyOnGpu = true;
              }

              ix++;
            }
          }

          if (!(-tau[i] == 0.0)) {
            jA = in;
            for (j = 0; j <= lastv; j++) {
              if (L[iaii + j] != 0.0) {
                temp = L[iaii + j] * -tau[i];
                DEC_param = jA + 1;
                nr = lastc + jA;
                for (knt = 0; knt <= nr - DEC_param; knt++) {
                  ix = jA + knt;
                  if (old_flag_dirtyOnGpu) {
                    cudaMemcpy(work, *gpu_work, 40UL, cudaMemcpyDeviceToHost);
                  }

                  old_flag_dirtyOnGpu = false;
                  L[ix] += work[knt] * temp;
                  ipiv_t_dirtyOnGpu = true;
                }
              }

              jA += 5;
            }
          }
        }

        iaii = (i + im1n) - 1;
        knt = (i + in) + 2;
        if (tau[i] != 0.0) {
          lastv = 4 - i;
          b_i = (iaii - i) + 3;
          while ((lastv > 0) && (L[b_i] == 0.0)) {
            lastv--;
            b_i--;
          }

          lastc = 4 - i;
          exitg1 = false;
          while ((!exitg1) && (lastc > 0)) {
            DEC_param = knt + (lastc - 1) * 5;
            k = DEC_param;
            do {
              exitg2 = 0;
              if (k <= (DEC_param + lastv) - 1) {
                if (L[k - 1] != 0.0) {
                  exitg2 = 1;
                } else {
                  k++;
                }
              } else {
                lastc--;
                exitg2 = 2;
              }
            } while (exitg2 == 0);

            if (exitg2 == 1) {
              exitg1 = true;
            }
          }
        } else {
          lastv = 0;
          lastc = 0;
        }

        if (lastv > 0) {
          if (lastc != 0) {
            validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
              ((lastc - 1) + 1L), &grid, &block, 1024U, 65535U);
            if (validLaunchParams) {
              BEV_image_kernel22<<<grid, block>>>(lastc, *gpu_work);
              old_flag_dirtyOnGpu = true;
            }

            DEC_param = knt + 5 * (lastc - 1);
            validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
              ((DEC_param - knt) / 5 + 1L), &grid, &block, 1024U, 65535U);
            if (validLaunchParams) {
              if (ipiv_t_dirtyOnGpu) {
                cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
              }

              BEV_image_kernel23<<<grid, block>>>(iaii, *gpu_L, lastv, knt,
                DEC_param, *gpu_work);
              old_flag_dirtyOnGpu = true;
            }
          }

          if (!(-tau[i] == 0.0)) {
            jA = knt - 1;
            for (j = 0; j < lastc; j++) {
              if (old_flag_dirtyOnGpu) {
                cudaMemcpy(work, *gpu_work, 40UL, cudaMemcpyDeviceToHost);
              }

              old_flag_dirtyOnGpu = false;
              d = work[j];
              if (d != 0.0) {
                temp = d * -tau[i];
                DEC_param = jA + 1;
                nr = lastv + jA;
                for (knt = 0; knt <= nr - DEC_param; knt++) {
                  ix = jA + knt;
                  L[ix] += L[iaii + knt] * temp;
                }
              }

              jA += 5;
            }
          }
        }

        L[(i + 5 * i) + 1] = Y_AB;
        ipiv_t_dirtyOnGpu = true;
      }

      if (ipiv_t_dirtyOnGpu) {
        cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
      }

      BEV_image_kernel24<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L,
        *gpu_Vr);
      Vr_dirtyOnGpu = true;
      for (j = 0; j < 4; j++) {
        knt = 4 - j;
        k = (4 - j) * 5;
        for (i = 0; i < knt; i++) {
          if (Vr_dirtyOnGpu) {
            cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
          }

          Vr[k + i] = 0.0;
          Vr_dirtyOnGpu = false;
          Vr_dirtyOnCpu = true;
        }

        for (i = 0; i <= 3 - knt; i++) {
          b_i = (i - j) + 4;
          if (Vr_dirtyOnGpu) {
            cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
          }

          Vr[(k + b_i) + 1] = Vr[(k + b_i) - 4];
          Vr_dirtyOnGpu = false;
          Vr_dirtyOnCpu = true;
        }
      }

      if (Vr_dirtyOnCpu) {
        cudaMemcpy(*gpu_Vr, Vr, 200UL, cudaMemcpyHostToDevice);
      }

      BEV_image_kernel25<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_Vr);
      BEV_image_kernel26<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_Vr);
      Vr_dirtyOnCpu = false;
      Vr_dirtyOnGpu = true;
      BEV_image_kernel27<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_work);
      old_flag_dirtyOnGpu = true;
      for (i = 0; i < 4; i++) {
        b_i = 4 - i;
        iaii = ((3 - i) * 5 - i) + 15;
        if (4 - i < 4) {
          if (Vr_dirtyOnCpu) {
            cudaMemcpy(*gpu_Vr, Vr, 200UL, cudaMemcpyHostToDevice);
          }

          BEV_image_kernel28<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(iaii,
            *gpu_Vr);
          Vr_dirtyOnGpu = true;
          if (tau[3 - i] != 0.0) {
            lastv = i + 1;
            DEC_param = (iaii + i) - 4;
            exitg1 = false;
            while ((!exitg1) && (lastv > 0)) {
              if (Vr_dirtyOnGpu) {
                cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
              }

              Vr_dirtyOnGpu = false;
              if (Vr[DEC_param - 2] == 0.0) {
                lastv--;
                DEC_param--;
              } else {
                exitg1 = true;
              }
            }

            lastc = i;
            exitg1 = false;
            while ((!exitg1) && (lastc > 0)) {
              DEC_param = iaii + (lastc - 1) * 5;
              k = DEC_param;
              do {
                exitg2 = 0;
                if (k <= (DEC_param + lastv) - 1) {
                  if (Vr_dirtyOnGpu) {
                    cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
                  }

                  Vr_dirtyOnGpu = false;
                  if (Vr[k - 1] != 0.0) {
                    exitg2 = 1;
                  } else {
                    k++;
                  }
                } else {
                  lastc--;
                  exitg2 = 2;
                }
              } while (exitg2 == 0);

              if (exitg2 == 1) {
                exitg1 = true;
              }
            }
          } else {
            lastv = 0;
            lastc = 0;
          }

          if (lastv > 0) {
            if (lastc != 0) {
              validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
                ((lastc - 1) + 1L), &grid, &block, 1024U, 65535U);
              if (validLaunchParams) {
                BEV_image_kernel29<<<grid, block>>>(lastc, *gpu_work);
                old_flag_dirtyOnGpu = true;
              }

              DEC_param = iaii + 5 * (lastc - 1);
              validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
                ((DEC_param - iaii) / 5 + 1L), &grid, &block, 1024U, 65535U);
              if (validLaunchParams) {
                BEV_image_kernel30<<<grid, block>>>(*gpu_Vr, lastv, iaii,
                  DEC_param, *gpu_work);
                old_flag_dirtyOnGpu = true;
              }
            }

            if (!(-tau[3 - i] == 0.0)) {
              jA = iaii - 1;
              for (j = 0; j < lastc; j++) {
                if (old_flag_dirtyOnGpu) {
                  cudaMemcpy(work, *gpu_work, 40UL, cudaMemcpyDeviceToHost);
                }

                old_flag_dirtyOnGpu = false;
                d = work[j];
                if (d != 0.0) {
                  temp = d * -tau[3 - i];
                  DEC_param = jA + 1;
                  nr = lastv + jA;
                  for (knt = 0; knt <= nr - DEC_param; knt++) {
                    ix = jA + knt;
                    if (Vr_dirtyOnGpu) {
                      cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
                    }

                    Vr[ix] += Vr[(iaii + knt) - 6] * temp;
                    Vr_dirtyOnGpu = false;
                  }
                }

                jA += 5;
              }
            }
          }

          jA = iaii - 3;
          DEC_param = (iaii + i) - 4;
          for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
            k = (iaii + lastc) - 5;
            if (Vr_dirtyOnGpu) {
              cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
            }

            Vr[k] *= -tau[3 - i];
            Vr_dirtyOnGpu = false;
          }
        }

        if (Vr_dirtyOnGpu) {
          cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
        }

        Vr[iaii - 6] = 1.0 - tau[3 - i];
        Vr_dirtyOnGpu = false;
        Vr_dirtyOnCpu = true;
        for (j = 0; j <= b_i - 2; j++) {
          Vr[(iaii - j) - 7] = 0.0;
        }
      }

      BEV_image_kernel31<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v);
      v_dirtyOnGpu = true;
      if (L_dirtyOnGpu) {
        cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
      }

      L[2] = 0.0;
      L[3] = 0.0;
      L[8] = 0.0;
      L[9] = 0.0;
      L[14] = 0.0;
      ipiv_t_dirtyOnGpu = true;
      i = 4;
      exitg1 = false;
      while ((!exitg1) && (i + 1 >= 1)) {
        b_L = 1;
        goto150 = false;
        jA = 0;
        exitg3 = false;
        while ((!exitg3) && (jA < 301)) {
          lastc = i;
          exitg4 = false;
          while ((!exitg4) && ((lastc + 1 > b_L) && (!(std::abs(L[lastc + 5 *
                     (lastc - 1)]) <= 5.0104209000224319E-292)))) {
            if (ipiv_t_dirtyOnGpu) {
              cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
            }

            ipiv_t_dirtyOnGpu = false;
            if (Y_AB_dirtyOnCpu) {
              cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
            }

            BEV_image_kernel32<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L,
              lastc, gpu_Y_AB);
            Y_AB_dirtyOnCpu = false;
            cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
            Y_AB_dirtyOnGpu = false;
            if (Y_AB == 0.0) {
              if (lastc - 1 >= 1) {
                BEV_image_kernel33<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                  (*gpu_L, lastc, gpu_Y_AB);
                Y_AB_dirtyOnGpu = true;
              }

              if (lastc + 2 <= 5) {
                BEV_image_kernel34<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                  (*gpu_L, lastc, gpu_Y_AB);
                Y_AB_dirtyOnGpu = true;
              }
            }

            if (Y_AB_dirtyOnGpu) {
              cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
            }

            if (std::abs(L[lastc + 5 * (lastc - 1)]) <= 2.2204460492503131E-16 *
                Y_AB) {
              Y_AB = std::abs(L[lastc + 5 * (lastc - 1)]);
              X_AB = std::abs(L[(lastc + 5 * lastc) - 1]);
              if (Y_AB > X_AB) {
                Class_B_idx_9 = Y_AB;
                Class_B_idx_10 = X_AB;
              } else {
                Class_B_idx_9 = X_AB;
                Class_B_idx_10 = Y_AB;
              }

              Y_AB = std::abs(L[lastc + 5 * lastc]);
              Y_AB_dirtyOnCpu = true;
              X_AB = std::abs(L[(lastc + 5 * (lastc - 1)) - 1] - L[lastc + 5 *
                              lastc]);
              if (Y_AB > X_AB) {
                temp = Y_AB;
                Y_AB = X_AB;
              } else {
                temp = X_AB;
              }

              scale = temp + Class_B_idx_9;
              if (Class_B_idx_10 * (Class_B_idx_9 / scale) <= std::fmax
                  (5.0104209000224319E-292, 2.2204460492503131E-16 * (Y_AB *
                    (temp / scale)))) {
                exitg4 = true;
              } else {
                lastc--;
              }
            } else {
              lastc--;
            }
          }

          b_L = lastc + 1;
          if (lastc + 1 > 1) {
            L[lastc + 5 * (lastc - 1)] = 0.0;
            ipiv_t_dirtyOnGpu = true;
          }

          if (lastc + 1 >= i) {
            goto150 = true;
            exitg3 = true;
          } else {
            if (jA == 10) {
              scale = std::abs(L[(lastc + 5 * lastc) + 1]) + std::abs(L[(lastc +
                5 * (lastc + 1)) + 2]);
              X_AB = 0.75 * scale + L[lastc + 5 * lastc];
              Class_B_idx_9 = -0.4375 * scale;
              Class_B_idx_9_dirtyOnCpu = true;
              Y_AB = scale;
              temp = X_AB;
            } else if (jA == 20) {
              scale = std::abs(L[i + 5 * (i - 1)]) + std::abs(L[(i + 5 * (i - 2))
                - 1]);
              X_AB = 0.75 * scale + L[i + 5 * i];
              Class_B_idx_9 = -0.4375 * scale;
              Class_B_idx_9_dirtyOnCpu = true;
              Y_AB = scale;
              temp = X_AB;
            } else {
              X_AB = L[(i + 5 * (i - 1)) - 1];
              Y_AB = L[i + 5 * (i - 1)];
              Class_B_idx_9 = L[(i + 5 * i) - 1];
              Class_B_idx_9_dirtyOnCpu = true;
              temp = L[i + 5 * i];
            }

            scale = ((std::abs(X_AB) + std::abs(Class_B_idx_9)) + std::abs(Y_AB))
              + std::abs(temp);
            if (scale == 0.0) {
              t = 0.0;
              X_AB = 0.0;
              Class_B_idx_10 = 0.0;
              Y_AB = 0.0;
              Y_AB_dirtyOnCpu = true;
            } else {
              X_AB /= scale;
              Y_AB /= scale;
              Class_B_idx_9 /= scale;
              temp /= scale;
              Class_B_idx_10 = (X_AB + temp) / 2.0;
              X_AB = (X_AB - Class_B_idx_10) * (temp - Class_B_idx_10) -
                Class_B_idx_9 * Y_AB;
              Y_AB = std::sqrt(std::abs(X_AB));
              if (X_AB >= 0.0) {
                t = Class_B_idx_10 * scale;
                Class_B_idx_10 = t;
                X_AB = Y_AB * scale;
                Y_AB = -X_AB;
                Y_AB_dirtyOnCpu = true;
              } else {
                t = Class_B_idx_10 + Y_AB;
                Class_B_idx_10 -= Y_AB;
                if (std::abs(t - temp) <= std::abs(Class_B_idx_10 - temp)) {
                  t *= scale;
                  Class_B_idx_10 = t;
                } else {
                  Class_B_idx_10 *= scale;
                  t = Class_B_idx_10;
                }

                X_AB = 0.0;
                Y_AB = 0.0;
                Y_AB_dirtyOnCpu = true;
              }
            }

            m = i - 1;
            exitg4 = false;
            while ((!exitg4) && (m >= lastc + 1)) {
              scale = (std::abs(L[(m + 5 * (m - 1)) - 1] - Class_B_idx_10) + std::
                       abs(Y_AB)) + std::abs(L[m + 5 * (m - 1)]);
              Class_B_idx_9 = L[m + 5 * (m - 1)] / scale;
              if (v_dirtyOnGpu) {
                cudaMemcpy(v, *gpu_v, 24UL, cudaMemcpyDeviceToHost);
              }

              v[0] = (Class_B_idx_9 * L[(m + 5 * m) - 1] + (L[(m + 5 * (m - 1))
                       - 1] - t) * ((L[(m + 5 * (m - 1)) - 1] - Class_B_idx_10) /
                                    scale)) - X_AB * (Y_AB / scale);
              v[1] = Class_B_idx_9 * (((L[(m + 5 * (m - 1)) - 1] + L[m + 5 * m])
                - t) - Class_B_idx_10);
              v[2] = Class_B_idx_9 * L[(m + 5 * m) + 1];
              scale = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
              v[0] /= scale;
              v[1] /= scale;
              v[2] /= scale;
              v_dirtyOnGpu = false;
              v_dirtyOnCpu = true;
              if ((m == lastc + 1) || (std::abs(L[(m + 5 * (m - 2)) - 1]) * (std::
                    abs(v[1]) + std::abs(v[2])) <= 2.2204460492503131E-16 * std::
                   abs(v[0]) * ((std::abs(L[(m + 5 * (m - 2)) - 2]) + std::abs
                                 (L[(m + 5 * (m - 1)) - 1])) + std::abs(L[m + 5 *
                     m])))) {
                exitg4 = true;
              } else {
                m--;
              }
            }

            for (k = 0; k <= i - m; k++) {
              ix = m + k;
              nr = (i - ix) + 2;
              nr_dirtyOnCpu = true;
              if (3 < nr) {
                nr = 3;
              }

              if (ix > m) {
                validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
                  ((nr - 1) + 1L), &grid, &block, 1024U, 65535U);
                if (validLaunchParams) {
                  if (ipiv_t_dirtyOnGpu) {
                    cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
                  }

                  ipiv_t_dirtyOnGpu = false;
                  cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
                  nr_dirtyOnCpu = false;
                  if (v_dirtyOnCpu) {
                    cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
                  }

                  BEV_image_kernel35<<<grid, block>>>(*gpu_L, (ix + 5 * (ix - 2))
                    - 1, gpu_nr, *gpu_v);
                  v_dirtyOnCpu = false;
                  v_dirtyOnGpu = true;
                }
              }

              if (v_dirtyOnGpu) {
                cudaMemcpy(v, *gpu_v, 24UL, cudaMemcpyDeviceToHost);
              }

              v_dirtyOnGpu = false;
              Y_AB = v[0];
              Class_B_idx_10 = 0.0;
              Class_B_idx_10_dirtyOnGpu = false;
              old_flag_dirtyOnGpu = true;
              if (nr > 0) {
                X_AB = 0.0;
                if (nr - 1 >= 1) {
                  if (nr - 1 == 1) {
                    X_AB = std::abs(v[1]);
                  } else {
                    scale = 3.3121686421112381E-170;
                    Class_B_idx_9 = std::abs(v[1]);
                    if (Class_B_idx_9 > 3.3121686421112381E-170) {
                      X_AB = 1.0;
                      scale = Class_B_idx_9;
                    } else {
                      t = Class_B_idx_9 / 3.3121686421112381E-170;
                      X_AB = t * t;
                    }

                    Class_B_idx_9 = std::abs(v[2]);
                    Class_B_idx_9_dirtyOnCpu = true;
                    if (Class_B_idx_9 > scale) {
                      t = scale / Class_B_idx_9;
                      X_AB = X_AB * t * t + 1.0;
                      scale = Class_B_idx_9;
                    } else {
                      t = Class_B_idx_9 / scale;
                      X_AB += t * t;
                    }

                    X_AB = scale * std::sqrt(X_AB);
                  }
                }

                if (X_AB != 0.0) {
                  X_AB = rt_hypotd_snf(v[0], X_AB);
                  if (v[0] >= 0.0) {
                    X_AB = -X_AB;
                  }

                  if (std::abs(X_AB) < 1.0020841800044864E-292) {
                    knt = -1;
                    do {
                      knt++;
                      validLaunchParams = mwGetLaunchParameters1D(static_cast<
                        double>((nr - 2) + 1L), &grid, &block, 1024U, 65535U);
                      if (validLaunchParams) {
                        if (nr_dirtyOnCpu) {
                          cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
                        }

                        nr_dirtyOnCpu = false;
                        if (v_dirtyOnCpu) {
                          cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
                        }

                        BEV_image_kernel38<<<grid, block>>>(gpu_nr, *gpu_v);
                        v_dirtyOnCpu = false;
                        v_dirtyOnGpu = true;
                      }

                      X_AB *= 9.9792015476736E+291;
                      Y_AB *= 9.9792015476736E+291;
                    } while (!(std::abs(X_AB) >= 1.0020841800044864E-292));

                    X_AB = 0.0;
                    if (nr - 1 >= 1) {
                      if (nr - 1 == 1) {
                        if (v_dirtyOnCpu) {
                          cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
                        }

                        v_dirtyOnCpu = false;
                        cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
                        BEV_image_kernel39<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                          (*gpu_v, gpu_X_AB);
                        cudaMemcpy(&X_AB, gpu_X_AB, 8UL, cudaMemcpyDeviceToHost);
                      } else {
                        scale = 3.3121686421112381E-170;
                        if (v_dirtyOnGpu) {
                          cudaMemcpy(v, *gpu_v, 24UL, cudaMemcpyDeviceToHost);
                        }

                        Class_B_idx_9 = std::abs(v[1]);
                        if (Class_B_idx_9 > 3.3121686421112381E-170) {
                          X_AB = 1.0;
                          scale = Class_B_idx_9;
                        } else {
                          t = Class_B_idx_9 / 3.3121686421112381E-170;
                          X_AB = t * t;
                        }

                        Class_B_idx_9 = std::abs(v[2]);
                        Class_B_idx_9_dirtyOnCpu = true;
                        if (Class_B_idx_9 > scale) {
                          t = scale / Class_B_idx_9;
                          X_AB = X_AB * t * t + 1.0;
                          scale = Class_B_idx_9;
                        } else {
                          t = Class_B_idx_9 / scale;
                          X_AB += t * t;
                        }

                        X_AB = scale * std::sqrt(X_AB);
                      }
                    }

                    X_AB = rt_hypotd_snf(Y_AB, X_AB);
                    if (Y_AB >= 0.0) {
                      X_AB = -X_AB;
                    }

                    Class_B_idx_10 = (X_AB - Y_AB) / X_AB;
                    validLaunchParams = mwGetLaunchParameters1D(static_cast<
                      double>((nr - 2) + 1L), &grid, &block, 1024U, 65535U);
                    if (validLaunchParams) {
                      if (nr_dirtyOnCpu) {
                        cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
                      }

                      if (v_dirtyOnCpu) {
                        cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
                      }

                      BEV_image_kernel40<<<grid, block>>>(1.0 / (Y_AB - X_AB),
                        gpu_nr, *gpu_v);
                      v_dirtyOnCpu = false;
                    }

                    validLaunchParams = mwGetLaunchParameters(static_cast<double>
                      (knt + 1L), &grid, &block, 1024U, 65535U);
                    if (validLaunchParams) {
                      cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
                      BEV_image_kernel41<<<grid, block>>>(knt, gpu_X_AB);
                      cudaMemcpy(&X_AB, gpu_X_AB, 8UL, cudaMemcpyDeviceToHost);
                    }

                    Y_AB = X_AB;
                  } else {
                    if (v_dirtyOnCpu) {
                      cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
                    }

                    v_dirtyOnCpu = false;
                    cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
                    cudaMemcpy(gpu_Class_B_idx_10, &Class_B_idx_10, 8UL,
                               cudaMemcpyHostToDevice);
                    BEV_image_kernel36<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*
                      gpu_v, gpu_X_AB, gpu_Class_B_idx_10);
                    old_flag_dirtyOnGpu = false;
                    Class_B_idx_10_dirtyOnGpu = true;
                    a = 1.0 / (v[0] - X_AB);
                    validLaunchParams = mwGetLaunchParameters1D(static_cast<
                      double>((nr - 2) + 1L), &grid, &block, 1024U, 65535U);
                    if (validLaunchParams) {
                      if (nr_dirtyOnCpu) {
                        cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
                      }

                      BEV_image_kernel37<<<grid, block>>>(a, gpu_nr, *gpu_v);
                    }

                    Y_AB = X_AB;
                  }
                }
              }

              cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
              if (v_dirtyOnCpu) {
                cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
              }

              BEV_image_kernel42<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (gpu_Y_AB, *gpu_v);
              v_dirtyOnCpu = false;
              if (ix > m) {
                L[(ix + 5 * (ix - 2)) - 1] = Y_AB;
                L[ix + 5 * (ix - 2)] = 0.0;
                ipiv_t_dirtyOnGpu = true;
                if (ix < i) {
                  L[(ix + 5 * (ix - 2)) + 1] = 0.0;
                }
              } else if (m > lastc + 1) {
                if (Class_B_idx_10_dirtyOnGpu) {
                  cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                             cudaMemcpyDeviceToHost);
                }

                Class_B_idx_10_dirtyOnGpu = false;
                L[(ix + 5 * (ix - 2)) - 1] *= 1.0 - Class_B_idx_10;
                ipiv_t_dirtyOnGpu = true;
              }

              cudaMemcpy(v, *gpu_v, 24UL, cudaMemcpyDeviceToHost);
              v_dirtyOnGpu = false;
              d = v[1];
              if (old_flag_dirtyOnGpu) {
                cudaMemcpy(gpu_Class_B_idx_10, &Class_B_idx_10, 8UL,
                           cudaMemcpyHostToDevice);
              }

              BEV_image_kernel43<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v,
                gpu_Class_B_idx_10, gpu_Y_AB);
              Y_AB_dirtyOnCpu = false;
              Y_AB_dirtyOnGpu = true;
              if (nr == 3) {
                scale = v[2];
                if (Class_B_idx_9_dirtyOnCpu) {
                  cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL,
                             cudaMemcpyHostToDevice);
                }

                BEV_image_kernel44<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                  (*gpu_v, gpu_Class_B_idx_10, gpu_Class_B_idx_9);
                Class_B_idx_9_dirtyOnCpu = false;
                old_flag_dirtyOnGpu = true;
                for (j = 0; j <= 5 - ix; j++) {
                  knt = (ix + j) - 1;
                  X_AB = (L[(ix + 5 * knt) - 1] + d * L[ix + 5 * knt]) + scale *
                    L[(ix + 5 * knt) + 1];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  L[(ix + 5 * knt) - 1] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  L[ix + 5 * knt] -= X_AB * Y_AB;
                  if (old_flag_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_9, gpu_Class_B_idx_9, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  old_flag_dirtyOnGpu = false;
                  L[(ix + 5 * knt) + 1] -= X_AB * Class_B_idx_9;
                  ipiv_t_dirtyOnGpu = true;
                }

                if (ix + 3 < i + 1) {
                  DEC_param = ix + 2;
                } else {
                  DEC_param = i;
                }

                for (j = 0; j <= DEC_param; j++) {
                  X_AB = (L[j + 5 * (ix - 1)] + d * L[j + 5 * ix]) + scale * L[j
                    + 5 * (ix + 1)];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  L[j + 5 * (ix - 1)] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  L[j + 5 * ix] -= X_AB * Y_AB;
                  if (old_flag_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_9, gpu_Class_B_idx_9, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  old_flag_dirtyOnGpu = false;
                  L[j + 5 * (ix + 1)] -= X_AB * Class_B_idx_9;
                  ipiv_t_dirtyOnGpu = true;
                }

                for (j = 0; j < 5; j++) {
                  if (Vr_dirtyOnGpu) {
                    cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
                  }

                  X_AB = (Vr[j + 5 * (ix - 1)] + d * Vr[j + 5 * ix]) + scale *
                    Vr[j + 5 * (ix + 1)];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  Vr[j + 5 * (ix - 1)] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  Vr[j + 5 * ix] -= X_AB * Y_AB;
                  if (old_flag_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_9, gpu_Class_B_idx_9, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  old_flag_dirtyOnGpu = false;
                  Vr[j + 5 * (ix + 1)] -= X_AB * Class_B_idx_9;
                  Vr_dirtyOnGpu = false;
                  Vr_dirtyOnCpu = true;
                }
              } else if (nr == 2) {
                for (j = 0; j <= 5 - ix; j++) {
                  knt = (ix + j) - 1;
                  X_AB = L[(ix + 5 * knt) - 1] + d * L[ix + 5 * knt];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  L[(ix + 5 * knt) - 1] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  L[ix + 5 * knt] -= X_AB * Y_AB;
                  ipiv_t_dirtyOnGpu = true;
                }

                for (j = 0; j <= i; j++) {
                  X_AB = L[j + 5 * (ix - 1)] + d * L[j + 5 * ix];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  L[j + 5 * (ix - 1)] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  L[j + 5 * ix] -= X_AB * Y_AB;
                  ipiv_t_dirtyOnGpu = true;
                }

                for (j = 0; j < 5; j++) {
                  if (Vr_dirtyOnGpu) {
                    cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
                  }

                  X_AB = Vr[j + 5 * (ix - 1)] + d * Vr[j + 5 * ix];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  Vr[j + 5 * (ix - 1)] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  Vr[j + 5 * ix] -= X_AB * Y_AB;
                  Vr_dirtyOnGpu = false;
                  Vr_dirtyOnCpu = true;
                }
              }
            }

            jA++;
          }
        }

        if (!goto150) {
          exitg1 = true;
        } else {
          if ((b_L != i + 1) && (b_L == i)) {
            old_Prob_cv = L[(i + 5 * (i - 1)) - 1];
            x = L[(i + 5 * i) - 1];
            b_x = L[i + 5 * (i - 1)];
            a = L[i + 5 * i];
            if (L[i + 5 * (i - 1)] == 0.0) {
              scale = 1.0;
              old_Prob_ctrv = 0.0;
            } else if (L[(i + 5 * i) - 1] == 0.0) {
              scale = 0.0;
              old_Prob_ctrv = 1.0;
              a = L[(i + 5 * (i - 1)) - 1];
              old_Prob_cv = L[i + 5 * i];
              x = -L[i + 5 * (i - 1)];
              b_x = 0.0;
            } else if ((L[(i + 5 * (i - 1)) - 1] - L[i + 5 * i] == 0.0) && ((L
                         [(i + 5 * i) - 1] < 0.0) != (L[i + 5 * (i - 1)] < 0.0)))
            {
              scale = 1.0;
              old_Prob_ctrv = 0.0;
            } else {
              temp = L[(i + 5 * (i - 1)) - 1] - L[i + 5 * i];
              Class_B_idx_9 = 0.5 * temp;
              Y_AB = std::fmax(std::abs(L[(i + 5 * i) - 1]), std::abs(L[i + 5 *
                (i - 1)]));
              Y_AB_dirtyOnCpu = true;
              if (!(L[(i + 5 * i) - 1] < 0.0)) {
                DEC_param = 1;
              } else {
                DEC_param = -1;
              }

              if (!(L[i + 5 * (i - 1)] < 0.0)) {
                jA = 1;
              } else {
                jA = -1;
              }

              X_AB = std::fmin(std::abs(L[(i + 5 * i) - 1]), std::abs(L[i + 5 *
                (i - 1)])) * static_cast<double>(DEC_param) * static_cast<double>
                (jA);
              scale = std::fmax(std::abs(Class_B_idx_9), Y_AB);
              Class_B_idx_10 = Class_B_idx_9 / scale * Class_B_idx_9 + Y_AB /
                scale * X_AB;
              if (Class_B_idx_10 >= 8.8817841970012523E-16) {
                a = std::sqrt(scale) * std::sqrt(Class_B_idx_10);
                if (Class_B_idx_9 < 0.0) {
                  a = -a;
                }

                Class_B_idx_10 = Class_B_idx_9 + a;
                old_Prob_cv = L[i + 5 * i] + Class_B_idx_10;
                a = L[i + 5 * i] - Y_AB / Class_B_idx_10 * X_AB;
                t = rt_hypotd_snf(L[i + 5 * (i - 1)], Class_B_idx_10);
                scale = Class_B_idx_10 / t;
                old_Prob_ctrv = L[i + 5 * (i - 1)] / t;
                x = L[(i + 5 * i) - 1] - L[i + 5 * (i - 1)];
                b_x = 0.0;
              } else {
                X_AB = L[(i + 5 * i) - 1] + L[i + 5 * (i - 1)];
                t = rt_hypotd_snf(X_AB, temp);
                scale = std::sqrt(0.5 * (std::abs(X_AB) / t + 1.0));
                if (!(X_AB < 0.0)) {
                  b_i = 1;
                } else {
                  b_i = -1;
                }

                old_Prob_ctrv = -(Class_B_idx_9 / (t * scale)) * static_cast<
                  double>(b_i);
                temp = L[(i + 5 * (i - 1)) - 1] * scale + L[(i + 5 * i) - 1] *
                  old_Prob_ctrv;
                X_AB = -L[(i + 5 * (i - 1)) - 1] * old_Prob_ctrv + L[(i + 5 * i)
                  - 1] * scale;
                Y_AB = L[i + 5 * (i - 1)] * scale + L[i + 5 * i] * old_Prob_ctrv;
                Class_B_idx_9 = -L[i + 5 * (i - 1)] * old_Prob_ctrv + L[i + 5 *
                  i] * scale;
                x = X_AB * scale + Class_B_idx_9 * old_Prob_ctrv;
                b_x = -temp * old_Prob_ctrv + Y_AB * scale;
                temp = 0.5 * ((temp * scale + Y_AB * old_Prob_ctrv) + (-X_AB *
                  old_Prob_ctrv + Class_B_idx_9 * scale));
                old_Prob_cv = temp;
                a = temp;
                if (b_x != 0.0) {
                  if (x != 0.0) {
                    if ((x < 0.0) == (b_x < 0.0)) {
                      X_AB = std::sqrt(std::abs(x));
                      Class_B_idx_10 = std::sqrt(std::abs(b_x));
                      a = X_AB * Class_B_idx_10;
                      if (!(b_x < 0.0)) {
                        Class_B_idx_9 = a;
                      } else {
                        Class_B_idx_9 = -a;
                      }

                      t = 1.0 / std::sqrt(std::abs(x + b_x));
                      old_Prob_cv = temp + Class_B_idx_9;
                      a = temp - Class_B_idx_9;
                      x -= b_x;
                      b_x = 0.0;
                      Y_AB = X_AB * t;
                      X_AB = Class_B_idx_10 * t;
                      temp = scale * Y_AB - old_Prob_ctrv * X_AB;
                      old_Prob_ctrv = scale * X_AB + old_Prob_ctrv * Y_AB;
                      scale = temp;
                    }
                  } else {
                    x = -b_x;
                    b_x = 0.0;
                    temp = scale;
                    scale = -old_Prob_ctrv;
                    old_Prob_ctrv = temp;
                  }
                }
              }
            }

            L[(i + 5 * (i - 1)) - 1] = old_Prob_cv;
            L[(i + 5 * i) - 1] = x;
            L[i + 5 * (i - 1)] = b_x;
            L[i + 5 * i] = a;
            ipiv_t_dirtyOnGpu = true;
            if (5 > i + 1) {
              DEC_param = 3 - i;
              ix = (i + (i + 1) * 5) - 1;
              jA = i + (i + 1) * 5;
              for (lastc = 0; lastc <= DEC_param; lastc++) {
                temp = scale * L[ix + lastc * 5] + old_Prob_ctrv * L[jA + lastc *
                  5];
                L[jA + lastc * 5] = scale * L[jA + lastc * 5] - old_Prob_ctrv *
                  L[ix + lastc * 5];
                L[ix + lastc * 5] = temp;
              }
            }

            if (i - 1 >= 1) {
              ix = (i - 1) * 5;
              jA = i * 5;
              for (lastc = 0; lastc <= i - 2; lastc++) {
                temp = scale * L[ix + lastc] + old_Prob_ctrv * L[jA + lastc];
                L[jA + lastc] = scale * L[jA + lastc] - old_Prob_ctrv * L[ix +
                  lastc];
                L[ix + lastc] = temp;
              }
            }

            ix = (i - 1) * 5;
            jA = i * 5;
            for (lastc = 0; lastc < 5; lastc++) {
              if (Vr_dirtyOnGpu) {
                cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
              }

              temp = scale * Vr[ix + lastc] + old_Prob_ctrv * Vr[jA + lastc];
              Vr[jA + lastc] = scale * Vr[jA + lastc] - old_Prob_ctrv * Vr[ix +
                lastc];
              Vr[ix + lastc] = temp;
              Vr_dirtyOnGpu = false;
              Vr_dirtyOnCpu = true;
            }
          }

          i = b_L - 2;
        }
      }

      for (j = 0; j < 2; j++) {
        DEC_param = j + 3;
        for (i = 0; i <= 4 - DEC_param; i++) {
          L[(DEC_param + i) + 5 * j] = 0.0;
          ipiv_t_dirtyOnGpu = true;
        }
      }

      if (Vr_dirtyOnCpu) {
        cudaMemcpy(*gpu_Vr, Vr, 200UL, cudaMemcpyHostToDevice);
      }

      Vr_dirtyOnCpu = false;
      if (ipiv_t_dirtyOnGpu) {
        cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
      }

      ipiv_t_dirtyOnGpu = false;
      if (U_dirtyOnCpu) {
        cudaMemcpy(*gpu_U, U, 400UL, cudaMemcpyHostToDevice);
      }

      BEV_image_kernel45<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_Vr,
        *gpu_L, *gpu_U, *gpu_L_tmp);
      U_dirtyOnCpu = false;
      nr_dirtyOnCpu = false;
      validLaunchParams = true;
      for (m = 0; m < 4; m++) {
        ix = 5 - m;
        d = L[(5 * (3 - m) - m) + 4];
        if (d != 0.0) {
          a = L[(5 * (3 - m) - m) + 3];
          old_Prob_cv = L[(5 * (4 - m) - m) + 3];
          Class_B_idx_10 = d;
          if (!(d == 0.0)) {
            if (old_Prob_cv == 0.0) {
              a = L[(5 * (4 - m) - m) + 4];
              old_Prob_cv = -d;
              Class_B_idx_10 = 0.0;
            } else if ((!(a - L[(5 * (4 - m) - m) + 4] == 0.0)) || ((old_Prob_cv
              < 0.0) == (d < 0.0))) {
              temp = a - L[(5 * (4 - m) - m) + 4];
              Class_B_idx_9 = 0.5 * temp;
              Y_AB = std::fmax(std::abs(old_Prob_cv), std::abs(d));
              if (!(old_Prob_cv < 0.0)) {
                DEC_param = 1;
              } else {
                DEC_param = -1;
              }

              if (!(d < 0.0)) {
                jA = 1;
              } else {
                jA = -1;
              }

              scale = std::fmax(std::abs(Class_B_idx_9), Y_AB);
              Class_B_idx_10 = Class_B_idx_9 / scale * Class_B_idx_9 + Y_AB /
                scale * (std::fmin(std::abs(old_Prob_cv), std::abs(d)) *
                         static_cast<double>(DEC_param) * static_cast<double>(jA));
              if (Class_B_idx_10 >= 8.8817841970012523E-16) {
                a = std::sqrt(scale) * std::sqrt(Class_B_idx_10);
                if (Class_B_idx_9 < 0.0) {
                  a = -a;
                }

                a = L[(5 * (4 - m) - m) + 4] + (Class_B_idx_9 + a);
                old_Prob_cv -= d;
                Class_B_idx_10 = 0.0;
              } else {
                X_AB = old_Prob_cv + d;
                t = rt_hypotd_snf(X_AB, temp);
                scale = std::sqrt(0.5 * (std::abs(X_AB) / t + 1.0));
                if (!(X_AB < 0.0)) {
                  b_i = 1;
                } else {
                  b_i = -1;
                }

                old_Prob_ctrv = -(Class_B_idx_9 / (t * scale)) * static_cast<
                  double>(b_i);
                temp = a * scale + old_Prob_cv * old_Prob_ctrv;
                X_AB = -a * old_Prob_ctrv + old_Prob_cv * scale;
                Y_AB = d * scale + L[(5 * (4 - m) - m) + 4] * old_Prob_ctrv;
                Class_B_idx_9 = -d * old_Prob_ctrv + L[(5 * (4 - m) - m) + 4] *
                  scale;
                old_Prob_cv = X_AB * scale + Class_B_idx_9 * old_Prob_ctrv;
                Class_B_idx_10 = -temp * old_Prob_ctrv + Y_AB * scale;
                temp = 0.5 * ((temp * scale + Y_AB * old_Prob_ctrv) + (-X_AB *
                  old_Prob_ctrv + Class_B_idx_9 * scale));
                a = temp;
                if (Class_B_idx_10 != 0.0) {
                  if (old_Prob_cv != 0.0) {
                    if ((old_Prob_cv < 0.0) == (Class_B_idx_10 < 0.0)) {
                      a = std::sqrt(std::abs(old_Prob_cv)) * std::sqrt(std::abs
                        (Class_B_idx_10));
                      if (Class_B_idx_10 < 0.0) {
                        a = -a;
                      }

                      a += temp;
                      old_Prob_cv -= Class_B_idx_10;
                      Class_B_idx_10 = 0.0;
                    }
                  } else {
                    old_Prob_cv = -Class_B_idx_10;
                    Class_B_idx_10 = 0.0;
                  }
                }
              }
            }
          }

          Class_B_idx_9 = a - L[(5 * (4 - m) - m) + 4];
          if (Class_B_idx_10 == 0.0) {
            X_AB = 0.0;
          } else {
            X_AB = std::sqrt(std::abs(old_Prob_cv)) * std::sqrt(std::abs
              (Class_B_idx_10));
          }

          Y_AB = rt_hypotd_snf(rt_hypotd_snf(Class_B_idx_9, X_AB), d);
          if (X_AB == 0.0) {
            cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL,
                       cudaMemcpyHostToDevice);
            BEV_image_kernel47<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_Y_AB,
              gpu_Class_B_idx_9, gpu_c);
            c_dirtyOnGpu = true;
          } else if (Class_B_idx_9 == 0.0) {
            if (c_dirtyOnGpu) {
              cudaMemcpy(&c, gpu_c, 16UL, cudaMemcpyDeviceToHost);
            }

            c.re = 0.0;
            c.im = X_AB / Y_AB;
            c_dirtyOnGpu = false;
            c_dirtyOnCpu = true;
          } else {
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL,
                       cudaMemcpyHostToDevice);
            BEV_image_kernel46<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_X_AB,
              gpu_Y_AB, gpu_Class_B_idx_9, gpu_c);
            c_dirtyOnGpu = true;
          }

          scale = d / Y_AB;
          for (j = 0; j <= 6 - ix; j++) {
            knt = (j - m) + 3;
            if (validLaunchParams) {
              cudaMemcpy(L_tmp, *gpu_L_tmp, 400UL, cudaMemcpyDeviceToHost);
            }

            t1 = L_tmp[(5 * knt - m) + 3];
            d = L_tmp[(5 * knt - m) + 3].re;
            if (c_dirtyOnGpu) {
              cudaMemcpy(&c, gpu_c, 16UL, cudaMemcpyDeviceToHost);
            }

            L_tmp[(5 * knt - m) + 3].re = (c.re * L_tmp[(5 * knt - m) + 3].re +
              c.im * L_tmp[(5 * knt - m) + 3].im) + scale * L_tmp[(5 * knt - m)
              + 4].re;
            L_tmp[(5 * knt - m) + 3].im = (c.re * L_tmp[(5 * knt - m) + 3].im -
              c.im * d) + scale * L_tmp[(5 * knt - m) + 4].im;
            X_AB = c.re * L_tmp[(5 * knt - m) + 4].im + c.im * L_tmp[(5 * knt -
              m) + 4].re;
            c_dirtyOnGpu = false;
            L_tmp[(5 * knt - m) + 4].re = (c.re * L_tmp[(5 * knt - m) + 4].re -
              c.im * L_tmp[(5 * knt - m) + 4].im) - scale * t1.re;
            L_tmp[(5 * knt - m) + 4].im = X_AB - scale * t1.im;
            validLaunchParams = false;
            nr_dirtyOnCpu = true;
          }

          validLaunchParams = mwGetLaunchParameters1D(static_cast<double>((4 - m)
            + 1L), &grid, &block, 1024U, 65535U);
          if (validLaunchParams) {
            cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
            if (c_dirtyOnCpu) {
              cudaMemcpy(gpu_c, &c, 16UL, cudaMemcpyHostToDevice);
            }

            c_dirtyOnCpu = false;
            if (nr_dirtyOnCpu) {
              cudaMemcpy(*gpu_L_tmp, L_tmp, 400UL, cudaMemcpyHostToDevice);
            }

            BEV_image_kernel48<<<grid, block>>>(gpu_scale, gpu_c, 5 - m,
              *gpu_L_tmp);
            nr_dirtyOnCpu = false;
          } else {
            cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
          }

          if (c_dirtyOnCpu) {
            cudaMemcpy(gpu_c, &c, 16UL, cudaMemcpyHostToDevice);
          }

          c_dirtyOnCpu = false;
          BEV_image_kernel49<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_scale,
            gpu_c, 5 - m, *gpu_U);
          if (nr_dirtyOnCpu) {
            cudaMemcpy(*gpu_L_tmp, L_tmp, 400UL, cudaMemcpyHostToDevice);
          }

          BEV_image_kernel50<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(5 - m,
            *gpu_L_tmp);
          nr_dirtyOnCpu = false;
          validLaunchParams = true;
        }
      }
    }

    if (R_dirtyOnCpu) {
      cudaMemcpy(*gpu_R, R, 400UL, cudaMemcpyHostToDevice);
    }

    BEV_image_kernel53<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_R);
    R_dirtyOnCpu = false;
    v_dirtyOnGpu = true;
    j = 0;
    do {
      exitg5 = 0;
      if (j + 1 < 6) {
        i = 1;
        do {
          exitg2 = 0;
          if (i <= j) {
            if (validLaunchParams) {
              cudaMemcpy(L_tmp, *gpu_L_tmp, 400UL, cudaMemcpyDeviceToHost);
            }

            validLaunchParams = false;
            if ((L_tmp[(i + 5 * j) - 1].re != 0.0) || (L_tmp[(i + 5 * j) - 1].im
                 != 0.0)) {
              for (j = 0; j < 5; j++) {
                Class_B_idx_9 = L_tmp[j + 5 * j].re;
                Class_B_idx_10 = L_tmp[j + 5 * j].im;
                if (Class_B_idx_10 == 0.0) {
                  if (Class_B_idx_9 < 0.0) {
                    X_AB = 0.0;
                    Class_B_idx_9 = std::sqrt(-Class_B_idx_9);
                  } else {
                    X_AB = std::sqrt(Class_B_idx_9);
                    Class_B_idx_9 = 0.0;
                  }
                } else if (Class_B_idx_9 == 0.0) {
                  if (Class_B_idx_10 < 0.0) {
                    X_AB = std::sqrt(-Class_B_idx_10 / 2.0);
                    Class_B_idx_9 = -X_AB;
                  } else {
                    X_AB = std::sqrt(Class_B_idx_10 / 2.0);
                    Class_B_idx_9 = X_AB;
                  }
                } else if (std::isnan(Class_B_idx_9)) {
                  X_AB = Class_B_idx_9;
                } else if (std::isnan(Class_B_idx_10)) {
                  X_AB = Class_B_idx_10;
                  Class_B_idx_9 = Class_B_idx_10;
                } else if (std::isinf(Class_B_idx_10)) {
                  X_AB = std::abs(Class_B_idx_10);
                  Class_B_idx_9 = Class_B_idx_10;
                } else if (std::isinf(Class_B_idx_9)) {
                  if (Class_B_idx_9 < 0.0) {
                    X_AB = 0.0;
                    Class_B_idx_9 = Class_B_idx_10 * -Class_B_idx_9;
                  } else {
                    X_AB = Class_B_idx_9;
                    Class_B_idx_9 = 0.0;
                  }
                } else {
                  Y_AB = std::abs(Class_B_idx_9);
                  X_AB = std::abs(Class_B_idx_10);
                  if ((Y_AB > 4.4942328371557893E+307) || (X_AB >
                       4.4942328371557893E+307)) {
                    Y_AB *= 0.5;
                    X_AB = rt_hypotd_snf(Y_AB, X_AB * 0.5);
                    if (X_AB > Y_AB) {
                      X_AB = std::sqrt(X_AB) * std::sqrt(Y_AB / X_AB + 1.0);
                    } else {
                      X_AB = std::sqrt(X_AB) * 1.4142135623730951;
                    }
                  } else {
                    X_AB = std::sqrt((rt_hypotd_snf(Y_AB, X_AB) + Y_AB) * 0.5);
                  }

                  if (Class_B_idx_9 > 0.0) {
                    Class_B_idx_9 = 0.5 * (Class_B_idx_10 / X_AB);
                  } else {
                    if (Class_B_idx_10 < 0.0) {
                      Class_B_idx_9 = -X_AB;
                    } else {
                      Class_B_idx_9 = X_AB;
                    }

                    X_AB = 0.5 * (Class_B_idx_10 / Class_B_idx_9);
                  }
                }

                if (v_dirtyOnGpu) {
                  cudaMemcpy(R, *gpu_R, 400UL, cudaMemcpyDeviceToHost);
                }

                R[j + 5 * j].re = X_AB;
                R[j + 5 * j].im = Class_B_idx_9;
                v_dirtyOnGpu = false;
                R_dirtyOnCpu = true;
                for (i = 0; i < j; i++) {
                  b_i = (j - i) - 1;
                  X_AB = 0.0;
                  Y_AB = 0.0;
                  DEC_param = b_i + 2;
                  for (lastc = 0; lastc <= j - DEC_param; lastc++) {
                    k = (b_i + lastc) + 1;
                    X_AB += R[b_i + 5 * k].re * R[k + 5 * j].re - R[b_i + 5 * k]
                      .im * R[k + 5 * j].im;
                    Y_AB += R[b_i + 5 * k].re * R[k + 5 * j].im + R[b_i + 5 * k]
                      .im * R[k + 5 * j].re;
                  }

                  old_Prob_ctrv = L_tmp[b_i + 5 * j].re - X_AB;
                  temp = L_tmp[b_i + 5 * j].im - Y_AB;
                  Y_AB = R[b_i + 5 * b_i].re + R[j + 5 * j].re;
                  Class_B_idx_9 = R[b_i + 5 * b_i].im + R[j + 5 * j].im;
                  if (Class_B_idx_9 == 0.0) {
                    if (temp == 0.0) {
                      t = old_Prob_ctrv / Y_AB;
                      X_AB = 0.0;
                    } else if (old_Prob_ctrv == 0.0) {
                      t = 0.0;
                      X_AB = temp / Y_AB;
                    } else {
                      t = old_Prob_ctrv / Y_AB;
                      X_AB = temp / Y_AB;
                    }
                  } else if (Y_AB == 0.0) {
                    if (old_Prob_ctrv == 0.0) {
                      t = temp / Class_B_idx_9;
                      X_AB = 0.0;
                    } else if (temp == 0.0) {
                      t = 0.0;
                      X_AB = -(old_Prob_ctrv / Class_B_idx_9);
                    } else {
                      t = temp / Class_B_idx_9;
                      X_AB = -(old_Prob_ctrv / Class_B_idx_9);
                    }
                  } else {
                    Class_B_idx_10 = std::abs(Y_AB);
                    X_AB = std::abs(Class_B_idx_9);
                    if (Class_B_idx_10 > X_AB) {
                      scale = Class_B_idx_9 / Y_AB;
                      old_Prob_cv = Y_AB + scale * Class_B_idx_9;
                      t = (old_Prob_ctrv + scale * temp) / old_Prob_cv;
                      X_AB = (temp - scale * old_Prob_ctrv) / old_Prob_cv;
                    } else if (X_AB == Class_B_idx_10) {
                      if (Y_AB > 0.0) {
                        Y_AB = 0.5;
                      } else {
                        Y_AB = -0.5;
                      }

                      if (Class_B_idx_9 > 0.0) {
                        X_AB = 0.5;
                      } else {
                        X_AB = -0.5;
                      }

                      t = (old_Prob_ctrv * Y_AB + temp * X_AB) / Class_B_idx_10;
                      X_AB = (temp * Y_AB - old_Prob_ctrv * X_AB) /
                        Class_B_idx_10;
                    } else {
                      scale = Y_AB / Class_B_idx_9;
                      old_Prob_cv = Class_B_idx_9 + scale * Y_AB;
                      t = (scale * old_Prob_ctrv + temp) / old_Prob_cv;
                      X_AB = (scale * temp - old_Prob_ctrv) / old_Prob_cv;
                    }
                  }

                  R[b_i + 5 * j].re = t;
                  R[b_i + 5 * j].im = X_AB;
                }
              }

              exitg2 = 1;
            } else {
              i++;
            }
          } else {
            j++;
            exitg2 = 2;
          }
        } while (exitg2 == 0);

        if (exitg2 == 1) {
          exitg5 = 1;
        }
      } else {
        BEV_image_kernel54<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp, *
          gpu_R);
        exitg5 = 1;
      }
    } while (exitg5 == 0);

    for (k = 0; k < 5; k++) {
      if (R_dirtyOnCpu) {
        cudaMemcpy(*gpu_R, R, 400UL, cudaMemcpyHostToDevice);
      }

      R_dirtyOnCpu = false;
      if (U_dirtyOnCpu) {
        cudaMemcpy(*gpu_U, U, 400UL, cudaMemcpyHostToDevice);
      }

      cudaMemcpy(gpu_k, &k, 4UL, cudaMemcpyHostToDevice);
      BEV_image_kernel55<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_R, *gpu_U,
        gpu_k, *b_gpu_U);
      U_dirtyOnCpu = false;
      BEV_image_kernel56<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*b_gpu_U,
        *gpu_U, gpu_k, *gpu_L_tmp);
    }

    BEV_image_kernel57<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp,
      *gpu_L);
    temp = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 5)) {
      scale = 0.0;
      cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel58<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L, j,
        gpu_scale);
      cudaMemcpy(&scale, gpu_scale, 8UL, cudaMemcpyDeviceToHost);
      if (std::isnan(scale)) {
        temp = rtNaN;
        exitg1 = true;
      } else {
        if (scale > temp) {
          temp = scale;
        }

        j++;
      }
    }

    X_AB = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 5)) {
      scale = 0.0;
      cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel59<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp, j,
        gpu_scale);
      cudaMemcpy(&scale, gpu_scale, 8UL, cudaMemcpyDeviceToHost);
      if (std::isnan(scale)) {
        X_AB = rtNaN;
        exitg1 = true;
      } else {
        if (scale > X_AB) {
          X_AB = scale;
        }

        j++;
      }
    }

    if (temp <= 1.1102230246251565E-14 * X_AB) {
      BEV_image_kernel60<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp);
    }

    BEV_image_kernel61<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp,
      *gpu_L);

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Sampling
    BEV_image_kernel62<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>(*gpu_X_k);
    Class_B_idx_9_dirtyOnCpu = true;
    BEV_image_kernel63<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_W);
    Vr_dirtyOnGpu = true;
    for (i = 0; i < 11; i++) {
      if (i == 0) {
        BEV_image_kernel68<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*gpu_x_ctrv_out, *gpu_X_k);
        BEV_image_kernel69<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_W);
      } else if (i <= 5) {
        BEV_image_kernel66<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L, i -
          1, *gpu_x_ctrv_out, i, *gpu_X_k);
        BEV_image_kernel67<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(i, *gpu_W);
      } else {
        BEV_image_kernel64<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L, i -
          6, *gpu_x_ctrv_out, i, *gpu_X_k);
        BEV_image_kernel65<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(i, *gpu_W);
      }
    }

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Model prediction
    BEV_image_kernel70<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_k_hat);
    BEV_image_kernel71<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L);
    for (i = 0; i < 11; i++) {
      //  x_k = [Y X Yaw v yawrate]';
      //  ts = 0.01;
      if (Class_B_idx_9_dirtyOnCpu) {
        cudaMemcpy(X_k, *gpu_X_k, 440UL, cudaMemcpyDeviceToHost);
      }

      Class_B_idx_9_dirtyOnCpu = false;
      d = X_k[5 * i + 4];
      if (d != 0.0) {
        scale = X_k[5 * i + 2];
        cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
        cudaMemcpy(gpu_d, &d, 8UL, cudaMemcpyHostToDevice);
        BEV_image_kernel73<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_scale,
          ts, gpu_d, X_k[5 * i + 3], *gpu_X_k, i, *gpu_X_hat);
      } else {
        //  x_kk(1) = x_k(1) + x_k(4)/x_k(5)*(-cos(x_k(5)*ts+x_k(3)) + cos(x_k(3)));
        //  x_kk(2) = x_k(2) + x_k(4)/x_k(5)*(-sin(x_k(3)) + sin(x_k(5)*ts+x_k(3)));
        //  x_kk(3) = x_k(3) + x_k(5)*ts;
        //  x_kk(4) = x_k(4);
        //  x_kk(5) = -sign(x_k(3))*10^(-1);
        cudaMemcpy(gpu_d, &d, 8UL, cudaMemcpyHostToDevice);
        BEV_image_kernel72<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_d,
          *gpu_X_k, i, *gpu_X_hat);
      }

      if (Vr_dirtyOnGpu) {
        cudaMemcpy(W, *gpu_W, 88UL, cudaMemcpyDeviceToHost);
      }

      Vr_dirtyOnGpu = false;
      X_AB = W[i];
      cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel74<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_X_AB, i,
        *gpu_X_hat, *gpu_x_k_hat, *gpu_X_k_hat);
    }

    for (i = 0; i < 11; i++) {
      if (Vr_dirtyOnGpu) {
        cudaMemcpy(W, *gpu_W, 88UL, cudaMemcpyDeviceToHost);
      }

      Vr_dirtyOnGpu = false;
      X_AB = W[i];
      cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel75<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_X_AB,
        *gpu_x_k_hat, *gpu_X_k_hat, i, *gpu_tmp_ego_y, *gpu_work);
      BEV_image_kernel76<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_tmp_ego_y,
        *gpu_work, *gpu_L);
    }

    P_ctrv_dirtyOnCpu = false;
    BEV_image_kernel77<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_ctrv,
      *gpu_L);

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Measurement update
    if (v_dirtyOnCpu) {
      cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
    }

    BEV_image_kernel78<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v);
    v_dirtyOnCpu = false;
    BEV_image_kernel79<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz);
    BEV_image_kernel80<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_xz);
    Class_B_idx_10_dirtyOnGpu = true;
    for (i = 0; i < 11; i++) {
      if (a_dirtyOnCpu) {
        cudaMemcpy(*gpu_a, b_a, 15UL, cudaMemcpyHostToDevice);
      }

      a_dirtyOnCpu = false;
      BEV_image_kernel81<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_X_k,
        *gpu_a, i, *gpu_Y_k_hat);
      if (Vr_dirtyOnGpu) {
        cudaMemcpy(W, *gpu_W, 88UL, cudaMemcpyDeviceToHost);
      }

      Vr_dirtyOnGpu = false;
      X_AB = W[i];
      cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel82<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_Y_k_hat,
        i, gpu_X_AB, *gpu_v);
    }

    for (i = 0; i < 11; i++) {
      if (Vr_dirtyOnGpu) {
        cudaMemcpy(W, *gpu_W, 88UL, cudaMemcpyDeviceToHost);
      }

      Vr_dirtyOnGpu = false;
      X_AB = W[i];
      cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel83<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_X_AB,
        *gpu_v, *gpu_Y_k_hat, i, *b_gpu_Y_k_hat, *b_gpu_W);
      BEV_image_kernel84<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*b_gpu_Y_k_hat,
        *b_gpu_W, *gpu_P_zz);
      BEV_image_kernel85<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_k_hat, *
        gpu_X_k_hat, i, gpu_X_AB, *gpu_work);
      BEV_image_kernel86<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v,
        *gpu_Y_k_hat, i, *b_gpu_Y_k_hat);
      BEV_image_kernel87<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*b_gpu_Y_k_hat,
        *gpu_work, *gpu_P_xz);
    }

    if (R_CTRV_IMM_dirtyOnCpu) {
      cudaMemcpy(*gpu_R_CTRV_IMM, R_CTRV_IMM, 72UL, cudaMemcpyHostToDevice);
    }

    if (A_dirtyOnCpu) {
      cudaMemcpy(*gpu_A, A, 72UL, cudaMemcpyHostToDevice);
    }

    BEV_image_kernel88<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_R_CTRV_IMM,
      *gpu_A, *gpu_P_zz);
    jA = 0;
    ix = 1;
    nr = 2;
    cudaMemcpy(P_zz, *gpu_P_zz, 72UL, cudaMemcpyDeviceToHost);
    X_AB = std::abs(P_zz[0]);
    Y_AB = std::abs(P_zz[1]);
    if (Y_AB > X_AB) {
      X_AB = Y_AB;
      jA = 1;
      ix = 0;
    }

    if (std::abs(P_zz[2]) > X_AB) {
      jA = 2;
      ix = 1;
      nr = 0;
    }

    BEV_image_kernel89<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, *gpu_P_zz,
      ix, *gpu_A);
    cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
    BEV_image_kernel90<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, gpu_nr,
      *gpu_A);
    BEV_image_kernel91<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, ix, *gpu_A);
    BEV_image_kernel92<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, gpu_nr,
      *gpu_A);
    BEV_image_kernel93<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, ix, *gpu_A);
    BEV_image_kernel94<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, gpu_nr,
      *gpu_A);
    cudaMemcpy(A, *gpu_A, 72UL, cudaMemcpyDeviceToHost);
    if (std::abs(A[nr + 3]) > std::abs(A[ix + 3])) {
      DEC_param = ix;
      ix = nr;
      nr = DEC_param;
    }

    A[nr + 3] /= A[ix + 3];
    A[nr + 6] -= A[nr + 3] * A[ix + 6];
    for (lastc = 0; lastc < 5; lastc++) {
      if (Class_B_idx_10_dirtyOnGpu) {
        cudaMemcpy(P_xz, *gpu_P_xz, 120UL, cudaMemcpyDeviceToHost);
      }

      K[lastc + 5 * jA] = P_xz[lastc] / A[jA];
      K[lastc + 5 * ix] = P_xz[lastc + 5] - K[lastc + 5 * jA] * A[jA + 3];
      Class_B_idx_10_dirtyOnGpu = false;
      K[lastc + 5 * nr] = P_xz[lastc + 10] - K[lastc + 5 * jA] * A[jA + 6];
      K[lastc + 5 * ix] /= A[ix + 3];
      K[lastc + 5 * nr] -= K[lastc + 5 * ix] * A[ix + 6];
      K[lastc + 5 * nr] /= A[nr + 6];
      K[lastc + 5 * ix] -= K[lastc + 5 * nr] * A[nr + 3];
      K[lastc + 5 * jA] -= K[lastc + 5 * nr] * A[nr];
      K[lastc + 5 * jA] -= K[lastc + 5 * ix] * A[ix];
      K_dirtyOnCpu = true;
    }

    cudaMemcpy(*gpu_A, A, 72UL, cudaMemcpyHostToDevice);
    BEV_image_kernel95<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz,
      *gpu_A);
    cusolverDnDgetrf_bufferSize(getCuSolverGlobalHandle(), 3, 3, (double *)
      &(*gpu_A)[0], 3, getCuSolverWorkspaceReq());
    setCuSolverWorkspaceTypeSize(8);
    cusolverInitWorkspace();
    cusolverDnDgetrf(getCuSolverGlobalHandle(), 3, 3, (double *)&(*gpu_A)[0], 3,
                     (double *)getCuSolverWorkspaceBuff(), &(*gpu_ipiv_t)[0],
                     gpu_info);
    cudaMemcpy(&info, gpu_info, 4UL, cudaMemcpyDeviceToHost);
    if (info < 0) {
      BEV_image_kernel97<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_A);
      BEV_image_kernel98<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_ipiv);
    } else {
      BEV_image_kernel96<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_ipiv_t,
        *gpu_ipiv);
    }

    temp = 1.0;
    cudaMemcpy(gpu_temp, &temp, 8UL, cudaMemcpyHostToDevice);
    BEV_image_kernel99<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_A, gpu_temp);
    old_flag_dirtyOnGpu = true;
    cudaMemcpy(ipiv, *gpu_ipiv, 12UL, cudaMemcpyDeviceToHost);
    goto150 = (ipiv[0] > 1);
    if (ipiv[1] > 2) {
      goto150 = !goto150;
    }

    if (goto150) {
      cudaMemcpy(&temp, gpu_temp, 8UL, cudaMemcpyDeviceToHost);
      temp = -temp;
      old_flag_dirtyOnGpu = false;
    }

    BEV_image_kernel100<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz,
      *gpu_A);
    DEC_param = 0;
    jA = 3;
    ix = 6;
    X_AB = std::abs(P_zz[0]);
    Y_AB = std::abs(P_zz[1]);
    Class_B_idx_9 = std::abs(P_zz[2]);
    if ((Y_AB > X_AB) && (Y_AB > Class_B_idx_9)) {
      DEC_param = 3;
      jA = 0;
      BEV_image_kernel102<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz,
        *gpu_A);
    } else if (Class_B_idx_9 > X_AB) {
      DEC_param = 6;
      ix = 0;
      BEV_image_kernel101<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz,
        *gpu_A);
    }

    BEV_image_kernel103<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_A);
    BEV_image_kernel104<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_A);
    cudaMemcpy(A, *gpu_A, 72UL, cudaMemcpyDeviceToHost);
    A[7] -= A[1] * A[6];
    A[8] -= A[2] * A[6];
    if (std::abs(A[5]) > std::abs(A[4])) {
      nr = jA;
      jA = ix;
      ix = nr;
      Class_B_idx_10 = A[1];
      A[1] = A[2];
      A[2] = Class_B_idx_10;
      Class_B_idx_10 = A[4];
      A[4] = A[5];
      A[5] = Class_B_idx_10;
      Class_B_idx_10 = A[7];
      A[7] = A[8];
      A[8] = Class_B_idx_10;
    }

    A[5] /= A[4];
    A[8] -= A[5] * A[7];
    Class_B_idx_9 = (A[1] * A[5] - A[2]) / A[8];
    Y_AB = -(A[1] + A[7] * Class_B_idx_9) / A[4];
    if (y_dirtyOnGpu) {
      cudaMemcpy(y, *gpu_y, 72UL, cudaMemcpyDeviceToHost);
    }

    y[DEC_param] = ((1.0 - A[3] * Y_AB) - A[6] * Class_B_idx_9) / A[0];
    y[DEC_param + 1] = Y_AB;
    y[DEC_param + 2] = Class_B_idx_9;
    Class_B_idx_9 = -A[5] / A[8];
    Y_AB = (1.0 - A[7] * Class_B_idx_9) / A[4];
    y[jA] = -(A[3] * Y_AB + A[6] * Class_B_idx_9) / A[0];
    y[jA + 1] = Y_AB;
    y[jA + 2] = Class_B_idx_9;
    Class_B_idx_9 = 1.0 / A[8];
    cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL, cudaMemcpyHostToDevice);
    cudaMemcpy(*gpu_A, A, 72UL, cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
    BEV_image_kernel105<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (gpu_Class_B_idx_9, *gpu_A, gpu_Y_AB);
    Y_AB_dirtyOnCpu = false;
    cudaMemcpy(A, *gpu_A, 72UL, cudaMemcpyDeviceToHost);
    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
    y[ix] = -(A[3] * Y_AB + A[6] * Class_B_idx_9) / A[0];
    y[ix + 1] = Y_AB;
    y[ix + 2] = Class_B_idx_9;
    if (y_out_dirtyOnCpu) {
      cudaMemcpy(*gpu_y_out, y_out, 24UL, cudaMemcpyHostToDevice);
    }

    BEV_image_kernel106<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v,
      *gpu_y_out, *b_gpu_Y_k_hat);
    cudaMemcpy(*gpu_y, y, 72UL, cudaMemcpyHostToDevice);
    BEV_image_kernel107<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_y,
      *b_gpu_Y_k_hat, *gpu_dv1);
    d = 0.0;
    y_out_dirtyOnCpu = false;
    cudaMemcpy(gpu_d, &d, 8UL, cudaMemcpyHostToDevice);
    BEV_image_kernel108<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v,
      *gpu_y_out, *gpu_dv1, gpu_d);
    if (old_flag_dirtyOnGpu) {
      cudaMemcpy(&temp, gpu_temp, 8UL, cudaMemcpyDeviceToHost);
    }

    cudaMemcpy(&d, gpu_d, 8UL, cudaMemcpyDeviceToHost);
    laneInfoL_idx_2 = 1.0 / std::sqrt(248.05021344239853 * temp) * std::exp(d);

    //  P_kk = P_k_hat - K*S*K';
    // -------------------------------------------------------------------------
    if ((flag[0] == 0.0) && (old_flag[0] == 1.0)) {
      BEV_image_kernel115<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_ctrv);
    } else if ((flag[0] == 1.0) && (old_flag[0] == 0.0)) {
      BEV_image_kernel114<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_ini,
        *gpu_x_ctrv);
    } else if (flag[0] != 0.0) {
      BEV_image_kernel110<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v,
        *gpu_y_out, *b_gpu_W);
      if (K_dirtyOnCpu) {
        cudaMemcpy(*gpu_K, K, 120UL, cudaMemcpyHostToDevice);
      }

      BEV_image_kernel111<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_k_hat,
        *b_gpu_W, *gpu_K, *gpu_x_ctrv);
      BEV_image_kernel112<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz,
        *gpu_K, *gpu_P_xz);
      K_dirtyOnCpu = false;
      BEV_image_kernel113<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L,
        sample_ts, *gpu_K, *gpu_P_xz, *gpu_P_ctrv_tmp);
    } else {
      BEV_image_kernel109<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_ctrv);
    }

    // -------------------------------------------------------------------------
    //  if time >=63.230
    //  figure(1); hold on
    //  cla;
    //  plot(x_kk(1),x_kk(2),'xr');
    //  plot(x_k(1),x_k(2),'xk')
    //  plot([x_kk(1) x_k(1)],[x_kk(2) x_k(2)])
    //  plot(y_k(1),y_k(2),'or')
    //  axis([-6 6 -5 10])
    //  axis equal
    //  drawnow
    //      time = time;
    //  end
    if (P_cv_dirtyOnCpu) {
      cudaMemcpy(*gpu_P_cv, P_cv, 200UL, cudaMemcpyHostToDevice);
    }

    BEV_image_kernel116<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_cv,
      sample_ts, *gpu_P_cv_tmp);

    //  if time >=18.49
    //      time = time;
    //  end
    //  x_k = [y x yaw v yawrate]';
    // -------------------------------------------------------------------------
    //  Parameter
    //  L = real(sqrtm((n_x+KAPPA)*P_cv_k));
    BEV_image_kernel117<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_cv_out, *
      gpu_L);
    L_dirtyOnGpu = true;
    goto150 = true;
    for (lastc = 0; lastc < 25; lastc++) {
      if (goto150) {
        if (L_dirtyOnGpu) {
          cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
        }

        L_dirtyOnGpu = false;
        if (std::isinf(L[lastc]) || std::isnan(L[lastc])) {
          goto150 = false;
        }
      } else {
        goto150 = false;
      }
    }

    if (!goto150) {
      if (U_dirtyOnCpu) {
        cudaMemcpy(*gpu_U, U, 400UL, cudaMemcpyHostToDevice);
      }

      BEV_image_kernel156<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_U);
      U_dirtyOnCpu = false;
      Y_AB_dirtyOnGpu = true;
      for (j = 0; j < 3; j++) {
        DEC_param = j + 2;
        for (i = 0; i <= 4 - DEC_param; i++) {
          if (Y_AB_dirtyOnGpu) {
            cudaMemcpy(U, *gpu_U, 400UL, cudaMemcpyDeviceToHost);
          }

          U[(DEC_param + i) + 5 * j].re = 0.0;
          U[(DEC_param + i) + 5 * j].im = 0.0;
          Y_AB_dirtyOnGpu = false;
          U_dirtyOnCpu = true;
        }
      }

      BEV_image_kernel157<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp);
      validLaunchParams = true;
    } else {
      BEV_image_kernel118<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_work);
      old_flag_dirtyOnGpu = true;
      for (i = 0; i < 4; i++) {
        im1n = i * 5 + 2;
        in = (i + 1) * 5;
        BEV_image_kernel119<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L, i,
          gpu_Y_AB);
        Y_AB_dirtyOnGpu = true;
        DEC_param = i + 3;
        if (i + 3 >= 5) {
          DEC_param = 5;
        }

        jA = (DEC_param + i * 5) - 1;
        tau[i] = 0.0;
        X_AB = 0.0;
        if (3 - i >= 1) {
          if (3 - i == 1) {
            if (L_dirtyOnGpu) {
              cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
            }

            L_dirtyOnGpu = false;
            X_AB = std::abs(L[jA]);
          } else {
            scale = 3.3121686421112381E-170;
            DEC_param = (jA - i) + 2;
            for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
              if (L_dirtyOnGpu) {
                cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
              }

              L_dirtyOnGpu = false;
              Class_B_idx_9 = std::abs(L[jA + lastc]);
              if (Class_B_idx_9 > scale) {
                t = scale / Class_B_idx_9;
                X_AB = X_AB * t * t + 1.0;
                scale = Class_B_idx_9;
              } else {
                t = Class_B_idx_9 / scale;
                X_AB += t * t;
              }
            }

            X_AB = scale * std::sqrt(X_AB);
          }
        }

        if (X_AB != 0.0) {
          if (L_dirtyOnGpu) {
            cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
          }

          X_AB = rt_hypotd_snf(L[(i + 5 * i) + 1], X_AB);
          if (L[(i + 5 * i) + 1] >= 0.0) {
            X_AB = -X_AB;
          }

          if (std::abs(X_AB) < 1.0020841800044864E-292) {
            knt = -1;
            DEC_param = (jA - i) + 2;
            do {
              knt++;
              for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
                k = jA + lastc;
                L[k] *= 9.9792015476736E+291;
                ipiv_t_dirtyOnGpu = true;
              }

              X_AB *= 9.9792015476736E+291;
              if (Y_AB_dirtyOnGpu) {
                cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
              }

              Y_AB *= 9.9792015476736E+291;
              Y_AB_dirtyOnGpu = false;
            } while (!(std::abs(X_AB) >= 1.0020841800044864E-292));

            X_AB = 0.0;
            if (3 - i >= 1) {
              if (3 - i == 1) {
                X_AB = std::abs(L[jA]);
              } else {
                scale = 3.3121686421112381E-170;
                DEC_param = (jA - i) + 2;
                for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
                  Class_B_idx_9 = std::abs(L[jA + lastc]);
                  if (Class_B_idx_9 > scale) {
                    t = scale / Class_B_idx_9;
                    X_AB = X_AB * t * t + 1.0;
                    scale = Class_B_idx_9;
                  } else {
                    t = Class_B_idx_9 / scale;
                    X_AB += t * t;
                  }
                }

                X_AB = scale * std::sqrt(X_AB);
              }
            }

            X_AB = rt_hypotd_snf(Y_AB, X_AB);
            if (Y_AB >= 0.0) {
              X_AB = -X_AB;
            }

            tau[i] = (X_AB - Y_AB) / X_AB;
            a = 1.0 / (Y_AB - X_AB);
            DEC_param = (jA - i) + 2;
            for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
              k = jA + lastc;
              L[k] *= a;
              ipiv_t_dirtyOnGpu = true;
            }

            validLaunchParams = mwGetLaunchParameters(static_cast<double>(knt +
              1L), &grid, &block, 1024U, 65535U);
            if (validLaunchParams) {
              cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
              BEV_image_kernel120<<<grid, block>>>(knt, gpu_X_AB);
              cudaMemcpy(&X_AB, gpu_X_AB, 8UL, cudaMemcpyDeviceToHost);
            }

            Y_AB = X_AB;
            Y_AB_dirtyOnCpu = true;
          } else {
            tau[i] = (X_AB - L[(i + 5 * i) + 1]) / X_AB;
            a = 1.0 / (L[(i + 5 * i) + 1] - X_AB);
            DEC_param = (jA - i) + 2;
            for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
              k = jA + lastc;
              L[k] *= a;
              ipiv_t_dirtyOnGpu = true;
            }

            Y_AB = X_AB;
            Y_AB_dirtyOnCpu = true;
          }
        }

        if (ipiv_t_dirtyOnGpu) {
          cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
        }

        BEV_image_kernel121<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(i, *gpu_L);
        ipiv_t_dirtyOnGpu = false;
        L_dirtyOnGpu = true;
        iaii = (i + im1n) - 1;
        knt = in + 1;
        if (tau[i] != 0.0) {
          lastv = 3 - i;
          b_i = (iaii - i) + 3;
          exitg1 = false;
          while ((!exitg1) && (lastv + 1 > 0)) {
            if (L_dirtyOnGpu) {
              cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
            }

            L_dirtyOnGpu = false;
            if (L[b_i] == 0.0) {
              lastv--;
              b_i--;
            } else {
              exitg1 = true;
            }
          }

          lastc = 5;
          exitg1 = false;
          while ((!exitg1) && (lastc > 0)) {
            DEC_param = in + lastc;
            k = DEC_param;
            do {
              exitg2 = 0;
              if (k <= DEC_param + lastv * 5) {
                if (L_dirtyOnGpu) {
                  cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
                }

                L_dirtyOnGpu = false;
                if (L[k - 1] != 0.0) {
                  exitg2 = 1;
                } else {
                  k += 5;
                }
              } else {
                lastc--;
                exitg2 = 2;
              }
            } while (exitg2 == 0);

            if (exitg2 == 1) {
              exitg1 = true;
            }
          }
        } else {
          lastv = -1;
          lastc = 0;
        }

        if (lastv + 1 > 0) {
          if (lastc != 0) {
            validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
              ((lastc - 1) + 1L), &grid, &block, 1024U, 65535U);
            if (validLaunchParams) {
              BEV_image_kernel122<<<grid, block>>>(lastc, *gpu_work);
              old_flag_dirtyOnGpu = true;
            }

            ix = iaii;
            DEC_param = (in + 5 * lastv) + 1;
            for (m = 0; m <= (DEC_param - knt) / 5; m++) {
              b_L = in + m * 5;
              jA = (b_L + lastc) - 1;
              validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
                ((jA - b_L) + 1L), &grid, &block, 1024U, 65535U);
              if (validLaunchParams) {
                cudaMemcpy(b_gpu_L, &b_L, 4UL, cudaMemcpyHostToDevice);
                BEV_image_kernel123<<<grid, block>>>(ix, *gpu_L, b_gpu_L, jA,
                  *gpu_work);
                old_flag_dirtyOnGpu = true;
              }

              ix++;
            }
          }

          if (!(-tau[i] == 0.0)) {
            jA = in;
            for (j = 0; j <= lastv; j++) {
              if (L_dirtyOnGpu) {
                cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
              }

              L_dirtyOnGpu = false;
              if (L[iaii + j] != 0.0) {
                temp = L[iaii + j] * -tau[i];
                DEC_param = jA + 1;
                nr = lastc + jA;
                for (knt = 0; knt <= nr - DEC_param; knt++) {
                  ix = jA + knt;
                  if (old_flag_dirtyOnGpu) {
                    cudaMemcpy(work, *gpu_work, 40UL, cudaMemcpyDeviceToHost);
                  }

                  old_flag_dirtyOnGpu = false;
                  L[ix] += work[knt] * temp;
                  ipiv_t_dirtyOnGpu = true;
                }
              }

              jA += 5;
            }
          }
        }

        iaii = (i + im1n) - 1;
        knt = (i + in) + 2;
        if (tau[i] != 0.0) {
          lastv = 4 - i;
          b_i = (iaii - i) + 3;
          exitg1 = false;
          while ((!exitg1) && (lastv > 0)) {
            if (L_dirtyOnGpu) {
              cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
            }

            L_dirtyOnGpu = false;
            if (L[b_i] == 0.0) {
              lastv--;
              b_i--;
            } else {
              exitg1 = true;
            }
          }

          lastc = 4 - i;
          exitg1 = false;
          while ((!exitg1) && (lastc > 0)) {
            DEC_param = knt + (lastc - 1) * 5;
            k = DEC_param;
            do {
              exitg2 = 0;
              if (k <= (DEC_param + lastv) - 1) {
                if (L_dirtyOnGpu) {
                  cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
                }

                L_dirtyOnGpu = false;
                if (L[k - 1] != 0.0) {
                  exitg2 = 1;
                } else {
                  k++;
                }
              } else {
                lastc--;
                exitg2 = 2;
              }
            } while (exitg2 == 0);

            if (exitg2 == 1) {
              exitg1 = true;
            }
          }
        } else {
          lastv = 0;
          lastc = 0;
        }

        if (lastv > 0) {
          if (lastc != 0) {
            validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
              ((lastc - 1) + 1L), &grid, &block, 1024U, 65535U);
            if (validLaunchParams) {
              BEV_image_kernel124<<<grid, block>>>(lastc, *gpu_work);
              old_flag_dirtyOnGpu = true;
            }

            DEC_param = knt + 5 * (lastc - 1);
            validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
              ((DEC_param - knt) / 5 + 1L), &grid, &block, 1024U, 65535U);
            if (validLaunchParams) {
              if (ipiv_t_dirtyOnGpu) {
                cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
              }

              ipiv_t_dirtyOnGpu = false;
              BEV_image_kernel125<<<grid, block>>>(iaii, *gpu_L, lastv, knt,
                DEC_param, *gpu_work);
              old_flag_dirtyOnGpu = true;
            }
          }

          if (!(-tau[i] == 0.0)) {
            jA = knt - 1;
            for (j = 0; j < lastc; j++) {
              if (old_flag_dirtyOnGpu) {
                cudaMemcpy(work, *gpu_work, 40UL, cudaMemcpyDeviceToHost);
              }

              old_flag_dirtyOnGpu = false;
              d = work[j];
              if (d != 0.0) {
                temp = d * -tau[i];
                DEC_param = jA + 1;
                nr = lastv + jA;
                for (knt = 0; knt <= nr - DEC_param; knt++) {
                  ix = jA + knt;
                  if (L_dirtyOnGpu) {
                    cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
                  }

                  L[ix] += L[iaii + knt] * temp;
                  L_dirtyOnGpu = false;
                  ipiv_t_dirtyOnGpu = true;
                }
              }

              jA += 5;
            }
          }
        }

        if (Y_AB_dirtyOnCpu) {
          cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
        }

        Y_AB_dirtyOnCpu = false;
        if (ipiv_t_dirtyOnGpu) {
          cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
        }

        BEV_image_kernel126<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_Y_AB, i,
          *gpu_L);
        ipiv_t_dirtyOnGpu = false;
        L_dirtyOnGpu = true;
      }

      BEV_image_kernel127<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L,
        *gpu_Vr);
      Vr_dirtyOnGpu = true;
      for (j = 0; j < 4; j++) {
        knt = 4 - j;
        k = (4 - j) * 5;
        for (i = 0; i < knt; i++) {
          if (Vr_dirtyOnGpu) {
            cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
          }

          Vr[k + i] = 0.0;
          Vr_dirtyOnGpu = false;
          Vr_dirtyOnCpu = true;
        }

        for (i = 0; i <= 3 - knt; i++) {
          b_i = (i - j) + 4;
          if (Vr_dirtyOnGpu) {
            cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
          }

          Vr[(k + b_i) + 1] = Vr[(k + b_i) - 4];
          Vr_dirtyOnGpu = false;
          Vr_dirtyOnCpu = true;
        }
      }

      if (Vr_dirtyOnCpu) {
        cudaMemcpy(*gpu_Vr, Vr, 200UL, cudaMemcpyHostToDevice);
      }

      BEV_image_kernel128<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_Vr);
      BEV_image_kernel129<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_Vr);
      Vr_dirtyOnCpu = false;
      Vr_dirtyOnGpu = true;
      BEV_image_kernel130<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_work);
      old_flag_dirtyOnGpu = true;
      for (i = 0; i < 4; i++) {
        b_i = 4 - i;
        iaii = ((3 - i) * 5 - i) + 15;
        if (4 - i < 4) {
          if (Vr_dirtyOnCpu) {
            cudaMemcpy(*gpu_Vr, Vr, 200UL, cudaMemcpyHostToDevice);
          }

          BEV_image_kernel131<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(iaii,
            *gpu_Vr);
          Vr_dirtyOnGpu = true;
          if (tau[3 - i] != 0.0) {
            lastv = i + 1;
            DEC_param = (iaii + i) - 4;
            exitg1 = false;
            while ((!exitg1) && (lastv > 0)) {
              if (Vr_dirtyOnGpu) {
                cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
              }

              Vr_dirtyOnGpu = false;
              if (Vr[DEC_param - 2] == 0.0) {
                lastv--;
                DEC_param--;
              } else {
                exitg1 = true;
              }
            }

            lastc = i;
            exitg1 = false;
            while ((!exitg1) && (lastc > 0)) {
              DEC_param = iaii + (lastc - 1) * 5;
              k = DEC_param;
              do {
                exitg2 = 0;
                if (k <= (DEC_param + lastv) - 1) {
                  if (Vr_dirtyOnGpu) {
                    cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
                  }

                  Vr_dirtyOnGpu = false;
                  if (Vr[k - 1] != 0.0) {
                    exitg2 = 1;
                  } else {
                    k++;
                  }
                } else {
                  lastc--;
                  exitg2 = 2;
                }
              } while (exitg2 == 0);

              if (exitg2 == 1) {
                exitg1 = true;
              }
            }
          } else {
            lastv = 0;
            lastc = 0;
          }

          if (lastv > 0) {
            if (lastc != 0) {
              validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
                ((lastc - 1) + 1L), &grid, &block, 1024U, 65535U);
              if (validLaunchParams) {
                BEV_image_kernel132<<<grid, block>>>(lastc, *gpu_work);
                old_flag_dirtyOnGpu = true;
              }

              DEC_param = iaii + 5 * (lastc - 1);
              validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
                ((DEC_param - iaii) / 5 + 1L), &grid, &block, 1024U, 65535U);
              if (validLaunchParams) {
                BEV_image_kernel133<<<grid, block>>>(*gpu_Vr, lastv, iaii,
                  DEC_param, *gpu_work);
                old_flag_dirtyOnGpu = true;
              }
            }

            if (!(-tau[3 - i] == 0.0)) {
              jA = iaii - 1;
              for (j = 0; j < lastc; j++) {
                if (old_flag_dirtyOnGpu) {
                  cudaMemcpy(work, *gpu_work, 40UL, cudaMemcpyDeviceToHost);
                }

                old_flag_dirtyOnGpu = false;
                d = work[j];
                if (d != 0.0) {
                  temp = d * -tau[3 - i];
                  DEC_param = jA + 1;
                  nr = lastv + jA;
                  for (knt = 0; knt <= nr - DEC_param; knt++) {
                    ix = jA + knt;
                    if (Vr_dirtyOnGpu) {
                      cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
                    }

                    Vr[ix] += Vr[(iaii + knt) - 6] * temp;
                    Vr_dirtyOnGpu = false;
                  }
                }

                jA += 5;
              }
            }
          }

          jA = iaii - 3;
          DEC_param = (iaii + i) - 4;
          for (lastc = 0; lastc <= DEC_param - jA; lastc++) {
            k = (iaii + lastc) - 5;
            if (Vr_dirtyOnGpu) {
              cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
            }

            Vr[k] *= -tau[3 - i];
            Vr_dirtyOnGpu = false;
          }
        }

        if (Vr_dirtyOnGpu) {
          cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
        }

        Vr[iaii - 6] = 1.0 - tau[3 - i];
        Vr_dirtyOnGpu = false;
        Vr_dirtyOnCpu = true;
        for (j = 0; j <= b_i - 2; j++) {
          Vr[(iaii - j) - 7] = 0.0;
        }
      }

      BEV_image_kernel134<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v);
      v_dirtyOnGpu = true;
      BEV_image_kernel135<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L);
      L_dirtyOnGpu = true;
      i = 4;
      exitg1 = false;
      while ((!exitg1) && (i + 1 >= 1)) {
        b_L = 1;
        goto150 = false;
        jA = 0;
        exitg3 = false;
        while ((!exitg3) && (jA < 301)) {
          lastc = i;
          exitg4 = false;
          while ((!exitg4) && (lastc + 1 > b_L)) {
            if (L_dirtyOnGpu) {
              cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
            }

            L_dirtyOnGpu = false;
            if (std::abs(L[lastc + 5 * (lastc - 1)]) <= 5.0104209000224319E-292)
            {
              exitg4 = true;
            } else {
              if (ipiv_t_dirtyOnGpu) {
                cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
              }

              ipiv_t_dirtyOnGpu = false;
              if (Y_AB_dirtyOnCpu) {
                cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
              }

              BEV_image_kernel136<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (*gpu_L, lastc, gpu_Y_AB);
              Y_AB_dirtyOnCpu = false;
              cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
              Y_AB_dirtyOnGpu = false;
              if (Y_AB == 0.0) {
                if (lastc - 1 >= 1) {
                  BEV_image_kernel137<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                    (*gpu_L, lastc, gpu_Y_AB);
                  Y_AB_dirtyOnGpu = true;
                }

                if (lastc + 2 <= 5) {
                  BEV_image_kernel138<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                    (*gpu_L, lastc, gpu_Y_AB);
                  Y_AB_dirtyOnGpu = true;
                }
              }

              if (Y_AB_dirtyOnGpu) {
                cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
              }

              if (std::abs(L[lastc + 5 * (lastc - 1)]) <= 2.2204460492503131E-16
                  * Y_AB) {
                Y_AB = std::abs(L[lastc + 5 * (lastc - 1)]);
                X_AB = std::abs(L[(lastc + 5 * lastc) - 1]);
                if (Y_AB > X_AB) {
                  Class_B_idx_9 = Y_AB;
                  Class_B_idx_10 = X_AB;
                } else {
                  Class_B_idx_9 = X_AB;
                  Class_B_idx_10 = Y_AB;
                }

                Y_AB = std::abs(L[lastc + 5 * lastc]);
                Y_AB_dirtyOnCpu = true;
                X_AB = std::abs(L[(lastc + 5 * (lastc - 1)) - 1] - L[lastc + 5 *
                                lastc]);
                if (Y_AB > X_AB) {
                  temp = Y_AB;
                  Y_AB = X_AB;
                } else {
                  temp = X_AB;
                }

                scale = temp + Class_B_idx_9;
                if (Class_B_idx_10 * (Class_B_idx_9 / scale) <= std::fmax
                    (5.0104209000224319E-292, 2.2204460492503131E-16 * (Y_AB *
                      (temp / scale)))) {
                  exitg4 = true;
                } else {
                  lastc--;
                }
              } else {
                lastc--;
              }
            }
          }

          b_L = lastc + 1;
          if (lastc + 1 > 1) {
            if (ipiv_t_dirtyOnGpu) {
              cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
            }

            BEV_image_kernel139<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(lastc, *
              gpu_L);
            ipiv_t_dirtyOnGpu = false;
            L_dirtyOnGpu = true;
          }

          if (lastc + 1 >= i) {
            goto150 = true;
            exitg3 = true;
          } else {
            if (jA == 10) {
              if (L_dirtyOnGpu) {
                cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
              }

              scale = std::abs(L[(lastc + 5 * lastc) + 1]) + std::abs(L[(lastc +
                5 * (lastc + 1)) + 2]);
              L_dirtyOnGpu = false;
              X_AB = 0.75 * scale + L[lastc + 5 * lastc];
              Class_B_idx_9 = -0.4375 * scale;
              Class_B_idx_9_dirtyOnCpu = true;
              Y_AB = scale;
              temp = X_AB;
            } else if (jA == 20) {
              if (L_dirtyOnGpu) {
                cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
              }

              scale = std::abs(L[i + 5 * (i - 1)]) + std::abs(L[(i + 5 * (i - 2))
                - 1]);
              L_dirtyOnGpu = false;
              X_AB = 0.75 * scale + L[i + 5 * i];
              Class_B_idx_9 = -0.4375 * scale;
              Class_B_idx_9_dirtyOnCpu = true;
              Y_AB = scale;
              temp = X_AB;
            } else {
              if (L_dirtyOnGpu) {
                cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
              }

              X_AB = L[(i + 5 * (i - 1)) - 1];
              Y_AB = L[i + 5 * (i - 1)];
              Class_B_idx_9 = L[(i + 5 * i) - 1];
              Class_B_idx_9_dirtyOnCpu = true;
              L_dirtyOnGpu = false;
              temp = L[i + 5 * i];
            }

            scale = ((std::abs(X_AB) + std::abs(Class_B_idx_9)) + std::abs(Y_AB))
              + std::abs(temp);
            if (scale == 0.0) {
              t = 0.0;
              X_AB = 0.0;
              Class_B_idx_10 = 0.0;
              Y_AB = 0.0;
              Y_AB_dirtyOnCpu = true;
            } else {
              X_AB /= scale;
              Y_AB /= scale;
              Class_B_idx_9 /= scale;
              temp /= scale;
              Class_B_idx_10 = (X_AB + temp) / 2.0;
              X_AB = (X_AB - Class_B_idx_10) * (temp - Class_B_idx_10) -
                Class_B_idx_9 * Y_AB;
              Y_AB = std::sqrt(std::abs(X_AB));
              if (X_AB >= 0.0) {
                t = Class_B_idx_10 * scale;
                Class_B_idx_10 = t;
                X_AB = Y_AB * scale;
                Y_AB = -X_AB;
                Y_AB_dirtyOnCpu = true;
              } else {
                t = Class_B_idx_10 + Y_AB;
                Class_B_idx_10 -= Y_AB;
                if (std::abs(t - temp) <= std::abs(Class_B_idx_10 - temp)) {
                  t *= scale;
                  Class_B_idx_10 = t;
                } else {
                  Class_B_idx_10 *= scale;
                  t = Class_B_idx_10;
                }

                X_AB = 0.0;
                Y_AB = 0.0;
                Y_AB_dirtyOnCpu = true;
              }
            }

            m = i - 1;
            exitg4 = false;
            while ((!exitg4) && (m >= lastc + 1)) {
              scale = (std::abs(L[(m + 5 * (m - 1)) - 1] - Class_B_idx_10) + std::
                       abs(Y_AB)) + std::abs(L[m + 5 * (m - 1)]);
              Class_B_idx_9 = L[m + 5 * (m - 1)] / scale;
              if (v_dirtyOnGpu) {
                cudaMemcpy(v, *gpu_v, 24UL, cudaMemcpyDeviceToHost);
              }

              v[0] = (Class_B_idx_9 * L[(m + 5 * m) - 1] + (L[(m + 5 * (m - 1))
                       - 1] - t) * ((L[(m + 5 * (m - 1)) - 1] - Class_B_idx_10) /
                                    scale)) - X_AB * (Y_AB / scale);
              v[1] = Class_B_idx_9 * (((L[(m + 5 * (m - 1)) - 1] + L[m + 5 * m])
                - t) - Class_B_idx_10);
              v[2] = Class_B_idx_9 * L[(m + 5 * m) + 1];
              scale = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
              v[0] /= scale;
              v[1] /= scale;
              v[2] /= scale;
              v_dirtyOnGpu = false;
              v_dirtyOnCpu = true;
              if ((m == lastc + 1) || (std::abs(L[(m + 5 * (m - 2)) - 1]) * (std::
                    abs(v[1]) + std::abs(v[2])) <= 2.2204460492503131E-16 * std::
                   abs(v[0]) * ((std::abs(L[(m + 5 * (m - 2)) - 2]) + std::abs
                                 (L[(m + 5 * (m - 1)) - 1])) + std::abs(L[m + 5 *
                     m])))) {
                exitg4 = true;
              } else {
                m--;
              }
            }

            for (k = 0; k <= i - m; k++) {
              ix = m + k;
              nr = (i - ix) + 2;
              nr_dirtyOnCpu = true;
              if (3 < nr) {
                nr = 3;
              }

              if (ix > m) {
                validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
                  ((nr - 1) + 1L), &grid, &block, 1024U, 65535U);
                if (validLaunchParams) {
                  if (ipiv_t_dirtyOnGpu) {
                    cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
                  }

                  ipiv_t_dirtyOnGpu = false;
                  cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
                  nr_dirtyOnCpu = false;
                  if (v_dirtyOnCpu) {
                    cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
                  }

                  BEV_image_kernel140<<<grid, block>>>(*gpu_L, (ix + 5 * (ix - 2))
                    - 1, gpu_nr, *gpu_v);
                  v_dirtyOnCpu = false;
                  v_dirtyOnGpu = true;
                }
              }

              if (v_dirtyOnGpu) {
                cudaMemcpy(v, *gpu_v, 24UL, cudaMemcpyDeviceToHost);
              }

              v_dirtyOnGpu = false;
              Y_AB = v[0];
              Class_B_idx_10 = 0.0;
              Class_B_idx_10_dirtyOnGpu = false;
              old_flag_dirtyOnGpu = true;
              if (nr > 0) {
                X_AB = 0.0;
                if (nr - 1 >= 1) {
                  if (nr - 1 == 1) {
                    X_AB = std::abs(v[1]);
                  } else {
                    scale = 3.3121686421112381E-170;
                    Class_B_idx_9 = std::abs(v[1]);
                    if (Class_B_idx_9 > 3.3121686421112381E-170) {
                      X_AB = 1.0;
                      scale = Class_B_idx_9;
                    } else {
                      t = Class_B_idx_9 / 3.3121686421112381E-170;
                      X_AB = t * t;
                    }

                    Class_B_idx_9 = std::abs(v[2]);
                    Class_B_idx_9_dirtyOnCpu = true;
                    if (Class_B_idx_9 > scale) {
                      t = scale / Class_B_idx_9;
                      X_AB = X_AB * t * t + 1.0;
                      scale = Class_B_idx_9;
                    } else {
                      t = Class_B_idx_9 / scale;
                      X_AB += t * t;
                    }

                    X_AB = scale * std::sqrt(X_AB);
                  }
                }

                if (X_AB != 0.0) {
                  X_AB = rt_hypotd_snf(v[0], X_AB);
                  if (v[0] >= 0.0) {
                    X_AB = -X_AB;
                  }

                  if (std::abs(X_AB) < 1.0020841800044864E-292) {
                    knt = -1;
                    do {
                      knt++;
                      validLaunchParams = mwGetLaunchParameters1D(static_cast<
                        double>((nr - 2) + 1L), &grid, &block, 1024U, 65535U);
                      if (validLaunchParams) {
                        if (nr_dirtyOnCpu) {
                          cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
                        }

                        nr_dirtyOnCpu = false;
                        if (v_dirtyOnCpu) {
                          cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
                        }

                        BEV_image_kernel143<<<grid, block>>>(gpu_nr, *gpu_v);
                        v_dirtyOnCpu = false;
                        v_dirtyOnGpu = true;
                      }

                      X_AB *= 9.9792015476736E+291;
                      Y_AB *= 9.9792015476736E+291;
                    } while (!(std::abs(X_AB) >= 1.0020841800044864E-292));

                    X_AB = 0.0;
                    if (nr - 1 >= 1) {
                      if (nr - 1 == 1) {
                        if (v_dirtyOnCpu) {
                          cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
                        }

                        v_dirtyOnCpu = false;
                        cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
                        BEV_image_kernel144<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                          (*gpu_v, gpu_X_AB);
                        cudaMemcpy(&X_AB, gpu_X_AB, 8UL, cudaMemcpyDeviceToHost);
                      } else {
                        scale = 3.3121686421112381E-170;
                        if (v_dirtyOnGpu) {
                          cudaMemcpy(v, *gpu_v, 24UL, cudaMemcpyDeviceToHost);
                        }

                        Class_B_idx_9 = std::abs(v[1]);
                        if (Class_B_idx_9 > 3.3121686421112381E-170) {
                          X_AB = 1.0;
                          scale = Class_B_idx_9;
                        } else {
                          t = Class_B_idx_9 / 3.3121686421112381E-170;
                          X_AB = t * t;
                        }

                        Class_B_idx_9 = std::abs(v[2]);
                        Class_B_idx_9_dirtyOnCpu = true;
                        if (Class_B_idx_9 > scale) {
                          t = scale / Class_B_idx_9;
                          X_AB = X_AB * t * t + 1.0;
                          scale = Class_B_idx_9;
                        } else {
                          t = Class_B_idx_9 / scale;
                          X_AB += t * t;
                        }

                        X_AB = scale * std::sqrt(X_AB);
                      }
                    }

                    X_AB = rt_hypotd_snf(Y_AB, X_AB);
                    if (Y_AB >= 0.0) {
                      X_AB = -X_AB;
                    }

                    Class_B_idx_10 = (X_AB - Y_AB) / X_AB;
                    validLaunchParams = mwGetLaunchParameters1D(static_cast<
                      double>((nr - 2) + 1L), &grid, &block, 1024U, 65535U);
                    if (validLaunchParams) {
                      if (nr_dirtyOnCpu) {
                        cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
                      }

                      if (v_dirtyOnCpu) {
                        cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
                      }

                      BEV_image_kernel145<<<grid, block>>>(1.0 / (Y_AB - X_AB),
                        gpu_nr, *gpu_v);
                      v_dirtyOnCpu = false;
                    }

                    validLaunchParams = mwGetLaunchParameters(static_cast<double>
                      (knt + 1L), &grid, &block, 1024U, 65535U);
                    if (validLaunchParams) {
                      cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
                      BEV_image_kernel146<<<grid, block>>>(knt, gpu_X_AB);
                      cudaMemcpy(&X_AB, gpu_X_AB, 8UL, cudaMemcpyDeviceToHost);
                    }

                    Y_AB = X_AB;
                  } else {
                    if (v_dirtyOnCpu) {
                      cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
                    }

                    v_dirtyOnCpu = false;
                    cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
                    cudaMemcpy(gpu_Class_B_idx_10, &Class_B_idx_10, 8UL,
                               cudaMemcpyHostToDevice);
                    BEV_image_kernel141<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                      (*gpu_v, gpu_X_AB, gpu_Class_B_idx_10);
                    old_flag_dirtyOnGpu = false;
                    Class_B_idx_10_dirtyOnGpu = true;
                    a = 1.0 / (v[0] - X_AB);
                    validLaunchParams = mwGetLaunchParameters1D(static_cast<
                      double>((nr - 2) + 1L), &grid, &block, 1024U, 65535U);
                    if (validLaunchParams) {
                      if (nr_dirtyOnCpu) {
                        cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
                      }

                      BEV_image_kernel142<<<grid, block>>>(a, gpu_nr, *gpu_v);
                    }

                    Y_AB = X_AB;
                  }
                }
              }

              cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
              if (v_dirtyOnCpu) {
                cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
              }

              BEV_image_kernel147<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (gpu_Y_AB, *gpu_v);
              v_dirtyOnCpu = false;
              if (ix > m) {
                L[(ix + 5 * (ix - 2)) - 1] = Y_AB;
                L[ix + 5 * (ix - 2)] = 0.0;
                ipiv_t_dirtyOnGpu = true;
                if (ix < i) {
                  L[(ix + 5 * (ix - 2)) + 1] = 0.0;
                }
              } else if (m > lastc + 1) {
                if (Class_B_idx_10_dirtyOnGpu) {
                  cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                             cudaMemcpyDeviceToHost);
                }

                Class_B_idx_10_dirtyOnGpu = false;
                L[(ix + 5 * (ix - 2)) - 1] *= 1.0 - Class_B_idx_10;
                ipiv_t_dirtyOnGpu = true;
              }

              cudaMemcpy(v, *gpu_v, 24UL, cudaMemcpyDeviceToHost);
              v_dirtyOnGpu = false;
              d = v[1];
              if (old_flag_dirtyOnGpu) {
                cudaMemcpy(gpu_Class_B_idx_10, &Class_B_idx_10, 8UL,
                           cudaMemcpyHostToDevice);
              }

              BEV_image_kernel148<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (*gpu_v, gpu_Class_B_idx_10, gpu_Y_AB);
              Y_AB_dirtyOnCpu = false;
              Y_AB_dirtyOnGpu = true;
              if (nr == 3) {
                scale = v[2];
                if (Class_B_idx_9_dirtyOnCpu) {
                  cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL,
                             cudaMemcpyHostToDevice);
                }

                BEV_image_kernel149<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                  (*gpu_v, gpu_Class_B_idx_10, gpu_Class_B_idx_9);
                Class_B_idx_9_dirtyOnCpu = false;
                old_flag_dirtyOnGpu = true;
                for (j = 0; j <= 5 - ix; j++) {
                  knt = (ix + j) - 1;
                  X_AB = (L[(ix + 5 * knt) - 1] + d * L[ix + 5 * knt]) + scale *
                    L[(ix + 5 * knt) + 1];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  L[(ix + 5 * knt) - 1] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  L[ix + 5 * knt] -= X_AB * Y_AB;
                  if (old_flag_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_9, gpu_Class_B_idx_9, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  old_flag_dirtyOnGpu = false;
                  L[(ix + 5 * knt) + 1] -= X_AB * Class_B_idx_9;
                  ipiv_t_dirtyOnGpu = true;
                }

                if (ix + 3 < i + 1) {
                  DEC_param = ix + 2;
                } else {
                  DEC_param = i;
                }

                for (j = 0; j <= DEC_param; j++) {
                  X_AB = (L[j + 5 * (ix - 1)] + d * L[j + 5 * ix]) + scale * L[j
                    + 5 * (ix + 1)];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  L[j + 5 * (ix - 1)] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  L[j + 5 * ix] -= X_AB * Y_AB;
                  if (old_flag_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_9, gpu_Class_B_idx_9, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  old_flag_dirtyOnGpu = false;
                  L[j + 5 * (ix + 1)] -= X_AB * Class_B_idx_9;
                  ipiv_t_dirtyOnGpu = true;
                }

                for (j = 0; j < 5; j++) {
                  if (Vr_dirtyOnGpu) {
                    cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
                  }

                  X_AB = (Vr[j + 5 * (ix - 1)] + d * Vr[j + 5 * ix]) + scale *
                    Vr[j + 5 * (ix + 1)];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  Vr[j + 5 * (ix - 1)] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  Vr[j + 5 * ix] -= X_AB * Y_AB;
                  if (old_flag_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_9, gpu_Class_B_idx_9, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  old_flag_dirtyOnGpu = false;
                  Vr[j + 5 * (ix + 1)] -= X_AB * Class_B_idx_9;
                  Vr_dirtyOnGpu = false;
                  Vr_dirtyOnCpu = true;
                }
              } else if (nr == 2) {
                for (j = 0; j <= 5 - ix; j++) {
                  knt = (ix + j) - 1;
                  X_AB = L[(ix + 5 * knt) - 1] + d * L[ix + 5 * knt];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  L[(ix + 5 * knt) - 1] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  L[ix + 5 * knt] -= X_AB * Y_AB;
                  ipiv_t_dirtyOnGpu = true;
                }

                for (j = 0; j <= i; j++) {
                  X_AB = L[j + 5 * (ix - 1)] + d * L[j + 5 * ix];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  L[j + 5 * (ix - 1)] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  L[j + 5 * ix] -= X_AB * Y_AB;
                  ipiv_t_dirtyOnGpu = true;
                }

                for (j = 0; j < 5; j++) {
                  if (Vr_dirtyOnGpu) {
                    cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
                  }

                  X_AB = Vr[j + 5 * (ix - 1)] + d * Vr[j + 5 * ix];
                  if (Class_B_idx_10_dirtyOnGpu) {
                    cudaMemcpy(&Class_B_idx_10, gpu_Class_B_idx_10, 8UL,
                               cudaMemcpyDeviceToHost);
                  }

                  Class_B_idx_10_dirtyOnGpu = false;
                  Vr[j + 5 * (ix - 1)] -= X_AB * Class_B_idx_10;
                  if (Y_AB_dirtyOnGpu) {
                    cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
                  }

                  Y_AB_dirtyOnGpu = false;
                  Vr[j + 5 * ix] -= X_AB * Y_AB;
                  Vr_dirtyOnGpu = false;
                  Vr_dirtyOnCpu = true;
                }
              }
            }

            jA++;
          }
        }

        if (!goto150) {
          exitg1 = true;
        } else {
          if ((b_L != i + 1) && (b_L == i)) {
            if (L_dirtyOnGpu) {
              cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
            }

            d2 = L[(i + 5 * (i - 1)) - 1];
            x = L[(i + 5 * i) - 1];
            b_x = L[i + 5 * (i - 1)];
            Class_B_idx_9 = L[i + 5 * i];
            L_dirtyOnGpu = false;
            if (L[i + 5 * (i - 1)] == 0.0) {
              scale = 1.0;
              old_Prob_ctrv = 0.0;
            } else if (L[(i + 5 * i) - 1] == 0.0) {
              scale = 0.0;
              old_Prob_ctrv = 1.0;
              Class_B_idx_9 = L[(i + 5 * (i - 1)) - 1];
              d2 = L[i + 5 * i];
              x = -L[i + 5 * (i - 1)];
              b_x = 0.0;
            } else if ((L[(i + 5 * (i - 1)) - 1] - L[i + 5 * i] == 0.0) && ((L
                         [(i + 5 * i) - 1] < 0.0) != (L[i + 5 * (i - 1)] < 0.0)))
            {
              scale = 1.0;
              old_Prob_ctrv = 0.0;
            } else {
              temp = L[(i + 5 * (i - 1)) - 1] - L[i + 5 * i];
              Class_B_idx_9 = 0.5 * temp;
              Y_AB = std::fmax(std::abs(L[(i + 5 * i) - 1]), std::abs(L[i + 5 *
                (i - 1)]));
              Y_AB_dirtyOnCpu = true;
              if (!(L[(i + 5 * i) - 1] < 0.0)) {
                DEC_param = 1;
              } else {
                DEC_param = -1;
              }

              if (!(L[i + 5 * (i - 1)] < 0.0)) {
                jA = 1;
              } else {
                jA = -1;
              }

              X_AB = std::fmin(std::abs(L[(i + 5 * i) - 1]), std::abs(L[i + 5 *
                (i - 1)])) * static_cast<double>(DEC_param) * static_cast<double>
                (jA);
              scale = std::fmax(std::abs(Class_B_idx_9), Y_AB);
              Class_B_idx_10 = Class_B_idx_9 / scale * Class_B_idx_9 + Y_AB /
                scale * X_AB;
              if (Class_B_idx_10 >= 8.8817841970012523E-16) {
                a = std::sqrt(scale) * std::sqrt(Class_B_idx_10);
                if (Class_B_idx_9 < 0.0) {
                  a = -a;
                }

                Class_B_idx_10 = Class_B_idx_9 + a;
                d2 = L[i + 5 * i] + Class_B_idx_10;
                Class_B_idx_9 = L[i + 5 * i] - Y_AB / Class_B_idx_10 * X_AB;
                t = rt_hypotd_snf(L[i + 5 * (i - 1)], Class_B_idx_10);
                scale = Class_B_idx_10 / t;
                old_Prob_ctrv = L[i + 5 * (i - 1)] / t;
                x = L[(i + 5 * i) - 1] - L[i + 5 * (i - 1)];
                b_x = 0.0;
              } else {
                X_AB = L[(i + 5 * i) - 1] + L[i + 5 * (i - 1)];
                t = rt_hypotd_snf(X_AB, temp);
                scale = std::sqrt(0.5 * (std::abs(X_AB) / t + 1.0));
                if (!(X_AB < 0.0)) {
                  b_i = 1;
                } else {
                  b_i = -1;
                }

                old_Prob_ctrv = -(Class_B_idx_9 / (t * scale)) * static_cast<
                  double>(b_i);
                temp = L[(i + 5 * (i - 1)) - 1] * scale + L[(i + 5 * i) - 1] *
                  old_Prob_ctrv;
                X_AB = -L[(i + 5 * (i - 1)) - 1] * old_Prob_ctrv + L[(i + 5 * i)
                  - 1] * scale;
                Y_AB = L[i + 5 * (i - 1)] * scale + L[i + 5 * i] * old_Prob_ctrv;
                Class_B_idx_9 = -L[i + 5 * (i - 1)] * old_Prob_ctrv + L[i + 5 *
                  i] * scale;
                x = X_AB * scale + Class_B_idx_9 * old_Prob_ctrv;
                b_x = -temp * old_Prob_ctrv + Y_AB * scale;
                temp = 0.5 * ((temp * scale + Y_AB * old_Prob_ctrv) + (-X_AB *
                  old_Prob_ctrv + Class_B_idx_9 * scale));
                d2 = temp;
                Class_B_idx_9 = temp;
                if (b_x != 0.0) {
                  if (x != 0.0) {
                    if ((x < 0.0) == (b_x < 0.0)) {
                      X_AB = std::sqrt(std::abs(x));
                      Class_B_idx_10 = std::sqrt(std::abs(b_x));
                      a = X_AB * Class_B_idx_10;
                      if (!(b_x < 0.0)) {
                        Class_B_idx_9 = a;
                      } else {
                        Class_B_idx_9 = -a;
                      }

                      t = 1.0 / std::sqrt(std::abs(x + b_x));
                      d2 = temp + Class_B_idx_9;
                      Class_B_idx_9 = temp - Class_B_idx_9;
                      x -= b_x;
                      b_x = 0.0;
                      Y_AB = X_AB * t;
                      X_AB = Class_B_idx_10 * t;
                      temp = scale * Y_AB - old_Prob_ctrv * X_AB;
                      old_Prob_ctrv = scale * X_AB + old_Prob_ctrv * Y_AB;
                      scale = temp;
                    }
                  } else {
                    x = -b_x;
                    b_x = 0.0;
                    temp = scale;
                    scale = -old_Prob_ctrv;
                    old_Prob_ctrv = temp;
                  }
                }
              }
            }

            L[(i + 5 * (i - 1)) - 1] = d2;
            L[(i + 5 * i) - 1] = x;
            L[i + 5 * (i - 1)] = b_x;
            L[i + 5 * i] = Class_B_idx_9;
            ipiv_t_dirtyOnGpu = true;
            if (5 > i + 1) {
              DEC_param = 3 - i;
              ix = (i + (i + 1) * 5) - 1;
              jA = i + (i + 1) * 5;
              for (lastc = 0; lastc <= DEC_param; lastc++) {
                temp = scale * L[ix + lastc * 5] + old_Prob_ctrv * L[jA + lastc *
                  5];
                L[jA + lastc * 5] = scale * L[jA + lastc * 5] - old_Prob_ctrv *
                  L[ix + lastc * 5];
                L[ix + lastc * 5] = temp;
              }
            }

            if (i - 1 >= 1) {
              ix = (i - 1) * 5;
              jA = i * 5;
              for (lastc = 0; lastc <= i - 2; lastc++) {
                temp = scale * L[ix + lastc] + old_Prob_ctrv * L[jA + lastc];
                L[jA + lastc] = scale * L[jA + lastc] - old_Prob_ctrv * L[ix +
                  lastc];
                L[ix + lastc] = temp;
              }
            }

            ix = (i - 1) * 5;
            jA = i * 5;
            for (lastc = 0; lastc < 5; lastc++) {
              if (Vr_dirtyOnGpu) {
                cudaMemcpy(Vr, *gpu_Vr, 200UL, cudaMemcpyDeviceToHost);
              }

              temp = scale * Vr[ix + lastc] + old_Prob_ctrv * Vr[jA + lastc];
              Vr[jA + lastc] = scale * Vr[jA + lastc] - old_Prob_ctrv * Vr[ix +
                lastc];
              Vr[ix + lastc] = temp;
              Vr_dirtyOnGpu = false;
              Vr_dirtyOnCpu = true;
            }
          }

          i = b_L - 2;
        }
      }

      for (j = 0; j < 2; j++) {
        DEC_param = j + 3;
        for (i = 0; i <= 4 - DEC_param; i++) {
          if (L_dirtyOnGpu) {
            cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
          }

          L[(DEC_param + i) + 5 * j] = 0.0;
          L_dirtyOnGpu = false;
          ipiv_t_dirtyOnGpu = true;
        }
      }

      if (Vr_dirtyOnCpu) {
        cudaMemcpy(*gpu_Vr, Vr, 200UL, cudaMemcpyHostToDevice);
      }

      Vr_dirtyOnCpu = false;
      if (ipiv_t_dirtyOnGpu) {
        cudaMemcpy(*gpu_L, L, 200UL, cudaMemcpyHostToDevice);
      }

      if (U_dirtyOnCpu) {
        cudaMemcpy(*gpu_U, U, 400UL, cudaMemcpyHostToDevice);
      }

      BEV_image_kernel150<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_Vr,
        *gpu_L, *gpu_U, *gpu_L_tmp);
      nr_dirtyOnCpu = false;
      validLaunchParams = true;
      U_dirtyOnCpu = false;
      for (m = 0; m < 4; m++) {
        ix = 5 - m;
        if (L_dirtyOnGpu) {
          cudaMemcpy(L, *gpu_L, 200UL, cudaMemcpyDeviceToHost);
        }

        L_dirtyOnGpu = false;
        d = L[(5 * (3 - m) - m) + 4];
        if (d != 0.0) {
          a = L[(5 * (3 - m) - m) + 3];
          old_Prob_cv = L[(5 * (4 - m) - m) + 3];
          Class_B_idx_10 = d;
          if (!(d == 0.0)) {
            if (old_Prob_cv == 0.0) {
              a = L[(5 * (4 - m) - m) + 4];
              old_Prob_cv = -d;
              Class_B_idx_10 = 0.0;
            } else if ((!(a - L[(5 * (4 - m) - m) + 4] == 0.0)) || ((old_Prob_cv
              < 0.0) == (d < 0.0))) {
              temp = a - L[(5 * (4 - m) - m) + 4];
              Class_B_idx_9 = 0.5 * temp;
              Y_AB = std::fmax(std::abs(old_Prob_cv), std::abs(d));
              if (!(old_Prob_cv < 0.0)) {
                DEC_param = 1;
              } else {
                DEC_param = -1;
              }

              if (!(d < 0.0)) {
                jA = 1;
              } else {
                jA = -1;
              }

              scale = std::fmax(std::abs(Class_B_idx_9), Y_AB);
              Class_B_idx_10 = Class_B_idx_9 / scale * Class_B_idx_9 + Y_AB /
                scale * (std::fmin(std::abs(old_Prob_cv), std::abs(d)) *
                         static_cast<double>(DEC_param) * static_cast<double>(jA));
              if (Class_B_idx_10 >= 8.8817841970012523E-16) {
                a = std::sqrt(scale) * std::sqrt(Class_B_idx_10);
                if (Class_B_idx_9 < 0.0) {
                  a = -a;
                }

                a = L[(5 * (4 - m) - m) + 4] + (Class_B_idx_9 + a);
                old_Prob_cv -= d;
                Class_B_idx_10 = 0.0;
              } else {
                X_AB = old_Prob_cv + d;
                t = rt_hypotd_snf(X_AB, temp);
                scale = std::sqrt(0.5 * (std::abs(X_AB) / t + 1.0));
                if (!(X_AB < 0.0)) {
                  b_i = 1;
                } else {
                  b_i = -1;
                }

                old_Prob_ctrv = -(Class_B_idx_9 / (t * scale)) * static_cast<
                  double>(b_i);
                temp = a * scale + old_Prob_cv * old_Prob_ctrv;
                X_AB = -a * old_Prob_ctrv + old_Prob_cv * scale;
                Y_AB = d * scale + L[(5 * (4 - m) - m) + 4] * old_Prob_ctrv;
                Class_B_idx_9 = -d * old_Prob_ctrv + L[(5 * (4 - m) - m) + 4] *
                  scale;
                old_Prob_cv = X_AB * scale + Class_B_idx_9 * old_Prob_ctrv;
                Class_B_idx_10 = -temp * old_Prob_ctrv + Y_AB * scale;
                temp = 0.5 * ((temp * scale + Y_AB * old_Prob_ctrv) + (-X_AB *
                  old_Prob_ctrv + Class_B_idx_9 * scale));
                a = temp;
                if (Class_B_idx_10 != 0.0) {
                  if (old_Prob_cv != 0.0) {
                    if ((old_Prob_cv < 0.0) == (Class_B_idx_10 < 0.0)) {
                      a = std::sqrt(std::abs(old_Prob_cv)) * std::sqrt(std::abs
                        (Class_B_idx_10));
                      if (Class_B_idx_10 < 0.0) {
                        a = -a;
                      }

                      a += temp;
                      old_Prob_cv -= Class_B_idx_10;
                      Class_B_idx_10 = 0.0;
                    }
                  } else {
                    old_Prob_cv = -Class_B_idx_10;
                    Class_B_idx_10 = 0.0;
                  }
                }
              }
            }
          }

          Class_B_idx_9 = a - L[(5 * (4 - m) - m) + 4];
          if (Class_B_idx_10 == 0.0) {
            X_AB = 0.0;
          } else {
            X_AB = std::sqrt(std::abs(old_Prob_cv)) * std::sqrt(std::abs
              (Class_B_idx_10));
          }

          Y_AB = rt_hypotd_snf(rt_hypotd_snf(Class_B_idx_9, X_AB), d);
          if (X_AB == 0.0) {
            cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL,
                       cudaMemcpyHostToDevice);
            BEV_image_kernel152<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (gpu_Y_AB, gpu_Class_B_idx_9, gpu_c);
            c_dirtyOnGpu = true;
          } else if (Class_B_idx_9 == 0.0) {
            if (c_dirtyOnGpu) {
              cudaMemcpy(&c, gpu_c, 16UL, cudaMemcpyDeviceToHost);
            }

            c.re = 0.0;
            c.im = X_AB / Y_AB;
            c_dirtyOnGpu = false;
            c_dirtyOnCpu = true;
          } else {
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL,
                       cudaMemcpyHostToDevice);
            BEV_image_kernel151<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (gpu_X_AB, gpu_Y_AB, gpu_Class_B_idx_9, gpu_c);
            c_dirtyOnGpu = true;
          }

          scale = d / Y_AB;
          for (j = 0; j <= 6 - ix; j++) {
            knt = (j - m) + 3;
            if (validLaunchParams) {
              cudaMemcpy(L_tmp, *gpu_L_tmp, 400UL, cudaMemcpyDeviceToHost);
            }

            t1 = L_tmp[(5 * knt - m) + 3];
            d = L_tmp[(5 * knt - m) + 3].re;
            if (c_dirtyOnGpu) {
              cudaMemcpy(&c, gpu_c, 16UL, cudaMemcpyDeviceToHost);
            }

            L_tmp[(5 * knt - m) + 3].re = (c.re * L_tmp[(5 * knt - m) + 3].re +
              c.im * L_tmp[(5 * knt - m) + 3].im) + scale * L_tmp[(5 * knt - m)
              + 4].re;
            L_tmp[(5 * knt - m) + 3].im = (c.re * L_tmp[(5 * knt - m) + 3].im -
              c.im * d) + scale * L_tmp[(5 * knt - m) + 4].im;
            X_AB = c.re * L_tmp[(5 * knt - m) + 4].im + c.im * L_tmp[(5 * knt -
              m) + 4].re;
            c_dirtyOnGpu = false;
            L_tmp[(5 * knt - m) + 4].re = (c.re * L_tmp[(5 * knt - m) + 4].re -
              c.im * L_tmp[(5 * knt - m) + 4].im) - scale * t1.re;
            L_tmp[(5 * knt - m) + 4].im = X_AB - scale * t1.im;
            validLaunchParams = false;
            nr_dirtyOnCpu = true;
          }

          validLaunchParams = mwGetLaunchParameters1D(static_cast<double>((4 - m)
            + 1L), &grid, &block, 1024U, 65535U);
          if (validLaunchParams) {
            cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
            if (c_dirtyOnCpu) {
              cudaMemcpy(gpu_c, &c, 16UL, cudaMemcpyHostToDevice);
            }

            c_dirtyOnCpu = false;
            if (nr_dirtyOnCpu) {
              cudaMemcpy(*gpu_L_tmp, L_tmp, 400UL, cudaMemcpyHostToDevice);
            }

            BEV_image_kernel153<<<grid, block>>>(gpu_scale, gpu_c, 5 - m,
              *gpu_L_tmp);
            nr_dirtyOnCpu = false;
          } else {
            cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
          }

          if (c_dirtyOnCpu) {
            cudaMemcpy(gpu_c, &c, 16UL, cudaMemcpyHostToDevice);
          }

          c_dirtyOnCpu = false;
          BEV_image_kernel154<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_scale,
            gpu_c, 5 - m, *gpu_U);
          if (nr_dirtyOnCpu) {
            cudaMemcpy(*gpu_L_tmp, L_tmp, 400UL, cudaMemcpyHostToDevice);
          }

          BEV_image_kernel155<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(5 - m,
            *gpu_L_tmp);
          nr_dirtyOnCpu = false;
          validLaunchParams = true;
        }
      }
    }

    if (R_dirtyOnCpu) {
      cudaMemcpy(*gpu_R, R, 400UL, cudaMemcpyHostToDevice);
    }

    BEV_image_kernel158<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_R);
    R_dirtyOnCpu = false;
    v_dirtyOnGpu = true;
    j = 0;
    do {
      exitg5 = 0;
      if (j + 1 < 6) {
        i = 1;
        do {
          exitg2 = 0;
          if (i <= j) {
            if (validLaunchParams) {
              cudaMemcpy(L_tmp, *gpu_L_tmp, 400UL, cudaMemcpyDeviceToHost);
            }

            validLaunchParams = false;
            if ((L_tmp[(i + 5 * j) - 1].re != 0.0) || (L_tmp[(i + 5 * j) - 1].im
                 != 0.0)) {
              for (j = 0; j < 5; j++) {
                Class_B_idx_9 = L_tmp[j + 5 * j].re;
                Class_B_idx_10 = L_tmp[j + 5 * j].im;
                if (Class_B_idx_10 == 0.0) {
                  if (Class_B_idx_9 < 0.0) {
                    X_AB = 0.0;
                    Class_B_idx_9 = std::sqrt(-Class_B_idx_9);
                  } else {
                    X_AB = std::sqrt(Class_B_idx_9);
                    Class_B_idx_9 = 0.0;
                  }
                } else if (Class_B_idx_9 == 0.0) {
                  if (Class_B_idx_10 < 0.0) {
                    X_AB = std::sqrt(-Class_B_idx_10 / 2.0);
                    Class_B_idx_9 = -X_AB;
                  } else {
                    X_AB = std::sqrt(Class_B_idx_10 / 2.0);
                    Class_B_idx_9 = X_AB;
                  }
                } else if (std::isnan(Class_B_idx_9)) {
                  X_AB = Class_B_idx_9;
                } else if (std::isnan(Class_B_idx_10)) {
                  X_AB = Class_B_idx_10;
                  Class_B_idx_9 = Class_B_idx_10;
                } else if (std::isinf(Class_B_idx_10)) {
                  X_AB = std::abs(Class_B_idx_10);
                  Class_B_idx_9 = Class_B_idx_10;
                } else if (std::isinf(Class_B_idx_9)) {
                  if (Class_B_idx_9 < 0.0) {
                    X_AB = 0.0;
                    Class_B_idx_9 = Class_B_idx_10 * -Class_B_idx_9;
                  } else {
                    X_AB = Class_B_idx_9;
                    Class_B_idx_9 = 0.0;
                  }
                } else {
                  Y_AB = std::abs(Class_B_idx_9);
                  X_AB = std::abs(Class_B_idx_10);
                  if ((Y_AB > 4.4942328371557893E+307) || (X_AB >
                       4.4942328371557893E+307)) {
                    Y_AB *= 0.5;
                    X_AB = rt_hypotd_snf(Y_AB, X_AB * 0.5);
                    if (X_AB > Y_AB) {
                      X_AB = std::sqrt(X_AB) * std::sqrt(Y_AB / X_AB + 1.0);
                    } else {
                      X_AB = std::sqrt(X_AB) * 1.4142135623730951;
                    }
                  } else {
                    X_AB = std::sqrt((rt_hypotd_snf(Y_AB, X_AB) + Y_AB) * 0.5);
                  }

                  if (Class_B_idx_9 > 0.0) {
                    Class_B_idx_9 = 0.5 * (Class_B_idx_10 / X_AB);
                  } else {
                    if (Class_B_idx_10 < 0.0) {
                      Class_B_idx_9 = -X_AB;
                    } else {
                      Class_B_idx_9 = X_AB;
                    }

                    X_AB = 0.5 * (Class_B_idx_10 / Class_B_idx_9);
                  }
                }

                if (v_dirtyOnGpu) {
                  cudaMemcpy(R, *gpu_R, 400UL, cudaMemcpyDeviceToHost);
                }

                R[j + 5 * j].re = X_AB;
                R[j + 5 * j].im = Class_B_idx_9;
                v_dirtyOnGpu = false;
                R_dirtyOnCpu = true;
                for (i = 0; i < j; i++) {
                  b_i = (j - i) - 1;
                  X_AB = 0.0;
                  Y_AB = 0.0;
                  DEC_param = b_i + 2;
                  for (lastc = 0; lastc <= j - DEC_param; lastc++) {
                    k = (b_i + lastc) + 1;
                    X_AB += R[b_i + 5 * k].re * R[k + 5 * j].re - R[b_i + 5 * k]
                      .im * R[k + 5 * j].im;
                    Y_AB += R[b_i + 5 * k].re * R[k + 5 * j].im + R[b_i + 5 * k]
                      .im * R[k + 5 * j].re;
                  }

                  old_Prob_ctrv = L_tmp[b_i + 5 * j].re - X_AB;
                  temp = L_tmp[b_i + 5 * j].im - Y_AB;
                  Y_AB = R[b_i + 5 * b_i].re + R[j + 5 * j].re;
                  Class_B_idx_9 = R[b_i + 5 * b_i].im + R[j + 5 * j].im;
                  if (Class_B_idx_9 == 0.0) {
                    if (temp == 0.0) {
                      t = old_Prob_ctrv / Y_AB;
                      X_AB = 0.0;
                    } else if (old_Prob_ctrv == 0.0) {
                      t = 0.0;
                      X_AB = temp / Y_AB;
                    } else {
                      t = old_Prob_ctrv / Y_AB;
                      X_AB = temp / Y_AB;
                    }
                  } else if (Y_AB == 0.0) {
                    if (old_Prob_ctrv == 0.0) {
                      t = temp / Class_B_idx_9;
                      X_AB = 0.0;
                    } else if (temp == 0.0) {
                      t = 0.0;
                      X_AB = -(old_Prob_ctrv / Class_B_idx_9);
                    } else {
                      t = temp / Class_B_idx_9;
                      X_AB = -(old_Prob_ctrv / Class_B_idx_9);
                    }
                  } else {
                    Class_B_idx_10 = std::abs(Y_AB);
                    X_AB = std::abs(Class_B_idx_9);
                    if (Class_B_idx_10 > X_AB) {
                      scale = Class_B_idx_9 / Y_AB;
                      old_Prob_cv = Y_AB + scale * Class_B_idx_9;
                      t = (old_Prob_ctrv + scale * temp) / old_Prob_cv;
                      X_AB = (temp - scale * old_Prob_ctrv) / old_Prob_cv;
                    } else if (X_AB == Class_B_idx_10) {
                      if (Y_AB > 0.0) {
                        Y_AB = 0.5;
                      } else {
                        Y_AB = -0.5;
                      }

                      if (Class_B_idx_9 > 0.0) {
                        X_AB = 0.5;
                      } else {
                        X_AB = -0.5;
                      }

                      t = (old_Prob_ctrv * Y_AB + temp * X_AB) / Class_B_idx_10;
                      X_AB = (temp * Y_AB - old_Prob_ctrv * X_AB) /
                        Class_B_idx_10;
                    } else {
                      scale = Y_AB / Class_B_idx_9;
                      old_Prob_cv = Class_B_idx_9 + scale * Y_AB;
                      t = (scale * old_Prob_ctrv + temp) / old_Prob_cv;
                      X_AB = (scale * temp - old_Prob_ctrv) / old_Prob_cv;
                    }
                  }

                  R[b_i + 5 * j].re = t;
                  R[b_i + 5 * j].im = X_AB;
                }
              }

              exitg2 = 1;
            } else {
              i++;
            }
          } else {
            j++;
            exitg2 = 2;
          }
        } while (exitg2 == 0);

        if (exitg2 == 1) {
          exitg5 = 1;
        }
      } else {
        BEV_image_kernel159<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp,
          *gpu_R);
        exitg5 = 1;
      }
    } while (exitg5 == 0);

    for (k = 0; k < 5; k++) {
      if (R_dirtyOnCpu) {
        cudaMemcpy(*gpu_R, R, 400UL, cudaMemcpyHostToDevice);
      }

      R_dirtyOnCpu = false;
      if (U_dirtyOnCpu) {
        cudaMemcpy(*gpu_U, U, 400UL, cudaMemcpyHostToDevice);
      }

      cudaMemcpy(gpu_k, &k, 4UL, cudaMemcpyHostToDevice);
      BEV_image_kernel160<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_R,
        *gpu_U, gpu_k, *b_gpu_U);
      U_dirtyOnCpu = false;
      BEV_image_kernel161<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*b_gpu_U,
        *gpu_U, gpu_k, *gpu_L_tmp);
    }

    BEV_image_kernel162<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp,
      *gpu_L);
    temp = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 5)) {
      scale = 0.0;
      cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel163<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L, j,
        gpu_scale);
      cudaMemcpy(&scale, gpu_scale, 8UL, cudaMemcpyDeviceToHost);
      if (std::isnan(scale)) {
        temp = rtNaN;
        exitg1 = true;
      } else {
        if (scale > temp) {
          temp = scale;
        }

        j++;
      }
    }

    X_AB = 0.0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 5)) {
      scale = 0.0;
      cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel164<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp, j,
        gpu_scale);
      cudaMemcpy(&scale, gpu_scale, 8UL, cudaMemcpyDeviceToHost);
      if (std::isnan(scale)) {
        X_AB = rtNaN;
        exitg1 = true;
      } else {
        if (scale > X_AB) {
          X_AB = scale;
        }

        j++;
      }
    }

    if (temp <= 1.1102230246251565E-14 * X_AB) {
      BEV_image_kernel165<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp);
    }

    BEV_image_kernel166<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L_tmp,
      *gpu_L);

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Sampling
    BEV_image_kernel167<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>(*gpu_X_k);
    Class_B_idx_9_dirtyOnCpu = true;
    BEV_image_kernel168<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_W);
    Vr_dirtyOnGpu = true;
    for (i = 0; i < 11; i++) {
      if (i == 0) {
        BEV_image_kernel173<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*gpu_x_cv_out, *gpu_X_k);
        BEV_image_kernel174<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_W);
      } else if (i <= 5) {
        BEV_image_kernel171<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L, i -
          1, *gpu_x_cv_out, i, *gpu_X_k);
        BEV_image_kernel172<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(i, *gpu_W);
      } else {
        BEV_image_kernel169<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L, i -
          6, *gpu_x_cv_out, i, *gpu_X_k);
        BEV_image_kernel170<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(i, *gpu_W);
      }
    }

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Model prediction
    BEV_image_kernel175<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_k_hat);
    BEV_image_kernel176<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L);
    BEV_image_kernel177<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_work);
    for (i = 0; i < 11; i++) {
      //  x_k = [Y X Yaw v yawrate]';
      if (Class_B_idx_9_dirtyOnCpu) {
        cudaMemcpy(X_k, *gpu_X_k, 440UL, cudaMemcpyDeviceToHost);
      }

      d = X_k[5 * i + 3];
      Class_B_idx_9_dirtyOnCpu = false;
      scale = X_k[5 * i + 2];
      cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
      cudaMemcpy(gpu_d, &d, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel178<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_scale, ts,
        gpu_d, *gpu_X_k, i, *gpu_work);
      if (Vr_dirtyOnGpu) {
        cudaMemcpy(W, *gpu_W, 88UL, cudaMemcpyDeviceToHost);
      }

      Vr_dirtyOnGpu = false;
      X_AB = W[i];
      cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel179<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_X_AB, i, *
        gpu_work, *gpu_x_k_hat, *gpu_X_k_hat);
    }

    for (i = 0; i < 11; i++) {
      if (Vr_dirtyOnGpu) {
        cudaMemcpy(W, *gpu_W, 88UL, cudaMemcpyDeviceToHost);
      }

      Vr_dirtyOnGpu = false;
      X_AB = W[i];
      cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel180<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_X_AB,
        *gpu_x_k_hat, *gpu_X_k_hat, i, *gpu_tmp_ego_y, *gpu_work);
      BEV_image_kernel181<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*gpu_tmp_ego_y, *gpu_work, *gpu_L);
    }

    P_cv_dirtyOnCpu = false;
    BEV_image_kernel182<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_cv,
      *gpu_L);

    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    //  Measurement update
    if (v_dirtyOnCpu) {
      cudaMemcpy(*gpu_v, v, 24UL, cudaMemcpyHostToDevice);
    }

    BEV_image_kernel183<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v);
    v_dirtyOnCpu = false;
    BEV_image_kernel184<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz);
    BEV_image_kernel185<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_xz);
    Class_B_idx_10_dirtyOnGpu = true;
    for (i = 0; i < 11; i++) {
      if (a_dirtyOnCpu) {
        cudaMemcpy(*gpu_a, b_a, 15UL, cudaMemcpyHostToDevice);
      }

      a_dirtyOnCpu = false;
      BEV_image_kernel186<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_X_k,
        *gpu_a, i, *gpu_Y_k_hat);
      if (Vr_dirtyOnGpu) {
        cudaMemcpy(W, *gpu_W, 88UL, cudaMemcpyDeviceToHost);
      }

      Vr_dirtyOnGpu = false;
      X_AB = W[i];
      cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel187<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_Y_k_hat,
        i, gpu_X_AB, *gpu_v);
    }

    for (i = 0; i < 11; i++) {
      if (Vr_dirtyOnGpu) {
        cudaMemcpy(W, *gpu_W, 88UL, cudaMemcpyDeviceToHost);
      }

      Vr_dirtyOnGpu = false;
      X_AB = W[i];
      cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
      BEV_image_kernel188<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_X_AB,
        *gpu_v, *gpu_Y_k_hat, i, *b_gpu_Y_k_hat, *b_gpu_W);
      BEV_image_kernel189<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*b_gpu_Y_k_hat, *b_gpu_W, *gpu_P_zz);
      BEV_image_kernel190<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_k_hat,
        *gpu_X_k_hat, i, gpu_X_AB, *gpu_work);
      BEV_image_kernel191<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v,
        *gpu_Y_k_hat, i, *b_gpu_Y_k_hat);
      BEV_image_kernel192<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*b_gpu_Y_k_hat, *gpu_work, *gpu_P_xz);
    }

    R_CTRV_IMM_dirtyOnCpu = false;
    BEV_image_kernel193<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_R_CTRV_IMM,
      *gpu_A, *gpu_P_zz);
    jA = 0;
    ix = 1;
    nr = 2;
    cudaMemcpy(P_zz, *gpu_P_zz, 72UL, cudaMemcpyDeviceToHost);
    X_AB = std::abs(P_zz[0]);
    Y_AB = std::abs(P_zz[1]);
    if (Y_AB > X_AB) {
      X_AB = Y_AB;
      jA = 1;
      ix = 0;
    }

    if (std::abs(P_zz[2]) > X_AB) {
      jA = 2;
      ix = 1;
      nr = 0;
    }

    BEV_image_kernel194<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, *gpu_P_zz,
      ix, *gpu_A);
    cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
    BEV_image_kernel195<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, gpu_nr,
      *gpu_A);
    BEV_image_kernel196<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, ix, *gpu_A);
    BEV_image_kernel197<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, gpu_nr,
      *gpu_A);
    BEV_image_kernel198<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, ix, *gpu_A);
    BEV_image_kernel199<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA, gpu_nr,
      *gpu_A);
    cudaMemcpy(A, *gpu_A, 72UL, cudaMemcpyDeviceToHost);
    if (std::abs(A[nr + 3]) > std::abs(A[ix + 3])) {
      DEC_param = ix;
      ix = nr;
      nr = DEC_param;
    }

    A[nr + 3] /= A[ix + 3];
    A[nr + 6] -= A[nr + 3] * A[ix + 6];
    for (lastc = 0; lastc < 5; lastc++) {
      if (Class_B_idx_10_dirtyOnGpu) {
        cudaMemcpy(P_xz, *gpu_P_xz, 120UL, cudaMemcpyDeviceToHost);
      }

      K[lastc + 5 * jA] = P_xz[lastc] / A[jA];
      K[lastc + 5 * ix] = P_xz[lastc + 5] - K[lastc + 5 * jA] * A[jA + 3];
      Class_B_idx_10_dirtyOnGpu = false;
      K[lastc + 5 * nr] = P_xz[lastc + 10] - K[lastc + 5 * jA] * A[jA + 6];
      K[lastc + 5 * ix] /= A[ix + 3];
      K[lastc + 5 * nr] -= K[lastc + 5 * ix] * A[ix + 6];
      K[lastc + 5 * nr] /= A[nr + 6];
      K[lastc + 5 * ix] -= K[lastc + 5 * nr] * A[nr + 3];
      K[lastc + 5 * jA] -= K[lastc + 5 * nr] * A[nr];
      K[lastc + 5 * jA] -= K[lastc + 5 * ix] * A[ix];
      K_dirtyOnCpu = true;
    }

    cudaMemcpy(*gpu_A, A, 72UL, cudaMemcpyHostToDevice);
    BEV_image_kernel200<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz,
      *gpu_A);
    cusolverDnDgetrf_bufferSize(getCuSolverGlobalHandle(), 3, 3, (double *)
      &(*gpu_A)[0], 3, getCuSolverWorkspaceReq());
    setCuSolverWorkspaceTypeSize(8);
    cusolverInitWorkspace();
    cusolverDnDgetrf(getCuSolverGlobalHandle(), 3, 3, (double *)&(*gpu_A)[0], 3,
                     (double *)getCuSolverWorkspaceBuff(), &(*gpu_ipiv_t)[0],
                     b_gpu_info);
    ipiv_t_dirtyOnGpu = true;
    cudaMemcpy(&b_info, b_gpu_info, 4UL, cudaMemcpyDeviceToHost);
    if (b_info < 0) {
      BEV_image_kernel202<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_A);
      BEV_image_kernel203<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_ipiv);
    } else {
      BEV_image_kernel201<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_ipiv_t, *
        gpu_ipiv);
    }

    temp = 1.0;
    cudaMemcpy(gpu_temp, &temp, 8UL, cudaMemcpyHostToDevice);
    BEV_image_kernel204<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_A,
      gpu_temp);
    old_flag_dirtyOnGpu = true;
    cudaMemcpy(ipiv, *gpu_ipiv, 12UL, cudaMemcpyDeviceToHost);
    goto150 = (ipiv[0] > 1);
    if (ipiv[1] > 2) {
      goto150 = !goto150;
    }

    if (goto150) {
      cudaMemcpy(&temp, gpu_temp, 8UL, cudaMemcpyDeviceToHost);
      temp = -temp;
      old_flag_dirtyOnGpu = false;
    }

    BEV_image_kernel205<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz,
      *gpu_A);
    DEC_param = 0;
    jA = 3;
    ix = 6;
    X_AB = std::abs(P_zz[0]);
    Y_AB = std::abs(P_zz[1]);
    Class_B_idx_9 = std::abs(P_zz[2]);
    if ((Y_AB > X_AB) && (Y_AB > Class_B_idx_9)) {
      DEC_param = 3;
      jA = 0;
      BEV_image_kernel207<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz,
        *gpu_A);
    } else if (Class_B_idx_9 > X_AB) {
      DEC_param = 6;
      ix = 0;
      BEV_image_kernel206<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz,
        *gpu_A);
    }

    BEV_image_kernel208<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_A);
    BEV_image_kernel209<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_A);
    cudaMemcpy(A, *gpu_A, 72UL, cudaMemcpyDeviceToHost);
    A[7] -= A[1] * A[6];
    A[8] -= A[2] * A[6];
    if (std::abs(A[5]) > std::abs(A[4])) {
      nr = jA;
      jA = ix;
      ix = nr;
      Class_B_idx_10 = A[1];
      A[1] = A[2];
      A[2] = Class_B_idx_10;
      Class_B_idx_10 = A[4];
      A[4] = A[5];
      A[5] = Class_B_idx_10;
      Class_B_idx_10 = A[7];
      A[7] = A[8];
      A[8] = Class_B_idx_10;
    }

    A[5] /= A[4];
    A[8] -= A[5] * A[7];
    A_dirtyOnCpu = true;
    Class_B_idx_9 = (A[1] * A[5] - A[2]) / A[8];
    Y_AB = -(A[1] + A[7] * Class_B_idx_9) / A[4];
    y[DEC_param] = ((1.0 - A[3] * Y_AB) - A[6] * Class_B_idx_9) / A[0];
    y[DEC_param + 1] = Y_AB;
    y[DEC_param + 2] = Class_B_idx_9;
    Class_B_idx_9 = -A[5] / A[8];
    Y_AB = (1.0 - A[7] * Class_B_idx_9) / A[4];
    y[jA] = -(A[3] * Y_AB + A[6] * Class_B_idx_9) / A[0];
    y[jA + 1] = Y_AB;
    y[jA + 2] = Class_B_idx_9;
    Class_B_idx_9 = 1.0 / A[8];
    Y_AB = -A[7] * Class_B_idx_9 / A[4];
    y[ix] = -(A[3] * Y_AB + A[6] * Class_B_idx_9) / A[0];
    cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
    cudaMemcpy(*gpu_y, y, 72UL, cudaMemcpyHostToDevice);
    BEV_image_kernel210<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_Y_AB, ix,
      *gpu_y);
    cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL, cudaMemcpyHostToDevice);
    BEV_image_kernel211<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (gpu_Class_B_idx_9, ix, *gpu_y);
    y_dirtyOnGpu = true;
    BEV_image_kernel212<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v,
      *gpu_y_out, *b_gpu_Y_k_hat);
    BEV_image_kernel213<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_y,
      *b_gpu_Y_k_hat, *gpu_dv1);
    d = 0.0;
    cudaMemcpy(gpu_d, &d, 8UL, cudaMemcpyHostToDevice);
    BEV_image_kernel214<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v,
      *gpu_y_out, *gpu_dv1, gpu_d);
    if (old_flag_dirtyOnGpu) {
      cudaMemcpy(&temp, gpu_temp, 8UL, cudaMemcpyDeviceToHost);
    }

    cudaMemcpy(&d, gpu_d, 8UL, cudaMemcpyDeviceToHost);
    X_AB = 1.0 / std::sqrt(248.05021344239853 * temp) * std::exp(d);

    //  P_kk = P_k_hat - K*S*K';
    // -------------------------------------------------------------------------
    if ((flag[0] == 0.0) && (old_flag[0] == 1.0)) {
      BEV_image_kernel221<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_work);
    } else if ((flag[0] == 1.0) && (old_flag[0] == 0.0)) {
      BEV_image_kernel220<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_ini,
        *gpu_work);
    } else if (flag[0] != 0.0) {
      BEV_image_kernel216<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_v,
        *gpu_y_out, *b_gpu_W);
      if (K_dirtyOnCpu) {
        cudaMemcpy(*gpu_K, K, 120UL, cudaMemcpyHostToDevice);
      }

      BEV_image_kernel217<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_x_k_hat,
        *b_gpu_W, *gpu_K, *gpu_work);
      BEV_image_kernel218<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_zz,
        *gpu_K, *gpu_P_xz);
      K_dirtyOnCpu = false;
      BEV_image_kernel219<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_L,
        sample_ts, *gpu_K, *gpu_P_xz, *gpu_P_cv_tmp);
    } else {
      BEV_image_kernel215<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_work);
    }

    // -------------------------------------------------------------------------
    //  if time >=63.230
    //  figure(1); hold on
    //  cla;
    //  plot(x_kk(1),x_kk(2),'xr');
    //  plot(x_k(1),x_k(2),'xk')
    //  plot([x_kk(1) x_k(1)],[x_kk(2) x_k(2)])
    //  plot(y_k(1),y_k(2),'or')
    //  axis([-6 6 -5 10])
    //  axis equal
    //  drawnow
    //      time = time;
    //  end
    //  if time >= 180
    //      time = time;
    //  end
    //
    //  if time >= 240
    //      time = time;
    //  end
    //  x_k = [y x yaw v yawrate]';
    // -------------------------------------------------------------------------
    //  Parameter
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    Y_AB = laneInfoL_idx_3 * laneInfoL_idx_2 / (laneInfoL_idx_3 *
      laneInfoL_idx_2 + laneInfoL_idx_4 * X_AB);
    X_AB = laneInfoL_idx_4 * X_AB / (laneInfoL_idx_3 * laneInfoL_idx_2 +
      laneInfoL_idx_4 * X_AB);
    cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
    cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
    Y_AB_dirtyOnCpu = false;
    BEV_image_kernel222<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_work,
      gpu_X_AB, *gpu_x_ctrv, gpu_Y_AB, sample_ts, *gpu_X_pred);

    // -------------------------------------------------------------------------
    //  if time >= 0
    //      time = time;
    //  end
    //          [Prob_ctrv,Prob_cv,X_c,P_c] = Mixing(c_ctrv,x_ctrv,P_ctrv,mu_ctrv,c_cv,x_cv,P_cv,mu_cv);
    if (sample_ts + 1 == 1) {
      out_Prob_ctrv = Y_AB;
      out_Prob_cv = X_AB;
    }
  }

  cudaMemcpy(*gpu_out_P_ctrv, out_P_ctrv, 200UL, cudaMemcpyHostToDevice);
  cudaMemcpy(*gpu_out_P_cv, out_P_cv, 200UL, cudaMemcpyHostToDevice);
  BEV_image_kernel223<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_P_cv_tmp,
    *gpu_P_ctrv_tmp, *gpu_out_P_cv, *gpu_out_P_ctrv);
  cudaMemcpy(out_P_cv, *gpu_out_P_cv, 200UL, cudaMemcpyDeviceToHost);
  cudaMemcpy(out_P_ctrv, *gpu_out_P_ctrv, 200UL, cudaMemcpyDeviceToHost);

  //  Threat Assessment
  //  Add Threat Metric to Training Data
  t = Chassis[6] + Training_data_data[7];
  temp = Chassis[7] + Training_data_data[8];

  //     %% TTC
  //  TTC Calculates time to collision in ROI.
  //
  //  TTC_out = TTC(rel_pos_x, rel_pos_y, rel_vel_x, ROI)
  //  rel_pos_x {double} : Relative longitudinal position (m)
  //  rel_pos_y {double} : Relative lateral position (m)
  //  rel_vel_x {double} : Relative longitudinal velocity (m/s)
  //  TTC_PARAM {struct} : Parameters for calculation of TTC
  //                        TTC_PARAM.ROI.Y_MIN : minimum relative lateral position of ROI
  //                        TTC_PARAM.ROI.Y_MAX : maximum relative lateral position of ROI
  //                        TTC_PARAM.ROI.X_MIN : minimum relative longitudinal position of ROI
  //                        TTC_PARAM.ROI.X_MAX : maximum relative longitudinal position of ROI
  //                        TTC_PARAM.TTC_MAX   : default value for exception
  if (std::abs(Training_data_data[6]) <= 2.0) {
    Class_B_idx_10 = -Training_data_data[5] / Training_data_data[7];
  } else {
    Class_B_idx_10 = 11.0;
  }

  if (Class_B_idx_10 < 0.0) {
    Class_B_idx_10 = 11.0;
  }

  Y_AB = 1.0 / Class_B_idx_10;
  if (Y_AB > 100.0) {
    Y_AB = 100.0;
  }

  Training_data_data[11] = Class_B_idx_10;
  Training_data_data[13] = Y_AB;

  //     %% TLC
  //  TLC Calculates Time to Line Crossing.
  //
  //  [TLC_out, DLC_out] = TLC(rel_pos_y, rel_vel_y, heading_angle, target_width,...
  //                            target_length, distance_to_leftlane, distance_to_rightlane, TLC_PARAM)
  //  rel_pos_y {double}             : Relative lateral position (m)
  //  rel_vel_y {double}             : Relative lateral velocity (m/s)
  //  heading_angle {double}         : Relative heading angle (rad)
  //  target_width {double}          : width of target vehicle (m)
  //  target_length {double}         : length of target vehicle (m)
  //  distance_to_leftlane {double}  : distance between ego vehicle center to left lane (m), positive number
  //  distance_to_rightlane {double} : distance between ego vehicle center to right lane (m), negative number
  //
  //  TLC_PARAM {struct}           : Parameters for calculation of time to line crossing
  //                                    TLC_PARAM.TLC_MAX : default value for exception
  //
  //  Reference : Mammar, Said, S?bastien Glaser, and Mariana Netto. "Time to line crossing for lane departure avoidance: A theoretical study and an experimental setting." IEEE Transactions on intelligent transportation systems 7.2 (2006): 226-241.
  // --------------------------------------------------------------------------
  //  Parameter
  // --------------------------------------------------------------------------
  //  parameter in algorithm
  //  parameter for initialization/exception
  // --------------------------------------------------------------------------
  //  initialization
  // --------------------------------------------------------------------------
  //  calculate DLC
  if (Training_data_data[6] >= Lane[7]) {
    //  outside of leftlane : calculate DLC( DLC > 0 )
    if (Training_data_data[10] < 0.0) {
      old_Prob_ctrv = -1.0;
    } else if (Training_data_data[10] > 0.0) {
      old_Prob_ctrv = 1.0;
    } else if (Training_data_data[10] == 0.0) {
      old_Prob_ctrv = 0.0;
    } else {
      old_Prob_ctrv = Training_data_data[10];
    }

    X_AB = Training_data_data[6] - ((Traffic[8] / 2.0 * std::sin(old_Prob_ctrv *
      Training_data_data[10]) + Traffic[9] / 2.0 * std::cos(Training_data_data
      [10])) + Lane[7]);
    if (X_AB < 0.0) {
      X_AB = 0.0;
    }
  } else if (Training_data_data[6] <= Lane[8]) {
    //  outside of rightlane : calculate DLC( DLC < 0 )
    if (Training_data_data[10] < 0.0) {
      old_Prob_ctrv = -1.0;
    } else if (Training_data_data[10] > 0.0) {
      old_Prob_ctrv = 1.0;
    } else if (Training_data_data[10] == 0.0) {
      old_Prob_ctrv = 0.0;
    } else {
      old_Prob_ctrv = Training_data_data[10];
    }

    X_AB = Training_data_data[6] + ((Traffic[8] / 2.0 * std::sin(old_Prob_ctrv *
      Training_data_data[10]) + Traffic[9] / 2.0 * std::cos(Training_data_data
      [10])) - Lane[8]);
    if (X_AB > 0.0) {
      X_AB = 0.0;
    }
  } else {
    //  in-lane : do not calculate DLC
    X_AB = 0.0;
  }

  //  calculate TLC
  if (Training_data_data[8] != 0.0) {
    if (std::abs(Training_data_data[8]) > 0.01) {
      if (X_AB != 0.0) {
        Class_B_idx_9 = -X_AB / Training_data_data[8];
      } else {
        //  exception : In case of DLC = 0, 1/TLC is -inf
        Class_B_idx_9 = 0.0;
      }
    } else {
      //  exception : In case of small value of rel vel y, TLC has too big value
      Class_B_idx_9 = 11.0;
    }

    //  prevent infinite TLC
  } else if (X_AB == 0.0) {
    Class_B_idx_9 = 0.0;
  } else {
    Class_B_idx_9 = 11.0;
  }

  if (Class_B_idx_9 < 0.0) {
    Class_B_idx_9 = 11.0;
  }

  if (std::abs(Training_data_data[6]) < 2.0) {
    Class_B_idx_9 = 0.0;
  }

  X_AB = 1.0 / Class_B_idx_9;
  if (X_AB > 100.0) {
    X_AB = 100.0;
  }

  Training_data_data[12] = Class_B_idx_9;
  Training_data_data[14] = X_AB;

  //     %% Ilat (lateral collision index)
  //  lateral: Ilat(combined and single),DLC and TLC
  //  longitudinal : Ilong,dw,dbr,xp and TTC
  //          I_LAT_PARAM.TTC_INVERSE_THRESHOLD=4;
  //          I_LAT_PARAM.A_X_MAX=-10;
  //          I_LAT_PARAM.A_X_MAX=-1;
  //  I_lat Calculates lateral collision index.
  //
  //  I_lat_out = I_lat(rel_pos_x, rel_pos_y, rel_vel_x, rel_vel_y, heading_angle,...
  //                    target_width, target_length, ego_length, distance_to_leftlane, distance_to_rightlane, I_LAT_PARAM)
  //  rel_pos_x {double}             : Relative longitudinal position (m)
  //  rel_pos_y {double}             : Relative lateral position (m)
  //  rel_vel_x {double}             : Relative longitudinal velocity (m/s)
  //  rel_vel_y {double}             : Relative lateral velocity (m/s)
  //  heading_angle {double}         : Relative heading angle (rad)
  //  target_width {double}          : width of target vehicle (m)
  //  target_length {double}         : length of target vehicle (m)
  //  ego_length {double}            : length of ego vehicle (m)
  //  distance_to_leftlane {double}  : distance between ego vehicle center to left lane (m)
  //  distance_to_rightlane {double} : distance between ego vehicle center to right lane (m)
  //
  //  I_LAT_PARAM {struct}           : Parameters for calculation of lateral collision index
  //                                    I_LAT_PARAM.TLC_THRESHOLD : minimum relative lateral position of ROI
  //                                    I_LAT_PARAM.T_THINKING : the delay in human response between recognition and manipulation (s)
  //                                    I_LAT_PARAM.T_BRAKE : the system delay (s)
  //                                    I_LAT_PARAM.A_X_MAX : maximum deceleration of vehicle(negative number) (m/s^2)
  //                                    I_LAT_PARAM.X_THRESHOLD : if warning index below threshold, it indicates current situation is dangerous
  //                                    I_LAT_PARAM.TTC_INVERSE_THRESHOLD : if TTC inverse beyond threshold threshold, it indecates current situation is dangerous
  //                                    I_LAT_PARAM.P_LONG_DEFAULT_VALUE : default value for initialization
  //                                    I_LAT_PARAM.TTC_MAX : default value for exception
  //                                    I_LAT_PARAM.TLC_MAX : default value for exception
  //
  //  Reference : K. Kim, “Predicted potential risk-based vehicle motion control of automated vehicles for integrated risk management,” Doctoral Dissertation, Seoul National University Graduate, 2016.
  // --------------------------------------------------------------------------
  //  Parameter
  // --------------------------------------------------------------------------
  //  parameter in algorithm
  //  X_THRESHOLD=10;
  //  parameter for initialization/exception
  // --------------------------------------------------------------------------
  //  initialization
  // --------------------------------------------------------------------------
  // --------------------------------------------------------------------------
  //  longitudinal collision index
  // --------------------------------------------------------------------------
  //  calculate the clearance between two vehicles(p_long)
  if (Training_data_data[5] >= 0.0) {
    X_AB = Training_data_data[5];
  } else {
    //  exception for adjacent and rear vehicle
    X_AB = (Training_data_data[5] + Chassis[10]) + Traffic[8];
  }

  //  calculate TTC(time to collision) : TTC positive = dangerous, TTC negative = safe
  Class_B_idx_10 = 0.0;
  if (X_AB != 300.0) {
    if (Training_data_data[7] < 0.0) {
      if (((Training_data_data[5] > 0.0) && (X_AB > 0.0)) ||
          ((Training_data_data[5] < 0.0) && (X_AB < 0.0))) {
        //  TTC for front vehicle || TTC for rear vehicle
        Class_B_idx_10 = X_AB / -Training_data_data[7];
      }
    } else {
      //  prevent infinite TTC
      Class_B_idx_10 = 11.0;
      if ((Training_data_data[5] < 0.0) && (X_AB < 0.0)) {
        //  exception : RT_LFL2R_IN rear collision
        Class_B_idx_10 = X_AB / -Training_data_data[7];
      }

      if ((Training_data_data[5] <= 0.0) && (X_AB > 0.0)) {
        //  exception : TTC for adjacent vehicle PSD-22-13(1020) LK_CIR_MER_data_2
        Class_B_idx_10 = 0.0;
      }
    }
  } else {
    Class_B_idx_10 = 11.0;
  }

  if (Class_B_idx_10 > 11.0) {
    Class_B_idx_10 = 11.0;
  }

  //  calculate warning braking critical distance(d_br)
  Y_AB = -Training_data_data[7] * 0.1 - (Chassis[6] * Chassis[6] - t * t) /
    -20.0;

  //  calculate warning critical distance(d_w)
  Class_B_idx_9 = (-Training_data_data[7] * 0.1 + -Training_data_data[7] * 0.1)
    - (Chassis[6] * Chassis[6] - t * t) / -20.0;
  if (t < 0.0) {
    Y_AB = -Training_data_data[7] * 0.1 + (t * t + Chassis[6] * Chassis[6]) /
      -20.0;
    Class_B_idx_9 = (-Training_data_data[7] * 0.1 + -Training_data_data[7] * 0.1)
      + (t * t + Chassis[6] * Chassis[6]) / -20.0;
  }

  //  calculate warning index(x_p)
  Y_AB = (X_AB - Y_AB) / (Class_B_idx_9 - Y_AB);
  if ((Y_AB > 4.5) || (Chassis[6] - t < 0.0)) {
    Y_AB = 4.5;
  }

  if (Y_AB < 0.0) {
    Y_AB = 0.0;
  }

  //  calculate collision risk index(I_long)
  Y_AB = std::fmax((4.5 - Y_AB) / 10.5, std::abs(1.0 / Class_B_idx_10) / 2.0);
  if (Y_AB > 1.0) {
    Y_AB = 1.0;
  }

  // --------------------------------------------------------------------------
  //  lateral collision index
  // --------------------------------------------------------------------------
  //  calculate DLC
  if (Training_data_data[6] >= Lane[7]) {
    //  outside of leftlane : calculate DLC, DLC > 0
    if (Training_data_data[10] < 0.0) {
      old_Prob_ctrv = -1.0;
    } else if (Training_data_data[10] > 0.0) {
      old_Prob_ctrv = 1.0;
    } else if (Training_data_data[10] == 0.0) {
      old_Prob_ctrv = 0.0;
    } else {
      old_Prob_ctrv = Training_data_data[10];
    }

    X_AB = (Training_data_data[6] - (Traffic[8] / 2.0 * std::sin(old_Prob_ctrv *
              Training_data_data[10]) + Traffic[9] / 2.0 * std::cos
             (Training_data_data[10]))) - Lane[7];
    if (X_AB < 0.0) {
      X_AB = 0.0;
    }
  } else if (Training_data_data[6] <= Lane[8]) {
    //  outside of rightlane : calculate DLC, DLC < 0
    if (Training_data_data[10] < 0.0) {
      old_Prob_ctrv = -1.0;
    } else if (Training_data_data[10] > 0.0) {
      old_Prob_ctrv = 1.0;
    } else if (Training_data_data[10] == 0.0) {
      old_Prob_ctrv = 0.0;
    } else {
      old_Prob_ctrv = Training_data_data[10];
    }

    X_AB = (Training_data_data[6] + (Traffic[8] / 2.0 * std::sin(old_Prob_ctrv *
              Training_data_data[10]) + Traffic[9] / 2.0 * std::cos
             (Training_data_data[10]))) - Lane[8];
    if (X_AB > 0.0) {
      X_AB = 0.0;
    }
  } else {
    //  in-lane : do not calculate DLC
    X_AB = 0.0;
  }

  //  calculate TLC
  if (Training_data_data[8] != 0.0) {
    if (std::abs(Training_data_data[8]) > 0.01) {
      if (X_AB != 0.0) {
        Class_B_idx_9 = -X_AB / Training_data_data[8];
        if (Class_B_idx_9 < 0.0) {
          Class_B_idx_9 = 11.0;
        }
      } else {
        //  exception : In case of DLC = 0, 1/TLC is -inf, then min(TLC_THRESHOLD/TLC,1) is -inf
        Class_B_idx_9 = 0.0;
      }
    } else {
      //  exception : In case of small value of rel vel y, TLC has too big value
      Class_B_idx_9 = 11.0;
      if (X_AB == 0.0) {
        Class_B_idx_9 = 0.0;
      }
    }
  } else if (X_AB == 0.0) {
    Class_B_idx_9 = 0.0;
  } else {
    //  prevent infinite TLC
    Class_B_idx_9 = 11.0;
  }

  //  out of FOV
  //  if abs(rel_pos_y)>2
  //      TLC=TLC_MAX;
  //  end
  //  if abs(DLC)>2||rel_vel_x > -0.2
  //      I_long=0;
  //  end
  //  calculate lateral collision index
  Training_data_data[15] = std::fmin(Y_AB, 1.0) * std::fmin(0.5 / Class_B_idx_9,
    1.0);
  Training_data_data[16] = Y_AB;

  //          I_LAT_PARAM.A_X_MAX=-10;
  //  I_lat Calculates lateral collision index.
  //
  //  I_lat_out = I_lat(rel_pos_x, rel_pos_y, rel_vel_x, rel_vel_y, heading_angle,...
  //                    target_width, target_length, ego_length, distance_to_leftlane, distance_to_rightlane, I_LAT_PARAM)
  //  rel_pos_x {double}             : Relative longitudinal position (m)
  //  rel_pos_y {double}             : Relative lateral position (m)
  //  rel_vel_x {double}             : Relative longitudinal velocity (m/s)
  //  rel_vel_y {double}             : Relative lateral velocity (m/s)
  //  heading_angle {double}         : Relative heading angle (rad)
  //  target_width {double}          : width of target vehicle (m)
  //  target_length {double}         : length of target vehicle (m)
  //  ego_length {double}            : length of ego vehicle (m)
  //  distance_to_leftlane {double}  : distance between ego vehicle center to left lane (m)
  //  distance_to_rightlane {double} : distance between ego vehicle center to right lane (m)
  //
  //  I_LAT_PARAM {struct}           : Parameters for calculation of lateral collision index
  //                                    I_LAT_PARAM.TLC_THRESHOLD : minimum relative lateral position of ROI
  //                                    I_LAT_PARAM.T_THINKING : the delay in human response between recognition and manipulation (s)
  //                                    I_LAT_PARAM.T_BRAKE : the system delay (s)
  //                                    I_LAT_PARAM.A_X_MAX : maximum deceleration of vehicle(negative number) (m/s^2)
  //                                    I_LAT_PARAM.X_THRESHOLD : if warning index below threshold, it indicates current situation is dangerous
  //                                    I_LAT_PARAM.TTC_INVERSE_THRESHOLD : if TTC inverse beyond threshold threshold, it indecates current situation is dangerous
  //                                    I_LAT_PARAM.P_LONG_DEFAULT_VALUE : default value for initialization
  //                                    I_LAT_PARAM.TTC_MAX : default value for exception
  //                                    I_LAT_PARAM.TLC_MAX : default value for exception
  //
  //  Reference : K. Kim, “Predicted potential risk-based vehicle motion control of automated vehicles for integrated risk management,” Doctoral Dissertation, Seoul National University Graduate, 2016.
  // --------------------------------------------------------------------------
  //  Parameter
  // --------------------------------------------------------------------------
  //  parameter in algorithm
  //  X_THRESHOLD=10;
  //  parameter for initialization/exception
  // --------------------------------------------------------------------------
  //  initialization
  // --------------------------------------------------------------------------
  // --------------------------------------------------------------------------
  //  longitudinal collision index
  // --------------------------------------------------------------------------
  //  calculate the clearance between two vehicles(p_long)
  if (Training_data_data[5] >= 0.0) {
    X_AB = Training_data_data[5];
  } else {
    //  exception for adjacent and rear vehicle
    X_AB = (Training_data_data[5] + Chassis[10]) + Traffic[8];
  }

  //  calculate TTC(time to collision) : TTC positive = dangerous, TTC negative = safe
  Class_B_idx_10 = 0.0;
  if (X_AB != 300.0) {
    if (Training_data_data[7] < 0.0) {
      if (((Training_data_data[5] > 0.0) && (X_AB > 0.0)) ||
          ((Training_data_data[5] < 0.0) && (X_AB < 0.0))) {
        //  TTC for front vehicle || TTC for rear vehicle
        Class_B_idx_10 = X_AB / -Training_data_data[7];
      }
    } else {
      //  prevent infinite TTC
      Class_B_idx_10 = 11.0;
      if ((Training_data_data[5] < 0.0) && (X_AB < 0.0)) {
        //  exception : RT_LFL2R_IN rear collision
        Class_B_idx_10 = X_AB / -Training_data_data[7];
      }

      if ((Training_data_data[5] <= 0.0) && (X_AB > 0.0)) {
        //  exception : TTC for adjacent vehicle PSD-22-13(1020) LK_CIR_MER_data_2
        Class_B_idx_10 = 0.0;
      }
    }
  } else {
    Class_B_idx_10 = 11.0;
  }

  if (Class_B_idx_10 > 11.0) {
    Class_B_idx_10 = 11.0;
  }

  //  calculate warning braking critical distance(d_br)
  Y_AB = -Training_data_data[7] * 0.1 - (Chassis[6] * Chassis[6] - t * t) /
    -20.0;

  //  calculate warning critical distance(d_w)
  Class_B_idx_9 = (-Training_data_data[7] * 0.1 + -Training_data_data[7] * 0.1)
    - (Chassis[6] * Chassis[6] - t * t) / -20.0;
  if (t < 0.0) {
    Y_AB = -Training_data_data[7] * 0.1 + (t * t + Chassis[6] * Chassis[6]) /
      -20.0;
    Class_B_idx_9 = (-Training_data_data[7] * 0.1 + -Training_data_data[7] * 0.1)
      + (t * t + Chassis[6] * Chassis[6]) / -20.0;
  }

  //  calculate warning index(x_p)
  Y_AB = (X_AB - Y_AB) / (Class_B_idx_9 - Y_AB);
  if ((Y_AB > 4.5) || (Chassis[6] - t < 0.0)) {
    Y_AB = 4.5;
  }

  if (Y_AB < 0.0) {
    Y_AB = 0.0;
  }

  //  calculate collision risk index(I_long)
  Y_AB = std::fmax((4.5 - Y_AB) / 10.5, std::abs(1.0 / Class_B_idx_10) / 2.0);
  if (Y_AB > 1.0) {
    Y_AB = 1.0;
  }

  // --------------------------------------------------------------------------
  //  lateral collision index
  // --------------------------------------------------------------------------
  //  calculate DLC
  if (Training_data_data[6] >= Lane[7]) {
    //  outside of leftlane : calculate DLC, DLC > 0
    if (Training_data_data[10] < 0.0) {
      old_Prob_ctrv = -1.0;
    } else if (Training_data_data[10] > 0.0) {
      old_Prob_ctrv = 1.0;
    } else if (Training_data_data[10] == 0.0) {
      old_Prob_ctrv = 0.0;
    } else {
      old_Prob_ctrv = Training_data_data[10];
    }

    X_AB = (Training_data_data[6] - (Traffic[8] / 2.0 * std::sin(old_Prob_ctrv *
              Training_data_data[10]) + Traffic[9] / 2.0 * std::cos
             (Training_data_data[10]))) - Lane[7];
    if (X_AB < 0.0) {
      X_AB = 0.0;
    }
  } else if (Training_data_data[6] <= Lane[8]) {
    //  outside of rightlane : calculate DLC, DLC < 0
    if (Training_data_data[10] < 0.0) {
      old_Prob_ctrv = -1.0;
    } else if (Training_data_data[10] > 0.0) {
      old_Prob_ctrv = 1.0;
    } else if (Training_data_data[10] == 0.0) {
      old_Prob_ctrv = 0.0;
    } else {
      old_Prob_ctrv = Training_data_data[10];
    }

    X_AB = (Training_data_data[6] + (Traffic[8] / 2.0 * std::sin(old_Prob_ctrv *
              Training_data_data[10]) + Traffic[9] / 2.0 * std::cos
             (Training_data_data[10]))) - Lane[8];
    if (X_AB > 0.0) {
      X_AB = 0.0;
    }
  } else {
    //  in-lane : do not calculate DLC
    X_AB = 0.0;
  }

  //  calculate TLC
  if (Training_data_data[8] != 0.0) {
    if (std::abs(Training_data_data[8]) > 0.01) {
      if (X_AB != 0.0) {
        Class_B_idx_9 = -X_AB / Training_data_data[8];
        if (Class_B_idx_9 < 0.0) {
          Class_B_idx_9 = 11.0;
        }
      } else {
        //  exception : In case of DLC = 0, 1/TLC is -inf, then min(TLC_THRESHOLD/TLC,1) is -inf
        Class_B_idx_9 = 0.0;
      }
    } else {
      //  exception : In case of small value of rel vel y, TLC has too big value
      Class_B_idx_9 = 11.0;
      if (X_AB == 0.0) {
        Class_B_idx_9 = 0.0;
      }
    }
  } else if (X_AB == 0.0) {
    Class_B_idx_9 = 0.0;
  } else {
    //  prevent infinite TLC
    Class_B_idx_9 = 11.0;
  }

  //  out of FOV
  //  if abs(rel_pos_y)>2
  //      TLC=TLC_MAX;
  //  end
  //  if abs(DLC)>2||rel_vel_x > -0.2
  //      I_long=0;
  //  end
  //  calculate lateral collision index
  Training_data_data[27] = std::fmin(Y_AB, 1.0) * std::fmin(0.5 / Class_B_idx_9,
    1.0);
  Training_data_data[26] = Y_AB;

  //     %% RSS (minimum safe distance x and y)
  //  Responsibility-Sensitive Safety(RSS) model
  //
  //  RSS_model - Calulate the safe longitudinal/lateral distance.
  //  [RSS_x, RSS_y] = RSS_model(Rel_x,Rel_y,vxA,vxB,vyA,vyB,RSS_Param)
  //
  //  RSS_model Input
  //
  //  Rel_x : Relative longitudinal position (m)
  //  Rel_y : Relative lateral position (m)
  //  vxA : Longitudinal velocity of ego vehicle (m/s)
  //  vxB : Longitudinal velocity of target vehicle (m/s)
  //  vyA : Lateral velocity of ego vehicle (m/s)
  //  vyB : Lateral velocity of target vehicle (m/s)
  //
  //  RSS_Param {struct} : Parameters of RSS_model
  //                       AMAX_XA  : Maximum long acc of ego veh.
  //                       AMIN_XA  : Minimum long decc of ego veh.
  //                       AMAX_XB  : Maximum long acc of target veh.
  //                       RHO_X    : Response time.
  //                       AMAX_Y   : Maximum lat acc of veh.
  //                       AMIN_Y   : Minimum lat decc of veh.
  //                       F        : Fluctutation margin.
  //                       RHO_Y    : Response time.
  //                       L_F      : Distance of the front tire from the c.g. of the veh.
  //                       W        : Width of ego veh.
  //
  //  RSS_model output
  //  RSS_x : Safe longitudinal distance between ego veh and target veh.
  //  RSS_y : Safe lateral distance between ego veh and target veh.
  //  Reference : Shai Shalev-Shwartz et al, “On a Formal Model of Safe and Scalable Self-driving Cars,”
  //  https://arxiv.org/abs/1708.06374, Mobileye, 2017.
  // %%%%%%%%%%%%%%%%%%%%% Parameter %%%%%%%%%%%%%%%%%%%%%%%%
  // %%%%%%%%%%%%%%%%%%%%% long model %%%%%%%%%%%%%%%%%%%%%%%%
  Y_AB = (Training_data_data[5] - t * t / 20.0) + (Chassis[6] * 0.1 + Chassis[6]
    * Chassis[6] / 20.0);
  if (t < -0.1) {
    Y_AB = (Training_data_data[5] + t * t / 20.0) + (Chassis[6] * 0.1 + Chassis
      [6] * Chassis[6] / 20.0);
  }

  if (Y_AB < -10.0) {
    Y_AB = -10.0;
  }

  if (Y_AB > 20.0) {
    Y_AB = 20.0;
  }

  // %%%%%%%%%%%%%%%%%%%%% lat model %%%%%%%%%%%%%%%%%%%%%%%%%
  //  if Rel_y>=0
  //      vyB=-vyB;
  //  end
  X_AB = Training_data_data[6] - 1.7;
  if (Training_data_data[6] < 0.0) {
    X_AB = -Training_data_data[6] - 1.7;
  }

  if (X_AB < 0.0) {
    X_AB = 0.0;
  }

  a = Chassis[7] + Chassis[7];
  Class_B_idx_9 = -2.0 * temp;
  X_AB = (X_AB - ((0.3 * ((Chassis[7] + Chassis[7]) / 2.0) + 0.1) + a * a / 8.0))
    + (0.3 * ((temp + temp) / 2.0) - Class_B_idx_9 * Class_B_idx_9 / 8.0);
  if (X_AB < -10.0) {
    X_AB = -10.0;
  }

  if (X_AB > 10.0) {
    X_AB = 10.0;
  }

  Training_data_data[17] = Y_AB;
  Training_data_data[18] = X_AB;

  //     %% Honda warning and avoidance algorithm (dw,dbr)
  //   HONDA calculates Honda's time to collision algorithm in ROI.
  //
  //  HONDA_out = HONDA(rel_pos_x, rel_pos_y, rel_vel_x, ROI)
  //  rel_pos_x {double} : Relative longitudinal position (m)
  //  rel_pos_y {double} : Relative lateral position (m)
  //  rel_vel_x {double} : Relative longitudinal velocity (m/s)
  //  HONDA_PARAM {struct} : Parameters for calculation of Honda algorithm
  //                        HONDA_PARAM.ROI.Y_MIN : minimum relative lateral position of ROI
  //                        HONDA_PARAM.ROI.Y_MAX : maximum relative lateral position of ROI
  //                        HONDA_PARAM.ROI.X_MIN : minimum relative longitudinal position of ROI
  //                        HONDA_PARAM.ROI.X_MAX : maximum relative longitudinal position of ROI
  //                        HONDA_PARAM.HONDA_MIN   : default value for exception
  if ((Training_data_data[6] >= -2.0) && (Training_data_data[6] <= 2.0) &&
      (Training_data_data[5] >= -1.0) && (Training_data_data[5] <= 120.0)) {
    X_AB = -2.2 * Training_data_data[7] + 6.2;
  } else {
    X_AB = 1.0;
  }

  if (Chassis[6] >= 11.67) {
    Y_AB = (0.2 * -Training_data_data[7] - 0.20000000000000004) -
      0.05000000000000001;
  } else {
    Y_AB = (0.2 * Chassis[6] - 0.05000000000000001) - t * t / 20.0;
  }

  if (t < -0.1) {
    Y_AB = (0.2 * Chassis[6] - 0.05000000000000001) + t * t / 20.0;
  }

  if (Y_AB < 0.0) {
    Y_AB = 0.0;
  }

  //  THM(HONDA)
  cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
  cudaMemcpy(*gpu_Training_data_data, Training_data_data, 224UL,
             cudaMemcpyHostToDevice);
  BEV_image_kernel224<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_X_AB,
    gpu_Y_AB, *gpu_Training_data_data);
  BEV_image_kernel225<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_X_AB,
    *gpu_Training_data_data);
  Y_AB_dirtyOnCpu = false;
  BEV_image_kernel226<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_Y_AB,
    *gpu_Training_data_data);
  BEV_image_kernel227<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Training_data_data);
  BEV_image_kernel228<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Training_data_data);
  BEV_image_kernel229<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Training_data_data);
  BEV_image_kernel230<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Training_data_data);

  //
  cudaMemcpy(*gpu_State, State, 20160UL, cudaMemcpyHostToDevice);
  BEV_image_kernel231<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Training_data_data, *gpu_State);
  cudaMemcpy(State, *gpu_State, 20160UL, cudaMemcpyDeviceToHost);

  //  90개
  //  Output
  temp = 6.0 * Lane[1];
  L_dirtyOnGpu = true;
  ts = 2.0 * Lane[3];
  laneInfoL_idx_2 = Lane[5];
  laneInfoL_idx_3 = Lane[7];
  laneInfoL_idx_4 = Lane[9];
  laneInfoR_idx_0 = 6.0 * Lane[2];
  laneInfoR_idx_1 = 2.0 * Lane[4];
  laneInfoR_idx_2 = Lane[6];
  laneInfoR_idx_3 = Lane[8];
  laneInfoR_idx_4 = Lane[10];
  range_X_min = 25.0;
  for (i = 0; i < 250; i++) {
    d = dv1[i + 1];
    if (range_X_min > d) {
      range_X_min = d;
    }
  }

  range_Y_min = 6.0;
  for (i = 0; i < 60; i++) {
    d = dv2[i + 1];
    if (range_Y_min > d) {
      range_Y_min = d;
    }
  }

  cudaMemcpy(*gpu_dv2, dv2, 488UL, cudaMemcpyHostToDevice);
  cudaMemcpy(*b_gpu_dv1, dv1, 2008UL, cudaMemcpyHostToDevice);
  BEV_image_kernel232<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>(*gpu_dv2,
    *b_gpu_dv1, *gpu_Ry, *gpu_Rx);
  for (b_L = 0; b_L < 90; b_L++) {
    d = State[b_L + 450];
    if (d != 0.0) {
      scale = State[b_L + 540];
      if ((scale != 0.0) && (d >= -25.0) && (d <= 25.0) && (scale >= -6.0) &&
          (scale <= 6.0)) {
        bool b_guard1{ false };

        bool guard1{ false };

        //                  Heading_Angle = Tmp_State(Data_Backward_index - image_index + 1, 5);
        for (DEC_param = 0; DEC_param < 90; DEC_param++) {
          X_AB = 25.0;
          for (i = 0; i < 250; i++) {
            d = dv1[i + 1];
            if (X_AB > d) {
              X_AB = d;
            }
          }

          d = State[DEC_param + 450];
          if ((d >= X_AB) && (d <= 25.0)) {
            X_AB = 6.0;
            for (i = 0; i < 60; i++) {
              scale = dv2[i + 1];
              if (X_AB > scale) {
                X_AB = scale;
              }
            }

            if ((State[DEC_param + 540] >= X_AB) && (State[DEC_param + 540] <=
                 6.0)) {
              scale = State[DEC_param + 1350];
              if (dv_dirtyOnCpu) {
                cudaMemcpy(*b_gpu_dv, dv, 2040UL, cudaMemcpyHostToDevice);
              }

              dv_dirtyOnCpu = false;
              cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
              BEV_image_kernel233<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>
                (*b_gpu_dv, gpu_scale, *gpu_x);
              nr = 0;
              cudaMemcpy(c_x, *gpu_x, 2040UL, cudaMemcpyDeviceToHost);
              X_AB = c_x[0];
              for (i = 0; i < 254; i++) {
                scale = c_x[i + 1];
                if (std::isnan(scale)) {
                  goto150 = false;
                } else if (std::isnan(X_AB)) {
                  goto150 = true;
                } else {
                  goto150 = (X_AB > scale);
                }

                if (goto150) {
                  X_AB = scale;
                  nr = i + 1;
                }
              }

              //              [~,Image_Position_X] = min(abs(Tmp_State(Data_Backward_index - image_index + 1,TRAINING.REL_POS_X) - RANGE.X_RANGE));
              X_AB = std::round(d * -5.0 + 126.0);

              //              [~,Image_Position_Y] = min(abs(Tmp_State(Data_Backward_index - image_index + 1,TRAINING.REL_POS_Y) - RANGE.Y_RANGE));
              Y_AB = std::round(State[DEC_param + 540] * -5.0 + 31.0);
              Y_AB_dirtyOnCpu = true;
              if (X_AB > 251.0) {
                X_AB = 251.0;
              }

              if (Y_AB < 1.0) {
                Y_AB = 1.0;
              } else if (Y_AB > 61.0) {
                Y_AB = 61.0;
              }

              //              [~,Velocity_uint8] = min(abs(Tmp_State(Data_Backward_index - image_index + 1,TRAINING.VELOCITY) - RANGE.V_RANGE));
              //              [~,Heading_uint8] = min(abs(Heading_Angle - RANGE.HEADING_RANGE));
              //  image에 입력할 때 1을 빼는 이유 : 위에서 min함수 이용해서 각 index를 찾으면 1~256으로 나오지만 image에는 0~255 범위로 입력해야하므로 1을 빼준다
              //                  if CHANNEL.Add_Line_to_BEV_Switch
              if (BEV_image_dirtyOnGpu) {
                cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL,
                           cudaMemcpyDeviceToHost);
              }

              b_BEV_image[(static_cast<int>(X_AB) + 251 * (static_cast<int>(Y_AB)
                - 1)) - 1] = static_cast<unsigned char>(nr);

              //  R
              b_BEV_image[(static_cast<int>(X_AB) + 251 * (static_cast<int>(Y_AB)
                - 1)) + 45932] = static_cast<unsigned char>(nr);

              //  R
              b_BEV_image[(static_cast<int>(X_AB) + 251 * (static_cast<int>(Y_AB)
                - 1)) + 91865] = static_cast<unsigned char>(nr);

              //  R
              b_BEV_image[(static_cast<int>(X_AB) + 251 * (static_cast<int>(Y_AB)
                - 1)) + 137798] = static_cast<unsigned char>(nr);
              BEV_image_dirtyOnGpu = false;
              BEV_image_dirtyOnCpu = true;

              //  R
              //                  else
              //                      BEV_Window_out(Image_Position_X,Image_Position_Y,1) = I_LAT_uint8-1; % R
              //  %                     BEV_Window_out(Image_Position_X,Image_Position_Y,2) = Velocity_uint8-1; % G
              //                      BEV_Window_out(Image_Position_X,Image_Position_Y,3) = Heading_uint8-1; % B
              //                  end
            }
          }
        }

        old_flag_dirtyOnGpu = true;
        lastc = 0;
        exitg1 = false;
        while ((!exitg1) && (lastc < 367464)) {
          if (BEV_image_dirtyOnGpu) {
            cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL,
                       cudaMemcpyDeviceToHost);
          }

          BEV_image_dirtyOnGpu = false;
          if (!(empty_black_image[lastc] == b_BEV_image[lastc])) {
            old_flag_dirtyOnGpu = false;
            exitg1 = true;
          } else {
            lastc++;
          }
        }

        goto150 = old_flag_dirtyOnGpu;
        if (!goto150) {
          //  && track_number == 1
          for (Y_AB = 0.0; Y_AB < laneInfoL_idx_4; Y_AB++) {
            X_AB = Y_AB * 3.5;
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            if (varargin_1_dirtyOnCpu) {
              cudaMemcpy(*gpu_varargin_1, varargin_1, 2008UL,
                         cudaMemcpyHostToDevice);
            }

            varargin_1_dirtyOnCpu = false;
            if (b_dirtyOnCpu) {
              cudaMemcpy(*gpu_b, b, 2008UL, cudaMemcpyHostToDevice);
            }

            b_dirtyOnCpu = false;
            if (b_b_dirtyOnCpu) {
              cudaMemcpy(*b_gpu_b, b_b, 2008UL, cudaMemcpyHostToDevice);
            }

            b_b_dirtyOnCpu = false;
            if (L_dirtyOnGpu) {
              cudaMemcpy(gpu_temp, &temp, 8UL, cudaMemcpyHostToDevice);
            }

            L_dirtyOnGpu = false;
            BEV_image_kernel299<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>
              (gpu_X_AB, laneInfoL_idx_3, *gpu_varargin_1, laneInfoL_idx_2,
               *gpu_b, ts, *b_gpu_b, gpu_temp, *gpu_tmp_lane_y);
            old_flag_dirtyOnGpu = true;
            for (jA = 0; jA < 251; jA++) {
              X_AB = dv1[jA];
              if (X_AB >= range_X_min) {
                if (old_flag_dirtyOnGpu) {
                  cudaMemcpy(tmp_lane_y, *gpu_tmp_lane_y, 2008UL,
                             cudaMemcpyDeviceToHost);
                }

                old_flag_dirtyOnGpu = false;
                if ((tmp_lane_y[jA] >= range_Y_min) && (tmp_lane_y[jA] <= 6.0))
                {
                  cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
                  BEV_image_kernel300<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>(*
                    gpu_varargin_1, gpu_X_AB, *b_gpu_y);
                  nr = 0;
                  cudaMemcpy(b_y, *b_gpu_y, 2008UL, cudaMemcpyDeviceToHost);
                  X_AB = b_y[0];
                  for (i = 0; i < 250; i++) {
                    d = b_y[i + 1];
                    if (X_AB > d) {
                      X_AB = d;
                      nr = i + 1;
                    }
                  }

                  X_AB = tmp_lane_y[jA];
                  if (b_varargin_1_dirtyOnCpu) {
                    cudaMemcpy(*b_gpu_varargin_1, b_varargin_1, 488UL,
                               cudaMemcpyHostToDevice);
                  }

                  b_varargin_1_dirtyOnCpu = false;
                  cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
                  BEV_image_kernel301<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
                    (*b_gpu_varargin_1, gpu_X_AB, *b_gpu_x);
                  DEC_param = 0;
                  cudaMemcpy(d_x, *b_gpu_x, 488UL, cudaMemcpyDeviceToHost);
                  X_AB = d_x[0];
                  for (i = 0; i < 60; i++) {
                    d = d_x[i + 1];
                    if (std::isnan(d)) {
                      goto150 = false;
                    } else if (std::isnan(X_AB)) {
                      goto150 = true;
                    } else {
                      goto150 = (X_AB > d);
                    }

                    if (goto150) {
                      X_AB = d;
                      DEC_param = i + 1;
                    }
                  }

                  if (BEV_image_dirtyOnGpu) {
                    cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL,
                               cudaMemcpyDeviceToHost);
                  }

                  b_BEV_image[(nr + 251 * DEC_param) + 15311] = MAX_uint8_T;
                  BEV_image_dirtyOnGpu = false;
                  BEV_image_dirtyOnCpu = true;
                }
              }
            }
          }

          //
          //          [in_tmp,~]=inpolygon(Rxv,Ryv,tmp_ego_x,tmp_ego_y);
          //          [Image_Position_Y,Image_Position_X]=find(reshape(in_tmp,[length(RANGE.Y_RANGE),length(RANGE.X_RANGE)])==1);
          Y_AB = 0.0;
          Y_AB_dirtyOnCpu = true;
          while (Y_AB < laneInfoR_idx_4) {
            X_AB = Y_AB * 3.5;
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            if (varargin_1_dirtyOnCpu) {
              cudaMemcpy(*gpu_varargin_1, varargin_1, 2008UL,
                         cudaMemcpyHostToDevice);
            }

            varargin_1_dirtyOnCpu = false;
            if (b_dirtyOnCpu) {
              cudaMemcpy(*gpu_b, b, 2008UL, cudaMemcpyHostToDevice);
            }

            b_dirtyOnCpu = false;
            if (b_b_dirtyOnCpu) {
              cudaMemcpy(*b_gpu_b, b_b, 2008UL, cudaMemcpyHostToDevice);
            }

            b_b_dirtyOnCpu = false;
            BEV_image_kernel296<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>
              (gpu_X_AB, laneInfoR_idx_3, *gpu_varargin_1, laneInfoR_idx_2,
               *gpu_b, laneInfoR_idx_1, *b_gpu_b, laneInfoR_idx_0,
               *gpu_tmp_lane_y);
            old_flag_dirtyOnGpu = true;
            for (jA = 0; jA < 251; jA++) {
              X_AB = dv1[jA];
              if (X_AB >= range_X_min) {
                if (old_flag_dirtyOnGpu) {
                  cudaMemcpy(tmp_lane_y, *gpu_tmp_lane_y, 2008UL,
                             cudaMemcpyDeviceToHost);
                }

                old_flag_dirtyOnGpu = false;
                if ((tmp_lane_y[jA] >= range_Y_min) && (tmp_lane_y[jA] <= 6.0))
                {
                  cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
                  BEV_image_kernel297<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>(*
                    gpu_varargin_1, gpu_X_AB, *b_gpu_y);
                  nr = 0;
                  cudaMemcpy(b_y, *b_gpu_y, 2008UL, cudaMemcpyDeviceToHost);
                  X_AB = b_y[0];
                  for (i = 0; i < 250; i++) {
                    d = b_y[i + 1];
                    if (X_AB > d) {
                      X_AB = d;
                      nr = i + 1;
                    }
                  }

                  X_AB = tmp_lane_y[jA];
                  if (b_varargin_1_dirtyOnCpu) {
                    cudaMemcpy(*b_gpu_varargin_1, b_varargin_1, 488UL,
                               cudaMemcpyHostToDevice);
                  }

                  b_varargin_1_dirtyOnCpu = false;
                  cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
                  BEV_image_kernel298<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
                    (*b_gpu_varargin_1, gpu_X_AB, *b_gpu_x);
                  DEC_param = 0;
                  cudaMemcpy(d_x, *b_gpu_x, 488UL, cudaMemcpyDeviceToHost);
                  X_AB = d_x[0];
                  for (i = 0; i < 60; i++) {
                    d = d_x[i + 1];
                    if (std::isnan(d)) {
                      goto150 = false;
                    } else if (std::isnan(X_AB)) {
                      goto150 = true;
                    } else {
                      goto150 = (X_AB > d);
                    }

                    if (goto150) {
                      X_AB = d;
                      DEC_param = i + 1;
                    }
                  }

                  if (BEV_image_dirtyOnGpu) {
                    cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL,
                               cudaMemcpyDeviceToHost);
                  }

                  b_BEV_image[(nr + 251 * DEC_param) + 15311] = MAX_uint8_T;
                  BEV_image_dirtyOnGpu = false;
                  BEV_image_dirtyOnCpu = true;
                }
              }
            }

            Y_AB++;
          }

          if (BEV_image_dirtyOnCpu) {
            cudaMemcpy(*gpu_BEV_image, b_BEV_image, 367464UL,
                       cudaMemcpyHostToDevice);
          }

          BEV_image_kernel234<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>
            (*gpu_BEV_image);
          BEV_image_kernel235<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>
            (*gpu_BEV_image);
          BEV_image_kernel236<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>
            (*gpu_BEV_image);
          BEV_image_kernel237<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>
            (*gpu_BEV_image);
          BEV_image_dirtyOnCpu = false;
          BEV_image_dirtyOnGpu = true;
        }

        old_flag_dirtyOnGpu = true;
        lastc = 0;
        exitg1 = false;
        while ((!exitg1) && (lastc < 367464)) {
          if (BEV_image_dirtyOnGpu) {
            cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL,
                       cudaMemcpyDeviceToHost);
          }

          BEV_image_dirtyOnGpu = false;
          if (!(empty_black_image[lastc] == b_BEV_image[lastc])) {
            old_flag_dirtyOnGpu = false;
            exitg1 = true;
          } else {
            lastc++;
          }
        }

        goto150 = old_flag_dirtyOnGpu;
        if (!goto150) {
          // && track_number == Traffic_Number
          if (Chassis_dirtyOnCpu) {
            cudaMemcpy(*gpu_Chassis, Chassis, 96UL, cudaMemcpyHostToDevice);
          }

          Chassis_dirtyOnCpu = false;
          BEV_image_kernel238<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*gpu_Chassis, *gpu_tmp_ego_y);
          tmp_ego_y_dirtyOnGpu = true;
          if (tmp_ego_x_dirtyOnGpu) {
            cudaMemcpy(tmp_ego_x, *gpu_tmp_ego_x, 40UL, cudaMemcpyDeviceToHost);
          }

          tmp_ego_x[0] = 0.0;
          tmp_ego_x[1] = -Chassis[10];
          tmp_ego_x[2] = -Chassis[10];
          tmp_ego_x[3] = 0.0;
          tmp_ego_x[4] = 0.0;
          tmp_ego_x_dirtyOnGpu = false;
          old_flag_dirtyOnGpu = true;
          BEV_image_kernel239<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>
            (*gpu_in_tmp);
          nr = 0;
          nr_dirtyOnCpu = true;
          BEV_image_kernel240<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_last,
            *gpu_first);
          Y_AB_dirtyOnGpu = true;
          lastc = 0;
          while ((lastc + 1 <= 5) && std::isnan(tmp_ego_x[lastc])) {
            lastc++;
          }

          while (lastc + 1 <= 5) {
            nr++;
            DEC_param = lastc;
            cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
            BEV_image_kernel293<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(lastc,
              gpu_nr, *gpu_first);
            exitg1 = false;
            while ((!exitg1) && (lastc + 1 < 5)) {
              lastc++;
              if (std::isnan(tmp_ego_x[lastc])) {
                lastc--;
                exitg1 = true;
              } else {
                if (tmp_ego_y_dirtyOnGpu) {
                  cudaMemcpy(tmp_ego_y, *gpu_tmp_ego_y, 40UL,
                             cudaMemcpyDeviceToHost);
                }

                tmp_ego_y_dirtyOnGpu = false;
                if (std::isnan(tmp_ego_y[lastc])) {
                  lastc--;
                  exitg1 = true;
                }
              }
            }

            guard1 = false;
            if (tmp_ego_x[lastc] == tmp_ego_x[DEC_param]) {
              if (tmp_ego_y_dirtyOnGpu) {
                cudaMemcpy(tmp_ego_y, *gpu_tmp_ego_y, 40UL,
                           cudaMemcpyDeviceToHost);
              }

              tmp_ego_y_dirtyOnGpu = false;
              if (tmp_ego_y[lastc] == tmp_ego_y[DEC_param]) {
                nr_dirtyOnCpu = false;
                BEV_image_kernel295<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                  (lastc, gpu_nr, *gpu_last);
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }

            if (guard1) {
              nr_dirtyOnCpu = false;
              BEV_image_kernel294<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(lastc,
                gpu_nr, *gpu_last);
            }

            lastc += 2;
            while ((lastc + 1 <= 5) && std::isnan(tmp_ego_x[lastc])) {
              lastc++;
            }
          }

          if (nr != 0) {
            cudaMemcpy(first, *gpu_first, 5UL, cudaMemcpyDeviceToHost);
            X_AB = tmp_ego_x[first[0] - 1];
            Y_AB = tmp_ego_x[first[0] - 1];
            for (lastc = 0; lastc < nr; lastc++) {
              DEC_param = first[lastc];
              if (Y_AB_dirtyOnGpu) {
                cudaMemcpy(last, *gpu_last, 20UL, cudaMemcpyDeviceToHost);
              }

              Y_AB_dirtyOnGpu = false;
              jA = last[lastc];
              for (j = 0; j <= jA - DEC_param; j++) {
                knt = (DEC_param + j) - 1;
                if (tmp_ego_x[knt] < X_AB) {
                  X_AB = tmp_ego_x[knt];
                } else if (tmp_ego_x[knt] > Y_AB) {
                  Y_AB = tmp_ego_x[knt];
                }
              }
            }

            if (tmp_ego_y_dirtyOnGpu) {
              cudaMemcpy(tmp_ego_y, *gpu_tmp_ego_y, 40UL, cudaMemcpyDeviceToHost);
            }

            Class_B_idx_9 = tmp_ego_y[first[0] - 1];
            tmp_ego_y_dirtyOnGpu = false;
            Class_B_idx_10 = tmp_ego_y[first[0] - 1];
            for (lastc = 0; lastc < nr; lastc++) {
              DEC_param = first[lastc];
              if (Y_AB_dirtyOnGpu) {
                cudaMemcpy(last, *gpu_last, 20UL, cudaMemcpyDeviceToHost);
              }

              Y_AB_dirtyOnGpu = false;
              jA = last[lastc];
              for (j = 0; j <= jA - DEC_param; j++) {
                knt = (DEC_param + j) - 1;
                if (tmp_ego_y[knt] < Class_B_idx_9) {
                  Class_B_idx_9 = tmp_ego_y[knt];
                } else if (tmp_ego_y[knt] > Class_B_idx_10) {
                  Class_B_idx_10 = tmp_ego_y[knt];
                }
              }
            }

            BEV_image_kernel241<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (*gpu_work);
            for (j = 0; j < nr; j++) {
              DEC_param = first[j];
              if (Y_AB_dirtyOnGpu) {
                cudaMemcpy(last, *gpu_last, 20UL, cudaMemcpyDeviceToHost);
              }

              jA = last[j] - 1;
              validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
                ((jA - DEC_param) + 1L), &grid, &block, 1024U, 65535U);
              if (validLaunchParams) {
                if (old_flag_dirtyOnGpu) {
                  cudaMemcpy(*gpu_tmp_ego_x, tmp_ego_x, 40UL,
                             cudaMemcpyHostToDevice);
                }

                old_flag_dirtyOnGpu = false;
                BEV_image_kernel242<<<grid, block>>>(*gpu_tmp_ego_y,
                  *gpu_tmp_ego_x, DEC_param, jA, *gpu_work);
              }

              a = std::abs(0.5 * (tmp_ego_x[last[j] - 1] + tmp_ego_x[first[j] -
                                  1]));
              Y_AB_dirtyOnGpu = false;
              old_Prob_cv = std::abs(0.5 * (tmp_ego_y[last[j] - 1] +
                tmp_ego_y[first[j] - 1]));
              if ((a > 1.0) && (old_Prob_cv > 1.0)) {
                a *= old_Prob_cv;
              } else if ((old_Prob_cv > a) || std::isnan(a)) {
                a = old_Prob_cv;
              }

              BEV_image_kernel243<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(a, j,
                *gpu_last, *gpu_work);
            }

            if (old_flag_dirtyOnGpu) {
              cudaMemcpy(*gpu_tmp_ego_x, tmp_ego_x, 40UL, cudaMemcpyHostToDevice);
            }

            if (nr_dirtyOnCpu) {
              cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
            }

            cudaMemcpy(gpu_Class_B_idx_10, &Class_B_idx_10, 8UL,
                       cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL,
                       cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
            Y_AB_dirtyOnCpu = false;
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            BEV_image_kernel244<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>
              (*gpu_work, *gpu_last, *gpu_tmp_ego_y, *gpu_tmp_ego_x, *gpu_first,
               gpu_nr, gpu_Class_B_idx_10, gpu_Class_B_idx_9, gpu_Y_AB, gpu_X_AB,
               *gpu_Ry, *gpu_Rx, *gpu_in_tmp);
          }

          BEV_image_kernel245<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>
            (*gpu_in_tmp, *c_gpu_x);
          old_flag_dirtyOnGpu = true;
          ix = 0;
          DEC_param = 1;
          jA = 1;
          exitg1 = false;
          while ((!exitg1) && (jA <= 251)) {
            if (old_flag_dirtyOnGpu) {
              cudaMemcpy(e_x, *c_gpu_x, 15311UL, cudaMemcpyDeviceToHost);
            }

            old_flag_dirtyOnGpu = false;
            b_guard1 = false;
            if (e_x[(DEC_param + 61 * (jA - 1)) - 1]) {
              ix++;
              i_data[ix - 1] = DEC_param;
              i_data_dirtyOnCpu = true;
              j_data[ix - 1] = static_cast<unsigned char>(jA);
              j_data_dirtyOnCpu = true;
              if (ix >= 15311) {
                exitg1 = true;
              } else {
                b_guard1 = true;
              }
            } else {
              b_guard1 = true;
            }

            if (b_guard1) {
              DEC_param++;
              if (DEC_param > 61) {
                DEC_param = 1;
                jA++;
              }
            }
          }

          if (ipiv_t_dirtyOnGpu) {
            cudaMemcpy(ipiv_t, *gpu_ipiv_t, 12UL, cudaMemcpyDeviceToHost);
          }

          if (1 > ix) {
            ipiv_t[0] = 0;
            ipiv_t[1] = 0;
          } else {
            ipiv_t[0] = ix;
            ipiv_t[1] = ix;
          }

          ipiv_t_dirtyOnGpu = false;
          validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
            (((ipiv_t[0] - 1) + 1L) * (((ipiv_t[1] - 1) + 1L) * 24L)), &grid,
            &block, 1024U, 65535U);
          if (validLaunchParams) {
            if (i_data_dirtyOnCpu) {
              cudaMemcpy(*gpu_i_data, i_data, 61244UL, cudaMemcpyHostToDevice);
            }

            i_data_dirtyOnCpu = false;
            if (j_data_dirtyOnCpu) {
              cudaMemcpy(*gpu_j_data, j_data, 15311UL, cudaMemcpyHostToDevice);
            }

            j_data_dirtyOnCpu = false;
            if (BEV_image_dirtyOnCpu) {
              cudaMemcpy(*gpu_BEV_image, b_BEV_image, 367464UL,
                         cudaMemcpyHostToDevice);
            }

            BEV_image_kernel246<<<grid, block>>>(*gpu_i_data, *gpu_j_data,
              ipiv_t[0] - 1, ipiv_t[1] - 1, *gpu_BEV_image);
            BEV_image_dirtyOnCpu = false;
            BEV_image_dirtyOnGpu = true;
          }

          //          for i = 1:length(Image_Position_X)
          //              BEV_Window_out(Image_Position_X(i), Image_Position_Y(i), :) = 255;
          //          end
        }

        old_flag_dirtyOnGpu = true;
        lastc = 0;
        exitg1 = false;
        while ((!exitg1) && (lastc < 367464)) {
          if (BEV_image_dirtyOnGpu) {
            cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL,
                       cudaMemcpyDeviceToHost);
          }

          BEV_image_dirtyOnGpu = false;
          if (!(empty_black_image[lastc] == b_BEV_image[lastc])) {
            old_flag_dirtyOnGpu = false;
            exitg1 = true;
          } else {
            lastc++;
          }
        }

        goto150 = old_flag_dirtyOnGpu;
        if (!goto150) {
          if (tmp_ego_y_dirtyOnGpu) {
            cudaMemcpy(tmp_ego_y, *gpu_tmp_ego_y, 40UL, cudaMemcpyDeviceToHost);
          }

          tmp_ego_y[0] = -State[270] / 2.0;
          tmp_ego_y[1] = -State[270] / 2.0;
          tmp_ego_y[2] = State[270] / 2.0;
          tmp_ego_y[3] = State[270] / 2.0;
          tmp_ego_y[4] = -State[270] / 2.0;
          tmp_ego_y_dirtyOnGpu = false;
          if (tmp_ego_x_dirtyOnGpu) {
            cudaMemcpy(tmp_ego_x, *gpu_tmp_ego_x, 40UL, cudaMemcpyDeviceToHost);
          }

          tmp_ego_x[0] = 0.0;
          tmp_ego_x[1] = State[360];
          tmp_ego_x[2] = State[360];
          cudaMemcpy(*gpu_tmp_ego_x, tmp_ego_x, 40UL, cudaMemcpyHostToDevice);
          BEV_image_kernel247<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*gpu_tmp_ego_x);
          x = std::sin(State[900]);
          b_x = std::cos(State[900]);
          X_AB = std::cos(State[900]);
          Y_AB = std::sin(State[900]);
          d = State[540];
          scale = State[450];
          cudaMemcpy(gpu_scale, &scale, 8UL, cudaMemcpyHostToDevice);
          cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
          Y_AB_dirtyOnCpu = false;
          cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
          cudaMemcpy(gpu_d, &d, 8UL, cudaMemcpyHostToDevice);
          cudaMemcpy(*gpu_tmp_ego_y, tmp_ego_y, 40UL, cudaMemcpyHostToDevice);
          BEV_image_kernel248<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(gpu_scale,
            gpu_Y_AB, gpu_X_AB, gpu_d, b_x, x, *gpu_tmp_ego_y, *gpu_target_y,
            *gpu_tmp_ego_x);
          old_flag_dirtyOnGpu = true;
          tmp_ego_x_dirtyOnGpu = true;
          BEV_image_kernel249<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>
            (*gpu_in_tmp);
          nr = 0;
          nr_dirtyOnCpu = true;
          BEV_image_kernel250<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*gpu_last,
            *gpu_first);
          Y_AB_dirtyOnGpu = true;
          lastc = 0;
          exitg1 = false;
          while ((!exitg1) && (lastc + 1 <= 5)) {
            if (tmp_ego_x_dirtyOnGpu) {
              cudaMemcpy(tmp_ego_x, *gpu_tmp_ego_x, 40UL, cudaMemcpyDeviceToHost);
            }

            tmp_ego_x_dirtyOnGpu = false;
            if (!std::isnan(tmp_ego_x[lastc])) {
              exitg1 = true;
            } else {
              lastc++;
            }
          }

          while (lastc + 1 <= 5) {
            nr++;
            DEC_param = lastc;
            cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
            BEV_image_kernel290<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(lastc,
              gpu_nr, *gpu_first);
            exitg1 = false;
            while ((!exitg1) && (lastc + 1 < 5)) {
              lastc++;
              if (tmp_ego_x_dirtyOnGpu) {
                cudaMemcpy(tmp_ego_x, *gpu_tmp_ego_x, 40UL,
                           cudaMemcpyDeviceToHost);
              }

              tmp_ego_x_dirtyOnGpu = false;
              if (std::isnan(tmp_ego_x[lastc])) {
                lastc--;
                exitg1 = true;
              } else {
                if (old_flag_dirtyOnGpu) {
                  cudaMemcpy(target_y, *gpu_target_y, 40UL,
                             cudaMemcpyDeviceToHost);
                }

                old_flag_dirtyOnGpu = false;
                if (std::isnan(target_y[lastc])) {
                  lastc--;
                  exitg1 = true;
                }
              }
            }

            if (tmp_ego_x_dirtyOnGpu) {
              cudaMemcpy(tmp_ego_x, *gpu_tmp_ego_x, 40UL, cudaMemcpyDeviceToHost);
            }

            tmp_ego_x_dirtyOnGpu = false;
            guard1 = false;
            if (tmp_ego_x[lastc] == tmp_ego_x[DEC_param]) {
              if (old_flag_dirtyOnGpu) {
                cudaMemcpy(target_y, *gpu_target_y, 40UL, cudaMemcpyDeviceToHost);
              }

              old_flag_dirtyOnGpu = false;
              if (target_y[lastc] == target_y[DEC_param]) {
                nr_dirtyOnCpu = false;
                BEV_image_kernel292<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                  (lastc, gpu_nr, *gpu_last);
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }

            if (guard1) {
              nr_dirtyOnCpu = false;
              BEV_image_kernel291<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(lastc,
                gpu_nr, *gpu_last);
            }

            lastc += 2;
            while ((lastc + 1 <= 5) && std::isnan(tmp_ego_x[lastc])) {
              lastc++;
            }
          }

          if (nr != 0) {
            if (tmp_ego_x_dirtyOnGpu) {
              cudaMemcpy(tmp_ego_x, *gpu_tmp_ego_x, 40UL, cudaMemcpyDeviceToHost);
            }

            cudaMemcpy(first, *gpu_first, 5UL, cudaMemcpyDeviceToHost);
            X_AB = tmp_ego_x[first[0] - 1];
            tmp_ego_x_dirtyOnGpu = false;
            Y_AB = tmp_ego_x[first[0] - 1];
            for (lastc = 0; lastc < nr; lastc++) {
              DEC_param = first[lastc];
              if (Y_AB_dirtyOnGpu) {
                cudaMemcpy(last, *gpu_last, 20UL, cudaMemcpyDeviceToHost);
              }

              Y_AB_dirtyOnGpu = false;
              jA = last[lastc];
              for (j = 0; j <= jA - DEC_param; j++) {
                knt = (DEC_param + j) - 1;
                if (tmp_ego_x[knt] < X_AB) {
                  X_AB = tmp_ego_x[knt];
                } else if (tmp_ego_x[knt] > Y_AB) {
                  Y_AB = tmp_ego_x[knt];
                }
              }
            }

            if (old_flag_dirtyOnGpu) {
              cudaMemcpy(target_y, *gpu_target_y, 40UL, cudaMemcpyDeviceToHost);
            }

            Class_B_idx_9 = target_y[first[0] - 1];
            Class_B_idx_10 = target_y[first[0] - 1];
            for (lastc = 0; lastc < nr; lastc++) {
              DEC_param = first[lastc];
              if (Y_AB_dirtyOnGpu) {
                cudaMemcpy(last, *gpu_last, 20UL, cudaMemcpyDeviceToHost);
              }

              Y_AB_dirtyOnGpu = false;
              jA = last[lastc];
              for (j = 0; j <= jA - DEC_param; j++) {
                knt = (DEC_param + j) - 1;
                if (target_y[knt] < Class_B_idx_9) {
                  Class_B_idx_9 = target_y[knt];
                } else if (target_y[knt] > Class_B_idx_10) {
                  Class_B_idx_10 = target_y[knt];
                }
              }
            }

            BEV_image_kernel251<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (*gpu_work);
            for (j = 0; j < nr; j++) {
              DEC_param = first[j];
              if (Y_AB_dirtyOnGpu) {
                cudaMemcpy(last, *gpu_last, 20UL, cudaMemcpyDeviceToHost);
              }

              jA = last[j] - 1;
              validLaunchParams = mwGetLaunchParameters1D(static_cast<double>
                ((jA - DEC_param) + 1L), &grid, &block, 1024U, 65535U);
              if (validLaunchParams) {
                BEV_image_kernel252<<<grid, block>>>(*gpu_target_y,
                  *gpu_tmp_ego_x, DEC_param, jA, *gpu_work);
              }

              a = std::abs(0.5 * (tmp_ego_x[last[j] - 1] + tmp_ego_x[first[j] -
                                  1]));
              Y_AB_dirtyOnGpu = false;
              old_Prob_cv = std::abs(0.5 * (target_y[last[j] - 1] +
                target_y[first[j] - 1]));
              if ((a > 1.0) && (old_Prob_cv > 1.0)) {
                a *= old_Prob_cv;
              } else if ((old_Prob_cv > a) || std::isnan(a)) {
                a = old_Prob_cv;
              }

              BEV_image_kernel253<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(a, j,
                *gpu_last, *gpu_work);
            }

            if (nr_dirtyOnCpu) {
              cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
            }

            cudaMemcpy(gpu_Class_B_idx_10, &Class_B_idx_10, 8UL,
                       cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Class_B_idx_9, &Class_B_idx_9, 8UL,
                       cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
            Y_AB_dirtyOnCpu = false;
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            BEV_image_kernel254<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>
              (*gpu_work, *gpu_last, *gpu_target_y, *gpu_tmp_ego_x, *gpu_first,
               gpu_nr, gpu_Class_B_idx_10, gpu_Class_B_idx_9, gpu_Y_AB, gpu_X_AB,
               *gpu_Ry, *gpu_Rx, *gpu_in_tmp);
          }

          BEV_image_kernel255<<<dim3(30U, 1U, 1U), dim3(512U, 1U, 1U)>>>
            (*gpu_in_tmp, *c_gpu_x);
          old_flag_dirtyOnGpu = true;
          ix = 0;
          DEC_param = 1;
          jA = 1;
          exitg1 = false;
          while ((!exitg1) && (jA <= 251)) {
            if (old_flag_dirtyOnGpu) {
              cudaMemcpy(e_x, *c_gpu_x, 15311UL, cudaMemcpyDeviceToHost);
            }

            old_flag_dirtyOnGpu = false;
            b_guard1 = false;
            if (e_x[(DEC_param + 61 * (jA - 1)) - 1]) {
              ix++;
              i_data[ix - 1] = DEC_param;
              i_data_dirtyOnCpu = true;
              j_data[ix - 1] = static_cast<unsigned char>(jA);
              j_data_dirtyOnCpu = true;
              if (ix >= 15311) {
                exitg1 = true;
              } else {
                b_guard1 = true;
              }
            } else {
              b_guard1 = true;
            }

            if (b_guard1) {
              DEC_param++;
              if (DEC_param > 61) {
                DEC_param = 1;
                jA++;
              }
            }
          }

          if (1 > ix) {
            DEC_param = 0;
          } else {
            DEC_param = ix;
          }

          d = State[1350];
          if (dv_dirtyOnCpu) {
            cudaMemcpy(*b_gpu_dv, dv, 2040UL, cudaMemcpyHostToDevice);
          }

          dv_dirtyOnCpu = false;
          cudaMemcpy(gpu_d, &d, 8UL, cudaMemcpyHostToDevice);
          BEV_image_kernel256<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>
            (*b_gpu_dv, gpu_d, *gpu_x);
          nr = 0;
          cudaMemcpy(c_x, *gpu_x, 2040UL, cudaMemcpyDeviceToHost);
          X_AB = c_x[0];
          for (i = 0; i < 254; i++) {
            d = c_x[i + 1];
            if (std::isnan(d)) {
              goto150 = false;
            } else if (std::isnan(X_AB)) {
              goto150 = true;
            } else {
              goto150 = (X_AB > d);
            }

            if (goto150) {
              X_AB = d;
              nr = i + 1;
            }
          }

          guard1 = false;
          if (DEC_param == 0) {
            if (1 > ix) {
              ix = 0;
            }

            if (ix != 0) {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }

          if (guard1) {
            for (i = 0; i < DEC_param; i++) {
              //                      if CHANNEL.Add_Line_to_BEV_Switch
              //                  BEV_Window_out(Image_Position_X(i),Image_Position_Y(i),1) = I_LONG_uint8-1; % R
              //                  BEV_Window_out(Image_Position_X(i),Image_Position_Y(i),4) = I_LONG_less_uint8-1; % R
              if (BEV_image_dirtyOnGpu) {
                cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL,
                           cudaMemcpyDeviceToHost);
              }

              b_BEV_image[(j_data[i] + 251 * (static_cast<int>(static_cast<float>
                (i_data[i])) - 1)) - 1] = static_cast<unsigned char>(nr);

              //  R
              b_BEV_image[(j_data[i] + 251 * (static_cast<int>(static_cast<float>
                (i_data[i])) - 1)) + 45932] = static_cast<unsigned char>(nr);

              //  R
              b_BEV_image[(j_data[i] + 251 * (static_cast<int>(static_cast<float>
                (i_data[i])) - 1)) + 91865] = static_cast<unsigned char>(nr);

              //  R
              b_BEV_image[(j_data[i] + 251 * (static_cast<int>(static_cast<float>
                (i_data[i])) - 1)) + 137798] = static_cast<unsigned char>(nr);

              //  R
              b_BEV_image[(j_data[i] + 251 * (static_cast<int>(static_cast<float>
                (i_data[i])) - 1)) + 183731] = static_cast<unsigned char>(nr);

              //  R
              b_BEV_image[(j_data[i] + 251 * (static_cast<int>(static_cast<float>
                (i_data[i])) - 1)) + 229664] = static_cast<unsigned char>(nr);

              //  R
              b_BEV_image[(j_data[i] + 251 * (static_cast<int>(static_cast<float>
                (i_data[i])) - 1)) + 275597] = static_cast<unsigned char>(nr);

              //  R
              b_BEV_image[(j_data[i] + 251 * (static_cast<int>(static_cast<float>
                (i_data[i])) - 1)) + 321530] = static_cast<unsigned char>(nr);
              BEV_image_dirtyOnGpu = false;
              BEV_image_dirtyOnCpu = true;

              //  R
            }
          }
        }

        //          BEV_Window_out(:,:,1) = BEV_Window_out(:,:,1);
        //          BEV_Window_out(:,:,4) = BEV_Window_out(:,:,1);
        //          BEV_Window_out(:,:,7) = BEV_Window_out(:,:,1);
        //          BEV_Window_out(:,:,10) = BEV_Window_out(:,:,1);
        //          BEV_Window_out(:,:,13) = BEV_Window_out(:,:,1);
        //          BEV_Window_out(:,:,16) = BEV_Window_out(:,:,1);
        //          BEV_Window_out(:,:,19) = BEV_Window_out(:,:,1);
        //          BEV_Window_out(:,:,22) = BEV_Window_out(:,:,1);
        d = State[1350];
        if (dv_dirtyOnCpu) {
          cudaMemcpy(*b_gpu_dv, dv, 2040UL, cudaMemcpyHostToDevice);
        }

        dv_dirtyOnCpu = false;
        cudaMemcpy(gpu_d, &d, 8UL, cudaMemcpyHostToDevice);
        BEV_image_kernel257<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>(*b_gpu_dv,
          gpu_d, *gpu_x);
        nr = 0;
        cudaMemcpy(c_x, *gpu_x, 2040UL, cudaMemcpyDeviceToHost);
        X_AB = c_x[0];
        for (i = 0; i < 254; i++) {
          d = c_x[i + 1];
          if (std::isnan(d)) {
            goto150 = false;
          } else if (std::isnan(X_AB)) {
            goto150 = true;
          } else {
            goto150 = (X_AB > d);
          }

          if (goto150) {
            X_AB = d;
            nr = i + 1;
          }
        }

        old_flag_dirtyOnGpu = true;
        lastc = 0;
        exitg1 = false;
        while ((!exitg1) && (lastc < 367464)) {
          if (BEV_image_dirtyOnGpu) {
            cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL,
                       cudaMemcpyDeviceToHost);
          }

          BEV_image_dirtyOnGpu = false;
          if (!(empty_black_image[lastc] == b_BEV_image[lastc])) {
            old_flag_dirtyOnGpu = false;
            exitg1 = true;
          } else {
            lastc++;
          }
        }

        goto150 = old_flag_dirtyOnGpu;
        if (!goto150) {
          a = -State[270] / 2.0;
          d2 = State[270] / 2.0;
          BEV_image_kernel258<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(a, d2,
            *gpu_TV_range_y);
          if (a == -d2) {
            X_AB = d2 / 9.0;
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            BEV_image_kernel261<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (gpu_X_AB, *gpu_TV_range_y);
          } else if (((a < 0.0) != (d2 < 0.0)) && ((std::abs(a) >
                       8.9884656743115785E+307) || (std::abs(d2) >
                       8.9884656743115785E+307))) {
            X_AB = a / 9.0;
            Y_AB = d2 / 9.0;
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
            Y_AB_dirtyOnCpu = false;
            BEV_image_kernel260<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (gpu_X_AB, gpu_Y_AB, a, *gpu_TV_range_y);
          } else {
            X_AB = (d2 - a) / 9.0;
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            BEV_image_kernel259<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (gpu_X_AB, a, *gpu_TV_range_y);
          }

          if (TV_range_x_dirtyOnGpu) {
            cudaMemcpy(TV_range_x, *gpu_TV_range_x, 80UL, cudaMemcpyDeviceToHost);
          }

          TV_range_x[9] = State[360];
          cudaMemcpy(*gpu_TV_range_x, TV_range_x, 80UL, cudaMemcpyHostToDevice);
          BEV_image_kernel262<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*gpu_TV_range_x);
          if (0.0 == -State[360]) {
            X_AB = State[360] / 9.0;
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            BEV_image_kernel265<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (gpu_X_AB, *gpu_TV_range_x);
            TV_range_x_dirtyOnGpu = true;
          } else if ((State[360] < 0.0) && (std::abs(State[360]) >
                      8.9884656743115785E+307)) {
            Y_AB = State[360] / 9.0;
            cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
            Y_AB_dirtyOnCpu = false;
            BEV_image_kernel264<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (gpu_Y_AB, *gpu_TV_range_x);
            TV_range_x_dirtyOnGpu = true;
          } else {
            X_AB = State[360] / 9.0;
            cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
            BEV_image_kernel263<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (gpu_X_AB, *gpu_TV_range_x);
            TV_range_x_dirtyOnGpu = true;
          }

          if (yp4_dirtyOnCpu) {
            cudaMemcpy(*gpu_yp4, yp4, 800UL, cudaMemcpyHostToDevice);
          }

          if (xp4_dirtyOnCpu) {
            cudaMemcpy(*gpu_xp4, xp4, 800UL, cudaMemcpyHostToDevice);
          }

          if (yp3_dirtyOnCpu) {
            cudaMemcpy(*gpu_yp3, yp3, 800UL, cudaMemcpyHostToDevice);
          }

          if (xp3_dirtyOnCpu) {
            cudaMemcpy(*gpu_xp3, xp3, 800UL, cudaMemcpyHostToDevice);
          }

          if (yp2_dirtyOnCpu) {
            cudaMemcpy(*gpu_yp2, yp2, 800UL, cudaMemcpyHostToDevice);
          }

          if (xp2_dirtyOnCpu) {
            cudaMemcpy(*gpu_xp2, xp2, 800UL, cudaMemcpyHostToDevice);
          }

          if (yp1_dirtyOnCpu) {
            cudaMemcpy(*gpu_yp1, yp1, 800UL, cudaMemcpyHostToDevice);
          }

          if (xp1_dirtyOnCpu) {
            cudaMemcpy(*gpu_xp1, xp1, 800UL, cudaMemcpyHostToDevice);
          }

          BEV_image_kernel266<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
            (*gpu_TV_range_x, *gpu_TV_range_y, *gpu_X_pred, *gpu_yp4, *gpu_xp4, *
             gpu_yp3, *gpu_xp3, *gpu_yp2, *gpu_xp2, *gpu_yp1, *gpu_xp1);
          BEV_image_kernel267<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>(*gpu_yp4,
            *gpu_xp4, *gpu_yp3, *gpu_xp3, *gpu_yp2, *gpu_xp2, *gpu_yp1, *gpu_xp1);
          xp1_dirtyOnCpu = false;
          v_dirtyOnGpu = true;
          yp1_dirtyOnCpu = false;
          Class_B_idx_9_dirtyOnCpu = true;
          xp2_dirtyOnCpu = false;
          Class_B_idx_10_dirtyOnGpu = true;
          yp2_dirtyOnCpu = false;
          old_flag_dirtyOnGpu = true;
          xp3_dirtyOnCpu = false;
          Y_AB_dirtyOnGpu = true;
          yp3_dirtyOnCpu = false;
          Vr_dirtyOnGpu = true;
          xp4_dirtyOnCpu = false;
          nr_dirtyOnCpu = true;
          yp4_dirtyOnCpu = false;
          validLaunchParams = true;
          for (j = 0; j < 100; j++) {
            if (v_dirtyOnGpu) {
              cudaMemcpy(xp1, *gpu_xp1, 800UL, cudaMemcpyDeviceToHost);
            }

            v_dirtyOnGpu = false;
            d = xp1[j];
            if (d < 1.0) {
              d = 1.0;
              xp1[j] = 1.0;
              xp1_dirtyOnCpu = true;
            } else if (d > 251.0) {
              d = 251.0;
              xp1[j] = 251.0;
              xp1_dirtyOnCpu = true;
            }

            if (Class_B_idx_9_dirtyOnCpu) {
              cudaMemcpy(yp1, *gpu_yp1, 800UL, cudaMemcpyDeviceToHost);
            }

            Class_B_idx_9_dirtyOnCpu = false;
            scale = yp1[j];
            if (scale < 1.0) {
              scale = 1.0;
              yp1[j] = 1.0;
              yp1_dirtyOnCpu = true;
            } else if (scale > 61.0) {
              scale = 61.0;
              yp1[j] = 61.0;
              yp1_dirtyOnCpu = true;
            }

            if (Class_B_idx_10_dirtyOnGpu) {
              cudaMemcpy(xp2, *gpu_xp2, 800UL, cudaMemcpyDeviceToHost);
            }

            Class_B_idx_10_dirtyOnGpu = false;
            old_Prob_ctrv = xp2[j];
            if (old_Prob_ctrv < 1.0) {
              old_Prob_ctrv = 1.0;
              xp2[j] = 1.0;
              xp2_dirtyOnCpu = true;
            } else if (old_Prob_ctrv > 251.0) {
              old_Prob_ctrv = 251.0;
              xp2[j] = 251.0;
              xp2_dirtyOnCpu = true;
            }

            if (old_flag_dirtyOnGpu) {
              cudaMemcpy(yp2, *gpu_yp2, 800UL, cudaMemcpyDeviceToHost);
            }

            old_flag_dirtyOnGpu = false;
            X_AB = yp2[j];
            if (X_AB < 1.0) {
              X_AB = 1.0;
              yp2[j] = 1.0;
              yp2_dirtyOnCpu = true;
            } else if (X_AB > 61.0) {
              X_AB = 61.0;
              yp2[j] = 61.0;
              yp2_dirtyOnCpu = true;
            }

            if (Y_AB_dirtyOnGpu) {
              cudaMemcpy(xp3, *gpu_xp3, 800UL, cudaMemcpyDeviceToHost);
            }

            Y_AB_dirtyOnGpu = false;
            Y_AB = xp3[j];
            Y_AB_dirtyOnCpu = true;
            if (Y_AB < 1.0) {
              Y_AB = 1.0;
              xp3[j] = 1.0;
              xp3_dirtyOnCpu = true;
            } else if (Y_AB > 251.0) {
              Y_AB = 251.0;
              xp3[j] = 251.0;
              xp3_dirtyOnCpu = true;
            }

            if (Vr_dirtyOnGpu) {
              cudaMemcpy(yp3, *gpu_yp3, 800UL, cudaMemcpyDeviceToHost);
            }

            Vr_dirtyOnGpu = false;
            Class_B_idx_9 = yp3[j];
            if (Class_B_idx_9 < 1.0) {
              Class_B_idx_9 = 1.0;
              yp3[j] = 1.0;
              yp3_dirtyOnCpu = true;
            } else if (Class_B_idx_9 > 61.0) {
              Class_B_idx_9 = 61.0;
              yp3[j] = 61.0;
              yp3_dirtyOnCpu = true;
            }

            if (nr_dirtyOnCpu) {
              cudaMemcpy(xp4, *gpu_xp4, 800UL, cudaMemcpyDeviceToHost);
            }

            nr_dirtyOnCpu = false;
            Class_B_idx_10 = xp4[j];
            if (Class_B_idx_10 < 1.0) {
              Class_B_idx_10 = 1.0;
              xp4[j] = 1.0;
              xp4_dirtyOnCpu = true;
            } else if (Class_B_idx_10 > 251.0) {
              Class_B_idx_10 = 251.0;
              xp4[j] = 251.0;
              xp4_dirtyOnCpu = true;
            }

            if (validLaunchParams) {
              cudaMemcpy(yp4, *gpu_yp4, 800UL, cudaMemcpyDeviceToHost);
            }

            validLaunchParams = false;
            t = yp4[j];
            if (t < 1.0) {
              t = 1.0;
              yp4[j] = 1.0;
              yp4_dirtyOnCpu = true;
            } else if (t > 61.0) {
              t = 61.0;
              yp4[j] = 61.0;
              yp4_dirtyOnCpu = true;
            }

            if (BEV_image_dirtyOnGpu) {
              cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL,
                         cudaMemcpyDeviceToHost);
            }

            b_BEV_image[(static_cast<int>(d) + 251 * (static_cast<int>(scale) -
              1)) + 45932] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(old_Prob_ctrv) + 251 * (static_cast<
              int>(X_AB) - 1)) + 45932] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Y_AB) + 251 * (static_cast<int>
              (Class_B_idx_9) - 1)) + 45932] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Class_B_idx_10) + 251 * (static_cast<
              int>(t) - 1)) + 45932] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(d) + 251 * (static_cast<int>(scale) -
              1)) + 91865] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(old_Prob_ctrv) + 251 * (static_cast<
              int>(X_AB) - 1)) + 91865] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Y_AB) + 251 * (static_cast<int>
              (Class_B_idx_9) - 1)) + 91865] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Class_B_idx_10) + 251 * (static_cast<
              int>(t) - 1)) + 91865] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(d) + 251 * (static_cast<int>(scale) -
              1)) + 137798] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(old_Prob_ctrv) + 251 * (static_cast<
              int>(X_AB) - 1)) + 137798] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Y_AB) + 251 * (static_cast<int>
              (Class_B_idx_9) - 1)) + 137798] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Class_B_idx_10) + 251 * (static_cast<
              int>(t) - 1)) + 137798] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(d) + 251 * (static_cast<int>(scale) -
              1)) + 183731] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(old_Prob_ctrv) + 251 * (static_cast<
              int>(X_AB) - 1)) + 183731] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Y_AB) + 251 * (static_cast<int>
              (Class_B_idx_9) - 1)) + 183731] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Class_B_idx_10) + 251 * (static_cast<
              int>(t) - 1)) + 183731] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(d) + 251 * (static_cast<int>(scale) -
              1)) + 229664] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(old_Prob_ctrv) + 251 * (static_cast<
              int>(X_AB) - 1)) + 229664] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Y_AB) + 251 * (static_cast<int>
              (Class_B_idx_9) - 1)) + 229664] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Class_B_idx_10) + 251 * (static_cast<
              int>(t) - 1)) + 229664] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(d) + 251 * (static_cast<int>(scale) -
              1)) + 275597] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(old_Prob_ctrv) + 251 * (static_cast<
              int>(X_AB) - 1)) + 275597] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Y_AB) + 251 * (static_cast<int>
              (Class_B_idx_9) - 1)) + 275597] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Class_B_idx_10) + 251 * (static_cast<
              int>(t) - 1)) + 275597] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(d) + 251 * (static_cast<int>(scale) -
              1)) + 321530] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(old_Prob_ctrv) + 251 * (static_cast<
              int>(X_AB) - 1)) + 321530] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Y_AB) + 251 * (static_cast<int>
              (Class_B_idx_9) - 1)) + 321530] = static_cast<unsigned char>(nr);

            //  R
            b_BEV_image[(static_cast<int>(Class_B_idx_10) + 251 * (static_cast<
              int>(t) - 1)) + 321530] = static_cast<unsigned char>(nr);
            BEV_image_dirtyOnGpu = false;
            BEV_image_dirtyOnCpu = true;

            //  R
          }

          //          BEV_Window_out(:,:,4) = BEV_Window_out(:,:,4);
          //          BEV_Window_out(:,:,7) = BEV_Window_out(:,:,4);
          //          BEV_Window_out(:,:,10) = BEV_Window_out(:,:,4);
          //          BEV_Window_out(:,:,13) = BEV_Window_out(:,:,4);
          //          BEV_Window_out(:,:,16) = BEV_Window_out(:,:,4);
          //          BEV_Window_out(:,:,19) = BEV_Window_out(:,:,4);
          //          BEV_Window_out(:,:,22) = BEV_Window_out(:,:,4);
        }

        old_flag_dirtyOnGpu = true;
        lastc = 0;
        exitg1 = false;
        while ((!exitg1) && (lastc < 367464)) {
          if (BEV_image_dirtyOnGpu) {
            cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL,
                       cudaMemcpyDeviceToHost);
          }

          BEV_image_dirtyOnGpu = false;
          if (!(empty_black_image[lastc] == b_BEV_image[lastc])) {
            old_flag_dirtyOnGpu = false;
            exitg1 = true;
          } else {
            lastc++;
          }
        }

        goto150 = old_flag_dirtyOnGpu;
        if (!goto150) {
          // && track_number == Traffic_Number
          a = -Chassis[11] / 2.0;
          d2 = Chassis[11] / 2.0;
          old_Prob_cv = -Chassis[10];
          for (lastc = 0; lastc < 7; lastc++) {
            //  time,10 sample (prediction sample),[ACC DEC ESL ESR ELCL ELCR ESS]
            BEV_image_kernel268<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(a, d2, *
              gpu_TV_range_y);
            if (a == -d2) {
              X_AB = d2 / 9.0;
              cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
              old_flag_dirtyOnGpu = false;
              BEV_image_kernel271<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (gpu_X_AB, *gpu_TV_range_y);
            } else if (((a < 0.0) != (d2 < 0.0)) && ((std::abs(a) >
                         8.9884656743115785E+307) || (std::abs(d2) >
                         8.9884656743115785E+307))) {
              X_AB = a / 9.0;
              Y_AB = d2 / 9.0;
              cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
              old_flag_dirtyOnGpu = false;
              cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
              Y_AB_dirtyOnCpu = false;
              BEV_image_kernel270<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (gpu_X_AB, gpu_Y_AB, a, *gpu_TV_range_y);
            } else {
              X_AB = (d2 - a) / 9.0;
              cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
              old_flag_dirtyOnGpu = false;
              BEV_image_kernel269<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (gpu_X_AB, a, *gpu_TV_range_y);
            }

            if (Chassis_dirtyOnCpu) {
              cudaMemcpy(*gpu_Chassis, Chassis, 96UL, cudaMemcpyHostToDevice);
            }

            Chassis_dirtyOnCpu = false;
            BEV_image_kernel272<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
              (*gpu_Chassis, *gpu_TV_range_x);
            if (-Chassis[10] == -0.0) {
              BEV_image_kernel277<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (*gpu_TV_range_x);
              TV_range_x_dirtyOnGpu = true;
            } else if ((-Chassis[10] < 0.0) && (std::abs(-Chassis[10]) >
                        8.9884656743115785E+307)) {
              BEV_image_kernel275<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (*gpu_Chassis, gpu_X_AB);
              BEV_image_kernel276<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (gpu_X_AB, old_Prob_cv, *gpu_TV_range_x);
              TV_range_x_dirtyOnGpu = true;
            } else {
              BEV_image_kernel273<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (*gpu_Chassis, gpu_X_AB);
              BEV_image_kernel274<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (gpu_X_AB, old_Prob_cv, *gpu_TV_range_x);
              TV_range_x_dirtyOnGpu = true;
            }

            if (yp4_dirtyOnCpu) {
              cudaMemcpy(*gpu_yp4, yp4, 800UL, cudaMemcpyHostToDevice);
            }

            if (xp4_dirtyOnCpu) {
              cudaMemcpy(*gpu_xp4, xp4, 800UL, cudaMemcpyHostToDevice);
            }

            if (yp3_dirtyOnCpu) {
              cudaMemcpy(*gpu_yp3, yp3, 800UL, cudaMemcpyHostToDevice);
            }

            if (xp3_dirtyOnCpu) {
              cudaMemcpy(*gpu_xp3, xp3, 800UL, cudaMemcpyHostToDevice);
            }

            if (yp2_dirtyOnCpu) {
              cudaMemcpy(*gpu_yp2, yp2, 800UL, cudaMemcpyHostToDevice);
            }

            if (xp2_dirtyOnCpu) {
              cudaMemcpy(*gpu_xp2, xp2, 800UL, cudaMemcpyHostToDevice);
            }

            if (yp1_dirtyOnCpu) {
              cudaMemcpy(*gpu_yp1, yp1, 800UL, cudaMemcpyHostToDevice);
            }

            if (xp1_dirtyOnCpu) {
              cudaMemcpy(*gpu_xp1, xp1, 800UL, cudaMemcpyHostToDevice);
            }

            BEV_image_kernel278<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
              (*gpu_TV_range_y, *gpu_TV_range_x, *gpu_TJ_X, *gpu_TJ_Y, lastc,
               *gpu_yp4, *gpu_xp4, *gpu_yp3, *gpu_xp3, *gpu_yp2, *gpu_xp2,
               *gpu_yp1, *gpu_xp1);
            BEV_image_kernel279<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
              (*gpu_yp4, *gpu_xp4, *gpu_yp3, *gpu_xp3, *gpu_yp2, *gpu_xp2,
               *gpu_yp1, *gpu_xp1);
            xp1_dirtyOnCpu = false;
            v_dirtyOnGpu = true;
            yp1_dirtyOnCpu = false;
            Class_B_idx_9_dirtyOnCpu = true;
            xp2_dirtyOnCpu = false;
            Class_B_idx_10_dirtyOnGpu = true;
            yp2_dirtyOnCpu = false;
            xp3_dirtyOnCpu = false;
            yp3_dirtyOnCpu = false;
            Vr_dirtyOnGpu = true;
            xp4_dirtyOnCpu = false;
            nr_dirtyOnCpu = true;
            yp4_dirtyOnCpu = false;
            validLaunchParams = true;
            m = 3 * (lastc + 1);
            jA = 3 * (lastc + 1);
            ix = 3 * (lastc + 1);
            knt = 3 * (lastc + 1);
            for (j = 0; j < 100; j++) {
              if (v_dirtyOnGpu) {
                cudaMemcpy(xp1, *gpu_xp1, 800UL, cudaMemcpyDeviceToHost);
              }

              v_dirtyOnGpu = false;
              d = xp1[j];
              if (d < 1.0) {
                d = 1.0;
                xp1[j] = 1.0;
                xp1_dirtyOnCpu = true;
              } else if (d > 251.0) {
                d = 251.0;
                xp1[j] = 251.0;
                xp1_dirtyOnCpu = true;
              }

              if (Class_B_idx_9_dirtyOnCpu) {
                cudaMemcpy(yp1, *gpu_yp1, 800UL, cudaMemcpyDeviceToHost);
              }

              Class_B_idx_9_dirtyOnCpu = false;
              scale = yp1[j];
              if (scale < 1.0) {
                scale = 1.0;
                yp1[j] = 1.0;
                yp1_dirtyOnCpu = true;
              } else if (scale > 61.0) {
                scale = 61.0;
                yp1[j] = 61.0;
                yp1_dirtyOnCpu = true;
              }

              if (Class_B_idx_10_dirtyOnGpu) {
                cudaMemcpy(xp2, *gpu_xp2, 800UL, cudaMemcpyDeviceToHost);
              }

              Class_B_idx_10_dirtyOnGpu = false;
              old_Prob_ctrv = xp2[j];
              if (old_Prob_ctrv < 1.0) {
                old_Prob_ctrv = 1.0;
                xp2[j] = 1.0;
                xp2_dirtyOnCpu = true;
              } else if (old_Prob_ctrv > 251.0) {
                old_Prob_ctrv = 251.0;
                xp2[j] = 251.0;
                xp2_dirtyOnCpu = true;
              }

              if (old_flag_dirtyOnGpu) {
                cudaMemcpy(gpu_X_AB, &X_AB, 8UL, cudaMemcpyHostToDevice);
              }

              BEV_image_kernel280<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (*gpu_yp2, j, gpu_X_AB);
              old_flag_dirtyOnGpu = false;
              cudaMemcpy(&X_AB, gpu_X_AB, 8UL, cudaMemcpyDeviceToHost);
              if (X_AB < 1.0) {
                X_AB = 1.0;
                old_flag_dirtyOnGpu = true;
                BEV_image_kernel282<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(j, *
                  gpu_yp2);
              } else if (X_AB > 61.0) {
                X_AB = 61.0;
                old_flag_dirtyOnGpu = true;
                BEV_image_kernel281<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(j, *
                  gpu_yp2);
              }

              if (Y_AB_dirtyOnCpu) {
                cudaMemcpy(gpu_Y_AB, &Y_AB, 8UL, cudaMemcpyHostToDevice);
              }

              BEV_image_kernel283<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (*gpu_xp3, j, gpu_Y_AB);
              Y_AB_dirtyOnCpu = false;
              cudaMemcpy(&Y_AB, gpu_Y_AB, 8UL, cudaMemcpyDeviceToHost);
              if (Y_AB < 1.0) {
                Y_AB = 1.0;
                Y_AB_dirtyOnCpu = true;
                BEV_image_kernel285<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(j, *
                  gpu_xp3);
              } else if (Y_AB > 251.0) {
                Y_AB = 251.0;
                Y_AB_dirtyOnCpu = true;
                BEV_image_kernel284<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(j, *
                  gpu_xp3);
              }

              if (Vr_dirtyOnGpu) {
                cudaMemcpy(yp3, *gpu_yp3, 800UL, cudaMemcpyDeviceToHost);
              }

              Vr_dirtyOnGpu = false;
              Class_B_idx_9 = yp3[j];
              if (Class_B_idx_9 < 1.0) {
                Class_B_idx_9 = 1.0;
                yp3[j] = 1.0;
                yp3_dirtyOnCpu = true;
              } else if (Class_B_idx_9 > 61.0) {
                Class_B_idx_9 = 61.0;
                yp3[j] = 61.0;
                yp3_dirtyOnCpu = true;
              }

              if (nr_dirtyOnCpu) {
                cudaMemcpy(xp4, *gpu_xp4, 800UL, cudaMemcpyDeviceToHost);
              }

              nr_dirtyOnCpu = false;
              Class_B_idx_10 = xp4[j];
              if (Class_B_idx_10 < 1.0) {
                Class_B_idx_10 = 1.0;
                xp4[j] = 1.0;
                xp4_dirtyOnCpu = true;
              } else if (Class_B_idx_10 > 251.0) {
                Class_B_idx_10 = 251.0;
                xp4[j] = 251.0;
                xp4_dirtyOnCpu = true;
              }

              if (validLaunchParams) {
                cudaMemcpy(yp4, *gpu_yp4, 800UL, cudaMemcpyDeviceToHost);
              }

              validLaunchParams = false;
              t = yp4[j];
              if (t < 1.0) {
                t = 1.0;
                yp4[j] = 1.0;
                yp4_dirtyOnCpu = true;
              } else if (t > 61.0) {
                t = 61.0;
                yp4[j] = 61.0;
                yp4_dirtyOnCpu = true;
              }

              nr = static_cast<int>(scale) - 1;
              cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
              if (BEV_image_dirtyOnCpu) {
                cudaMemcpy(*gpu_BEV_image, b_BEV_image, 367464UL,
                           cudaMemcpyHostToDevice);
              }

              BEV_image_kernel286<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(m,
                gpu_nr, static_cast<int>(d) - 1, *gpu_BEV_image);

              //  R
              nr = static_cast<int>(X_AB) - 1;
              cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
              BEV_image_kernel287<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(jA,
                gpu_nr, static_cast<int>(old_Prob_ctrv) - 1, *gpu_BEV_image);

              //  R
              nr = static_cast<int>(Class_B_idx_9) - 1;
              cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
              BEV_image_kernel288<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(ix,
                gpu_nr, static_cast<int>(Y_AB) - 1, *gpu_BEV_image);

              //  R
              nr = static_cast<int>(t) - 1;
              cudaMemcpy(gpu_nr, &nr, 4UL, cudaMemcpyHostToDevice);
              BEV_image_kernel289<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(knt,
                gpu_nr, static_cast<int>(Class_B_idx_10) - 1, *gpu_BEV_image);
              BEV_image_dirtyOnCpu = false;
              BEV_image_dirtyOnGpu = true;

              //  R
            }

            //          for i = 1:length(Image_Position_X)
            //              BEV_Window_out(Image_Position_X(i), Image_Position_Y(i), :) = 255;
          }
        }
      }
    }
  }

  if (BEV_image_dirtyOnGpu) {
    cudaMemcpy(b_BEV_image, *gpu_BEV_image, 367464UL, cudaMemcpyDeviceToHost);
  }

  cudaFree(*gpu_flag);
  cudaFree(*gpu_old_flag);
  cudaFree(gpu_k);
  cudaFree(*gpu_State);
  cudaFree(*gpu_dv);
  cudaFree(gpu_X_AB);
  cudaFree(*gpu_TJ_X);
  cudaFree(*gpu_TJ_Y);
  cudaFree(gpu_temp);
  cudaFree(*gpu_x_ini);
  cudaFree(*gpu_tmp_ego_y);
  cudaFree(*gpu_work);
  cudaFree(*gpu_x_cv_out);
  cudaFree(*gpu_x_ctrv_out);
  cudaFree(*gpu_out_P_ctrv);
  cudaFree(*gpu_L);
  cudaFree(*gpu_out_P_cv);
  cudaFree(*gpu_Vr);
  cudaFree(gpu_Class_B_idx_9);
  cudaFree(*gpu_P_ctrv_out);
  cudaFree(*gpu_P_cv_out);
  cudaFree(*gpu_P_cv_tmp);
  cudaFree(*gpu_P_ctrv_tmp);
  cudaFree(*gpu_P_ctrv);
  cudaFree(*gpu_U);
  cudaFree(*gpu_L_tmp);
  cudaFree(*gpu_R);
  cudaFree(*gpu_v);
  cudaFree(*b_gpu_U);
  cudaFree(gpu_scale);
  cudaFree(gpu_Y_AB);
  cudaFree(*gpu_X_k);
  cudaFree(*gpu_W);
  cudaFree(b_gpu_L);
  cudaFree(gpu_nr);
  cudaFree(*gpu_x_k_hat);
  cudaFree(gpu_c);
  cudaFree(*gpu_X_k_hat);
  cudaFree(gpu_d);
  cudaFree(*gpu_X_hat);
  cudaFree(*gpu_P_zz);
  cudaFree(*gpu_P_xz);
  cudaFree(*gpu_a);
  cudaFree(*gpu_Y_k_hat);
  cudaFree(*gpu_R_CTRV_IMM);
  cudaFree(*gpu_A);
  cudaFree(*b_gpu_Y_k_hat);
  cudaFree(*b_gpu_W);
  cudaFree(gpu_Class_B_idx_10);
  cudaFree(*gpu_ipiv_t);
  cudaFree(gpu_info);
  cudaFree(*gpu_ipiv);
  cudaFree(*gpu_y_out);
  cudaFree(*gpu_y);
  cudaFree(*gpu_dv1);
  cudaFree(*gpu_x_ctrv);
  cudaFree(*gpu_K);
  cudaFree(*gpu_P_cv);
  cudaFree(b_gpu_info);
  cudaFree(*gpu_X_pred);
  cudaFree(*gpu_Training_data_data);
  cudaFree(*gpu_dv2);
  cudaFree(*b_gpu_dv1);
  cudaFree(*gpu_Ry);
  cudaFree(*gpu_Rx);
  cudaFree(*b_gpu_dv);
  cudaFree(*gpu_x);
  cudaFree(*gpu_varargin_1);
  cudaFree(*gpu_b);
  cudaFree(*b_gpu_b);
  cudaFree(*gpu_tmp_lane_y);
  cudaFree(*gpu_BEV_image);
  cudaFree(*gpu_Chassis);
  cudaFree(*b_gpu_y);
  cudaFree(*gpu_in_tmp);
  cudaFree(*b_gpu_varargin_1);
  cudaFree(*b_gpu_x);
  cudaFree(*gpu_last);
  cudaFree(*gpu_first);
  cudaFree(*gpu_tmp_ego_x);
  cudaFree(*c_gpu_x);
  cudaFree(*gpu_target_y);
  cudaFree(*gpu_i_data);
  cudaFree(*gpu_j_data);
  cudaFree(*gpu_TV_range_y);
  cudaFree(*gpu_TV_range_x);
  cudaFree(*gpu_yp4);
  cudaFree(*gpu_xp4);
  cudaFree(*gpu_yp3);
  cudaFree(*gpu_xp3);
  cudaFree(*gpu_yp2);
  cudaFree(*gpu_xp2);
  cudaFree(*gpu_yp1);
  cudaFree(*gpu_xp1);
}

//
// Arguments    : void
// Return Type  : void
//
void BEV_image_init()
{
  out_Prob_ctrv = 0.0;
  out_Prob_cv = 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
void just_one_check_not_empty_init()
{
  just_one_check_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void out_Prob_ctrv_not_empty_init()
{
  out_Prob_ctrv_not_empty = false;
}

//
// File trailer for BEV_image.cu
//
// [EOF]
//
