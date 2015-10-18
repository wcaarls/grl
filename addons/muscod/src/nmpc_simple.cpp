
/*
 *
 *  MUSCOD-II/MC2_TEST/nmpc1.c
 *  (c) Moritz Diehl, 2004
 *
 *  Simple  NMPC problem
 *
 */

#include <cmath>
#include <iostream>
#include <vector>
#include "def_usrmod.hpp"

#define  NMOS   1
#define  NP     0
#define  NRC    0
#define  NRCE   0

#define  NXD    1
#define  NXA    0
#define  NU     1
#define  NPR    0

const double w0 = 10.0;
const double w1 =  1.0;

const double sw0 = sqrt(w0);
const double sw1 = sqrt(w1);

extern "C" {
  void set_path(std::string new_problem_path, std::string new_lua_model){
  }
  void convert_obs_for_muscod(const double *from, double *to){
  }
}

static void lfcn(double *t, double *xd, double *xa, double *u,
  double *p, double *lval, double *rwh, long *iwh, InfoPtr *info) {
    *lval = 10 * xd[0] * xd[0] + u[0] * u[0];
}

static void mfcn_e(double *ts, double *xd, double *xa,
  double *p, double *pr, double *mval,  long *dpnd, InfoPtr *info)
{
  if (*dpnd) { *dpnd = MFCN_DPND(*ts, *xd, 0, 0, 0); return; }
  // choose endpoint objective instead of endpoint constraint
  *mval = 1000 * xd[0] * xd[0];
}

static void ffcn(double *t, double *xd, double *xa, double *u,
  double *p, double *rhs, double *rwh, long *iwh, InfoPtr *info)
{
  //     =           x + x**2  + u
  rhs[0] = (1 + xd[0]) * xd[0] + u[0];
}

static void lsqfcn(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{
  if (*dpnd) { *dpnd = RFCN_DPND(0, *sd, 0, *u, 0, 0); return; }

  res[0] = sw0 * sd[0];
  res[1] = sw1 *  u[0];
}

static void msqfcn(double *ts, double *sd, double *sa, double *u,
  double *p, double *pr, double *res, long *dpnd, InfoPtr *info)
{

  if (*dpnd) { *dpnd = RFCN_DPND(0, *sd, 0, *u, 0, 0); return; }

  res[0] = 2.0 * sw0 * sd[0];
  res[1] = 2.0 * sw1 *  u[0];
}

extern "C" void def_model(void)
{
  def_mdims(NMOS, NP, NRC, NRCE);
  def_mstage(
    0, // stage number
    NXD, NXA, NU, // variable dimensions
    NULL, NULL, // objective functions
    //mfcn_e, lfcn, // objective functions
    0, 0, 0, NULL, ffcn, NULL, // rhs
    NULL, NULL
  );
  // def_lsq(0, "c",0 , NXD+NU, lsqfcn);
  def_lsq(0, "s", 0, NXD+NU, lsqfcn);
  def_lsq(0, "i", 0, NXD+NU, lsqfcn);
  def_lsq(0, "e", 0, NXD+NU, msqfcn);
  def_mpc(0, "End Point", NPR, NXD, NXD, NULL, NULL);
}
