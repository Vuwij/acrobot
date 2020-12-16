/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: calc_P.c
 *
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 08-Apr-2020 04:22:08
 */

/* Include Files */
#include "calc_P.h"
#include "calc_B.h"
#include "calc_B_data.h"
#include "calc_B_initialize.h"
#include "calc_C.h"
#include "calc_D.h"
#include "calc_De.h"
#include "calc_E.h"
#include "calc_EE.h"
#include "calc_J.h"
#include "calc_dUde.h"
#include "calc_qd.h"
#include "calc_rend.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */

/*
 * CALC_P
 *     P = CALC_P(G,L1,LC1,LC2,M1,M2,Q1,Q2)
 * Arguments    : float g
 *                float l1
 *                float lc1
 *                float lc2
 *                float m1
 *                float m2
 *                float q1
 *                float q2
 *                float P[2]
 * Return Type  : void
 */
void calc_P(float g, float l1, float lc1, float lc2, float m1, float m2, float
            q1, float q2, float P[2])
{
  float t2;
  float t4;
  if (isInitialized_calc_B == false) {
    calc_B_initialize();
  }

  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     07-Apr-2020 19:04:18 */
  t2 = cosf(q1);
  t4 = cosf(q1 + q2);
  P[0] = g * m2 * (l1 * t2 + lc2 * t4) + g * m1 * t2 * (l1 - lc1);
  P[1] = g * lc2 * m2 * t4;
}

/*
 * File trailer for calc_P.c
 *
 * [EOF]
 */