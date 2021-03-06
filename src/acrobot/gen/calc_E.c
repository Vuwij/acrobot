/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: calc_E.c
 *
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 08-Apr-2020 04:22:08
 */

/* Include Files */
#include "calc_E.h"
#include "calc_B.h"
#include "calc_B_data.h"
#include "calc_B_initialize.h"
#include "calc_C.h"
#include "calc_D.h"
#include "calc_De.h"
#include "calc_EE.h"
#include "calc_J.h"
#include "calc_P.h"
#include "calc_dUde.h"
#include "calc_qd.h"
#include "calc_rend.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */

/*
 * CALC_E
 *     E = CALC_E(L1,L2,Q1,Q2)
 * Arguments    : float l1
 *                float l2
 *                float q1
 *                float q2
 *                float E[8]
 * Return Type  : void
 */
void calc_E(float l1, float l2, float q1, float q2, float E[8])
{
  float t2;
  float t5;
  if (isInitialized_calc_B == false) {
    calc_B_initialize();
  }

  /*     This function was generated by the Symbolic Math Toolbox version 8.4. */
  /*     07-Apr-2020 19:04:19 */
  t2 = q1 + q2;
  t5 = l2 * cosf(t2);
  t2 = -(l2 * sinf(t2));
  E[0] = t2 - l1 * sinf(q1);
  E[1] = t5 + l1 * cosf(q1);
  E[2] = t2;
  E[3] = t5;
  E[4] = 1.0F;
  E[5] = 0.0F;
  E[6] = 0.0F;
  E[7] = 1.0F;
}

/*
 * File trailer for calc_E.c
 *
 * [EOF]
 */
