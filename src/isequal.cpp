//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: isequal.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 10-Nov-2021 22:00:36
//

// Include Files
#include "isequal.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const unsigned char varargin_1[275598]
//                const unsigned char varargin_2[275598]
// Return Type  : bool
//
namespace coder {
bool isequal(const unsigned char varargin_1[275598],
             const unsigned char varargin_2[275598])
{
  int k;
  bool b_p;
  bool exitg1;
  bool p;
  p = false;
  b_p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 275598)) {
    if (varargin_1[k] != varargin_2[k]) {
      b_p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return b_p || p;
}

} // namespace coder

//
// File trailer for isequal.cpp
//
// [EOF]
//
