//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: find.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 09-Nov-2021 00:41:02
//

// Include Files
#include "find.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : const bool x[15311]
//                int i_data[]
//                int *i_size
//                int j_data[]
//                int *j_size
// Return Type  : void
//
namespace coder {
void eml_find(const bool x[15311], int i_data[], int *i_size, int j_data[],
              int *j_size)
{
  int idx;
  int ii;
  int jj;
  bool exitg1;
  idx = 0;
  ii = 1;
  jj = 1;
  exitg1 = false;
  while ((!exitg1) && (jj <= 251)) {
    bool guard1{false};
    guard1 = false;
    if (x[(ii + 61 * (jj - 1)) - 1]) {
      idx++;
      i_data[idx - 1] = ii;
      j_data[idx - 1] = jj;
      if (idx >= 15311) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      ii++;
      if (ii > 61) {
        ii = 1;
        jj++;
      }
    }
  }
  if (1 > idx) {
    *i_size = 0;
    *j_size = 0;
  } else {
    *i_size = idx;
    *j_size = idx;
  }
}

} // namespace coder

//
// File trailer for find.cpp
//
// [EOF]
//
