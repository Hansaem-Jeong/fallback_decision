//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: matlab_array2magick.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 12-Nov-2021 04:02:16
//

// Include Files
#include "matlab_array2magick.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const unsigned char in[275598]
//                unsigned char img[275598]
// Return Type  : void
//
void matlab_array2magick(const unsigned char in[275598],
                         unsigned char img[275598])
{
  int num;
  //  in : [251, 61, 18] 275598
  std::memset(&img[0], 0, 275598U * sizeof(unsigned char));
  // img = ones([251, 1098], 'uint8');
  num = 1;
  for (int idx{0}; idx < 6; idx++) {
    int i;
    i = 3 * idx;
    for (int jdx{0}; jdx < 61; jdx++) {
      int i1;
      for (i1 = 0; i1 < 251; i1++) {
        img[(num + 1098 * (250 - i1)) - 1] = in[(i1 + 251 * jdx) + 15311 * i];
      }
      for (i1 = 0; i1 < 251; i1++) {
        img[num + 1098 * (250 - i1)] = in[(i1 + 251 * jdx) + 15311 * (i + 1)];
      }
      for (i1 = 0; i1 < 251; i1++) {
        img[(num + 1098 * (250 - i1)) + 1] =
            in[(i1 + 251 * jdx) + 15311 * (i + 2)];
      }
      num += 3;
    }
  }
}

//
// File trailer for matlab_array2magick.cpp
//
// [EOF]
//
