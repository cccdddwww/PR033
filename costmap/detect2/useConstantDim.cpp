//
// File: useConstantDim.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

// Include Files
#include "useConstantDim.h"

// Function Definitions
//
// Arguments    : double varargin_2[40320]
//                int varargin_3
// Return Type  : void
//
namespace coder {
namespace internal {
void useConstantDim(double varargin_2[40320], int varargin_3)
{
  if (1 == varargin_3) {
    for (int k{0}; k < 180; k++) {
      for (int b_k{0}; b_k < 223; b_k++) {
        int i;
        i = b_k + 224 * k;
        varargin_2[i + 1] += varargin_2[i];
      }
    }
  } else {
    for (int k{0}; k < 179; k++) {
      for (int b_k{0}; b_k < 224; b_k++) {
        int i;
        i = b_k + 224 * (k + 1);
        varargin_2[i] += varargin_2[b_k + 224 * k];
      }
    }
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for useConstantDim.cpp
//
// [EOF]
//
