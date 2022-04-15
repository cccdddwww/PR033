//
// File: imbinarize.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

// Include Files
#include "imbinarize.h"
#include "adaptthresh.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const unsigned char b_I[32000]
//                bool BW[32000]
// Return Type  : void
//
namespace coder {
void imbinarize(const unsigned char b_I[32000], bool BW[32000])
{
  static double c_I[32000];
  static double t[32000];
  int k;
  for (k = 0; k < 32000; k++) {
    c_I[k] = static_cast<double>(b_I[k]) / 255.0;
  }
  localMeanThresh(c_I, t);
  for (k = 0; k < 32000; k++) {
    double d;
    d = std::fmax(std::fmin(t[k], 1.0), 0.0);
    t[k] = d;
    BW[k] = (b_I[k] > 255.0 * d);
  }
}

} // namespace coder

//
// File trailer for imbinarize.cpp
//
// [EOF]
//
