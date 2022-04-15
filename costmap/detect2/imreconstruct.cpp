//
// File: imreconstruct.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

// Include Files
#include "imreconstruct.h"
#include <algorithm>

// Function Definitions
//
// Arguments    : bool marker[32000]
//                const bool mask[32000]
// Return Type  : void
//
namespace coder {
void imreconstruct(bool marker[32000], const bool mask[32000])
{
  static unsigned char stack[512000];
  int i;
  int i_tmp;
  int j;
  int stackTop;
  int stack_tmp;
  bool bw1[32000];
  std::copy(&mask[0], &mask[32000], &bw1[0]);
  stackTop = -1;
  for (j = 0; j < 160; j++) {
    for (i = 0; i < 200; i++) {
      i_tmp = i + 200 * j;
      if (marker[i_tmp]) {
        stackTop++;
        stack_tmp = stackTop << 1;
        stack[stack_tmp] = static_cast<unsigned char>(i + 1);
        stack[stack_tmp + 1] = static_cast<unsigned char>(j + 1);
        marker[i_tmp] = false;
      }
    }
  }
  while (stackTop + 1 > 0) {
    i_tmp = stackTop << 1;
    i = stack[i_tmp] - 1;
    j = stack[i_tmp + 1] - 1;
    stackTop--;
    i_tmp = i + 200 * j;
    if (bw1[i_tmp]) {
      marker[i_tmp] = true;
      bw1[i_tmp] = false;
      if ((i + 2 <= 200) && bw1[i_tmp + 1]) {
        stackTop++;
        stack_tmp = stackTop << 1;
        stack[stack_tmp] = static_cast<unsigned char>(i + 2);
        stack[stack_tmp + 1] = static_cast<unsigned char>(j + 1);
      }
      if ((i >= 1) && bw1[i_tmp - 1]) {
        stackTop++;
        stack_tmp = stackTop << 1;
        stack[stack_tmp] = static_cast<unsigned char>(i);
        stack[stack_tmp + 1] = static_cast<unsigned char>(j + 1);
      }
      if ((j + 2 <= 160) && bw1[i + 200 * (j + 1)]) {
        stackTop++;
        stack_tmp = stackTop << 1;
        stack[stack_tmp] = static_cast<unsigned char>(i + 1);
        stack[stack_tmp + 1] = static_cast<unsigned char>(j + 2);
      }
      if ((j >= 1) && bw1[i + 200 * (j - 1)]) {
        stackTop++;
        stack_tmp = stackTop << 1;
        stack[stack_tmp] = static_cast<unsigned char>(i + 1);
        stack[stack_tmp + 1] = static_cast<unsigned char>(j);
      }
      if ((i >= 1) && (j + 2 <= 160) && bw1[(i + 200 * (j + 1)) - 1]) {
        stackTop++;
        stack_tmp = stackTop << 1;
        stack[stack_tmp] = static_cast<unsigned char>(i);
        stack[stack_tmp + 1] = static_cast<unsigned char>(j + 2);
      }
      if ((i + 2 <= 200) && (j + 2 <= 160) && bw1[(i + 200 * (j + 1)) + 1]) {
        stackTop++;
        stack_tmp = stackTop << 1;
        stack[stack_tmp] = static_cast<unsigned char>(i + 2);
        stack[stack_tmp + 1] = static_cast<unsigned char>(j + 2);
      }
      if ((i >= 1) && (j >= 1) && bw1[(i + 200 * (j - 1)) - 1]) {
        stackTop++;
        stack_tmp = stackTop << 1;
        stack[stack_tmp] = static_cast<unsigned char>(i);
        stack[stack_tmp + 1] = static_cast<unsigned char>(j);
      }
      if ((i + 2 <= 200) && (j >= 1) && bw1[(i + 200 * (j - 1)) + 1]) {
        stackTop++;
        stack_tmp = stackTop << 1;
        stack[stack_tmp] = static_cast<unsigned char>(i + 2);
        stack[stack_tmp + 1] = static_cast<unsigned char>(j);
      }
    }
  }
}

} // namespace coder

//
// File trailer for imreconstruct.cpp
//
// [EOF]
//
