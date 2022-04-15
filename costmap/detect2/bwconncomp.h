//
// File: bwconncomp.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

#ifndef BWCONNCOMP_H
#define BWCONNCOMP_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void bwconncomp(const bool varargin_1[32000], double *CC_Connectivity,
                double CC_ImageSize[2], double *CC_NumObjects,
                ::coder::array<double, 1U> &CC_RegionIndices,
                ::coder::array<int, 1U> &CC_RegionLengths);

}

#endif
//
// File trailer for bwconncomp.h
//
// [EOF]
//
