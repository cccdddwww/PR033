//
// File: regionprops.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

#ifndef REGIONPROPS_H
#define REGIONPROPS_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct c_struct_T;

struct e_struct_T;

struct struct_T;

struct d_struct_T;

// Function Declarations
namespace coder {
void ComputePixelIdxList(const ::coder::array<double, 1U> &CC_RegionIndices,
                         const ::coder::array<int, 1U> &CC_RegionLengths,
                         double numObjs, ::coder::array<e_struct_T, 1U> &stats,
                         struct_T *statsAlreadyComputed);

void b_regionprops(const bool varargin_1[32000],
                   ::coder::array<d_struct_T, 1U> &outstats);

void initializeStatsStruct(double numObjs,
                           ::coder::array<e_struct_T, 1U> &stats,
                           struct_T *statsAlreadyComputed);

void regionprops(const double varargin_1[32000],
                 ::coder::array<c_struct_T, 1U> &outstats);

} // namespace coder

#endif
//
// File trailer for regionprops.h
//
// [EOF]
//
