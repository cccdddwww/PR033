//
// File: detect2_internal_types.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

#ifndef DETECT2_INTERNAL_TYPES_H
#define DETECT2_INTERNAL_TYPES_H

// Include Files
#include "detect2_types.h"
#include "rtwtypes.h"
#include "coder_array.h"

// Type Definitions
struct struct_T {
  bool Area;
  bool Centroid;
  bool BoundingBox;
  bool MajorAxisLength;
  bool MinorAxisLength;
  bool Eccentricity;
  bool Orientation;
  bool Image;
  bool FilledImage;
  bool FilledArea;
  bool EulerNumber;
  bool Extrema;
  bool EquivDiameter;
  bool Extent;
  bool PixelIdxList;
  bool PixelList;
  bool Perimeter;
  bool Circularity;
  bool PixelValues;
  bool WeightedCentroid;
  bool MeanIntensity;
  bool MinIntensity;
  bool MaxIntensity;
  bool SubarrayIdx;
};

struct c_struct_T {
  double Centroid[2];
};

struct d_struct_T {
  double BoundingBox[4];
};

struct emxArray_boolean_T_0x0 {
  int size[2];
};

struct emxArray_real_T_0 {
  int size[1];
};

struct emxArray_real_T_1x0 {
  int size[2];
};

struct e_struct_T {
  double Area;
  double Centroid[2];
  double BoundingBox[4];
  double MajorAxisLength;
  double MinorAxisLength;
  double Eccentricity;
  double Orientation;
  emxArray_boolean_T_0x0 Image;
  emxArray_boolean_T_0x0 FilledImage;
  double FilledArea;
  double EulerNumber;
  double Extrema[16];
  double EquivDiameter;
  double Extent;
  coder::array<double, 1U> PixelIdxList;
  coder::array<double, 2U> PixelList;
  double Perimeter;
  double Circularity;
  emxArray_real_T_0 PixelValues;
  double WeightedCentroid[2];
  double MeanIntensity;
  double MinIntensity;
  double MaxIntensity;
  emxArray_real_T_1x0 SubarrayIdx;
  double SubarrayIdxLengths[2];
};

#endif
//
// File trailer for detect2_internal_types.h
//
// [EOF]
//
