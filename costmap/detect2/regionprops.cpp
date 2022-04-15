//
// File: regionprops.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

// Include Files
#include "regionprops.h"
#include "bwconncomp.h"
#include "detect2_internal_types.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 1U> &CC_RegionIndices
//                const ::coder::array<int, 1U> &CC_RegionLengths
//                double numObjs
//                ::coder::array<e_struct_T, 1U> &stats
//                struct_T *statsAlreadyComputed
// Return Type  : void
//
namespace coder {
void ComputePixelIdxList(const ::coder::array<double, 1U> &CC_RegionIndices,
                         const ::coder::array<int, 1U> &CC_RegionLengths,
                         double numObjs, ::coder::array<e_struct_T, 1U> &stats,
                         struct_T *statsAlreadyComputed)
{
  array<int, 1U> idxCount;
  array<int, 1U> regionLengths;
  statsAlreadyComputed->PixelIdxList = true;
  if (numObjs != 0.0) {
    int i;
    int k;
    int loop_ub;
    regionLengths.set_size(CC_RegionLengths.size(0));
    loop_ub = CC_RegionLengths.size(0);
    for (i = 0; i < loop_ub; i++) {
      regionLengths[i] = CC_RegionLengths[i];
    }
    if ((CC_RegionLengths.size(0) != 1) && (CC_RegionLengths.size(0) != 0) &&
        (CC_RegionLengths.size(0) != 1)) {
      i = CC_RegionLengths.size(0);
      for (k = 0; k <= i - 2; k++) {
        regionLengths[k + 1] = regionLengths[k] + regionLengths[k + 1];
      }
    }
    idxCount.set_size(regionLengths.size(0) + 1);
    idxCount[0] = 0;
    loop_ub = regionLengths.size(0);
    for (i = 0; i < loop_ub; i++) {
      idxCount[i + 1] = regionLengths[i];
    }
    i = stats.size(0);
    for (k = 0; k < i; k++) {
      int i1;
      int i2;
      i1 = idxCount[k + 1];
      if (idxCount[k] + 1 > i1) {
        i2 = 0;
        i1 = 0;
      } else {
        i2 = idxCount[k];
      }
      loop_ub = i1 - i2;
      stats[k].PixelIdxList.set_size(loop_ub);
      for (i1 = 0; i1 < loop_ub; i1++) {
        stats[k].PixelIdxList[i1] = CC_RegionIndices[i2 + i1];
      }
    }
  }
}

//
// Arguments    : const bool varargin_1[32000]
//                ::coder::array<d_struct_T, 1U> &outstats
// Return Type  : void
//
void b_regionprops(const bool varargin_1[32000],
                   ::coder::array<d_struct_T, 1U> &outstats)
{
  array<e_struct_T, 1U> stats;
  array<double, 1U> CC_RegionIndices;
  array<int, 1U> v1;
  array<int, 1U> vk;
  d_struct_T s;
  struct_T statsAlreadyComputed;
  double maxval[2];
  double min_corner[2];
  double CC_NumObjects;
  double expl_temp;
  int i;
  int i1;
  int k;
  int loop_ub_tmp;
  bwconncomp(varargin_1, &expl_temp, min_corner, &CC_NumObjects,
             CC_RegionIndices, v1);
  s.BoundingBox[0] = 0.0;
  s.BoundingBox[1] = 0.0;
  s.BoundingBox[2] = 0.0;
  s.BoundingBox[3] = 0.0;
  loop_ub_tmp = static_cast<int>(CC_NumObjects);
  outstats.set_size(loop_ub_tmp);
  for (i = 0; i < loop_ub_tmp; i++) {
    outstats[i] = s;
  }
  initializeStatsStruct(CC_NumObjects, stats, &statsAlreadyComputed);
  ComputePixelIdxList(CC_RegionIndices, v1, CC_NumObjects, stats,
                      &statsAlreadyComputed);
  i = stats.size(0);
  for (k = 0; k < i; k++) {
    loop_ub_tmp = stats[k].PixelIdxList.size(0);
    if (stats[k].PixelIdxList.size(0) != 0) {
      v1.set_size(stats[k].PixelIdxList.size(0));
      for (i1 = 0; i1 < loop_ub_tmp; i1++) {
        v1[i1] = static_cast<int>(stats[k].PixelIdxList[i1]) - 1;
      }
      vk.set_size(v1.size(0));
      loop_ub_tmp = v1.size(0);
      for (i1 = 0; i1 < loop_ub_tmp; i1++) {
        vk[i1] = v1[i1] / 200;
      }
      loop_ub_tmp = v1.size(0);
      for (i1 = 0; i1 < loop_ub_tmp; i1++) {
        v1[i1] = v1[i1] - vk[i1] * 200;
      }
      stats[k].PixelList.set_size(vk.size(0), 2);
      loop_ub_tmp = vk.size(0);
      for (i1 = 0; i1 < loop_ub_tmp; i1++) {
        stats[k].PixelList[i1] = vk[i1] + 1;
      }
      loop_ub_tmp = v1.size(0);
      for (i1 = 0; i1 < loop_ub_tmp; i1++) {
        stats[k].PixelList[i1 + stats[k].PixelList.size(0)] = v1[i1] + 1;
      }
    } else {
      stats[k].PixelList.set_size(0, 2);
    }
  }
  i = stats.size(0);
  for (k = 0; k < i; k++) {
    i1 = stats[k].PixelList.size(0);
    if (stats[k].PixelList.size(0) == 0) {
      stats[k].BoundingBox[0] = 0.5;
      stats[k].BoundingBox[1] = 0.5;
      stats[k].BoundingBox[2] = 0.0;
      stats[k].BoundingBox[3] = 0.0;
    } else {
      for (loop_ub_tmp = 0; loop_ub_tmp < 2; loop_ub_tmp++) {
        int b_i;
        bool p;
        min_corner[loop_ub_tmp] =
            stats[k].PixelList[stats[k].PixelList.size(0) * loop_ub_tmp];
        for (b_i = 2; b_i <= i1; b_i++) {
          expl_temp =
              stats[k]
                  .PixelList[(b_i + stats[k].PixelList.size(0) * loop_ub_tmp) -
                             1];
          if (std::isnan(expl_temp)) {
            p = false;
          } else if (std::isnan(min_corner[loop_ub_tmp])) {
            p = true;
          } else {
            p = (min_corner[loop_ub_tmp] > expl_temp);
          }
          if (p) {
            min_corner[loop_ub_tmp] = expl_temp;
          }
        }
        min_corner[loop_ub_tmp] -= 0.5;
        maxval[loop_ub_tmp] =
            stats[k].PixelList[stats[k].PixelList.size(0) * loop_ub_tmp];
        for (b_i = 2; b_i <= i1; b_i++) {
          expl_temp =
              stats[k]
                  .PixelList[(b_i + stats[k].PixelList.size(0) * loop_ub_tmp) -
                             1];
          if (std::isnan(expl_temp)) {
            p = false;
          } else if (std::isnan(maxval[loop_ub_tmp])) {
            p = true;
          } else {
            p = (maxval[loop_ub_tmp] < expl_temp);
          }
          if (p) {
            maxval[loop_ub_tmp] = expl_temp;
          }
        }
        expl_temp = min_corner[loop_ub_tmp];
        stats[k].BoundingBox[loop_ub_tmp] = expl_temp;
        stats[k].BoundingBox[loop_ub_tmp + 2] =
            (maxval[loop_ub_tmp] + 0.5) - expl_temp;
      }
    }
  }
  i = stats.size(0);
  for (k = 0; k < i; k++) {
    outstats[k].BoundingBox[0] = stats[k].BoundingBox[0];
    outstats[k].BoundingBox[1] = stats[k].BoundingBox[1];
    outstats[k].BoundingBox[2] = stats[k].BoundingBox[2];
    outstats[k].BoundingBox[3] = stats[k].BoundingBox[3];
  }
}

//
// Arguments    : double numObjs
//                ::coder::array<e_struct_T, 1U> &stats
//                struct_T *statsAlreadyComputed
// Return Type  : void
//
void initializeStatsStruct(double numObjs,
                           ::coder::array<e_struct_T, 1U> &stats,
                           struct_T *statsAlreadyComputed)
{
  e_struct_T statsOneObj;
  int loop_ub_tmp;
  statsAlreadyComputed->Area = false;
  statsOneObj.Area = 0.0;
  statsAlreadyComputed->Centroid = false;
  statsOneObj.Centroid[0] = 0.0;
  statsOneObj.Centroid[1] = 0.0;
  statsAlreadyComputed->BoundingBox = false;
  statsOneObj.BoundingBox[0] = 0.0;
  statsOneObj.BoundingBox[1] = 0.0;
  statsOneObj.BoundingBox[2] = 0.0;
  statsOneObj.BoundingBox[3] = 0.0;
  statsAlreadyComputed->MajorAxisLength = false;
  statsOneObj.MajorAxisLength = 0.0;
  statsAlreadyComputed->MinorAxisLength = false;
  statsOneObj.MinorAxisLength = 0.0;
  statsAlreadyComputed->Eccentricity = false;
  statsOneObj.Eccentricity = 0.0;
  statsAlreadyComputed->Orientation = false;
  statsOneObj.Orientation = 0.0;
  statsAlreadyComputed->Image = false;
  statsOneObj.Image.size[0] = 0;
  statsOneObj.Image.size[1] = 0;
  statsAlreadyComputed->FilledImage = false;
  statsOneObj.FilledImage.size[0] = 0;
  statsOneObj.FilledImage.size[1] = 0;
  statsAlreadyComputed->FilledArea = false;
  statsOneObj.FilledArea = 0.0;
  statsAlreadyComputed->EulerNumber = false;
  statsOneObj.EulerNumber = 0.0;
  statsAlreadyComputed->Extrema = false;
  std::memset(&statsOneObj.Extrema[0], 0, 16U * sizeof(double));
  statsAlreadyComputed->EquivDiameter = false;
  statsOneObj.EquivDiameter = 0.0;
  statsAlreadyComputed->Extent = false;
  statsOneObj.Extent = 0.0;
  statsAlreadyComputed->PixelIdxList = false;
  statsOneObj.PixelIdxList.set_size(0);
  statsAlreadyComputed->PixelList = false;
  statsOneObj.PixelList.set_size(0, 2);
  statsAlreadyComputed->Perimeter = false;
  statsOneObj.Perimeter = 0.0;
  statsAlreadyComputed->Circularity = false;
  statsOneObj.Circularity = 0.0;
  statsAlreadyComputed->PixelValues = false;
  statsOneObj.PixelValues.size[0] = 0;
  statsAlreadyComputed->WeightedCentroid = false;
  statsAlreadyComputed->MeanIntensity = false;
  statsOneObj.MeanIntensity = 0.0;
  statsAlreadyComputed->MinIntensity = false;
  statsOneObj.MinIntensity = 0.0;
  statsAlreadyComputed->MaxIntensity = false;
  statsOneObj.MaxIntensity = 0.0;
  statsAlreadyComputed->SubarrayIdx = false;
  statsOneObj.SubarrayIdx.size[0] = 1;
  statsOneObj.SubarrayIdx.size[1] = 0;
  statsOneObj.WeightedCentroid[0] = 0.0;
  statsOneObj.SubarrayIdxLengths[0] = 0.0;
  statsOneObj.WeightedCentroid[1] = 0.0;
  statsOneObj.SubarrayIdxLengths[1] = 0.0;
  loop_ub_tmp = static_cast<int>(numObjs);
  stats.set_size(loop_ub_tmp);
  for (int i{0}; i < loop_ub_tmp; i++) {
    stats[i] = statsOneObj;
  }
}

//
// Arguments    : const double varargin_1[32000]
//                ::coder::array<c_struct_T, 1U> &outstats
// Return Type  : void
//
void regionprops(const double varargin_1[32000],
                 ::coder::array<c_struct_T, 1U> &outstats)
{
  array<e_struct_T, 1U> stats;
  array<double, 1U> regionIndices;
  array<double, 1U> regionLengths;
  array<int, 1U> idxCount;
  array<int, 1U> v1;
  c_struct_T s;
  struct_T statsAlreadyComputed;
  double y[2];
  double bsum;
  double numObjs;
  int firstBlockLength;
  int hi;
  int i;
  int ib;
  int k;
  int lastBlockLength;
  int nblocks;
  int xblockoffset;
  numObjs = varargin_1[0];
  for (k = 0; k < 31999; k++) {
    bsum = varargin_1[k + 1];
    if (numObjs < bsum) {
      numObjs = bsum;
    }
  }
  numObjs = std::fmax(0.0, std::floor(numObjs));
  s.Centroid[0] = 0.0;
  s.Centroid[1] = 0.0;
  firstBlockLength = static_cast<int>(numObjs);
  outstats.set_size(firstBlockLength);
  for (i = 0; i < firstBlockLength; i++) {
    outstats[i] = s;
  }
  initializeStatsStruct(numObjs, stats, &statsAlreadyComputed);
  if (numObjs != 0.0) {
    regionLengths.set_size(firstBlockLength);
    for (i = 0; i < firstBlockLength; i++) {
      regionLengths[i] = 0.0;
    }
    for (lastBlockLength = 0; lastBlockLength < 160; lastBlockLength++) {
      for (firstBlockLength = 0; firstBlockLength < 200; firstBlockLength++) {
        bsum = varargin_1[firstBlockLength + 200 * lastBlockLength];
        if (static_cast<int>(bsum) > 0) {
          regionLengths[static_cast<int>(bsum) - 1] =
              regionLengths[static_cast<int>(bsum) - 1] + 1.0;
        }
      }
    }
    if (regionLengths.size(0) <= 1024) {
      firstBlockLength = regionLengths.size(0);
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = regionLengths.size(0) / 1024;
      lastBlockLength = regionLengths.size(0) - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    numObjs = regionLengths[0];
    for (k = 2; k <= firstBlockLength; k++) {
      numObjs += regionLengths[k - 1];
    }
    for (ib = 2; ib <= nblocks; ib++) {
      xblockoffset = (ib - 1) << 10;
      bsum = regionLengths[xblockoffset];
      if (ib == nblocks) {
        hi = lastBlockLength;
      } else {
        hi = 1024;
      }
      for (k = 2; k <= hi; k++) {
        bsum += regionLengths[(xblockoffset + k) - 1];
      }
      numObjs += bsum;
    }
    regionIndices.set_size(static_cast<int>(numObjs));
    if (regionLengths.size(0) != 1) {
      i = regionLengths.size(0);
      for (k = 0; k <= i - 2; k++) {
        regionLengths[k + 1] = regionLengths[k] + regionLengths[k + 1];
      }
    }
    v1.set_size(regionLengths.size(0) + 1);
    v1[0] = 0;
    firstBlockLength = regionLengths.size(0);
    for (i = 0; i < firstBlockLength; i++) {
      v1[i + 1] = static_cast<int>(regionLengths[i]);
    }
    idxCount.set_size(v1.size(0));
    firstBlockLength = v1.size(0);
    for (i = 0; i < firstBlockLength; i++) {
      idxCount[i] = v1[i];
    }
    lastBlockLength = 1;
    for (firstBlockLength = 0; firstBlockLength < 160; firstBlockLength++) {
      for (nblocks = 0; nblocks < 200; nblocks++) {
        bsum = varargin_1[nblocks + 200 * firstBlockLength];
        if (static_cast<int>(bsum) > 0) {
          idxCount[static_cast<int>(bsum) - 1] =
              idxCount[static_cast<int>(bsum) - 1] + 1;
          regionIndices[idxCount[static_cast<int>(bsum) - 1] - 1] =
              lastBlockLength;
        }
        lastBlockLength++;
      }
    }
    i = stats.size(0);
    for (k = 0; k < i; k++) {
      lastBlockLength = v1[k + 1];
      if (v1[k] + 1 > lastBlockLength) {
        nblocks = 0;
        lastBlockLength = 0;
      } else {
        nblocks = v1[k];
      }
      firstBlockLength = lastBlockLength - nblocks;
      stats[k].PixelIdxList.set_size(firstBlockLength);
      for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
           lastBlockLength++) {
        stats[k].PixelIdxList[lastBlockLength] =
            regionIndices[nblocks + lastBlockLength];
      }
    }
  }
  i = stats.size(0);
  for (k = 0; k < i; k++) {
    firstBlockLength = stats[k].PixelIdxList.size(0);
    if (stats[k].PixelIdxList.size(0) != 0) {
      v1.set_size(stats[k].PixelIdxList.size(0));
      for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
           lastBlockLength++) {
        v1[lastBlockLength] =
            static_cast<int>(stats[k].PixelIdxList[lastBlockLength]) - 1;
      }
      idxCount.set_size(v1.size(0));
      firstBlockLength = v1.size(0);
      for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
           lastBlockLength++) {
        idxCount[lastBlockLength] = v1[lastBlockLength] / 200;
      }
      firstBlockLength = v1.size(0);
      for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
           lastBlockLength++) {
        v1[lastBlockLength] =
            v1[lastBlockLength] - idxCount[lastBlockLength] * 200;
      }
      stats[k].PixelList.set_size(idxCount.size(0), 2);
      firstBlockLength = idxCount.size(0);
      for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
           lastBlockLength++) {
        stats[k].PixelList[lastBlockLength] = idxCount[lastBlockLength] + 1;
      }
      firstBlockLength = v1.size(0);
      for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
           lastBlockLength++) {
        stats[k].PixelList[lastBlockLength + stats[k].PixelList.size(0)] =
            v1[lastBlockLength] + 1;
      }
    } else {
      stats[k].PixelList.set_size(0, 2);
    }
  }
  i = stats.size(0);
  for (k = 0; k < i; k++) {
    if (stats[k].PixelList.size(0) == 0) {
      y[0] = 0.0;
      y[1] = 0.0;
    } else {
      if (stats[k].PixelList.size(0) <= 1024) {
        firstBlockLength = stats[k].PixelList.size(0);
        lastBlockLength = 0;
        nblocks = 1;
      } else {
        firstBlockLength = 1024;
        nblocks = stats[k].PixelList.size(0) / 1024;
        lastBlockLength = stats[k].PixelList.size(0) - (nblocks << 10);
        if (lastBlockLength > 0) {
          nblocks++;
        } else {
          lastBlockLength = 1024;
        }
      }
      for (int xi{0}; xi < 2; xi++) {
        int b_k;
        int xpageoffset;
        xpageoffset = xi * stats[k].PixelList.size(0);
        y[xi] = stats[k].PixelList[xpageoffset];
        for (b_k = 2; b_k <= firstBlockLength; b_k++) {
          y[xi] += stats[k].PixelList[(xpageoffset + b_k) - 1];
        }
        for (ib = 2; ib <= nblocks; ib++) {
          xblockoffset = xpageoffset + ((ib - 1) << 10);
          bsum = stats[k].PixelList[xblockoffset];
          if (ib == nblocks) {
            hi = lastBlockLength;
          } else {
            hi = 1024;
          }
          for (b_k = 2; b_k <= hi; b_k++) {
            bsum += stats[k].PixelList[(xblockoffset + b_k) - 1];
          }
          y[xi] += bsum;
        }
      }
    }
    stats[k].Centroid[0] =
        y[0] / static_cast<double>(stats[k].PixelList.size(0));
    stats[k].Centroid[1] =
        y[1] / static_cast<double>(stats[k].PixelList.size(0));
  }
  i = stats.size(0);
  for (k = 0; k < i; k++) {
    outstats[k].Centroid[0] = stats[k].Centroid[0];
    outstats[k].Centroid[1] = stats[k].Centroid[1];
  }
}

} // namespace coder

//
// File trailer for regionprops.cpp
//
// [EOF]
//
