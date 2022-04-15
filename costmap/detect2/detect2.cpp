//
// File: detect2.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

// Include Files
#include "detect2.h"
#include "bwconncomp.h"
#include "bwlabel.h"
#include "detect2_internal_types.h"
#include "imbinarize.h"
#include "imfilter.h"
#include "imreconstruct.h"
#include "inPainting.h"
#include "regionprops.h"
#include "coder_array.h"
#include <cmath>

// Type Definitions
struct b_struct_T {
  double Area;
};

// Function Definitions
//
// Arguments    : const unsigned char img[32000]
//                coder::array<double, 2U> &Result
//                double *t
// Return Type  : void
//
void detect2(const unsigned char img[32000], coder::array<double, 2U> &Result,
             double *t)
{
  static double L[32000];
  static bool BW[32000];
  coder::array<b_struct_T, 1U> Area1;
  coder::array<c_struct_T, 1U> centroid;
  coder::array<d_struct_T, 1U> Box1;
  coder::array<e_struct_T, 1U> stats;
  coder::array<double, 2U> filter_box;
  coder::array<double, 1U> CC_RegionIndices;
  coder::array<int, 1U> regionLengths;
  struct_T statsAlreadyComputed;
  double b_filter_box[5];
  double expl_temp[2];
  double bsum;
  double lx;
  double ly;
  double nz;
  int b_i;
  int hi;
  int i;
  int i1;
  unsigned int j;
  int xblockoffset;
  unsigned char g2[32000];
  unsigned char uv[32000];
  bool marker[32000];
  bool mask[32000];
  bool similarValuesImage[32000];
  for (i = 0; i < 32000; i++) {
    BW[i] = (img[i] != 0);
  }
  inPainting(img, BW, uv);
  coder::imfilter(uv, g2);
  coder::imbinarize(g2, BW);
  for (i = 0; i < 32000; i++) {
    BW[i] = (BW[i] * (g2[i] > 10) != 0);
  }
  coder::bwlabel(BW, L, &bsum);
  hi = static_cast<int>(bsum);
  regionLengths.set_size(hi);
  for (i = 0; i < hi; i++) {
    regionLengths[i] = 0;
  }
  for (hi = 0; hi < 32000; hi++) {
    i = static_cast<int>(L[hi]);
    if (i > 0) {
      regionLengths[i - 1] = regionLengths[i - 1] + 1;
    }
  }
  for (hi = 0; hi < 160; hi++) {
    for (xblockoffset = 0; xblockoffset < 200; xblockoffset++) {
      i = xblockoffset + 200 * hi;
      if (BW[i] && (regionLengths[static_cast<int>(L[i]) - 1] < 15)) {
        BW[i] = false;
      }
    }
  }
  coder::bwlabel(BW, L, &bsum);
  coder::regionprops(L, centroid);
  i = static_cast<int>(bsum);
  filter_box.set_size(i, 5);
  xblockoffset = static_cast<int>(bsum) * 5;
  for (i1 = 0; i1 < xblockoffset; i1++) {
    filter_box[i1] = 0.0;
  }
  j = 0U;
  for (b_i = 0; b_i < i; b_i++) {
    int ib;
    int k;
    unsigned int qY;
    short seed_indices_data;
    bsum = std::floor(centroid[b_i].Centroid[1]);
    nz = std::floor(centroid[b_i].Centroid[0]);
    hi = g2[(static_cast<int>(bsum) + 200 * (static_cast<int>(nz) - 1)) - 1];
    qY = hi - 5U;
    if (hi - 5U > static_cast<unsigned int>(hi)) {
      qY = 0U;
    }
    i1 = static_cast<int>(hi + 5U);
    if (hi + 5U > 255U) {
      i1 = 255;
    }
    seed_indices_data = static_cast<short>(
        static_cast<unsigned char>(bsum) +
        static_cast<short>(200 * (static_cast<unsigned char>(nz) - 1)));
    for (ib = 0; ib < 32000; ib++) {
      unsigned char u;
      bool b;
      u = g2[ib];
      b = ((u >= static_cast<unsigned char>(qY)) &&
           (u <= static_cast<unsigned char>(i1)));
      similarValuesImage[ib] = b;
      b = !b;
      BW[ib] = b;
      mask[ib] = !b;
      marker[ib] = false;
    }
    marker[seed_indices_data - 1] = mask[seed_indices_data - 1];
    coder::imreconstruct(marker, mask);
    for (i1 = 0; i1 < 32000; i1++) {
      BW[i1] = ((BW[i1] || marker[i1]) && similarValuesImage[i1]);
    }
    nz = BW[0];
    for (k = 0; k < 1023; k++) {
      nz += static_cast<double>(BW[k + 1]);
    }
    for (ib = 0; ib < 31; ib++) {
      xblockoffset = (ib + 1) << 10;
      bsum = BW[xblockoffset];
      if (ib + 2 == 32) {
        hi = 256;
      } else {
        hi = 1024;
      }
      for (k = 2; k <= hi; k++) {
        bsum += static_cast<double>(BW[(xblockoffset + k) - 1]);
      }
      nz += bsum;
    }
    if (!(nz / 200.0 / 160.0 > 0.125)) {
      double rx;
      double ry;
      int lastBlockLength;
      int nblocks;
      coder::b_regionprops(BW, Box1);
      coder::bwconncomp(BW, &bsum, expl_temp, &nz, CC_RegionIndices,
                        regionLengths);
      hi = static_cast<int>(nz);
      Area1.set_size(hi);
      for (i1 = 0; i1 < hi; i1++) {
        Area1[i1].Area = 0.0;
      }
      coder::initializeStatsStruct(nz, stats, &statsAlreadyComputed);
      coder::ComputePixelIdxList(CC_RegionIndices, regionLengths, nz, stats,
                                 &statsAlreadyComputed);
      i1 = stats.size(0);
      for (k = 0; k < i1; k++) {
        stats[k].Area = stats[k].PixelIdxList.size(0);
      }
      i1 = stats.size(0);
      for (k = 0; k < i1; k++) {
        Area1[k].Area = stats[k].Area;
      }
      lx = std::fmax(std::floor(Box1[0].BoundingBox[0]), 1.0);
      ly = std::fmax(std::floor(Box1[0].BoundingBox[1]), 1.0);
      rx = lx + Box1[0].BoundingBox[2];
      ry = ly + Box1[0].BoundingBox[3];
      if (ly > ry) {
        i1 = 0;
        ib = 0;
      } else {
        i1 = static_cast<int>(ly) - 1;
        ib = static_cast<int>(ry);
      }
      if (lx > rx) {
        hi = 0;
        nblocks = 0;
      } else {
        hi = static_cast<int>(lx) - 1;
        nblocks = static_cast<int>(rx);
      }
      xblockoffset = ib - i1;
      lastBlockLength = nblocks - hi;
      for (ib = 0; ib < lastBlockLength; ib++) {
        for (nblocks = 0; nblocks < xblockoffset; nblocks++) {
          BW[nblocks + xblockoffset * ib] =
              (img[(i1 + nblocks) + 200 * (hi + ib)] != 0);
        }
      }
      i1 = xblockoffset * lastBlockLength;
      if (i1 == 0) {
        nz = 0.0;
      } else {
        if (i1 <= 1024) {
          hi = i1;
          lastBlockLength = 0;
          nblocks = 1;
        } else {
          hi = 1024;
          nblocks = i1 / 1024;
          lastBlockLength = i1 - (nblocks << 10);
          if (lastBlockLength > 0) {
            nblocks++;
          } else {
            lastBlockLength = 1024;
          }
        }
        nz = BW[0];
        for (k = 2; k <= hi; k++) {
          nz += static_cast<double>(BW[k - 1]);
        }
        for (ib = 2; ib <= nblocks; ib++) {
          xblockoffset = (ib - 1) << 10;
          bsum = BW[xblockoffset];
          if (ib == nblocks) {
            hi = lastBlockLength;
          } else {
            hi = 1024;
          }
          for (k = 2; k <= hi; k++) {
            bsum += static_cast<double>(BW[(xblockoffset + k) - 1]);
          }
          nz += bsum;
        }
      }
      if ((!(nz / Area1[0].Area < 0.3)) && (!(Area1[0].Area < 15.0)) &&
          (!(lx < 5.0)) && (!(rx > 155.0)) && (!(ly < 5.0)) &&
          (!(ry > 195.0))) {
        j++;
        filter_box[static_cast<int>(j) - 1] = lx;
        filter_box[(static_cast<int>(j) + filter_box.size(0)) - 1] = ly;
        filter_box[(static_cast<int>(j) + filter_box.size(0) * 2) - 1] =
            rx - lx;
        filter_box[(static_cast<int>(j) + filter_box.size(0) * 3) - 1] =
            ry - ly;
        filter_box[(static_cast<int>(j) + filter_box.size(0) * 4) - 1] =
            Area1[0].Area;
      }
    }
  }
  b_i = 1;
  int exitg1;
  do {
    exitg1 = 0;
    if (b_i <= static_cast<int>(j) - 1) {
      bsum = filter_box[b_i - 1];
      ly = std::fmax(bsum, filter_box[b_i]);
      nz =
          std::fmin(bsum + filter_box[(b_i + filter_box.size(0) * 2) - 1],
                    filter_box[b_i] + filter_box[b_i + filter_box.size(0) * 2]);
      bsum = filter_box[(b_i + filter_box.size(0)) - 1];
      lx = std::fmax(bsum, filter_box[b_i + filter_box.size(0)]);
      bsum = std::fmin(bsum + filter_box[(b_i + filter_box.size(0) * 3) - 1],
                       filter_box[b_i + filter_box.size(0)] +
                           filter_box[b_i + filter_box.size(0) * 3]);
      if ((nz >= ly) && (bsum >= lx)) {
        bsum = (nz - ly) * (bsum - lx);
      } else {
        bsum = 0.0;
      }
      if (bsum != 0.0) {
        if (filter_box[(b_i + filter_box.size(0) * 4) - 1] >=
            filter_box[b_i + filter_box.size(0) * 4]) {
          for (i = 0; i < 5; i++) {
            b_filter_box[i] = filter_box[b_i + filter_box.size(0) * i];
          }
          for (i = 0; i < 5; i++) {
            filter_box[(b_i + filter_box.size(0) * i) - 1] = b_filter_box[i];
          }
          if (static_cast<int>(j) == b_i + 1) {
            for (i = 0; i < 5; i++) {
              filter_box[b_i + filter_box.size(0) * i] = 0.0;
            }
            exitg1 = 1;
          } else {
            for (i = 0; i < 5; i++) {
              b_filter_box[i] =
                  filter_box[(static_cast<int>(j) + filter_box.size(0) * i) -
                             1];
            }
            for (i = 0; i < 5; i++) {
              filter_box[b_i + filter_box.size(0) * i] = b_filter_box[i];
            }
            for (i = 0; i < 5; i++) {
              filter_box[(static_cast<int>(j) + filter_box.size(0) * i) - 1] =
                  0.0;
            }
            j = static_cast<unsigned int>(static_cast<int>(j) - 1);
          }
        } else {
          for (i = 0; i < 5; i++) {
            b_filter_box[i] =
                filter_box[(static_cast<int>(j) + filter_box.size(0) * i) - 1];
          }
          for (i = 0; i < 5; i++) {
            filter_box[b_i + filter_box.size(0) * i] = b_filter_box[i];
          }
          for (i = 0; i < 5; i++) {
            filter_box[(static_cast<int>(j) + filter_box.size(0) * i) - 1] =
                0.0;
          }
          j = static_cast<unsigned int>(static_cast<int>(j) - 1);
        }
      } else {
        b_i++;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  if (1 > static_cast<int>(j)) {
    xblockoffset = 0;
  } else {
    xblockoffset = static_cast<int>(j);
  }
  Result.set_size(xblockoffset, 4);
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < xblockoffset; i1++) {
      Result[i1 + Result.size(0) * i] = filter_box[i1 + filter_box.size(0) * i];
    }
  }
  *t = static_cast<double>(j) + 1.0;
}

//
// File trailer for detect2.cpp
//
// [EOF]
//
