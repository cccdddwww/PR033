//
// File: inPainting.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

// Include Files
#include "inPainting.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const unsigned char b_I[32000]
//                const bool mask[32000]
//                unsigned char R[32000]
// Return Type  : void
//
void inPainting(const unsigned char b_I[32000], const bool mask[32000],
                unsigned char R[32000])
{
  static double cols[288000];
  static double padR[32724];
  static double b_R[32000];
  static short b_tmp_data[32000];
  static short b_unknown_data[32000];
  static short tmp_data[32000];
  static short unknown_data[32000];
  static const unsigned char idxA[404]{
      1U,   1U,   2U,   3U,   4U,   5U,   6U,   7U,   8U,   9U,   10U,  11U,
      12U,  13U,  14U,  15U,  16U,  17U,  18U,  19U,  20U,  21U,  22U,  23U,
      24U,  25U,  26U,  27U,  28U,  29U,  30U,  31U,  32U,  33U,  34U,  35U,
      36U,  37U,  38U,  39U,  40U,  41U,  42U,  43U,  44U,  45U,  46U,  47U,
      48U,  49U,  50U,  51U,  52U,  53U,  54U,  55U,  56U,  57U,  58U,  59U,
      60U,  61U,  62U,  63U,  64U,  65U,  66U,  67U,  68U,  69U,  70U,  71U,
      72U,  73U,  74U,  75U,  76U,  77U,  78U,  79U,  80U,  81U,  82U,  83U,
      84U,  85U,  86U,  87U,  88U,  89U,  90U,  91U,  92U,  93U,  94U,  95U,
      96U,  97U,  98U,  99U,  100U, 101U, 102U, 103U, 104U, 105U, 106U, 107U,
      108U, 109U, 110U, 111U, 112U, 113U, 114U, 115U, 116U, 117U, 118U, 119U,
      120U, 121U, 122U, 123U, 124U, 125U, 126U, 127U, 128U, 129U, 130U, 131U,
      132U, 133U, 134U, 135U, 136U, 137U, 138U, 139U, 140U, 141U, 142U, 143U,
      144U, 145U, 146U, 147U, 148U, 149U, 150U, 151U, 152U, 153U, 154U, 155U,
      156U, 157U, 158U, 159U, 160U, 161U, 162U, 163U, 164U, 165U, 166U, 167U,
      168U, 169U, 170U, 171U, 172U, 173U, 174U, 175U, 176U, 177U, 178U, 179U,
      180U, 181U, 182U, 183U, 184U, 185U, 186U, 187U, 188U, 189U, 190U, 191U,
      192U, 193U, 194U, 195U, 196U, 197U, 198U, 199U, 200U, 200U, 1U,   1U,
      2U,   3U,   4U,   5U,   6U,   7U,   8U,   9U,   10U,  11U,  12U,  13U,
      14U,  15U,  16U,  17U,  18U,  19U,  20U,  21U,  22U,  23U,  24U,  25U,
      26U,  27U,  28U,  29U,  30U,  31U,  32U,  33U,  34U,  35U,  36U,  37U,
      38U,  39U,  40U,  41U,  42U,  43U,  44U,  45U,  46U,  47U,  48U,  49U,
      50U,  51U,  52U,  53U,  54U,  55U,  56U,  57U,  58U,  59U,  60U,  61U,
      62U,  63U,  64U,  65U,  66U,  67U,  68U,  69U,  70U,  71U,  72U,  73U,
      74U,  75U,  76U,  77U,  78U,  79U,  80U,  81U,  82U,  83U,  84U,  85U,
      86U,  87U,  88U,  89U,  90U,  91U,  92U,  93U,  94U,  95U,  96U,  97U,
      98U,  99U,  100U, 101U, 102U, 103U, 104U, 105U, 106U, 107U, 108U, 109U,
      110U, 111U, 112U, 113U, 114U, 115U, 116U, 117U, 118U, 119U, 120U, 121U,
      122U, 123U, 124U, 125U, 126U, 127U, 128U, 129U, 130U, 131U, 132U, 133U,
      134U, 135U, 136U, 137U, 138U, 139U, 140U, 141U, 142U, 143U, 144U, 145U,
      146U, 147U, 148U, 149U, 150U, 151U, 152U, 153U, 154U, 155U, 156U, 157U,
      158U, 159U, 160U, 160U, 0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U};
  coder::array<double, 2U> nb;
  coder::array<double, 2U> nbSum;
  coder::array<double, 2U> r;
  coder::array<int, 2U> nzs;
  coder::array<bool, 2U> b_nb;
  double colImageColIdx;
  int cols_tmp;
  int idx;
  int npages;
  int trueCount;
  bool exitg1;
  for (cols_tmp = 0; cols_tmp < 32000; cols_tmp++) {
    b_R[cols_tmp] = b_I[cols_tmp];
  }
  // // For precision
  // // Change - column major indices
  idx = 0;
  npages = 0;
  exitg1 = false;
  while ((!exitg1) && (npages < 32000)) {
    if (!mask[npages]) {
      idx++;
      unknown_data[idx - 1] = static_cast<short>(npages + 1);
      if (idx >= 32000) {
        exitg1 = true;
      } else {
        npages++;
      }
    } else {
      npages++;
    }
  }
  if (1 > idx) {
    trueCount = 0;
  } else {
    trueCount = idx;
  }
  // Find zeros in mask (area to be inpainted)
  // // Until we have searched all unknown pixels
  while (trueCount != 0) {
    int colImageRowIdx;
    int i;
    int kPixelColIdx;
    // // Change - take image at current iteration and
    // // create columns of pixel neighbourhoods
    for (idx = 0; idx < 162; idx++) {
      for (i = 0; i < 202; i++) {
        padR[i + 202 * idx] = b_R[(idxA[i] + 200 * (idxA[idx + 202] - 1)) - 1];
      }
    }
    // cols = im2col(padR, [2*n+1 2*n+1], 'sliding');
    //  -----------------------------------------------------------------------------------------------
    //  % [ mColumnImage ] = ImageToColumns( mInputImage, blockRadius )
    //    Creates an column image from the sliding neighborhood in mInpuImage
    //  Input:
    //    - mInputImage           -   Input image.
    //                                Matrix, 1 Channels, Floating Point
    //    - blockRadius        -      Local Window Radius.
    //                                Scalar, Floating Point, {1, 2, ..., inf}.
    //  Output:
    //    - mColumnImage          -   Input image.
    //                                Matrix, 1 Channels, Floating Point, [0, 1]
    //  Remarks:
    //    1.  Prefixes:
    //        -   'm' - Matrix.
    //        -   'v' - Vector.
    //    2.  C
    //  TODO:
    //    1.  I
    //    Release Notes:
    //    -   1.0.000     22/08/2014  R
    //        *   First release version.
    //  -----------------------------------------------------------------------------------------------
    //  %
    std::memset(&cols[0], 0, 288000U * sizeof(double));
    colImageColIdx = 0.0;
    for (idx = 0; idx < 160; idx++) {
      for (npages = 0; npages < 200; npages++) {
        colImageColIdx++;
        colImageRowIdx = -1;
        for (kPixelColIdx = 0; kPixelColIdx < 3; kPixelColIdx++) {
          i = npages + 202 * (idx + kPixelColIdx);
          cols_tmp =
              colImageRowIdx + 9 * (static_cast<int>(colImageColIdx) - 1);
          cols[cols_tmp + 1] = padR[i];
          cols[cols_tmp + 2] = padR[i + 1];
          cols[cols_tmp + 3] = padR[i + 2];
          colImageRowIdx += 3;
        }
      }
    }
    // // Change - Access the right pixel neighbourhoods
    // // denoted by unknown
    nb.set_size(9, trueCount);
    for (cols_tmp = 0; cols_tmp < trueCount; cols_tmp++) {
      for (idx = 0; idx < 9; idx++) {
        nb[idx + 9 * cols_tmp] = cols[idx + 9 * (unknown_data[cols_tmp] - 1)];
      }
    }
    // // Get total sum of each neighbourhood
    if (nb.size(1) == 0) {
      nbSum.set_size(1, 0);
    } else {
      npages = nb.size(1);
      nbSum.set_size(1, static_cast<int>(static_cast<short>(nb.size(1))));
      for (idx = 0; idx < npages; idx++) {
        colImageRowIdx = idx * 9;
        colImageColIdx = nb[colImageRowIdx];
        for (kPixelColIdx = 0; kPixelColIdx < 8; kPixelColIdx++) {
          colImageColIdx += nb[(colImageRowIdx + kPixelColIdx) + 1];
        }
        nbSum[idx] = colImageColIdx;
      }
    }
    // // Get total number of non-zero elements per pixel neighbourhood
    b_nb.set_size(9, nb.size(1));
    idx = 9 * nb.size(1);
    for (cols_tmp = 0; cols_tmp < idx; cols_tmp++) {
      b_nb[cols_tmp] = (nb[cols_tmp] != 0.0);
    }
    if (b_nb.size(1) == 0) {
      nzs.set_size(1, 0);
    } else {
      npages = b_nb.size(1);
      nzs.set_size(1, static_cast<int>(static_cast<short>(b_nb.size(1))));
      for (i = 0; i < npages; i++) {
        colImageRowIdx = i * 9;
        cols_tmp = b_nb[colImageRowIdx];
        for (kPixelColIdx = 0; kPixelColIdx < 8; kPixelColIdx++) {
          cols_tmp += b_nb[(colImageRowIdx + kPixelColIdx) + 1];
        }
        nzs[i] = cols_tmp;
      }
    }
    // // Replace the right pixels in the image with the mean
    idx = nzs.size(1) - 1;
    trueCount = 0;
    for (i = 0; i <= idx; i++) {
      if (nzs[i] != 0) {
        trueCount++;
      }
    }
    npages = 0;
    for (i = 0; i <= idx; i++) {
      if (nzs[i] != 0) {
        tmp_data[npages] = static_cast<short>(i + 1);
        npages++;
      }
    }
    r.set_size(1, trueCount);
    for (cols_tmp = 0; cols_tmp < trueCount; cols_tmp++) {
      idx = tmp_data[cols_tmp] - 1;
      r[cols_tmp] = nbSum[idx] / static_cast<double>(nzs[idx]);
    }
    idx = nzs.size(1);
    npages = 0;
    // // Find new unknown pixels to look at
    colImageRowIdx = nzs.size(1) - 1;
    trueCount = 0;
    for (i = 0; i < idx; i++) {
      cols_tmp = nzs[i];
      if (cols_tmp != 0) {
        b_R[unknown_data[i] - 1] = r[npages];
        npages++;
      }
      if (cols_tmp == 0) {
        trueCount++;
      }
    }
    npages = 0;
    for (i = 0; i <= colImageRowIdx; i++) {
      if (nzs[i] == 0) {
        b_tmp_data[npages] = static_cast<short>(i + 1);
        npages++;
      }
    }
    for (cols_tmp = 0; cols_tmp < trueCount; cols_tmp++) {
      b_unknown_data[cols_tmp] = unknown_data[b_tmp_data[cols_tmp] - 1];
    }
    if (0 <= trueCount - 1) {
      std::copy(&b_unknown_data[0], &b_unknown_data[trueCount],
                &unknown_data[0]);
    }
    // // Update image for next iteration
  }
  // // Cast back to the right type
  for (cols_tmp = 0; cols_tmp < 32000; cols_tmp++) {
    unsigned char u;
    colImageColIdx = std::round(b_R[cols_tmp]);
    if (colImageColIdx < 256.0) {
      if (colImageColIdx >= 0.0) {
        u = static_cast<unsigned char>(colImageColIdx);
      } else {
        u = 0U;
      }
    } else if (colImageColIdx >= 256.0) {
      u = MAX_uint8_T;
    } else {
      u = 0U;
    }
    R[cols_tmp] = u;
  }
}

//
// File trailer for inPainting.cpp
//
// [EOF]
//
