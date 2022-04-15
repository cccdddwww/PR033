//
// File: adaptthresh.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

// Include Files
#include "adaptthresh.h"
#include "useConstantDim.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// Arguments    : const double b_I[32000]
//                double T[32000]
// Return Type  : void
//
namespace coder {
void localMeanThresh(const double b_I[32000], double T[32000])
{
  static double intA[40725];
  static double paddedImage[40320];
  static const unsigned char idxA[448]{
      1U,   1U,   1U,   1U,   1U,   1U,   1U,   1U,   1U,   1U,   1U,   1U,
      1U,   2U,   3U,   4U,   5U,   6U,   7U,   8U,   9U,   10U,  11U,  12U,
      13U,  14U,  15U,  16U,  17U,  18U,  19U,  20U,  21U,  22U,  23U,  24U,
      25U,  26U,  27U,  28U,  29U,  30U,  31U,  32U,  33U,  34U,  35U,  36U,
      37U,  38U,  39U,  40U,  41U,  42U,  43U,  44U,  45U,  46U,  47U,  48U,
      49U,  50U,  51U,  52U,  53U,  54U,  55U,  56U,  57U,  58U,  59U,  60U,
      61U,  62U,  63U,  64U,  65U,  66U,  67U,  68U,  69U,  70U,  71U,  72U,
      73U,  74U,  75U,  76U,  77U,  78U,  79U,  80U,  81U,  82U,  83U,  84U,
      85U,  86U,  87U,  88U,  89U,  90U,  91U,  92U,  93U,  94U,  95U,  96U,
      97U,  98U,  99U,  100U, 101U, 102U, 103U, 104U, 105U, 106U, 107U, 108U,
      109U, 110U, 111U, 112U, 113U, 114U, 115U, 116U, 117U, 118U, 119U, 120U,
      121U, 122U, 123U, 124U, 125U, 126U, 127U, 128U, 129U, 130U, 131U, 132U,
      133U, 134U, 135U, 136U, 137U, 138U, 139U, 140U, 141U, 142U, 143U, 144U,
      145U, 146U, 147U, 148U, 149U, 150U, 151U, 152U, 153U, 154U, 155U, 156U,
      157U, 158U, 159U, 160U, 161U, 162U, 163U, 164U, 165U, 166U, 167U, 168U,
      169U, 170U, 171U, 172U, 173U, 174U, 175U, 176U, 177U, 178U, 179U, 180U,
      181U, 182U, 183U, 184U, 185U, 186U, 187U, 188U, 189U, 190U, 191U, 192U,
      193U, 194U, 195U, 196U, 197U, 198U, 199U, 200U, 200U, 200U, 200U, 200U,
      200U, 200U, 200U, 200U, 200U, 200U, 200U, 200U, 1U,   1U,   1U,   1U,
      1U,   1U,   1U,   1U,   1U,   1U,   1U,   2U,   3U,   4U,   5U,   6U,
      7U,   8U,   9U,   10U,  11U,  12U,  13U,  14U,  15U,  16U,  17U,  18U,
      19U,  20U,  21U,  22U,  23U,  24U,  25U,  26U,  27U,  28U,  29U,  30U,
      31U,  32U,  33U,  34U,  35U,  36U,  37U,  38U,  39U,  40U,  41U,  42U,
      43U,  44U,  45U,  46U,  47U,  48U,  49U,  50U,  51U,  52U,  53U,  54U,
      55U,  56U,  57U,  58U,  59U,  60U,  61U,  62U,  63U,  64U,  65U,  66U,
      67U,  68U,  69U,  70U,  71U,  72U,  73U,  74U,  75U,  76U,  77U,  78U,
      79U,  80U,  81U,  82U,  83U,  84U,  85U,  86U,  87U,  88U,  89U,  90U,
      91U,  92U,  93U,  94U,  95U,  96U,  97U,  98U,  99U,  100U, 101U, 102U,
      103U, 104U, 105U, 106U, 107U, 108U, 109U, 110U, 111U, 112U, 113U, 114U,
      115U, 116U, 117U, 118U, 119U, 120U, 121U, 122U, 123U, 124U, 125U, 126U,
      127U, 128U, 129U, 130U, 131U, 132U, 133U, 134U, 135U, 136U, 137U, 138U,
      139U, 140U, 141U, 142U, 143U, 144U, 145U, 146U, 147U, 148U, 149U, 150U,
      151U, 152U, 153U, 154U, 155U, 156U, 157U, 158U, 159U, 160U, 160U, 160U,
      160U, 160U, 160U, 160U, 160U, 160U, 160U, 160U, 0U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U};
  int i;
  int j;
  for (j = 0; j < 180; j++) {
    for (i = 0; i < 224; i++) {
      paddedImage[i + 224 * j] = b_I[(idxA[i] + 200 * (idxA[j + 224] - 1)) - 1];
    }
  }
  std::memset(&intA[0], 0, 40725U * sizeof(double));
  internal::useConstantDim(paddedImage, 1);
  internal::useConstantDim(paddedImage, 2);
  for (j = 0; j < 180; j++) {
    std::copy(&paddedImage[j * 224],
              &paddedImage[static_cast<int>(j * 224 + 224U)],
              &intA[j * 225 + 226]);
  }
  for (j = 0; j < 160; j++) {
    for (i = 0; i < 200; i++) {
      int T_tmp;
      int b_T_tmp;
      T_tmp = i + 225 * (j + 21);
      b_T_tmp = i + 225 * j;
      T[i + 200 * j] = 0.0020952380952380953 *
                       (((intA[T_tmp + 25] + intA[b_T_tmp]) - intA[T_tmp]) -
                        intA[b_T_tmp + 25]);
    }
  }
}

} // namespace coder

//
// File trailer for adaptthresh.cpp
//
// [EOF]
//
