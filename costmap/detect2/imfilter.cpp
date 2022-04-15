//
// File: imfilter.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

// Include Files
#include "imfilter.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const unsigned char varargin_1[32000]
//                unsigned char b[32000]
// Return Type  : void
//
namespace coder {
void imfilter(const unsigned char varargin_1[32000], unsigned char b[32000])
{
  static const double c_b[100]{
      0.0088575840774764072, 0.00921906894115239,   0.0094998313892405289,
      0.0096917407133314259, 0.00978914432683684,   0.00978914432683684,
      0.0096917407133314259, 0.0094998313892405289, 0.00921906894115239,
      0.0088575840774764072, 0.00921906894115239,   0.00959530628197382,
      0.0098875268629325854, 0.010087268155114369,  0.010188646885496077,
      0.010188646885496077,  0.010087268155114369,  0.0098875268629325854,
      0.00959530628197382,   0.00921906894115239,   0.0094998313892405289,
      0.0098875268629325854, 0.010188646885496077,  0.01039447120564253,
      0.01049893737801081,   0.01049893737801081,   0.01039447120564253,
      0.010188646885496077,  0.0098875268629325854, 0.0094998313892405289,
      0.0096917407133314259, 0.010087268155114369,  0.01039447120564253,
      0.010604453452866036,  0.010711029981903623,  0.010711029981903623,
      0.010604453452866036,  0.01039447120564253,   0.010087268155114369,
      0.0096917407133314259, 0.00978914432683684,   0.010188646885496077,
      0.01049893737801081,   0.010711029981903623,  0.010818677622865288,
      0.010818677622865288,  0.010711029981903623,  0.01049893737801081,
      0.010188646885496077,  0.00978914432683684,   0.00978914432683684,
      0.010188646885496077,  0.01049893737801081,   0.010711029981903623,
      0.010818677622865288,  0.010818677622865288,  0.010711029981903623,
      0.01049893737801081,   0.010188646885496077,  0.00978914432683684,
      0.0096917407133314259, 0.010087268155114369,  0.01039447120564253,
      0.010604453452866036,  0.010711029981903623,  0.010711029981903623,
      0.010604453452866036,  0.01039447120564253,   0.010087268155114369,
      0.0096917407133314259, 0.0094998313892405289, 0.0098875268629325854,
      0.010188646885496077,  0.01039447120564253,   0.01049893737801081,
      0.01049893737801081,   0.01039447120564253,   0.010188646885496077,
      0.0098875268629325854, 0.0094998313892405289, 0.00921906894115239,
      0.00959530628197382,   0.0098875268629325854, 0.010087268155114369,
      0.010188646885496077,  0.010188646885496077,  0.010087268155114369,
      0.0098875268629325854, 0.00959530628197382,   0.00921906894115239,
      0.0088575840774764072, 0.00921906894115239,   0.0094998313892405289,
      0.0096917407133314259, 0.00978914432683684,   0.00978914432683684,
      0.0096917407133314259, 0.0094998313892405289, 0.00921906894115239,
      0.0088575840774764072};
  static double b_b[32000];
  static const unsigned char idxA[420]{
      1U,   1U,   1U,   1U,   1U,   1U,   2U,   3U,   4U,   5U,   6U,   7U,
      8U,   9U,   10U,  11U,  12U,  13U,  14U,  15U,  16U,  17U,  18U,  19U,
      20U,  21U,  22U,  23U,  24U,  25U,  26U,  27U,  28U,  29U,  30U,  31U,
      32U,  33U,  34U,  35U,  36U,  37U,  38U,  39U,  40U,  41U,  42U,  43U,
      44U,  45U,  46U,  47U,  48U,  49U,  50U,  51U,  52U,  53U,  54U,  55U,
      56U,  57U,  58U,  59U,  60U,  61U,  62U,  63U,  64U,  65U,  66U,  67U,
      68U,  69U,  70U,  71U,  72U,  73U,  74U,  75U,  76U,  77U,  78U,  79U,
      80U,  81U,  82U,  83U,  84U,  85U,  86U,  87U,  88U,  89U,  90U,  91U,
      92U,  93U,  94U,  95U,  96U,  97U,  98U,  99U,  100U, 101U, 102U, 103U,
      104U, 105U, 106U, 107U, 108U, 109U, 110U, 111U, 112U, 113U, 114U, 115U,
      116U, 117U, 118U, 119U, 120U, 121U, 122U, 123U, 124U, 125U, 126U, 127U,
      128U, 129U, 130U, 131U, 132U, 133U, 134U, 135U, 136U, 137U, 138U, 139U,
      140U, 141U, 142U, 143U, 144U, 145U, 146U, 147U, 148U, 149U, 150U, 151U,
      152U, 153U, 154U, 155U, 156U, 157U, 158U, 159U, 160U, 161U, 162U, 163U,
      164U, 165U, 166U, 167U, 168U, 169U, 170U, 171U, 172U, 173U, 174U, 175U,
      176U, 177U, 178U, 179U, 180U, 181U, 182U, 183U, 184U, 185U, 186U, 187U,
      188U, 189U, 190U, 191U, 192U, 193U, 194U, 195U, 196U, 197U, 198U, 199U,
      200U, 200U, 200U, 200U, 200U, 200U, 1U,   1U,   1U,   1U,   1U,   1U,
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
      158U, 159U, 160U, 160U, 160U, 160U, 160U, 160U, 0U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,
      0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U,   0U};
  static unsigned char a[35700];
  double cj[200];
  double bij;
  int i;
  int j;
  unsigned char b_a[35321];
  for (j = 0; j < 170; j++) {
    for (i = 0; i < 210; i++) {
      a[i + 210 * j] = varargin_1[(idxA[i] + 200 * (idxA[j + 210] - 1)) - 1];
    }
  }
  for (j = 0; j < 169; j++) {
    std::copy(&a[j * 210 + 211], &a[j * 210 + 420], &b_a[j * 209]);
  }
  for (j = 0; j < 160; j++) {
    std::memset(&cj[0], 0, 200U * sizeof(double));
    for (int jb{0}; jb < 10; jb++) {
      for (int ib{0}; ib < 10; ib++) {
        bij = c_b[(10 * (9 - jb) - ib) + 9];
        for (i = 0; i < 200; i++) {
          cj[i] += bij * static_cast<double>(b_a[(i + ib) + 209 * (j + jb)]);
        }
      }
    }
    std::copy(&cj[0], &cj[200], &b_b[j * 200]);
  }
  for (j = 0; j < 32000; j++) {
    unsigned char u;
    bij = std::round(b_b[j]);
    if (bij < 256.0) {
      u = static_cast<unsigned char>(bij);
    } else {
      u = MAX_uint8_T;
    }
    b[j] = u;
  }
}

} // namespace coder

//
// File trailer for imfilter.cpp
//
// [EOF]
//