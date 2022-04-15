//
// File: bwlabel.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

// Include Files
#include "bwlabel.h"
#include <cmath>

// Function Declarations
static int mul_s32_sat(int a);

static void mul_wide_s32(int in0, unsigned int *ptrOutBitsHi,
                         unsigned int *ptrOutBitsLo);

// Function Definitions
//
// Arguments    : int a
// Return Type  : int
//
static int mul_s32_sat(int a)
{
  int result;
  unsigned int u32_chi;
  unsigned int u32_clo;
  mul_wide_s32(a, &u32_chi, &u32_clo);
  if ((static_cast<int>(u32_chi) > 0) ||
      ((u32_chi == 0U) && (u32_clo >= 2147483648U))) {
    result = MAX_int32_T;
  } else if ((static_cast<int>(u32_chi) < -1) ||
             ((static_cast<int>(u32_chi) == -1) && (u32_clo < 2147483648U))) {
    result = MIN_int32_T;
  } else {
    result = static_cast<int>(u32_clo);
  }
  return result;
}

//
// Arguments    : int in0
//                unsigned int *ptrOutBitsHi
//                unsigned int *ptrOutBitsLo
// Return Type  : void
//
static void mul_wide_s32(int in0, unsigned int *ptrOutBitsHi,
                         unsigned int *ptrOutBitsLo)
{
  unsigned int absIn0;
  int carry;
  unsigned int outBitsLo;
  unsigned int productHiLo;
  if (in0 < 0) {
    absIn0 = ~static_cast<unsigned int>(in0) + 1U;
  } else {
    absIn0 = static_cast<unsigned int>(in0);
  }
  productHiLo = (absIn0 >> 16U) * 100U;
  absIn0 = (absIn0 & 65535U) * 100U;
  carry = 0;
  outBitsLo = absIn0 + (productHiLo << 16U);
  if (outBitsLo < absIn0) {
    carry = 1;
  }
  absIn0 = carry + (productHiLo >> 16U);
  if (static_cast<int>((in0 != 0) && (in0 <= 0))) {
    absIn0 = ~absIn0;
    outBitsLo = ~outBitsLo;
    outBitsLo++;
    if (outBitsLo == 0U) {
      absIn0++;
    }
  }
  *ptrOutBitsHi = absIn0;
  *ptrOutBitsLo = outBitsLo;
}

//
// Arguments    : const bool varargin_1[32000]
//                double L[32000]
//                double *numComponents
// Return Type  : void
//
namespace coder {
void bwlabel(const bool varargin_1[32000], double L[32000],
             double *numComponents)
{
  static double P[16001];
  double d;
  double label;
  double rootj;
  int chunksSizeAndLabels[168];
  int b_c;
  int c;
  int exitg1;
  int i;
  int i2;
  int r;
  int stripeFirstLabel;
  P[0] = 0.0;
  for (int thread{0}; thread < 8; thread++) {
    int firstLabel;
    int startC;
    c = thread * 20 + 1;
    stripeFirstLabel = (thread + 1) * 20;
    chunksSizeAndLabels[c - 1] = stripeFirstLabel + 1;
    label = std::floor(static_cast<double>(c) / 2.0) * 100.0 + 1.0;
    firstLabel = static_cast<int>(label);
    startC = c;
    i = thread * 20;
    stripeFirstLabel -= i;
    for (b_c = 0; b_c < stripeFirstLabel; b_c++) {
      c = i + b_c;
      for (r = 0; r < 200; r++) {
        int i1;
        i1 = r + 200 * c;
        if (varargin_1[i1]) {
          int i3;
          bool guard1{false};
          bool guard2{false};
          bool guard3{false};
          bool guard4{false};
          guard1 = false;
          guard2 = false;
          guard3 = false;
          guard4 = false;
          if (c + 1 > startC) {
            i2 = r + 200 * (c - 1);
            if (varargin_1[i2]) {
              L[i1] = L[i2];
            } else {
              guard4 = true;
            }
          } else {
            guard4 = true;
          }
          if (guard4) {
            if ((r + 1 < 200) && (c + 1 > startC)) {
              i2 = (r + 200 * (c - 1)) + 1;
              if (varargin_1[i2]) {
                if ((c + 1 > startC) && (r + 1 > 1)) {
                  i3 = (r + 200 * (c - 1)) - 1;
                  if (varargin_1[i3]) {
                    L[i1] = L[i3];
                    do {
                      exitg1 = 0;
                      d = L[i1];
                      rootj = P[static_cast<int>(d + 1.0) - 1];
                      if (rootj < d) {
                        L[i1] = rootj;
                      } else {
                        exitg1 = 1;
                      }
                    } while (exitg1 == 0);
                    if (L[i3] != L[i2]) {
                      rootj = L[i2];
                      while (P[static_cast<int>(rootj + 1.0) - 1] < rootj) {
                        rootj = P[static_cast<int>(rootj + 1.0) - 1];
                      }
                      if (d > rootj) {
                        L[i1] = rootj;
                      }
                      do {
                        exitg1 = 0;
                        d = L[i2];
                        rootj = P[static_cast<int>(d + 1.0) - 1];
                        if (rootj < L[i2]) {
                          P[static_cast<int>(d + 1.0) - 1] = L[i1];
                          L[i2] = rootj;
                        } else {
                          exitg1 = 1;
                        }
                      } while (exitg1 == 0);
                      P[static_cast<int>(d + 1.0) - 1] = L[i1];
                    }
                    do {
                      exitg1 = 0;
                      i2 = static_cast<int>(L[i3] + 1.0) - 1;
                      d = P[i2];
                      if (d < L[i3]) {
                        P[i2] = L[i1];
                        L[i3] = d;
                      } else {
                        exitg1 = 1;
                      }
                    } while (exitg1 == 0);
                    P[i2] = L[i1];
                  } else {
                    guard3 = true;
                  }
                } else {
                  guard3 = true;
                }
              } else {
                guard2 = true;
              }
            } else {
              guard2 = true;
            }
          }
          if (guard3) {
            if (r + 1 > 1) {
              i3 = (r + 200 * c) - 1;
              if (varargin_1[i3]) {
                int i4;
                L[i1] = L[i3];
                do {
                  exitg1 = 0;
                  i4 = static_cast<int>(L[i1] + 1.0) - 1;
                  d = L[i1];
                  if (P[i4] < d) {
                    L[i1] = P[i4];
                  } else {
                    exitg1 = 1;
                  }
                } while (exitg1 == 0);
                if (L[i3] != L[i2]) {
                  rootj = L[i2];
                  while (P[static_cast<int>(rootj + 1.0) - 1] < rootj) {
                    rootj = P[static_cast<int>(rootj + 1.0) - 1];
                  }
                  if (d > rootj) {
                    L[i1] = rootj;
                  }
                  do {
                    exitg1 = 0;
                    i4 = static_cast<int>(L[i2] + 1.0) - 1;
                    d = P[i4];
                    if (d < L[i2]) {
                      P[i4] = L[i1];
                      L[i2] = d;
                    } else {
                      exitg1 = 1;
                    }
                  } while (exitg1 == 0);
                  P[i4] = L[i1];
                }
                do {
                  exitg1 = 0;
                  i2 = static_cast<int>(L[i3] + 1.0) - 1;
                  d = P[i2];
                  if (d < L[i3]) {
                    P[i2] = L[i1];
                    L[i3] = d;
                  } else {
                    exitg1 = 1;
                  }
                } while (exitg1 == 0);
                P[i2] = L[i1];
              } else {
                L[i1] = L[i2];
              }
            } else {
              L[i1] = L[i2];
            }
          }
          if (guard2) {
            if ((c + 1 > startC) && (r + 1 > 1)) {
              i2 = (r + 200 * (c - 1)) - 1;
              if (varargin_1[i2]) {
                L[i1] = L[i2];
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }
          }
          if (guard1) {
            if ((r + 1 > 1) && varargin_1[i1 - 1]) {
              L[i1] = L[i1 - 1];
            } else {
              L[i1] = label;
              P[static_cast<int>(label + 1.0) - 1] = label;
              label++;
            }
          }
        } else {
          L[i1] = 0.0;
        }
      }
    }
    d = label - static_cast<double>(firstLabel);
    if (d < 2.147483648E+9) {
      i = static_cast<int>(d);
    } else {
      i = MAX_int32_T;
    }
    chunksSizeAndLabels[startC] = i;
  }
  for (c = chunksSizeAndLabels[0] - 1; c + 1 <= 160;
       c = chunksSizeAndLabels[c] - 1) {
    for (r = 0; r < 200; r++) {
      i = r + 200 * c;
      d = L[i];
      if (d != 0.0) {
        double root;
        if (r + 1 > 1) {
          label = L[(r + 200 * (c - 1)) - 1];
          if (label != 0.0) {
            root = label;
            while (P[static_cast<int>(root + 1.0) - 1] < root) {
              root = P[static_cast<int>(root + 1.0) - 1];
            }
            if (label != d) {
              rootj = L[i];
              while (P[static_cast<int>(rootj + 1.0) - 1] < rootj) {
                rootj = P[static_cast<int>(rootj + 1.0) - 1];
              }
              if (root > rootj) {
                root = rootj;
              }
              rootj = L[i];
              do {
                exitg1 = 0;
                d = P[static_cast<int>(rootj + 1.0) - 1];
                if (d < rootj) {
                  P[static_cast<int>(rootj + 1.0) - 1] = root;
                  rootj = d;
                } else {
                  exitg1 = 1;
                }
              } while (exitg1 == 0);
              P[static_cast<int>(rootj + 1.0) - 1] = root;
            }
            do {
              exitg1 = 0;
              d = P[static_cast<int>(label + 1.0) - 1];
              if (d < label) {
                P[static_cast<int>(label + 1.0) - 1] = root;
                label = d;
              } else {
                exitg1 = 1;
              }
            } while (exitg1 == 0);
            P[static_cast<int>(label + 1.0) - 1] = root;
            L[i] = root;
          }
        }
        if (r + 1 < 200) {
          label = L[(r + 200 * (c - 1)) + 1];
          if (label != 0.0) {
            root = label;
            while (P[static_cast<int>(root + 1.0) - 1] < root) {
              root = P[static_cast<int>(root + 1.0) - 1];
            }
            if (label != L[i]) {
              rootj = L[i];
              while (P[static_cast<int>(rootj + 1.0) - 1] < rootj) {
                rootj = P[static_cast<int>(rootj + 1.0) - 1];
              }
              if (root > rootj) {
                root = rootj;
              }
              rootj = L[i];
              do {
                exitg1 = 0;
                d = P[static_cast<int>(rootj + 1.0) - 1];
                if (d < rootj) {
                  P[static_cast<int>(rootj + 1.0) - 1] = root;
                  rootj = d;
                } else {
                  exitg1 = 1;
                }
              } while (exitg1 == 0);
              P[static_cast<int>(rootj + 1.0) - 1] = root;
            }
            do {
              exitg1 = 0;
              d = P[static_cast<int>(label + 1.0) - 1];
              if (d < label) {
                P[static_cast<int>(label + 1.0) - 1] = root;
                label = d;
              } else {
                exitg1 = 1;
              }
            } while (exitg1 == 0);
            P[static_cast<int>(label + 1.0) - 1] = root;
            L[i] = root;
          }
        }
        label = L[r + 200 * (c - 1)];
        if (label != 0.0) {
          root = label;
          while (P[static_cast<int>(root + 1.0) - 1] < root) {
            root = P[static_cast<int>(root + 1.0) - 1];
          }
          if (label != L[i]) {
            rootj = L[i];
            while (P[static_cast<int>(rootj + 1.0) - 1] < rootj) {
              rootj = P[static_cast<int>(rootj + 1.0) - 1];
            }
            if (root > rootj) {
              root = rootj;
            }
            rootj = L[i];
            do {
              exitg1 = 0;
              d = P[static_cast<int>(rootj + 1.0) - 1];
              if (d < rootj) {
                P[static_cast<int>(rootj + 1.0) - 1] = root;
                rootj = d;
              } else {
                exitg1 = 1;
              }
            } while (exitg1 == 0);
            P[static_cast<int>(rootj + 1.0) - 1] = root;
          }
          do {
            exitg1 = 0;
            d = P[static_cast<int>(label + 1.0) - 1];
            if (d < label) {
              P[static_cast<int>(label + 1.0) - 1] = root;
              label = d;
            } else {
              exitg1 = 1;
            }
          } while (exitg1 == 0);
          P[static_cast<int>(label + 1.0) - 1] = root;
          L[i] = root;
        }
      }
    }
  }
  *numComponents = 1.0;
  c = 1;
  while (c <= 160) {
    if (c < -2147483647) {
      stripeFirstLabel = MIN_int32_T;
    } else {
      stripeFirstLabel = c - 1;
    }
    stripeFirstLabel = mul_s32_sat(static_cast<int>(
        std::round(static_cast<double>(stripeFirstLabel) / 2.0)));
    i = stripeFirstLabel + 2;
    b_c = chunksSizeAndLabels[c];
    if ((stripeFirstLabel + 1 < 0) && (b_c < MAX_int32_T - stripeFirstLabel)) {
      stripeFirstLabel = MIN_int32_T;
    } else if ((stripeFirstLabel + 1 > 0) &&
               (b_c > 2147483646 - stripeFirstLabel)) {
      stripeFirstLabel = MAX_int32_T;
    } else {
      stripeFirstLabel = (stripeFirstLabel + b_c) + 1;
    }
    for (b_c = i; b_c <= stripeFirstLabel; b_c++) {
      d = P[b_c - 1];
      if (d < static_cast<double>(b_c) - 1.0) {
        P[b_c - 1] = P[static_cast<int>(d + 1.0) - 1];
      } else {
        P[b_c - 1] = *numComponents;
        (*numComponents)++;
      }
    }
    c = chunksSizeAndLabels[c - 1];
  }
  (*numComponents)--;
  for (c = 0; c < 160; c++) {
    for (r = 0; r < 200; r++) {
      stripeFirstLabel = r + 200 * c;
      L[stripeFirstLabel] = P[static_cast<int>(L[stripeFirstLabel] + 1.0) - 1];
    }
  }
}

} // namespace coder

//
// File trailer for bwlabel.cpp
//
// [EOF]
//
