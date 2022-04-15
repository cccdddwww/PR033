//
// File: bwconncomp.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 03-Apr-2022 14:20:30
//

// Include Files
#include "bwconncomp.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : const bool varargin_1[32000]
//                double *CC_Connectivity
//                double CC_ImageSize[2]
//                double *CC_NumObjects
//                ::coder::array<double, 1U> &CC_RegionIndices
//                ::coder::array<int, 1U> &CC_RegionLengths
// Return Type  : void
//
namespace coder {
void bwconncomp(const bool varargin_1[32000], double *CC_Connectivity,
                double CC_ImageSize[2], double *CC_NumObjects,
                ::coder::array<double, 1U> &CC_RegionIndices,
                ::coder::array<int, 1U> &CC_RegionLengths)
{
  array<int, 1U> idxCount;
  array<int, 1U> labelsRenumbered;
  array<int, 1U> x;
  array<short, 1U> pixelIdxList;
  array<unsigned char, 1U> endRow;
  array<unsigned char, 1U> startCol;
  array<unsigned char, 1U> startRow;
  double numComponents;
  int firstRunOnPreviousColumn;
  int firstRunOnThisColumn;
  int k;
  int numRuns;
  numRuns = 0;
  for (firstRunOnPreviousColumn = 0; firstRunOnPreviousColumn < 160;
       firstRunOnPreviousColumn++) {
    if (varargin_1[200 * firstRunOnPreviousColumn]) {
      numRuns++;
    }
    for (k = 0; k < 199; k++) {
      firstRunOnThisColumn = k + 200 * firstRunOnPreviousColumn;
      if (varargin_1[firstRunOnThisColumn + 1] &&
          (!varargin_1[firstRunOnThisColumn])) {
        numRuns++;
      }
    }
  }
  if (numRuns == 0) {
    CC_ImageSize[0] = 200.0;
    CC_ImageSize[1] = 160.0;
    numComponents = 0.0;
    CC_RegionIndices.set_size(0);
    CC_RegionLengths.set_size(1);
    CC_RegionLengths[0] = 0;
  } else {
    double y;
    int lastRunOnPreviousColumn;
    int row;
    int runCounter;
    startRow.set_size(numRuns);
    endRow.set_size(numRuns);
    startCol.set_size(numRuns);
    runCounter = 0;
    for (firstRunOnPreviousColumn = 0; firstRunOnPreviousColumn < 160;
         firstRunOnPreviousColumn++) {
      row = 1;
      while (row <= 200) {
        while ((row <= 200) &&
               (!varargin_1[(row + 200 * firstRunOnPreviousColumn) - 1])) {
          row++;
        }
        if ((row <= 200) &&
            varargin_1[(row + 200 * firstRunOnPreviousColumn) - 1]) {
          startCol[runCounter] =
              static_cast<unsigned char>(firstRunOnPreviousColumn + 1);
          startRow[runCounter] = static_cast<unsigned char>(row);
          while ((row <= 200) &&
                 varargin_1[(row + 200 * firstRunOnPreviousColumn) - 1]) {
            row++;
          }
          endRow[runCounter] = static_cast<unsigned char>(row - 1);
          runCounter++;
        }
      }
    }
    CC_RegionLengths.set_size(numRuns);
    for (firstRunOnThisColumn = 0; firstRunOnThisColumn < numRuns;
         firstRunOnThisColumn++) {
      CC_RegionLengths[firstRunOnThisColumn] = 0;
    }
    k = 0;
    runCounter = 1;
    row = 1;
    firstRunOnPreviousColumn = -1;
    lastRunOnPreviousColumn = -1;
    firstRunOnThisColumn = 0;
    while (k + 1 <= numRuns) {
      if (startCol[k] == runCounter + 1) {
        firstRunOnPreviousColumn = firstRunOnThisColumn + 1;
        firstRunOnThisColumn = k;
        lastRunOnPreviousColumn = k;
        runCounter = startCol[k];
      } else if (startCol[k] > runCounter + 1) {
        firstRunOnPreviousColumn = -1;
        lastRunOnPreviousColumn = -1;
        firstRunOnThisColumn = k;
        runCounter = startCol[k];
      }
      if (firstRunOnPreviousColumn >= 0) {
        for (int p{firstRunOnPreviousColumn - 1}; p < lastRunOnPreviousColumn;
             p++) {
          if ((endRow[k] >= startRow[p] - 1) &&
              (startRow[k] <= endRow[p] + 1)) {
            if (CC_RegionLengths[k] == 0) {
              CC_RegionLengths[k] = CC_RegionLengths[p];
              row++;
            } else if (CC_RegionLengths[k] != CC_RegionLengths[p]) {
              int root_k;
              int root_p;
              for (root_k = k; root_k + 1 != CC_RegionLengths[root_k];
                   root_k = CC_RegionLengths[root_k] - 1) {
                CC_RegionLengths[root_k] =
                    CC_RegionLengths[CC_RegionLengths[root_k] - 1];
              }
              for (root_p = p; root_p + 1 != CC_RegionLengths[root_p];
                   root_p = CC_RegionLengths[root_p] - 1) {
                CC_RegionLengths[root_p] =
                    CC_RegionLengths[CC_RegionLengths[root_p] - 1];
              }
              if (root_k + 1 != root_p + 1) {
                if (root_p + 1 < root_k + 1) {
                  CC_RegionLengths[root_k] = root_p + 1;
                  CC_RegionLengths[k] = root_p + 1;
                } else {
                  CC_RegionLengths[root_p] = root_k + 1;
                  CC_RegionLengths[p] = root_k + 1;
                }
              }
            }
          }
        }
      }
      if (CC_RegionLengths[k] == 0) {
        CC_RegionLengths[k] = row;
        row++;
      }
      k++;
    }
    labelsRenumbered.set_size(CC_RegionLengths.size(0));
    numComponents = 0.0;
    for (k = 0; k < numRuns; k++) {
      if (CC_RegionLengths[k] == k + 1) {
        numComponents++;
        labelsRenumbered[k] = static_cast<int>(numComponents);
      }
      labelsRenumbered[k] = labelsRenumbered[CC_RegionLengths[k] - 1];
    }
    runCounter = static_cast<int>(numComponents);
    CC_RegionLengths.set_size(runCounter);
    for (firstRunOnThisColumn = 0; firstRunOnThisColumn < runCounter;
         firstRunOnThisColumn++) {
      CC_RegionLengths[firstRunOnThisColumn] = 0;
    }
    for (k = 0; k < numRuns; k++) {
      CC_RegionLengths[labelsRenumbered[k] - 1] =
          ((CC_RegionLengths[labelsRenumbered[k] - 1] + endRow[k]) -
           startRow[k]) +
          1;
    }
    if (CC_RegionLengths.size(0) == 0) {
      y = 0.0;
    } else {
      if (CC_RegionLengths.size(0) <= 1024) {
        runCounter = CC_RegionLengths.size(0);
        lastRunOnPreviousColumn = 0;
        firstRunOnPreviousColumn = 1;
      } else {
        runCounter = 1024;
        firstRunOnPreviousColumn = CC_RegionLengths.size(0) / 1024;
        lastRunOnPreviousColumn =
            CC_RegionLengths.size(0) - (firstRunOnPreviousColumn << 10);
        if (lastRunOnPreviousColumn > 0) {
          firstRunOnPreviousColumn++;
        } else {
          lastRunOnPreviousColumn = 1024;
        }
      }
      y = CC_RegionLengths[0];
      for (k = 2; k <= runCounter; k++) {
        y += static_cast<double>(CC_RegionLengths[k - 1]);
      }
      for (firstRunOnThisColumn = 2;
           firstRunOnThisColumn <= firstRunOnPreviousColumn;
           firstRunOnThisColumn++) {
        double bsum;
        runCounter = (firstRunOnThisColumn - 1) << 10;
        bsum = CC_RegionLengths[runCounter];
        if (firstRunOnThisColumn == firstRunOnPreviousColumn) {
          row = lastRunOnPreviousColumn;
        } else {
          row = 1024;
        }
        for (k = 2; k <= row; k++) {
          bsum += static_cast<double>(CC_RegionLengths[(runCounter + k) - 1]);
        }
        y += bsum;
      }
    }
    pixelIdxList.set_size(static_cast<int>(y));
    x.set_size(CC_RegionLengths.size(0));
    runCounter = CC_RegionLengths.size(0);
    for (firstRunOnThisColumn = 0; firstRunOnThisColumn < runCounter;
         firstRunOnThisColumn++) {
      x[firstRunOnThisColumn] = CC_RegionLengths[firstRunOnThisColumn];
    }
    if ((CC_RegionLengths.size(0) != 1) && (CC_RegionLengths.size(0) != 0) &&
        (CC_RegionLengths.size(0) != 1)) {
      firstRunOnThisColumn = CC_RegionLengths.size(0);
      for (k = 0; k <= firstRunOnThisColumn - 2; k++) {
        x[k + 1] = x[k] + x[k + 1];
      }
    }
    idxCount.set_size(x.size(0) + 1);
    idxCount[0] = 0;
    runCounter = x.size(0);
    for (firstRunOnThisColumn = 0; firstRunOnThisColumn < runCounter;
         firstRunOnThisColumn++) {
      idxCount[firstRunOnThisColumn + 1] = x[firstRunOnThisColumn];
    }
    for (k = 0; k < numRuns; k++) {
      runCounter = (startCol[k] - 1) * 200;
      row = labelsRenumbered[k] - 1;
      firstRunOnThisColumn = startRow[k];
      firstRunOnPreviousColumn = endRow[k];
      for (lastRunOnPreviousColumn = firstRunOnThisColumn;
           lastRunOnPreviousColumn <= firstRunOnPreviousColumn;
           lastRunOnPreviousColumn++) {
        idxCount[row] = idxCount[row] + 1;
        pixelIdxList[idxCount[row] - 1] =
            static_cast<short>(lastRunOnPreviousColumn + runCounter);
      }
    }
    CC_ImageSize[0] = 200.0;
    CC_ImageSize[1] = 160.0;
    CC_RegionIndices.set_size(pixelIdxList.size(0));
    runCounter = pixelIdxList.size(0);
    for (firstRunOnThisColumn = 0; firstRunOnThisColumn < runCounter;
         firstRunOnThisColumn++) {
      CC_RegionIndices[firstRunOnThisColumn] =
          pixelIdxList[firstRunOnThisColumn];
    }
  }
  *CC_Connectivity = 8.0;
  *CC_NumObjects = numComponents;
}

} // namespace coder

//
// File trailer for bwconncomp.cpp
//
// [EOF]
//
