#include "demosaic.h"
#include <cstring>

bool Demosaic::process(uint8_t* src, uint8_t* dst, uint32_t width, uint32_t height) {
  // Scalar reference: expand each mono sample into RGB triplet (naive)
  for (uint32_t i = 0; i < width * height; ++i) {
    uint8_t v = src[i];
    dst[3 * i + 0] = v;
    dst[3 * i + 1] = v;
    dst[3 * i + 2] = v;
  }
  return true;
}