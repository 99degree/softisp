#include "color_transform.h"

bool ColorTransform::process(uint8_t* src_rgb, uint8_t* dst_rgb, uint32_t pixels) {
  // Identity transform for stub. Replace with SIMD/NICHT implementation later.
  for (uint32_t i = 0; i < pixels * 3; ++i) dst_rgb[i] = src_rgb[i];
  return true;
}