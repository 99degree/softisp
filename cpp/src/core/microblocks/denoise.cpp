#include "denoise.h"
#include <cstring>

bool Denoise::process(uint8_t* src, uint8_t* dst, uint32_t width, uint32_t height) {
  // Simple box filter 3x3 (scalar fallback)
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      int sum = 0;
      int cnt = 0;
      for (int oy = -1; oy <= 1; ++oy) {
        int yy = (int)y + oy;
        if (yy < 0 || yy >= (int)height) continue;
        for (int ox = -1; ox <= 1; ++ox) {
          int xx = (int)x + ox;
          if (xx < 0 || xx >= (int)width) continue;
          sum += src[yy * width + xx];
          ++cnt;
        }
      }
      dst[y * width + x] = static_cast<uint8_t>(sum / cnt);
    }
  }
  return true;
}