#pragma once
#include <cstdint>

class ColorTransform {
public:
  ColorTransform() = default;
  ~ColorTransform() = default;
  bool process(uint8_t* src_rgb, uint8_t* dst_rgb, uint32_t pixels);
};