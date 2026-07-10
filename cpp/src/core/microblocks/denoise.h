#pragma once
#include <cstdint>

class Denoise {
public:
  Denoise() = default;
  ~Denoise() = default;
  bool process(uint8_t* src, uint8_t* dst, uint32_t width, uint32_t height);
};