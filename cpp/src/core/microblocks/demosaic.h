#pragma once
#include "../../include/process_item.h"
#include <cstdint>

class Demosaic {
public:
  Demosaic() = default;
  ~Demosaic() = default;

  // in-place demosaic; input/out layout is implementation defined in stub
  bool process(uint8_t* src, uint8_t* dst, uint32_t width, uint32_t height);
};