#pragma once

#include "core/common/Types.h"

#include <string>

class ConfigLoader {
 public:
  static SimParams load(const std::string& path);
};

