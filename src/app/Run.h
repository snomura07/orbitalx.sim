#pragma once

#include "core/common/Types.h"

class Run {
 public:
  explicit Run(const SimParams& params);

  int run();

 private:
  SimParams params;
};

