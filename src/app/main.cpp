#include "app/ConfigLoader.h"
#include "app/Run.h"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  std::string configPath;
  RunOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--stg") {
      options.odometryTraceMode = true;
      continue;
    }
    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << '\n';
      return 1;
    }
    if (configPath.empty()) {
      configPath = arg;
    } else {
      std::cerr << "unexpected argument: " << arg << '\n';
      return 1;
    }
  }

  if (configPath.empty()) {
    const std::string kDefault = "config/sim_params.conf";
    const std::string kFromBuildDir = "../config/sim_params.conf";
    if (std::filesystem::exists(kDefault)) {
      configPath = kDefault;
    } else if (std::filesystem::exists(kFromBuildDir)) {
      configPath = kFromBuildDir;
    } else {
      configPath = kDefault;
    }
  }

  std::cerr << "using config: " << configPath << '\n';
  const SimParams params = ConfigLoader::load(configPath);
  Run runner(params, options);
  return runner.run();
}
