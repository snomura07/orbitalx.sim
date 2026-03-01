#include "app/ConfigLoader.h"
#include "app/Run.h"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  std::string configPath;
  if (argc > 1) {
    configPath = argv[1];
  } else {
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
  Run runner(params);
  return runner.run();
}
