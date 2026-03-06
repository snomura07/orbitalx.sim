#include "app/ConfigLoader.h"
#include "app/Run.h"

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool parseCommandLine(int argc, char** argv, std::string& configPath, RunOptions& options) {
  std::vector<std::string> positionalArgs;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--stg") {
      options.odometryTraceMode = true;
      continue;
    }
    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << '\n';
      return false;
    }
    positionalArgs.push_back(arg);
  }

  if (positionalArgs.size() > 1) {
    std::cerr << "unexpected argument: " << positionalArgs[1] << '\n';
    return false;
  }
  if (!positionalArgs.empty()) {
    configPath = positionalArgs.front();
  }
  return true;
}

std::string resolveConfigPath(std::string configPath) {
  if (!configPath.empty()) {
    return configPath;
  }

  const std::string kInstalledDefault = "/usr/local/config/sim_params.conf";
  const std::string kDefault = "config/sim_params.conf";
  const std::string kFromBuildDir = "../config/sim_params.conf";
  if (std::filesystem::exists(kInstalledDefault)) {
    return kInstalledDefault;
  }
  if (std::filesystem::exists(kDefault)) {
    return kDefault;
  }
  if (std::filesystem::exists(kFromBuildDir)) {
    return kFromBuildDir;
  }
  return kInstalledDefault;
}

}  // namespace

int main(int argc, char** argv) {
  std::string configPath;
  RunOptions options;
  if (!parseCommandLine(argc, argv, configPath, options)) {
    return 1;
  }
  configPath = resolveConfigPath(configPath);

  std::cerr << "using config: " << configPath << '\n';
  const SimParams params = ConfigLoader::load(configPath);
  Run runner(params, options);
  return runner.run();
}
