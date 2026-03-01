#include "app/ConfigLoader.h"
#include "app/Run.h"

#include <string>

int main(int argc, char** argv) {
  const std::string configPath = (argc > 1) ? argv[1] : "config/sim_params.conf";
  const SimParams params = ConfigLoader::load(configPath);
  Run runner(params);
  return runner.run();
}
