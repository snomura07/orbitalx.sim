cmake_minimum_required(VERSION 3.16)
project(orbitalx_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(orbitalx_sim
  src/app/ConfigLoader.cpp
  src/app/Run.cpp
  src/app/main.cpp
  src/core/plant/battery/Battery.cpp
  src/core/plant/motor/Motor.cpp
  src/core/cpu/Cpu.cpp
  src/core/Sensor/BatterySensor.cpp
  src/core/Sensor/Encoder.cpp
  src/core/Sensor/LineSensor.cpp
)

target_include_directories(orbitalx_sim PRIVATE src)

install(TARGETS orbitalx_sim
  RUNTIME DESTINATION bin
)
