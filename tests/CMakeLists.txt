cmake_minimum_required(VERSION 3.14)
project(sensoregg_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Pure-logic source extracted from the Arduino sketch. Must stay free of
# Arduino headers so it compiles on a host toolchain. Same harness shape
# as the DovesDataLogger repo's tests/.
set(EGG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_executable(sensoregg_tests
  test_main.cpp
  pw_adv_encode_test.cpp
  mcp9600_regs_test.cpp
  thermistor_test.cpp
  battery_test.cpp
  ${EGG_DIR}/pw_adv_encode.cpp
  ${EGG_DIR}/mcp9600_regs.cpp
  ${EGG_DIR}/thermistor.cpp
  ${EGG_DIR}/battery.cpp
)

target_include_directories(sensoregg_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}  # for doctest.h
  ${EGG_DIR}                   # for the extracted module headers
)

# doctest.h is third-party; suppress warnings it triggers without
# muting them for our own code.
set_source_files_properties(test_main.cpp PROPERTIES
  COMPILE_FLAGS "-w"
)

target_compile_options(sensoregg_tests PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

enable_testing()
add_test(NAME sensoregg_tests COMMAND sensoregg_tests)
