cmake_minimum_required(VERSION 3.25)
project(marlin-calc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

set(MARLIN ${CMAKE_CURRENT_SOURCE_DIR}/../Marlin)

add_executable(marlin-calc
  src/main.cpp
  src/stubs.cpp
  ${MARLIN}/src/core/serial.cpp
  ${MARLIN}/src/feature/fwretract.cpp
  ${MARLIN}/src/gcode/config/M92.cpp
  ${MARLIN}/src/gcode/config/M200-M205.cpp
  ${MARLIN}/src/gcode/config/M220.cpp
  ${MARLIN}/src/gcode/config/M221.cpp
  ${MARLIN}/src/gcode/feature/advance/M900.cpp
  ${MARLIN}/src/gcode/feature/fwretract/G10_G11.cpp
  ${MARLIN}/src/gcode/feature/fwretract/M207-M209.cpp
  ${MARLIN}/src/gcode/gcode.cpp
  ${MARLIN}/src/gcode/geometry/G92.cpp
  ${MARLIN}/src/gcode/motion/G0_G1.cpp
  ${MARLIN}/src/gcode/motion/G2_G3.cpp
  ${MARLIN}/src/gcode/motion/M400.cpp
  ${MARLIN}/src/gcode/parser.cpp
  ${MARLIN}/src/gcode/queue.cpp
  ${MARLIN}/src/gcode/units/M82_M83.cpp
  ${MARLIN}/src/module/fans.cpp
  ${MARLIN}/src/module/motion.cpp
  ${MARLIN}/src/module/planner.cpp
  ${MARLIN}/src/HAL/LINUX/HAL.cpp
  ${MARLIN}/src/HAL/LINUX/arduino.cpp
  ${MARLIN}/src/HAL/LINUX/hardware/Clock.cpp
  ${MARLIN}/src/HAL/LINUX/hardware/Gpio.cpp
)
target_compile_definitions(marlin-calc PRIVATE __PLAT_LINUX__)
target_include_directories(marlin-calc PRIVATE src ${MARLIN} ${MARLIN}/src/HAL/LINUX/include)
target_compile_options(marlin-calc PRIVATE -Wall -Wno-expansion-to-defined)
find_package(Threads REQUIRED)
target_link_libraries(marlin-calc PRIVATE Threads::Threads)
if(NOT APPLE)
  target_link_options(marlin-calc PRIVATE -static)
endif()

# OctoPrint-PrintTimeGenius runs marlin-calc.<platform.machine()>, with a
# darwin- or windows- prefix on those systems.
if(APPLE)
  if(CMAKE_OSX_ARCHITECTURES)
    set(MARLIN_CALC_TARGET darwin-${CMAKE_OSX_ARCHITECTURES})
  else()
    set(MARLIN_CALC_TARGET darwin-${CMAKE_SYSTEM_PROCESSOR})
  endif()
elseif(WIN32)
  set(MARLIN_CALC_TARGET windows-${CMAKE_SYSTEM_PROCESSOR})
else()
  set(MARLIN_CALC_TARGET ${CMAKE_SYSTEM_PROCESSOR})
endif()
set_target_properties(marlin-calc PROPERTIES OUTPUT_NAME marlin-calc.${MARLIN_CALC_TARGET})

enable_testing()
find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
  set(test_args --marlin-calc $<TARGET_FILE:marlin-calc>)
  if(CMAKE_CROSSCOMPILING_EMULATOR)
    list(JOIN CMAKE_CROSSCOMPILING_EMULATOR " " emulator)
    list(APPEND test_args --emulator ${emulator})
  endif()
  add_test(NAME calc_tests
    COMMAND ${Python3_EXECUTABLE} -m pytest ${CMAKE_CURRENT_SOURCE_DIR}/tests
            -p no:cacheprovider ${test_args})
  set_tests_properties(calc_tests PROPERTIES ENVIRONMENT PYTHONDONTWRITEBYTECODE=1)
endif()
