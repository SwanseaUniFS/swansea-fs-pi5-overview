cmake_minimum_required(VERSION 3.16)
project(raspi_dash C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-DLV_CONF_INCLUDE_SIMPLE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

set(LVGL_DIR "${CMAKE_SOURCE_DIR}/third_party/lvgl")

include_directories(
  ${SDL2_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/squareline
  ${LVGL_DIR}
)

file(GLOB_RECURSE LVGL_SOURCES
  ${LVGL_DIR}/src/*.c
)
file(GLOB UI_SOURCES
  ${CMAKE_SOURCE_DIR}/squareline/*.c
)

add_executable(raspi_dash
  ${LVGL_SOURCES}
  ${UI_SOURCES}
  ${CMAKE_SOURCE_DIR}/main.cpp
  ${CMAKE_SOURCE_DIR}/socketcan.cpp
  ${CMAKE_SOURCE_DIR}/spi_ws2812.cpp    # <-- back in
)

target_link_libraries(raspi_dash
  ${SDL2_LIBRARIES}
  m
  pthread
)

message(STATUS "✅ Building raspi_dash with:")
message(STATUS "   LVGL directory: ${LVGL_DIR}")
message(STATUS "   SDL2 include:   ${SDL2_INCLUDE_DIRS}")
message(STATUS "   LVGL sources:   ${LVGL_SOURCES}")
