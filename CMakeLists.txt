cmake_minimum_required(VERSION 3.16)
project(raspi_dash LANGUAGES C CXX)

# --- language / defs ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_definitions(LV_CONF_INCLUDE_SIMPLE)

# --- SDL2 via pkg-config ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

# --- LVGL location (vendored) ---
set(LVGL_DIR "${CMAKE_SOURCE_DIR}/third_party/lvgl")

# --- rpi_ws281x (libws2811) ---
# Headers usually in /usr/local/include, library in /usr/local/lib
find_path(WS2811_INCLUDE_DIR ws2811.h
  PATHS /usr/local/include /usr/include
)
find_library(WS2811_LIB ws2811
  PATHS /usr/local/lib /usr/lib
)

if (NOT WS2811_INCLUDE_DIR OR NOT WS2811_LIB)
  message(FATAL_ERROR "Could not find rpi_ws281x (ws2811). Did you run 'sudo make install' in jgarff/rpi_ws281x?")
endif()

# --- include dirs ---
include_directories(
  ${SDL2_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/squareline
  ${LVGL_DIR}
  ${WS2811_INCLUDE_DIR}
)

# --- sources ---
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
  # spi_ws2812.cpp REMOVED
)

# --- link ---
target_link_libraries(raspi_dash
  ${SDL2_LIBRARIES}
  ${WS2811_LIB}
  m
  pthread
)

# --- status ---
message(STATUS "✅ Building raspi_dash with:")
message(STATUS "   LVGL directory: ${LVGL_DIR}")
message(STATUS "   SDL2 include:   ${SDL2_INCLUDE_DIRS}")
message(STATUS "   LVGL sources:   ${LVGL_SOURCES}")
message(STATUS "   ws2811 include: ${WS2811_INCLUDE_DIR}")
message(STATUS "   ws2811 lib:     ${WS2811_LIB}")
