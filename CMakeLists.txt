# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "main/esp-request" "main/libesphttpd" "main/md5" "main/adafruit")
set(INCLUDE_DIRS "main/esp-request/include" "main/libesphttpd/include" "main/md5" "main/adafruit")
set(SUPPORTED_TARGETS esp32)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_hd)
