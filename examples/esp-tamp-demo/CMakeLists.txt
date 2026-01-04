# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)    

# set(EXTRA_COMPONENT_DIRS "D:/Hanno/Documents/esp32/mycomponents")

if(NOT DEFINED ENV{IDF_TARGET})
    message("IDF_TARGET not set, defaulting to: esp32s3")
    set(ENV{IDF_TARGET} esp32s3)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tamp-demo)
