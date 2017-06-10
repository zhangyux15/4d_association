cmake_minimum_required(VERSION 3.0)
project(BaseProject)

include(CheckIncludeFileCXX)

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" "${CMAKE_MODULE_PATH}")

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

CHECK_INCLUDE_FILE_CXX(cstdint HAVE_CSTDINT)
if(HAVE_CSTDINT)
    add_definitions(-D_HAS_CSTDINT_=1)
endif(HAVE_CSTDINT)

add_subdirectory(cli_demo)
