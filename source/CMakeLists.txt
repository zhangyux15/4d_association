cmake_minimum_required(VERSION 3.0)
project(BaseProject)

include(CheckIncludeFileCXX)

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" "${CMAKE_MODULE_PATH}")

if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif(UNIX)

if (MSVC_VERSION GREATER_EQUAL "1900")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("/std:c++latest" _cpp_latest_flag_supported)
    if (_cpp_latest_flag_supported)
        add_compile_options("/std:c++latest")
    endif()
endif()

CHECK_INCLUDE_FILE_CXX(cstdint HAVE_CSTDINT)
if(HAVE_CSTDINT)
    add_definitions(-D_HAS_CSTDINT_=1)
endif(HAVE_CSTDINT)

find_package(OpenCV REQUIRED)

if(UNIX)
    find_package (Eigen3 3.3 REQUIRED NO_MODULE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(JSONCPP jsoncpp)
endif(UNIX)

add_subdirectory(cli_demo)
add_subdirectory(four_d_association)
