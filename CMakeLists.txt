cmake_minimum_required(VERSION 3.0)

# new feature to Cmake Version > 2.8.12
# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

# Set the version number for the library
set (GTSAM_VERSION_MAJOR 4)
set (GTSAM_VERSION_MINOR 2)
set (GTSAM_VERSION_PATCH 0)
set (GTSAM_PRERELEASE_VERSION "a9")
math (EXPR GTSAM_VERSION_NUMERIC "10000 * ${GTSAM_VERSION_MAJOR} + 100 * ${GTSAM_VERSION_MINOR} + ${GTSAM_VERSION_PATCH}")

if (${GTSAM_VERSION_PATCH} EQUAL 0)
    set (GTSAM_VERSION_STRING "${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}${GTSAM_PRERELEASE_VERSION}")
else()
    set (GTSAM_VERSION_STRING "${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH}${GTSAM_PRERELEASE_VERSION}")
endif()

project(GTSAM
        LANGUAGES CXX C
        VERSION "${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH}")

message(STATUS "GTSAM Version: ${GTSAM_VERSION_STRING}")

set (CMAKE_PROJECT_VERSION_MAJOR ${GTSAM_VERSION_MAJOR})
set (CMAKE_PROJECT_VERSION_MINOR ${GTSAM_VERSION_MINOR})
set (CMAKE_PROJECT_VERSION_PATCH ${GTSAM_VERSION_PATCH})

# Determine the platform.
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Darwin")
    set(OS_MACOSX 1)
    set(OS_POSIX 1)
    add_definitions(-D__APPLE__)
    add_definitions(-D__MACOSX__)
    add_definitions(-DABTOR_MAC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -fPIC -Wno-deprecated-declarations")  #屏蔽弃用方法的警告
    message(STATUS "You are building Darwin")
elseif ("${CMAKE_SYSTEM_NAME}" STREQUAL "Linux")
    set(OS_LINUX 1)
    set(OS_POSIX 1)
    add_definitions(-D__LINUX__)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -fPIC")
    message(STATUS "You are building Linux")
elseif ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
    set(OS_WINDOWS 1)
    add_definitions(-DUNICODE)
    add_compile_definitions(PLATFORM_WIN32)
    add_compile_definitions(TARGET_STATIC_LIB)
    add_compile_definitions(TENCENT_WINDOWS)
    add_compile_definitions(_TENCENT_BUILD_TARGET_WINDOWS)
    if (MSVC_VERSION EQUAL 1900 OR MSVC_VERSION GREATER 1900)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -fPIC /source-charset:utf-8")
    endif ()
    message(STATUS "You are building Windows")
elseif ("${CMAKE_SYSTEM_NAME}" STREQUAL "Android")
    set(OS_ANDROID 1)
    add_definitions(-DPLATFORM_ANDROID)
    add_compile_definitions(TENCENT_ANDROID)

    add_definitions(-DEIGEN_DONT_VECTORIZE)
    add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

    message(STATUS "You are building Android")
elseif ("${CMAKE_SYSTEM_NAME}" STREQUAL "iOS")
    set(OS_IOS 1)
    set(OS_IOS TRUE CACHE BOOL INTERNAL)
    set(DYNAMIC_LIB_EXTENSION "dylib" CACHE STRING INTERNAL)
    set(STATIC_LIB_EXTENSION "a" CACHE STRING INTERNAL)
    add_definitions(-DOS_IOS)
    add_compile_definitions(ABTOR_IOS)
    message(STATUS "You are building IOS")
endif ()

###############################################################################
# Gather information, perform checks, set defaults

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GtsamMakeConfigFile)
include(GNUInstallDirs)

# Load build type flags and default to Debug mode
include(GtsamBuildTypes)

# Use macros for creating tests/timing scripts
include(GtsamTesting)
include(GtsamPrinting)

# guard against in-source builds
if(${GTSAM_SOURCE_DIR} STREQUAL ${GTSAM_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# options
option(GTSAM_BUILD_TESTS                 "Enable/Disable building of tests"          OFF)
option(GTSAM_BUILD_EXAMPLES_ALWAYS       "Build examples with 'make all' (build with 'make examples' if not)"       ON)
option(GTSAM_BUILD_TIMING_ALWAYS         "Build timing scripts with 'make all' (build with 'make timing' if not"    OFF)
include(cmake/HandleGeneralOptions.cmake)   # CMake build options

set(USE_BOOST OFF)
set(GTSAM_WITH_TBB OFF)
# Libraries:
if(USE_BOOST)
    list(APPEND CMAKE_PREFIX_PATH  "/home/rayanzhao/source/linux_hdloc/third_party/boost/out/lib/cmake/Boost-1.83.0")
#    set(BOOST_ROOT  "/home/rayanzhao/source/linux_hdloc/third_party/boost/out/lib/cmake/Boost-1.83.0")
    add_definitions(-DUSING_BOOST_DEV)
    include(cmake/HandleBoost.cmake)            # Boost
endif()
include(cmake/HandleCCache.cmake)           # ccache
include(cmake/HandleCPack.cmake)            # CPack
include(cmake/HandleEigen.cmake)            # Eigen3
include(cmake/HandleMetis.cmake)            # metis
include(cmake/HandleMKL.cmake)              # MKL
include(cmake/HandleOpenMP.cmake)           # OpenMP
include(cmake/HandlePerfTools.cmake)        # Google perftools
include(cmake/HandlePython.cmake)           # Python options and commands
include(cmake/HandleTBB.cmake)              # TBB
include(cmake/HandleUninstall.cmake)        # for "make uninstall"
include(cmake/HandleAllocators.cmake)       # Must be after tbb, pertools
include(cmake/HandleGlobalBuildFlags.cmake) # Build flags

###############################################################################
# Build Tests
if(GTSAM_BUILD_TESTS)
    add_subdirectory(CppUnitLite)
    add_subdirectory(tests)
    # Build examples
    if(GTSAM_BUILD_EXAMPLES_ALWAYS)
        add_subdirectory(examples)
    endif()
endif()

# Build GTSAM library
add_subdirectory(gtsam)

# This is the new wrapper
if(GTSAM_BUILD_PYTHON OR GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Need to set this for the wrap package so we don't use the default value.
    set(WRAP_PYTHON_VERSION ${GTSAM_PYTHON_VERSION}
        CACHE STRING "The Python version to use for wrapping")
    # Set the include directory for matlab.h
    set(GTWRAP_INCLUDE_NAME "wrap")

    # Copy matlab.h to the correct folder.
    configure_file(${PROJECT_SOURCE_DIR}/wrap/matlab.h
               ${PROJECT_BINARY_DIR}/wrap/matlab.h COPYONLY)

    add_subdirectory(wrap)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/wrap/cmake")
endif()

# Python toolbox
if(GTSAM_BUILD_PYTHON)
    add_subdirectory(python)
endif()

# Matlab toolbox
if(GTSAM_INSTALL_MATLAB_TOOLBOX)
    add_subdirectory(matlab)
endif()

# Install config and export files
GtsamMakeConfigFile(GTSAM "${CMAKE_CURRENT_SOURCE_DIR}/gtsam_extra.cmake.in")
export(TARGETS ${GTSAM_EXPORTED_TARGETS} FILE GTSAM-exports.cmake)

# CMake Tools
add_subdirectory(cmake)

# Print configuration variables
include(cmake/HandlePrintConfiguration.cmake)

# Print warnings at the end
include(cmake/HandleFinalChecks.cmake)

# Include CPack *after* all flags
include(CPack)
