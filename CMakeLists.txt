cmake_minimum_required (VERSION 3.1)

# set a default build type if none was provided
# this has to be done before the project() instruction!
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

project (stitch)

# Detect the OS
if(${CMAKE_HOST_WIN32})
  set(WINDOWS 1)
elseif(${CMAKE_HOST_APPLE})
  set(MACOSX 1)
elseif(${CMAKE_HOST_UNIX})
  set(LINUX 1)
else()
  message(FATAL_ERROR "Unsupported operating system")
  return()
endif()

# Set some env vars
set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)

if (MSVC)
  if(MSVC_VERSION LESS 1800) # 1700 = VS 2013
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
  set(OBJ_EXT obj)
else()
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
  set(OBJ_EXT o)
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_definitions( -DUNICODE -D_UNICODE)
#################################################################################################################
# ============================================ General build definitions ====================================== #
#################################################################################################################
find_package(JPEG)
if(NOT JPEG_FOUND)
 set(JPEG_INCLUDE_DIR "$ENV{JPEG_INCLUDE_DIR}")
 if(NOT JPEG_INCLUDE_DIR)
  message(WARNING "Can not find the jpeg include directory, ignore the jpeg.")
 endif()
 set(JPEG_LIBRARY "$ENV{JPEG_LIBRARY}")
 if(NOT JPEG_LIBRARY)
  message(WARNING "Can not find the jpeg lib directory, ignore the jpeg.")
 endif()
 #find again.
 find_package(JPEG)
 if(NOT JPEG_FOUND)
  message(WARNING "Can not find the jpeg ")
 endif()
endif()

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
 set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
 if(NOT EIGEN3_INCLUDE_DIR)
   message(FATAL_ERROR "Can not find the eigen3 directory, please set the EIGEN3_INCLUDE_DIR as an environment variable, and make it point to the folder of eigen3(Make sure the file Eigen/Dense exist in it).")
 endif()
endif()

if(MSVC)

  add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE -DWIN32_LEAN_AND_MEAN -DVC_EXTRALEAN -DMSVC)

  # /Zo makes debug symbol in pdb in release mode in VS2015
  add_definitions(/fp:fast /GR- /Os /Zo /arch:AVX /openmp)
else()
  add_definitions(-O3 -g -march=native -fopenmp -Wall -Wextra)
  set(CMAKE_CXX_COMPILER g++)
endif()

add_subdirectory(src)
