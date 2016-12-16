cmake_minimum_required (VERSION 3.1)

# set a default build type if none was provided
# this has to be done before the project() instruction!
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

project (OpenPano)

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
if (WIN32)
	add_definitions( -DUNICODE -D_UNICODE)
endif()


# find all dependency libraries:
find_package(OpenMP)
if (NOT OPENMP_FOUND)
	message(WARNING "OpenMP is not found. Will be slow.")
endif()

#find_package(JPEG)
if(NOT JPEG_FOUND)
	set(JPEG_INCLUDE_DIR "$ENV{JPEG_INCLUDE_DIR}")
	set(JPEG_LIBRARY "$ENV{JPEG_LIBRARY}")
	if((NOT JPEG_LIBRARY) OR (NOT JPEG_INCLUDE_DIR))
		message(WARNING "libjpeg is not found. Will not read/write jpeg files.")
		add_definitions(-DDISABLE_JPEG)
	endif()
endif()

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
	set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
	if(NOT EIGEN3_INCLUDE_DIR)
		message(FATAL_ERROR "Eigen3 is not found. Please set EIGEN3_INCLUDE_DIR and make sure the file Eigen/Dense exists there.")
	endif()
endif()

# compiler options:
if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE -DWIN32_LEAN_AND_MEAN -DVC_EXTRALEAN -DMSVC)
	# /Zo makes debug symbol in pdb in release mode in VS2015
	add_definitions(/fp:fast /GR- /Os /Zo /arch:AVX /openmp)
else()
	add_definitions(-O3 -march=native -Wall -Wextra)
endif()
add_definitions(-DDEBUG)


add_subdirectory(src)
