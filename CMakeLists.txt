cmake_minimum_required (VERSION 3.1)
project (OpenPano LANGUAGES CXX)

# set a default build type if none was provided
# this has to be done before the project() instruction!
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

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
