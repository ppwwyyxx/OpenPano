cmake_minimum_required(VERSION 2.8)

if (OPENMP_FOUND)
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
include_directories(${JPEG_INCLUDE_DIR})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

include_directories(SYSTEM third-party)
include_directories(.)

file(GLOB SOURCES main.cc feature/*.[c,h]* stitch/*.[c.h]* lib/*.[c,h]*)

list(APPEND SOURCES
 ${CMAKE_CURRENT_SOURCE_DIR}/third-party/lodepng/lodepng.cc
)

if (CMAKE_VERSION VERSION_LESS "3.1")
	set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
else()
	set (CMAKE_CXX_STANDARD 11)
endif()

add_executable(image-stitching ${SOURCES})
target_link_libraries(image-stitching ${JPEG_LIBRARIES})

# Visual Studio handling of file glob
# This part is not maintained.. I don't know what it is
if(MSVC)
  set_target_properties(
    image-stitching
    PROPERTIES
    GENERATED true
    EXTERNAL_OBJECT true
    LINK_FLAGS "setargv.obj"
    )
endif()
