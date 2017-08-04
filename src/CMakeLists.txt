# find all dependency libraries:
find_package(OpenMP)
if (NOT OPENMP_FOUND)
	message(WARNING "OpenMP is not found. Will be slow.")
else()
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


find_package(JPEG)
if(NOT JPEG_FOUND)
	set(JPEG_INCLUDE_DIR "$ENV{JPEG_INCLUDE_DIR}")
	set(JPEG_LIBRARY "$ENV{JPEG_LIBRARY}")
	if((NOT JPEG_LIBRARY) OR (NOT JPEG_INCLUDE_DIR))
		message(WARNING "libjpeg is not found. Will not read/write jpeg files.")
		add_definitions(-DDISABLE_JPEG)
	endif()
else()
	include_directories(${JPEG_INCLUDE_DIR})
endif()

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
	set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
	if(NOT EIGEN3_INCLUDE_DIR)
		message(FATAL_ERROR "Eigen3 is not found. Please set EIGEN3_INCLUDE_DIR and make sure the file Eigen/Dense exists there.")
	endif()
else()
	include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(SYSTEM third-party)
include_directories(.)

# find sources
file(GLOB SOURCES main.cc feature/*.[c,h]* stitch/*.[c.h]* lib/*.[c,h]*)
list(APPEND SOURCES
 ${CMAKE_CURRENT_SOURCE_DIR}/third-party/lodepng/lodepng.cc
)

add_executable(image-stitching ${SOURCES})
target_link_libraries(image-stitching ${JPEG_LIBRARY})

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
