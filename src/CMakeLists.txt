# find all dependency libraries:
find_package(OpenMP)
if (NOT OPENMP_FOUND)
	message(WARNING "OpenMP is not found. Will be slow.")
else()
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


find_package(JPEG)
if(JPEG_FOUND)
	include_directories(${JPEG_INCLUDE_DIR})
endif()

find_package(Eigen3 REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third-party/)

# find sources
file(GLOB SOURCES feature/*.cc stitch/*.cc lib/*.cc)


add_library(openpano ${SOURCES})	# TODO shared?
target_link_libraries(openpano ${JPEG_LIBRARY} lodepng Eigen3::Eigen)

add_executable(image-stitching main.cc)
target_link_libraries(image-stitching openpano)

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
