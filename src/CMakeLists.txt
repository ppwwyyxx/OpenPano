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

add_definitions(${EIGEN3_DEFINITIONS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# find sources
file(GLOB SOURCES feature/*.cc stitch/*.cc lib/*.cc)


add_library(openpano ${SOURCES})	# TODO shared?
target_link_libraries(openpano ${JPEG_LIBRARY} lodepng)

add_executable(image-stitching main.cc)
target_link_libraries(image-stitching openpano)

if (NOT JPEG_FOUND)
	target_compile_definitions(openpano PUBLIC DISABLE_JPEG)
	target_compile_definitions(image-stitching PUBLIC DISABLE_JPEG)
endif()

find_package(Python3 COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)
if(Python3_Development_FOUND AND BUILD_SHARED_LIBS AND pybind11_FOUND)
  pybind11_add_module(pyopenpano python/pybind.cc)
  target_link_libraries(pyopenpano PRIVATE openpano)
endif()
