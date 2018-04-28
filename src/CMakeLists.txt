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
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	${EIGEN3_INCLUDE_DIR}
)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third-party/)

# find sources
file(GLOB SOURCES feature/*.cc stitch/*.cc lib/*.cc openpano.cc)

add_library(openpano STATIC ${SOURCES})	# TODO shared?
target_link_libraries(openpano
	${JPEG_LIBRARY}
	lodepng
)
set_target_properties(openpano PROPERTIES PUBLIC_HEADER "openpano.h")

add_executable(image-stitching main.cc)
target_link_libraries(image-stitching openpano)

if (NOT JPEG_FOUND)
	target_compile_definitions(openpano PUBLIC DISABLE_JPEG)
	target_compile_definitions(image-stitching PUBLIC DISABLE_JPEG)
endif()

find_package(PythonLibs)
if(PYTHONLIBS_FOUND AND BUILD_SHARED_LIBS)
	add_library(pyopenpano SHARED python/pybind.cc)
	target_link_libraries(pyopenpano openpano pybind11)
	target_include_directories(pyopenpano SYSTEM
		PRIVATE ${PYTHON_INCLUDE_DIRS}
	)
endif()

install(TARGETS openpano
	EXPORT openpano
	ARCHIVE DESTINATION /usr/lib
	PUBLIC_HEADER DESTINATION /usr/include
)

install(EXPORT openpano DESTINATION /usr/lib/cmake/openpano)
install(FILES openpano-config.cmake DESTINATION /usr/lib/cmake/openpano)

# export(TARGETS openpano FILE /usr/lib/cmake/openpano/openpano-config.cmake)
# export(PACKAGE openpano)
