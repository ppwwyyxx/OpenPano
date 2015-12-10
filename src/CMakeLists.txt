# Currently this doesn't work for Linux. Use the Makefile instead.
find_package(jpeg)
include_directories(${JPEG_INCLUDE_DIR})
include_directories(third-party)

# our sources
include_directories(.)
file(GLOB SOURCES *.[c,h]* feature/*.[c,h]* stitch/*.[c.h]* lib/*.[c,h]*)
list(APPEND SOURCES
 ${CMAKE_CURRENT_SOURCE_DIR}/third-party/lodepng/lodepng.cc
)

add_definitions(-DDEBUG)

add_executable(image_stitching ${SOURCES})
target_link_libraries(image_stitching ${JPEG_LIBRARY})

# Visual Studio handling of file glob
if(MSVC)
  set_target_properties(
    image_stitching
    PROPERTIES
    GENERATED true
    EXTERNAL_OBJECT true
    LINK_FLAGS "setargv.obj"
    )
endif()
