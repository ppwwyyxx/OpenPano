
add_library(lodepng lodepng/lodepng.cc)
target_include_directories(lodepng SYSTEM PUBLIC
	 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
)

add_library(flann INTERFACE)
target_include_directories(flann SYSTEM INTERFACE
	 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
)

add_library(CImg INTERFACE)
target_include_directories(CImg SYSTEM INTERFACE
	 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
)

add_library(pybind11 INTERFACE)
target_include_directories(pybind11 SYSTEM INTERFACE
	 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
)
