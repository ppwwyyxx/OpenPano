cmake_minimum_required (VERSION 3.20)
project (OpenPano LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
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
# --- installs (append after add_subdirectory(src)) ---------------------------
include(GNUInstallDirs)

# sensible rpath for installed targets
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")

# we'll set this to TRUE when any target is added to the export set
set(_OPENPANO_HAVE_EXPORT FALSE)

# Library: libopenpano.so (target likely named "openpano")
if(TARGET openpano)
  install(TARGETS openpano
          EXPORT OpenPanoTargets
          LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
          ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
          RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
          INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  set(_OPENPANO_HAVE_EXPORT TRUE)
endif()

# Executable: image-stitching
if(TARGET image-stitching)
  install(TARGETS image-stitching
          RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# Python extension module: pyopenpano
if(TARGET pyopenpano)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_paths()['platlib'])"
    OUTPUT_VARIABLE _PY_SITE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT _PY_SITE)
    set(_PY_SITE "${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
  endif()

  install(TARGETS pyopenpano
          LIBRARY DESTINATION "${_PY_SITE}"
          RUNTIME DESTINATION "${_PY_SITE}")
endif()

# Third-party lodepng (shared) — via target if present, else file fallback
# Third-party lodepng (shared/static) — add it to the same export set
if(TARGET lodepng)
  install(TARGETS lodepng
          EXPORT OpenPanoTargets
          LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
          ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
          RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  set(_OPENPANO_HAVE_EXPORT TRUE)
elseif(EXISTS "${CMAKE_BINARY_DIR}/src/third-party/liblodepng.so")
  # Fallback if it's built as a raw .so without a CMake target (no export)
  install(FILES "${CMAKE_BINARY_DIR}/src/third-party/liblodepng.so"
          DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

# Export CMake package only if something joined the export set
if(_OPENPANO_HAVE_EXPORT)
  install(EXPORT OpenPanoTargets
          NAMESPACE OpenPano::
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/OpenPano)
endif()

# Public headers (optional)
if(EXISTS "${PROJECT_SOURCE_DIR}/include")
  install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/"
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
          FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
endif()

