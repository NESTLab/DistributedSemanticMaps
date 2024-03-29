#
# Find the ARGoS package
#
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_${ARGOS_BUILD_FOR})
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
set(CMAKE_INSTALL_PREFIX ${ARGOS_PREFIX} CACHE STRING "Install path prefix, prepended onto install directories." FORCE)

#
# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
#
if(ARGOS_BUILD_FOR_SIMULATOR)
  include(ARGoSCheckQTOpenGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Find Lua
#
find_package(Lua53 REQUIRED)
if(LUA53_FOUND)
  include_directories(${LUA_INCLUDE_DIR})
endif(LUA53_FOUND)

#
# Set ARGoS include dir
#
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})

#
# Set ARGoS link dir
#
link_directories(${ARGOS_LIBRARY_DIRS})
