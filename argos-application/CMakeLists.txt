project(collective_perception)
cmake_minimum_required(VERSION 3.16.0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
include(ARGoSCheckQTOpenGL)

# Look for the (optional) galib library
find_package(GALIB)
if(GALIB_FOUND)
  include_directories(${GALIB_INCLUDE_DIRS})
endif(GALIB_FOUND)

# Find Lua
find_package(Lua "5.3" REQUIRED)
if(LUA_FOUND)
   include_directories(${LUA_INCLUDE_DIR})
endif(LUA_FOUND)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(swarm_mesh_library REQUIRED)
# if(swarm_mesh_library_FOUND)
#   message("Includes for swarmmesh")
#   message("${swarm_mesh_library_INCLUDE_DIRS}")
#   include_directories(${swarm_mesh_library_INCLUDE_DIRS})
# endif(swarm_mesh_library_FOUND)

set(SWARMMESH_INCLUDE_DIR "/usr/local/include/SwarmMeshLibrary/")
set(SWARMMESH_LIBRARY_DIR "/usr/local/lib/SwarmMeshLibrary/")

#set(SWARMMESH_INCLUDE_DIR "~/swarmmeshbundle/include/SwarmMeshLibrary/") #"/usr/local/include/SwarmMeshLibrary/")
#set(SWARMMESH_LIBRARY_DIR "~/swarmmeshbundle/lib/SwarmMeshLibrary/")

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${LUA_INCLUDE_DIR} ${SWARMMESH_INCLUDE_DIR})


# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS} ${SWARMMESH_LIBRARY_DIR})


# Descend into the controllers directory
add_subdirectory(controllers)

# Descend into the loop_functions directory
add_subdirectory(loop_functions)
