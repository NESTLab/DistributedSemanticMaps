#
# What is ARGoS being built for?
# Accepted values: "simulator" or a robot name (lowercase)
#
if(NOT DEFINED ARGOS_BUILD_FOR)
  # Variable was not set, set to default value
  set(ARGOS_BUILD_FOR "simulator" CACHE STRING "What is ARGoS being built for? \"simulator\" or a robot name (lowercase)")
else(NOT DEFINED ARGOS_BUILD_FOR)
  # Variable was set, make it public
  set(ARGOS_BUILD_FOR ${ARGOS_BUILD_FOR} CACHE STRING "What is ARGoS being built for? \"simulator\" or a robot name (lowercase)")
endif(NOT DEFINED ARGOS_BUILD_FOR)
# Set a macro according to value set in ARGOS_BUILD_FOR
add_definitions(-DARGOS_${ARGOS_BUILD_FOR}_BUILD)
# Create a convenience variable for checks in the CMake files
if(ARGOS_BUILD_FOR STREQUAL "simulator")
  set(ARGOS_BUILD_FOR_SIMULATOR TRUE)
else(ARGOS_BUILD_FOR STREQUAL "simulator")
  set(ARGOS_BUILD_FOR_SIMULATOR FALSE)
endif(ARGOS_BUILD_FOR STREQUAL "simulator")

#
# Optimize code for current platform?
#
if(NOT DEFINED ARGOS_BUILD_NATIVE)
  option(ARGOS_BUILD_NATIVE "ON -> compile with platform-specific optimizations, OFF -> compile to portable binary" OFF)
endif(NOT DEFINED ARGOS_BUILD_NATIVE)

