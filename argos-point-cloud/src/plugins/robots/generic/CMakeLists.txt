#
# headers
#
# argos3/plugins/robots/generic/control_interface
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_POINTCLOUD_CONTROLINTERFACE
  control_interface/ci_camera_sensor_algorithms/ci_camera_sensor_point_cloud_detector_algorithm.h)
# argos3/plugins/robots/generic/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_POINTCLOUD_SIMULATOR
    simulator/camera_sensor_algorithms/camera_sensor_point_cloud_detector_algorithm.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_POINTCLOUD
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_POINTCLOUD_CONTROLINTERFACE}
  control_interface/ci_camera_sensor_algorithms/ci_camera_sensor_point_cloud_detector_algorithm.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_POINTCLOUD
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_POINTCLOUD}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_POINTCLOUD_SIMULATOR}
    simulator/camera_sensor_algorithms/camera_sensor_point_cloud_detector_algorithm.cpp
    ../../simulator/media/point_cloud_medium.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  # if(ARGOS_COMPILE_QTOPENGL)
  #   set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KHEPERAIV_SIMULATOR
  #     ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KHEPERAIV_SIMULATOR}
  #     simulator/qtopengl_kheperaiv.h)
  #   set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KHEPERAIV
  #     ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KHEPERAIV}
  #     simulator/qtopengl_kheperaiv.h
  #     simulator/qtopengl_kheperaiv.cpp)
  # endif(ARGOS_COMPILE_QTOPENGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create point cloud plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_sensor SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_POINTCLOUD})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_sensor
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator)
if(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_sensor
    argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d
    argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator)
  # if(ARGOS_COMPILE_QTOPENGL)
  #   target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_kheperaiv argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
  # endif(ARGOS_COMPILE_QTOPENGL)
else(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_sensor
    -lpointcloud)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_POINTCLOUD_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/generic/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_POINTCLOUD_SIMULATOR} DESTINATION include/argos3/plugins/robots/generic/simulator)
  # install(FILES simulator/kheperaiv_texture_bottom.png simulator/kheperaiv_texture_side.png simulator/kheperaiv_texture_top.png DESTINATION include/argos3/plugins/simulator/visualizations/qt-opengl/textures)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_sensor
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(TARGETS argos3plugin_simulator_pointcloud_sensor
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/argos3
    ARCHIVE DESTINATION lib/argos3)
endif(ARGOS_BUILD_FOR_SIMULATOR)