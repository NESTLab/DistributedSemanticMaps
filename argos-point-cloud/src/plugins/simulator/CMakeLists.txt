#
# headers
#
# argos3/plugins/simulator/entities
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_ENTITIES
  entities/point_cloud_entity.h)
# argos3/plugins/simulator/media
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_MEDIA
    media/point_cloud_medium.h)

set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_ENGINES
    physics_engines/dynamics2d/dynamics2d_point_cloud_model.h)


#
# sources
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_POINTCLOUD_SIMULATOR
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_ENTITIES}
  entities/point_cloud_entity.cpp)


set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_POINTCLOUD_SIMULATOR
  ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_POINTCLOUD_SIMULATOR}
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_MEDIA}
  media/point_cloud_medium.cpp)


  set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_POINTCLOUD_SIMULATOR
  ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_POINTCLOUD_SIMULATOR}
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_ENGINES}
  physics_engines/dynamics2d/dynamics2d_point_cloud_model.cpp)


#
# Create point cloud plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator SHARED ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_POINTCLOUD_SIMULATOR})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
if(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator
    argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d)

else(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator
    -lpointcloud_simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_MEDIA} DESTINATION include/argos3/plugins/simulator/media)

install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_ENTITIES} DESTINATION include/argos3/plugins/simulator/entities)

install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_ENGINES} DESTINATION include/argos3/plugins/simulator/physics_engines)


install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(TARGETS argos3plugin_simulator_pointcloud_simulator
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/argos3
    ARCHIVE DESTINATION lib/argos3)
endif(ARGOS_BUILD_FOR_SIMULATOR)