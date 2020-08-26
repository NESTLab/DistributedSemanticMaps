#
# headers
#
# argos3/plugins/simulator/entities
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_ENTITIES
  entities/point_cloud_entity.h)
# argos3/plugins/simulator/media
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_MEDIA
    media/point_cloud_medium.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# kheperaiv sources
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_POINTCLOUD_SIMULATOR
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_ENTITIES}
  entities/point_cloud_entity.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_POINTCLOUD_SIMULATOR
    ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_POINTCLOUD_SIMULATOR}
    ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_MEDIA}
    media/point_cloud_medium.cpp)
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
add_library(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_POINTCLOUD_SIMULATOR})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot)
if(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator
    argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d)
#   if(ARGOS_COMPILE_QTOPENGL)
    # target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_kheperaiv argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
#   endif(ARGOS_COMPILE_QTOPENGL)
else(ARGOS_BUILD_FOR_SIMULATOR)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_pointcloud_simulator
    -lpointcloud_simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_MEDIA} DESTINATION include/argos3/plugins/simulator/media)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_POINTCLOUD_ENTITIES} DESTINATION include/argos3/plugins/simulator/entities)
  # install(FILES simulator/kheperaiv_texture_bottom.png simulator/kheperaiv_texture_side.png simulator/kheperaiv_texture_top.png DESTINATION include/argos3/plugins/simulator/visualizations/qt-opengl/textures)
endif(ARGOS_BUILD_FOR_SIMULATOR)

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