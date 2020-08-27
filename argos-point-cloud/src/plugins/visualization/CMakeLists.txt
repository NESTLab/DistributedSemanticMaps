#
# Headers
#
# argos3/plugins/simulator/visualizations/qt-opengl
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_VISUALIZATIONS_QTOPENGL
  qt-opengl/qtopengl_point_cloud.h)

#
# Source files
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_VISUALIZATIONS_QTOPENGL
  ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_VISUALIZATIONS_QTOPENGL}
  qt-opengl/qtopengl_point_cloud.cpp)

#
# Create Qt-OpenGL visualization
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_qtopengl_pointcloud SHARED
  ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_VISUALIZATIONS_QTOPENGL})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_qtopengl_pointcloud argos3plugin_${ARGOS_BUILD_FOR}_entities ${ARGOS_QTOPENGL_LIBRARIES})

install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_VISUALIZATIONS_QTOPENGL} DESTINATION include/argos3/plugins/simulator/visualizations/qt-opengl)
install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_qtopengl_pointcloud
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3
)