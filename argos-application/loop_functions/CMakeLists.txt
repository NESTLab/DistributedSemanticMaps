link_directories(${CMAKE_BINARY_DIR}/controllers/collective_perception_controller)
set(point_cloud_loop_functions_SOURCES point_cloud_loop_functions.cpp)

if(ARGOS_COMPILE_QTOPENGL)
  set(point_cloud_loop_functions_SOURCES
    ${point_cloud_loop_functions_SOURCES}
    point_cloud_qtuser_functions.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(point_cloud_loop_functions MODULE ${point_cloud_loop_functions_SOURCES})
target_link_libraries(point_cloud_loop_functions
  collective_perception_controller
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  argos3plugin_simulator_pointcloud_simulator
  argos3plugin_simulator_pointcloud_sensor
  swarm_mesh_library
  )

if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(point_cloud_loop_functions argos3plugin_simulator_qtopengl 
  argos3plugin_simulator_qtopengl_pointcloud
   ${ARGOS_QTOPENGL_LIBRARIES})
endif(ARGOS_COMPILE_QTOPENGL)

set(scenenn_point_cloud_loop_functions_SOURCES scenenn_point_cloud_loop_functions.cpp)

if(ARGOS_COMPILE_QTOPENGL)
  set(scenenn_point_cloud_loop_functions_SOURCES
    ${scenenn_point_cloud_loop_functions_SOURCES}
    point_cloud_qtuser_functions.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(scenenn_point_cloud_loop_functions MODULE ${scenenn_point_cloud_loop_functions_SOURCES})
target_link_libraries(scenenn_point_cloud_loop_functions
  collective_perception_controller
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media
  argos3plugin_simulator_pointcloud_simulator
  argos3plugin_simulator_pointcloud_sensor
  swarm_mesh_library
  )

if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(scenenn_point_cloud_loop_functions argos3plugin_simulator_qtopengl 
  argos3plugin_simulator_qtopengl_pointcloud
   ${ARGOS_QTOPENGL_LIBRARIES})
endif(ARGOS_COMPILE_QTOPENGL)

add_library(point_cloud_qtuser_functions MODULE 
  point_cloud_qtuser_functions.h
  point_cloud_qtuser_functions.cpp)

target_link_libraries(point_cloud_qtuser_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_pointcloud_simulator
  argos3plugin_simulator_qtopengl
  argos3plugin_simulator_qtopengl_pointcloud
  ${ARGOS_QTOPENGL_LIBRARIES})