cmake_minimum_required(VERSION 3.5.0)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

add_executable(mediation_layer mediation_layer_main.cc)
target_include_directories(mediation_layer PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(mediation_layer
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  lib_ros_node
  lib_ros_view
  yaml-cpp
  ${catkin_LIBRARIES}
)

set_target_properties(mediation_layer
  PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

add_executable(visualizer visualizer_main.cc)
target_include_directories(visualizer PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(visualizer
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  lib_ros_node
  lib_ros_view
  yaml-cpp
  ${catkin_LIBRARIES}
)

set_target_properties(visualizer
  PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

add_executable(physics_simulator physics_simulator_main.cc)
target_include_directories(physics_simulator PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(physics_simulator
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  lib_ros_node
  lib_ros_view
  lib_autonomy_protocol
  yaml-cpp
  ${catkin_LIBRARIES})
set_target_properties(physics_simulator
  PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

add_executable(example_autonomy_protocol example_autonomy_protocol_main.cc)
target_include_directories(example_autonomy_protocol PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(example_autonomy_protocol
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  lib_ros_node
  lib_ros_view
  lib_autonomy_protocol
  yaml-cpp
  ${catkin_LIBRARIES})
set_target_properties(example_autonomy_protocol
  PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

add_executable(student_autonomy_protocol student_autonomy_protocol_main.cc)
target_include_directories(student_autonomy_protocol PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(student_autonomy_protocol
  lib_geometry
  lib_environment
  lib_util
  lib_mediation_layer
  lib_ros_node
  lib_ros_view
  lib_autonomy_protocol
  yaml-cpp
  ${catkin_LIBRARIES})
set_target_properties(student_autonomy_protocol
  PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

add_executable(multi_quad_autonomy_protocol multi_quad_autonomy_protocol_main.cc)
target_include_directories(multi_quad_autonomy_protocol PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(multi_quad_autonomy_protocol
        lib_geometry
        lib_environment
        lib_util
        lib_mediation_layer
        lib_ros_node
        lib_ros_view
        lib_autonomy_protocol
        yaml-cpp
        ${catkin_LIBRARIES})
set_target_properties(multi_quad_autonomy_protocol
        PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        )

add_executable(red_team_autonomy_protocol red_team_autonomy_protocol_main.cc)
target_include_directories(red_team_autonomy_protocol PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(red_team_autonomy_protocol
        lib_geometry
        lib_environment
        lib_util
        lib_mediation_layer
        lib_ros_node
        lib_ros_view
        lib_autonomy_protocol
        yaml-cpp
        ${catkin_LIBRARIES})
set_target_properties(red_team_autonomy_protocol
        PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        )

add_executable(blue_team_autonomy_protocol blue_team_autonomy_protocol_main.cc)
target_include_directories(blue_team_autonomy_protocol PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(blue_team_autonomy_protocol
        lib_geometry
        lib_environment
        lib_util
        lib_mediation_layer
        lib_ros_node
        lib_ros_view
        lib_autonomy_protocol
        yaml-cpp
        ${catkin_LIBRARIES})
set_target_properties(blue_team_autonomy_protocol
        PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        )

add_executable(asset_games_protocol asset_games_protocol_main.cc)
target_include_directories(asset_games_protocol PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(asset_games_protocol
        lib_geometry
        lib_environment
        lib_util
        lib_mediation_layer
        lib_ros_node
        lib_ros_view
        lib_autonomy_protocol
        yaml-cpp
        ${catkin_LIBRARIES})
set_target_properties(asset_games_protocol
        PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        )

