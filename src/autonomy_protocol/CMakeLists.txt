cmake_minimum_required(VERSION 3.5.0)

set(TARGET lib_autonomy_protocol)

find_package (Eigen3 3.3 NO_MODULE)

set(SOURCE_FILES
  example_autonomy_protocol.cc
  game_snapshot.cc
  student_autonomy_protocol.cc
  occupancy_grid3d.cpp
  student_game_engine_visualizer.cpp 
  )

add_library(${TARGET} STATIC ${SOURCE_FILES})

target_include_directories(${TARGET} PUBLIC
  ${PROJECT_SOURCE_DIR}/src/P4/src
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TARGET} PUBLIC 
  lib_mediation_layer
  lib_environment
  lib_util
  lib_p4
  ${Boost_LIBRARIES}
  Eigen3::Eigen)

set_target_properties(${TARGET} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

target_compile_options(${TARGET} PRIVATE
  -Wfatal-errors
)
