cmake_minimum_required(VERSION 3.1)

# Confiugre yaml-cpp options
option(YAML_CPP_BUILD_TESTS "Enable testing" OFF)

# Build libraries
add_subdirectory(yaml-cpp)
add_subdirectory(mg-msgs)
add_subdirectory(P4)
