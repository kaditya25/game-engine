cmake_minimum_required(VERSION 3.1)

# Confiugre yaml-cpp options
option(YAML_CPP_BUILD_TESTS "Enable testing" OFF)

# Build libraries
add_subdirectory(mg-msgs)
add_subdirectory(P4)
# Suppress "unused parameter" warning just for the following subdirectory
add_compile_options("-Wno-unused-parameter")
add_subdirectory(yaml-cpp)
