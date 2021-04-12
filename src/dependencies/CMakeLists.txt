cmake_minimum_required(VERSION 3.4)

# Confiugre yaml-cpp options
option(YAML_CPP_BUILD_TESTS "Enable testing" OFF)

# Build libraries
add_subdirectory(mg-msgs)
add_subdirectory(P4)
# Suppress all warnings for yaml-cpp subdirectory
add_compile_options(-w)
add_subdirectory(yaml-cpp)
