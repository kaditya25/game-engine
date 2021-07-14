cmake_minimum_required(VERSION 2.8.3)

set(TARGET lib_asset_games)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(SOURCE_FILES
  nash.cpp
  sync_future.cpp
  helper.cpp
  vel_match_controller.cpp
  thread_pool.cpp
  future_combinator.cpp
  cost_calculator.cpp
  nash_controller.cpp
  )

add_library(${TARGET} STATIC ${SOURCE_FILES})

target_link_libraries(${TARGET} PUBLIC
  Eigen3::Eigen
  Threads::Threads)
