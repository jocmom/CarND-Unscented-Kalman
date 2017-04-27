project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   src/ukf.cpp
   src/main.cpp
   src/tools.cpp)

add_executable(UnscentedKF ${sources})

add_library(Catch INTERFACE)

set(TEST_SOURCES 
    src/ukf.cpp
    src/tests-ukf.cpp)
add_executable(tests ${TEST_SOURCES})
target_link_libraries(tests Catch)