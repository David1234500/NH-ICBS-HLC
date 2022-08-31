project(asoa_core)
set(CMAKE_BUILD_TYPE Debug)
cmake_minimum_required(VERSION 3.15)

## Compile and include Eigen
add_subdirectory(thirdparty/eigen)
include_directories(thirdparty/eigen)


## have to include ciaro manually... no cmake integration
#include_directories("/usr/local/Cellar/cairo/1.16.0_5/include")
#find_library(CAIRO_LIB_RELEASE cairo)

## Compile main planner library
include_directories("include")
file(GLOB SRC "src/Planner/*.cpp" "src/DynamicsModel/*.cpp" "src/util/*.cpp")
add_library(planner_lib SHARED ${SRC})
#target_link_libraries(planner_lib ${CAIRO_LIB_RELEASE})

add_executable(main src/main.cpp)
target_link_libraries(main planner_lib)

