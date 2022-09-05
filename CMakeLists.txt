project(asoa_core)
set(CMAKE_BUILD_TYPE Debug)
cmake_minimum_required(VERSION 3.15)

## Compile and include Eigen
add_subdirectory(thirdparty/eigen)
include_directories(thirdparty/eigen)

## Compile and inlcude nlohmann json
include_directories(thirdparty/json/include)
add_subdirectory(thirdparty/json)

## Compile main planner library
include_directories("include")
file(GLOB SRC "src/Planner/*.cpp" "src/DynamicsModel/*.cpp" "src/util/*.cpp")
add_library(planner_lib SHARED ${SRC})


add_executable(main src/main.cpp)
target_link_libraries(main planner_lib)

