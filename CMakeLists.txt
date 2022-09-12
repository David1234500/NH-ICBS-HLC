project(planner)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_BUILD_TYPE Debug)
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


file(GLOB EXEC_SRC "src/Exec/*.cpp")
foreach(src_ ${EXEC_SRC})
	get_filename_component(target ${src_} NAME)
    
	string(REPLACE ".cpp" "" target ${target})
	string(REPLACE "/" "_" target ${target})
    message(STATUS "Found executable : ${target}")

	add_executable(${target} ${src_})
    target_link_libraries(${target} planner_lib)
endforeach()




