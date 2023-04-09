project(planner)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE debug)
cmake_minimum_required(VERSION 3.10)

option(WITH_NLOPT "Inlcude nlopt dependend executables in compile" ON)

## Compile and include Eigen
add_subdirectory(thirdparty/eigen)
include_directories(thirdparty/eigen)

## Compile and inlcude nlohmann json
include_directories(thirdparty/json/include)
add_subdirectory(thirdparty/json)

## Compile main planner library
include_directories("include")


if(${WITH_NLOPT})

file(GLOB SRC "src/Planner/*.cpp" "src/MPComputeNL/*.cpp" "src/MPCompute/*.cpp" "src/DynamicsModel/*.cpp" "src/util/*.cpp" "src/*")
add_library(planner_lib SHARED ${SRC})
target_link_libraries(planner_lib pthread nlopt m)

else()

file(GLOB SRC "src/Planner/*.cpp"  "src/MPCompute/*.cpp" "src/DynamicsModel/*.cpp" "src/util/*.cpp" "src/*")
add_library(planner_lib SHARED ${SRC})
target_link_libraries(planner_lib pthread)

endif()

file(GLOB EXEC_SRC "src/Exec/*.cpp" "src/Experiments/*.cpp")
foreach(src_ ${EXEC_SRC})
	get_filename_component(target ${src_} NAME)
    
	string(REPLACE ".cpp" "" target ${target})
	string(REPLACE "/" "_" target ${target})
    message(STATUS "Found executable : ${target}")

	add_executable(${target} ${src_})
    target_link_libraries(${target} planner_lib)
endforeach()

file(GLOB EXEC_SRC "src/Tests/*.cpp")
foreach(src_ ${EXEC_SRC})
	get_filename_component(target ${src_} NAME)
    
	string(REPLACE ".cpp" "" target ${target})
	string(REPLACE "/" "_" target ${target})
    message(STATUS "Found executable : ${target}")

	add_executable(${target} ${src_})
    target_link_libraries(${target} planner_lib)
endforeach()

if(${WITH_NLOPT})

	file(GLOB EXEC_SRC "src/ExecNL/*.cpp")
	foreach(src_ ${EXEC_SRC})
		get_filename_component(target ${src_} NAME)
		
		string(REPLACE ".cpp" "" target ${target})
		string(REPLACE "/" "_" target ${target})
		message(STATUS "Found executable : ${target}")

		add_executable(${target} ${src_})
		target_link_libraries(${target} planner_lib)
	endforeach()

	file(GLOB EXEC_SRC "src/TestsNL/*.cpp")
	foreach(src_ ${EXEC_SRC})
		get_filename_component(target ${src_} NAME)
		
		string(REPLACE ".cpp" "" target ${target})
		string(REPLACE "/" "_" target ${target})
		message(STATUS "Found executable : ${target}")

		add_executable(${target} ${src_})
		target_link_libraries(${target} planner_lib)
	endforeach()
endif()


