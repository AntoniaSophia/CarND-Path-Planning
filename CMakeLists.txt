project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp 
            src/EgoVehicle.cpp
            src/AbstractVehicle.cpp 
            src/SensorObject.cpp 
            src/BehaviorPlanner.cpp
            src/TrajectoryPlanner.cpp
            src/TrajectoryPlannerWIP.cpp
            )



include_directories("${PROJECT_SOURCE_DIR}/src/Eigen-3.3/Eigen")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include "${PROJECT_SOURCE_DIR}/src/Eigen-3.3/Eigen")
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
