cmake_minimum_required(VERSION 3.8)
project(allan_variance_ros2)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable compile optimizations for release
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -Wall -g -ggdb")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
  message(STATUS "OpenMP found will try to link!")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
 src/ImuMeasurement.cpp
 src/yaml_parsers.cpp
 src/AllanVarianceComputor.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp rosbag2_cpp sensor_msgs yaml-cpp Boost)

add_executable(allan_variance src/allan_variance.cpp)
target_link_libraries(allan_variance ${PROJECT_NAME})
add_executable(imu_simulator src/ImuSimulator.cpp)
target_link_libraries(imu_simulator ${PROJECT_NAME})

install(TARGETS allan_variance imu_simulator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)

install(PROGRAMS
  scripts/analysis.py
  DESTINATION lib/${PROJECT_NAME}
)

file(GLOB PYTHON_SCRIPTS scripts/*.py)
foreach(script ${PYTHON_SCRIPTS})
  execute_process(
    COMMAND chmod +x ${script}
  )
endforeach()

ament_package()
