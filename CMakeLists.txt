cmake_minimum_required(VERSION 3.5)
project(yard_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
include_directories( include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} )

ament_auto_find_build_dependencies()

ament_auto_add_executable(yard_slam_node src/YardSlam.cpp src/EKFPose.cpp src/ObsList.cpp)
target_link_libraries(yard_slam_node
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)

install(DIRECTORY
  launch
  config
  
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
