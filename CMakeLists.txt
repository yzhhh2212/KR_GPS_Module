cmake_minimum_required(VERSION 3.8)
project(gps)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)

add_definitions(${PCL_DEFINITIONS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ./include/
  ./include/gps/
)

# 1. 添加源文件
add_executable(gps_shower src/gps.cpp src/gps_sub.cpp)

# 2. 添加依赖
ament_target_dependencies(gps_shower
  rclcpp
  message_filters
  cv_bridge
  nav_msgs
)

target_link_libraries( gps_shower
  ${EIGEN3_LIBS}
  ${GeographicLib_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
)


install(TARGETS
  gps_shower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
