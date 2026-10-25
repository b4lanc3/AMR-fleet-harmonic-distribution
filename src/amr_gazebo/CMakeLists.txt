cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
# Gazebo libraries through the ROS vendor packages (Harmonic on Jazzy,
# Jetty on newer distros) - versionless targets work for both.
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(sdformat_vendor REQUIRED)
find_package(sdformat REQUIRED)

add_library(amr_line_animator SHARED src/LineAnimator.cc)
target_link_libraries(amr_line_animator
  gz-sim::gz-sim gz-plugin::register gz-transport::core gz-msgs::core sdformat::sdformat)
add_library(amr_pedestrians SHARED src/Pedestrians.cc)
target_link_libraries(amr_pedestrians
  gz-sim::gz-sim gz-plugin::register gz-transport::core gz-msgs::core sdformat::sdformat)
install(TARGETS amr_line_animator amr_pedestrians LIBRARY DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
install(DIRECTORY launch worlds models maps DESTINATION share/${PROJECT_NAME})
ament_package()
