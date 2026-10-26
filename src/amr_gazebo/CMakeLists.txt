cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
# Gazebo libraries through the ROS vendor packages (Harmonic on Jazzy,
# Jetty on newer distros) - versionless targets work for both.
# ROS 2 Humble has no vendor packages: use the Gazebo Harmonic system
# libraries from packages.osrfoundation.org (gz-sim 8).
find_package(gz_sim_vendor QUIET)
if(gz_sim_vendor_FOUND)
  find_package(gz-sim REQUIRED)
  find_package(gz_plugin_vendor REQUIRED)
  find_package(gz-plugin REQUIRED COMPONENTS register)
  find_package(gz_transport_vendor REQUIRED)
  find_package(gz-transport REQUIRED)
  find_package(gz_msgs_vendor REQUIRED)
  find_package(gz-msgs REQUIRED)
  find_package(sdformat_vendor REQUIRED)
  find_package(sdformat REQUIRED)
  set(GZ_LIBS
    gz-sim::gz-sim gz-plugin::register gz-transport::core gz-msgs::core sdformat::sdformat)
else()
  find_package(gz-sim8 REQUIRED)
  find_package(gz-plugin2 REQUIRED COMPONENTS register)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  find_package(sdformat14 REQUIRED)
  set(GZ_LIBS
    gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::core gz-msgs10::core
    sdformat14::sdformat14)
endif()

add_library(amr_line_animator SHARED src/LineAnimator.cc)
target_link_libraries(amr_line_animator ${GZ_LIBS})
add_library(amr_pedestrians SHARED src/Pedestrians.cc)
target_link_libraries(amr_pedestrians ${GZ_LIBS})
add_library(amr_station_arms SHARED src/StationArms.cc)
target_link_libraries(amr_station_arms ${GZ_LIBS})
install(TARGETS amr_line_animator amr_pedestrians amr_station_arms LIBRARY DESTINATION lib/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
install(DIRECTORY launch worlds models maps DESTINATION share/${PROJECT_NAME})
ament_package()
