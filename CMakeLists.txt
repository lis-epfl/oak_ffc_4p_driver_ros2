cmake_minimum_required(VERSION 3.10)
project(oak_ffc_4p_driver_ros2)

# DepthAI v3 requires C++14 or newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_compile_options(-g) # compile options for debugging
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

set(DEPENDENCIES
  rclcpp
  image_transport
  cv_bridge)

add_library(oak_ffc_4p_driver src/oak_ffc_4p_driver.cpp)
target_include_directories(oak_ffc_4p_driver PUBLIC include/oak_ffc_4p_driver_ros2 ${OpenCV_INCLUDE_DIRS})

# CHANGED: Linked to depthai::core instead of depthai::opencv
target_link_libraries(oak_ffc_4p_driver ${OpenCV_LIBS} depthai::core)

ament_target_dependencies(oak_ffc_4p_driver ${DEPENDENCIES})

add_executable(oak_ffc_4p_driver_node src/oak_ffc_4p_driver_node.cpp)
target_link_libraries(oak_ffc_4p_driver_node oak_ffc_4p_driver)

add_executable(image_latency_node src/image_latency_node.cpp)
target_link_libraries(image_latency_node ${OpenCV_LIBS})
ament_target_dependencies(image_latency_node ${DEPENDENCIES})

# NOTE: These checkers are commented out because they use DepthAI v2 API
# and will break the v3 build until they are ported.
add_executable(oak_sync_checker src/oak_sync_checker.cpp)
target_link_libraries(oak_sync_checker
  depthai::core
  ${OpenCV_LIBS}
  pthread
)

add_executable(oak_focus_checker src/oak_focus_checker.cpp)
target_link_libraries(oak_focus_checker
  depthai::core
  ${OpenCV_LIBS}
  pthread
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  oak_ffc_4p_driver
  oak_ffc_4p_driver_node
  image_latency_node
  oak_sync_checker
  oak_focus_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
