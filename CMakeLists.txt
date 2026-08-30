cmake_minimum_required(VERSION 3.16)
project(insta360_camera)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -O2)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AV REQUIRED libavcodec libavutil libswscale)

# A colcon workspace has ament; a wheel build does not. Detect it rather than
# require it, so the reader and its Python bindings build with no ROS present.
find_package(ament_cmake QUIET)
option(INSTA360_BUILD_ROS "Build the ROS 2 node and package" ${ament_cmake_FOUND})
option(INSTA360_BUILD_PYTHON "Build the Python extension module" OFF)

if(INSTA360_BUILD_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(ffmpeg_image_transport_msgs REQUIRED)
endif()

set(SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk)

# ROS-free reader library
add_library(insta360_reader SHARED src/reader.cpp)
# Only reader.h is public (it is SDK-free); the SDK + ffmpeg headers are
# PRIVATE implementation details of reader.cpp.
target_include_directories(insta360_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(insta360_reader PRIVATE
  ${SDK_DIR}/include ${AV_INCLUDE_DIRS})
target_link_libraries(insta360_reader
  ${SDK_DIR}/lib/libCameraSDK.so ${AV_LIBRARIES})

if(INSTA360_BUILD_PYTHON)
  find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
  find_package(nanobind CONFIG REQUIRED)
  nanobind_add_module(_core STABLE_ABI NB_STATIC python/bindings.cpp)
  target_include_directories(_core PRIVATE ${AV_INCLUDE_DIRS})
  target_link_libraries(_core PRIVATE insta360_reader ${AV_LIBRARIES})
  # The wheel carries the reader and the vendored SDK beside the module, so
  # both have to be found relative to it rather than on the library path.
  set_target_properties(insta360_reader PROPERTIES INSTALL_RPATH "$ORIGIN")
  set_target_properties(_core PROPERTIES INSTALL_RPATH "$ORIGIN")
  install(TARGETS _core insta360_reader LIBRARY DESTINATION insta360_driver)
  install(FILES ${SDK_DIR}/lib/libCameraSDK.so DESTINATION insta360_driver)
endif()

if(NOT INSTA360_BUILD_ROS)
  return()
endif()

# Composable node (+ standalone executable)
add_library(camera_component SHARED src/camera_node.cpp)
# camera_node.cpp calls sws_scale directly, so link libav here, not via reader.
target_include_directories(camera_component PRIVATE ${AV_INCLUDE_DIRS})
target_link_libraries(camera_component
  insta360_reader
  ${AV_LIBRARIES}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${ffmpeg_image_transport_msgs_TARGETS})
rclcpp_components_register_node(camera_component
  PLUGIN "insta360_camera::CameraNode"
  EXECUTABLE camera_node)

install(TARGETS camera_component insta360_reader
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib)
install(FILES ${SDK_DIR}/lib/libCameraSDK.so DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Opt in with: colcon build --cmake-args -DINSTA360_BUILD_BENCHMARKS=ON
option(INSTA360_BUILD_BENCHMARKS
  "Build latency/benchmark tools (dev only, not for production)" OFF)
if(INSTA360_BUILD_BENCHMARKS)
  find_package(OpenCV REQUIRED)  # benchmark-only: aruco detector

  add_library(latency_probe_component SHARED test/latency_probe.cpp)
  target_link_libraries(latency_probe_component
    rclcpp::rclcpp rclcpp_components::component ${sensor_msgs_TARGETS})
  rclcpp_components_register_node(latency_probe_component
    PLUGIN "insta360_camera::LatencyProbe"
    EXECUTABLE latency_probe)

  add_library(aruco_latency_component SHARED test/aruco_latency_test.cpp)
  target_link_libraries(aruco_latency_component
    ${OpenCV_LIBS}
    rclcpp::rclcpp rclcpp_components::component ${sensor_msgs_TARGETS})
  rclcpp_components_register_node(aruco_latency_component
    PLUGIN "insta360_camera::ArucoLatencyTest"
    EXECUTABLE aruco_latency_test)

  install(TARGETS latency_probe_component aruco_latency_component
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib)
endif()

ament_package()
