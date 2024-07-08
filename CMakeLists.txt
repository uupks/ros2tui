cmake_minimum_required(VERSION 3.8)
project(ros2tui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG v5.0.0
)

FetchContent_GetProperties(ftxui)
if(NOT ftxui_POPULATED)
  FetchContent_Populate(ftxui)
  add_subdirectory(${ftxui_SOURCE_DIR} ${ftxui_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynmsg REQUIRED)
find_package(dynmsg_demo REQUIRED)
find_package(spdlog REQUIRED)

add_executable(ros2tui
  src/main.cpp
  src/ros2tui.cpp
)

target_link_libraries(ros2tui
  PRIVATE 
    ftxui::component
    ftxui::dom
    ftxui::screen
    rclcpp::rclcpp 
    dynmsg::dynmsg
    dynmsg_demo::dynmsg_demo_library
    spdlog::spdlog
)

target_include_directories(ros2tui
  PRIVATE include
)

install(
  TARGETS ros2tui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
