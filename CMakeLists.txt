cmake_minimum_required(VERSION 3.5)
project(node_registry)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)
ament_python_install_package(${PROJECT_NAME})

# rosnode cpp
add_library(${PROJECT_NAME} SHARED
  src/rosnode.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

target_link_libraries(${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(PROGRAMS
  ${PROJECT_NAME}/xnode.py
  ${PROJECT_NAME}/xnode_builder.py
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY ${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(node_registry test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python
  rclcpp
  rclcpp_components)
ament_package()
