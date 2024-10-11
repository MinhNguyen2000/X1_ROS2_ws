cmake_minimum_required(VERSION 3.8)
project(x1_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Instruction to generate the interfaces for the service messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
  "msg/PointArray.msg"
  "msg/ImageMsg.msg"
  "msg/Target.msg"
  "msg/TargetArray.msg"
  DEPENDENCIES geometry_msgs actionlib_msgs builtin_interfaces
)


# ## Export dependencies ##
# # This package depends on geometry_msgs, actionlib_msgs, and rosidl_default_runtime
# ament_export_dependencies(rosidl_default_runtime)
# ament_export_dependencies(geometry_msgs)
# ament_export_dependencies(actionlib_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
