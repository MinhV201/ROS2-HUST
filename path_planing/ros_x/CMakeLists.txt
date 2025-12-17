cmake_minimum_required(VERSION 3.8)
project(ros_x)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==========================================
# 1. DEPENDENCIES
# ==========================================
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ==========================================
# 2. INTERFACES (MSG & SRV)
# ==========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PFStatus.msg"
  "msg/PathSegment.msg"
  "srv/SetPath.srv"
  "msg/PathDef.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# ==========================================
# 3. BUILD LIBRARY (Thư viện thuật toán)
# ==========================================
add_library(pf_library
  src/PathLibs.cpp
  src/PF_algorithm.cpp
)


target_include_directories(pf_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ==========================================
# 4. BUILD NODE (Executable)
# ==========================================
add_executable(pf_node src/pf_node.cpp)

# --- QUAN TRỌNG: Link Node với Thư viện thuật toán ---
target_link_libraries(pf_node pf_library)

# Link Node với các thư viện ROS
ament_target_dependencies(pf_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Link Node với Message/Service sinh ra từ chính package này
rosidl_get_typesupport_target(cpp_typesupport_target
  "${PROJECT_NAME}"
  "rosidl_typesupport_cpp"
)
target_link_libraries(pf_node "${cpp_typesupport_target}")


# ==========================================
# 5. INSTALLATION 
# ==========================================
install(TARGETS
  pf_library
  pf_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES
  msg/PFStatus.msg
  srv/SetPath.srv
  DESTINATION share/${PROJECT_NAME}
)

# ==========================================
# 6. TESTING & PACKAGE
# ==========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()