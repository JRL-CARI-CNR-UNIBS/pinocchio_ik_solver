cmake_minimum_required(VERSION 3.16)
project(pinocchio_ik_solver)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT "$ENV{ROS_VERSION}" STREQUAL "2")
  message(FATAL_ERROR "pinocchio_ik_solver supports ROS 2 only")
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ik_solver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# ROS-free kinematics
add_library(pinocchio_kinematics SHARED src/pinocchio_kinematics.cpp)
target_include_directories(pinocchio_kinematics PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(pinocchio_kinematics PUBLIC pinocchio::pinocchio Eigen3::Eigen)

# Plugin
add_library(${PROJECT_NAME} SHARED src/pinocchio_ik_solver.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio_kinematics)
target_link_libraries(${PROJECT_NAME} PUBLIC ${rclcpp_TARGETS} ${ik_solver_TARGETS} ${pluginlib_TARGETS})

pluginlib_export_plugin_description_file(ik_solver plugin.xml)

install(TARGETS pinocchio_kinematics ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ur_description QUIET)
  find_program(XACRO_EXECUTABLE xacro)
  if(ur_description_FOUND AND XACRO_EXECUTABLE)
    # The test URDF is generated from ur_description, not stored in the package
    set(TEST_URDF ${CMAKE_CURRENT_BINARY_DIR}/ur10e.urdf)
    add_custom_command(OUTPUT ${TEST_URDF}
      COMMAND ${XACRO_EXECUTABLE} ${ur_description_DIR}/../urdf/ur.urdf.xacro ur_type:=ur10e name:=ur -o ${TEST_URDF}
      COMMENT "Generating the UR10e test URDF")
    add_custom_target(test_urdf DEPENDS ${TEST_URDF})

    ament_add_gtest(test_pinocchio_kinematics test/test_pinocchio_kinematics.cpp)
    add_dependencies(test_pinocchio_kinematics test_urdf)
    target_link_libraries(test_pinocchio_kinematics pinocchio_kinematics)
    target_compile_definitions(test_pinocchio_kinematics PRIVATE TEST_URDF_PATH="${TEST_URDF}")
  else()
    message(WARNING "ur_description or xacro not found: tests disabled")
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ik_solver pluginlib pinocchio Eigen3)
ament_package()
