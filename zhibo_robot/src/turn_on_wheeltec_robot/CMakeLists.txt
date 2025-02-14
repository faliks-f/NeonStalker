cmake_minimum_required(VERSION 3.8)
project(turn_on_wheeltec_robot)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)



set(tf2_geometry_msgs_INCLUDE_DIRS /opt/ros/humble/include/tf2_geometry_msgs)


include_directories(include ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${serial_INCLUDE_DIRS}
        ${tf2_geometry_msgs_INCLUDE_DIRS})

# print all found dependencies
message(STATUS "tf2_geometry_msgs_INCLUDE_DIRS: ${tf2_geometry_msgs_INCLUDE_DIRS}")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

add_executable(wheeltec_robot_node src/main.cpp src/wheeltec_robot.cpp src/quaternion_solution.cpp)
target_link_libraries(wheeltec_robot_node ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES} ${tf2_LIBRARIES} ${serial_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
        wheeltec_robot_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
