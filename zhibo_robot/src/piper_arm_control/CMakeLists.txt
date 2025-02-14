cmake_minimum_required(VERSION 3.8)
project(piper_arm_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(piper_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(include ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS}
${piper_msgs_INCLUDE_DIRS})

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

add_executable(sim_move_demo src/sim_move_demo.cpp)
target_link_libraries(sim_move_demo ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES}
${piper_msgs_LIBRARIES})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
        sim_move_demo
        DESTINATION lib/${PROJECT_NAME}
)


ament_package()
