cmake_minimum_required(VERSION 3.22)
project(waterlinked_sonar_driver)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(waterlinkedsonar REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} srv/SetRange.srv)
rosidl_get_typesupport_target(
    cpp_typesupport_target
    ${PROJECT_NAME}
    rosidl_typesupport_cpp
)

generate_parameter_library(
    ${PROJECT_NAME}_parameters
    src/waterlinked_sonar_driver_parameters.yaml
)

# rosidl_generate_interfaces owns the target named after the project.
add_library(${PROJECT_NAME}_component SHARED src/waterlinked_sonar_driver.cpp)
target_compile_features(${PROJECT_NAME}_component PRIVATE cxx_std_17)
target_compile_options(
    ${PROJECT_NAME}_component
    PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>
)
target_link_libraries(
    ${PROJECT_NAME}_component
    PRIVATE
        ${PROJECT_NAME}_parameters
        ${cpp_typesupport_target}
        ${diagnostic_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${std_srvs_TARGETS}
        diagnostic_updater::diagnostic_updater
        rclcpp::rclcpp
        rclcpp_components::component
        rclcpp_lifecycle::rclcpp_lifecycle
        waterlinkedsonar::waterlinkedsonar
)
rclcpp_components_register_node(
    ${PROJECT_NAME}_component
    PLUGIN "waterlinked::sonar::ros::WaterlinkedSonarDriver"
    EXECUTABLE ${PROJECT_NAME}_node
)

install(
    TARGETS ${PROJECT_NAME}_component
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_driver.py)
endif()

ament_package()
