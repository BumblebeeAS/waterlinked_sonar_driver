cmake_minimum_required(VERSION 3.22)
project(waterlinked_sonar_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(waterlinkedsonar REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    srv/SetRange.srv
)
rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp
)

generate_parameter_library(waterlinked_sonar_driver_parameters
    src/waterlinked_sonar_driver_parameters.yaml
)

add_library(waterlinked_sonar_driver_library SHARED)
target_sources(
    waterlinked_sonar_driver_library
    PRIVATE src/waterlinked_sonar_driver.cpp
)
target_include_directories(
    waterlinked_sonar_driver_library
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(waterlinked_sonar_driver_library PUBLIC cxx_std_17)
target_link_libraries(
    waterlinked_sonar_driver_library
    PRIVATE waterlinked_sonar_driver_parameters "${cpp_typesupport_target}"
    PUBLIC
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        rclcpp_components::component
        waterlinkedsonar::waterlinkedsonar
        diagnostic_updater::diagnostic_updater
        ${sensor_msgs_TARGETS}
        ${std_srvs_TARGETS}
)

# The interface-generation target owns the plain project name.
rclcpp_components_register_node(waterlinked_sonar_driver_library
    PLUGIN "waterlinked::sonar::ros::WaterlinkedSonarDriver"
    EXECUTABLE waterlinked_sonar_driver_node
)

install(
    TARGETS waterlinked_sonar_driver_library waterlinked_sonar_driver_parameters
    EXPORT export_waterlinked_sonar_driver_library
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY config launch DESTINATION share/waterlinked_sonar_driver)

ament_export_targets(export_waterlinked_sonar_driver_library
    HAS_LIBRARY_TARGET
)
ament_export_dependencies(
    "rclcpp"
    "rclcpp_lifecycle"
    "rclcpp_components"
    "sensor_msgs"
    "std_srvs"
    "diagnostic_updater"
    "waterlinkedsonar"
    "rosidl_default_runtime"
)

ament_package()
