cmake_minimum_required(VERSION 3.10)
project(axon_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect ROS version and set appropriate macro
if(DEFINED ENV{ROS_VERSION})
    if($ENV{ROS_VERSION} EQUAL 1)
        message(STATUS "Building for ROS 1")
        add_definitions(-DAXON_ROS1)
        set(AXON_ROS_VERSION 1)
    elseif($ENV{ROS_VERSION} EQUAL 2)
        message(STATUS "Building for ROS 2")
        add_definitions(-DAXON_ROS2)
        set(AXON_ROS_VERSION 2)
    endif()
else()
    message(WARNING "ROS_VERSION not set, defaulting to ROS 1")
    add_definitions(-DAXON_ROS1)
    set(AXON_ROS_VERSION 1)
endif()

# Find Arrow
find_package(Arrow REQUIRED)

# ROS 1 specific setup
if(AXON_ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        message_generation
    )
    
    # Generate service messages
    add_service_files(
        FILES
        srv/StartRecording.srv
        srv/StopRecording.srv
        srv/UpdateConfig.srv
        srv/GetStatus.srv
    )
    
    generate_messages(
        DEPENDENCIES
        std_msgs
    )
    
    catkin_package(
        INCLUDE_DIRS src
        LIBRARIES axon_recorder_lib
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs message_runtime
    )
    
    include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Arrow_INCLUDE_DIRS}
        src
        ../../cpp/src
        ../../c/include
    )
    
    # Library
    add_library(axon_recorder_lib
        src/ros_interface.cpp
        src/message_registry.cpp
        src/ros_introspection.cpp
        src/message_factory.cpp
        src/register_common_messages.cpp
        src/recorder_service.cpp
    )
    
    target_link_libraries(axon_recorder_lib
        ${catkin_LIBRARIES}
        Arrow::arrow_shared
        axon_cpp
        axon_lance_bridge
    )
    
    # Executable
    add_executable(axon_recorder_node
        src/recorder_node.cpp
    )
    
    target_link_libraries(axon_recorder_node
        axon_recorder_lib
        ${catkin_LIBRARIES}
        Arrow::arrow_shared
        axon_cpp
        axon_lance_bridge
    )
    
    add_dependencies(axon_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    
    # Install
    install(TARGETS axon_recorder_lib axon_recorder_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    
    install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
    
    install(DIRECTORY srv/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
        FILES_MATCHING PATTERN "*.srv"
    )
    
# ROS 2 specific setup
elseif(AXON_ROS_VERSION EQUAL 2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    
    # Generate service messages
    rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/StartRecording.srv"
        "srv/StopRecording.srv"
        "srv/UpdateConfig.srv"
        "srv/GetStatus.srv"
        DEPENDENCIES std_msgs
    )
    
    include_directories(
        ${Arrow_INCLUDE_DIRS}
        src
        ../../cpp/src
        ../../c/include
    )
    
    # Library
    add_library(axon_recorder_lib SHARED
        src/ros_interface.cpp
        src/message_registry.cpp
        src/ros_introspection.cpp
        src/message_factory.cpp
        src/register_common_messages.cpp
        src/recorder_service.cpp
    )
    
    ament_target_dependencies(axon_recorder_lib
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
    )
    
    target_link_libraries(axon_recorder_lib
        Arrow::arrow_shared
        axon_cpp
        axon_lance_bridge
    )
    
    # Executable
    add_executable(axon_recorder_node
        src/recorder_node.cpp
    )
    
    ament_target_dependencies(axon_recorder_node
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
    )
    
    target_link_libraries(axon_recorder_node
        axon_recorder_lib
        Arrow::arrow_shared
        axon_cpp
        axon_lance_bridge
    )
    
    rosidl_target_interfaces(axon_recorder_node
        ${PROJECT_NAME} rosidl_typesupport_cpp
    )
    
    # Install
    install(TARGETS axon_recorder_lib axon_recorder_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
    )
    
    install(DIRECTORY srv/
        DESTINATION share/${PROJECT_NAME}/srv
        FILES_MATCHING PATTERN "*.srv"
    )
    
    ament_package()
endif()
