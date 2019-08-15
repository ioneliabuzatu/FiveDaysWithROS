cmake_minimum_required(VERSION 2.8.3)
project(service_server_pkg)


find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        message_generation
        roscpp
        rospy
        std_msgs
        std_srvs
        )


## Generate services in the 'srv' folder
add_service_files(
        DIRECTORY srv
        FILES
        MyCustomServiceMessage.srv
)


generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs  # Or other packages containing msgs
)



catkin_package(
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime std_srvs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


add_executable(bb8_server src/bb8_server.cpp)
add_dependencies(bb8_server ${bb8_server_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bb8_server
        ${catkin_LIBRARIES}
        )

add_executable(bb8_client src/bb8_client.cpp)
add_dependencies(bb8_client ${bb8_client_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bb8_client
        ${catkin_LIBRARIES}
        )

## add_dependencies(source_file_name package_name_generate_messages_cpp)
add_dependencies(bb8_server service_server_pkg_generate_messages_cpp) ## this avoids fatal error: no such file found CRUCIAL!!!
add_dependencies(bb8_client service_server_pkg_generate_messages_cpp)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)