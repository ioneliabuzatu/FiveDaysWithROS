cmake_minimum_required(VERSION 3.14)
project(robot_avoiding_wall)

set(CMAKE_CXX_STANDARD 14)

add_executable(read_write_wall src/read_write_wall .cpp)
add_dependencies(read_write_wall  ${laser_subscriber_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_write_wall
        ${catkin_LIBRARIES}
        )


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        )

catkin_package(
)


include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

