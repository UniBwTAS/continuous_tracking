cmake_minimum_required(VERSION 3.0.2)
project(continuous_tracking)

find_package(catkin REQUIRED COMPONENTS roscpp tf2_eigen tf2_ros object_instance_msgs visualization_msgs dynamic_reconfigure message_filters image_geometry)

generate_dynamic_reconfigure_options(
        cfg/ContinuousTracker.cfg
)

catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES continuous_tracking
        CATKIN_DEPENDS roscpp tf2_eigen tf2_ros object_instance_msgs visualization_msgs dynamic_reconfigure message_filters image_geometry
        # DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(continuous_tracker_node src/continuous_tracker_node.cpp)
target_link_libraries(continuous_tracker_node ${catkin_LIBRARIES})
