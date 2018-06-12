cmake_minimum_required(VERSION 2.8.3)
project(hpr)

find_package(catkin REQUIRED COMPONENTS
    laserscan_stacker
    pointcloud2_segments_viz
)

catkin_package()

include_directories()
