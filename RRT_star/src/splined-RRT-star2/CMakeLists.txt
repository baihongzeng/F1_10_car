cmake_minimum_required(VERSION 3.14)
project(splined_RRT_star2)

set(CMAKE_CXX_STANDARD 11)

include_directories(.)

add_executable(splined_RRT_star2
        Collision_Constraints.cpp
        Collision_Constraints.h
        constants.h
        curve.cpp
        curve.h
        dubins.cpp
        dubins.h
        kdtree.cpp
        kdtree.h
        Line_Angle.cpp
        Line_Angle.h
        main.cpp
        NodeClass.cpp
        NodeClass.h
        plot.cpp
        plot.h
        splined-RRT-star.cpp
        splined-RRT-star.h)
