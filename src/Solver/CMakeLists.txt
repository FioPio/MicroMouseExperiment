cmake_minimum_required(VERSION 3.4)

add_library(Solver SHARED Solver.cpp)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(Solver ${OpenCV_LIBS} )
