cmake_minimum_required(VERSION 3.4)

add_library(Simulator SHARED Simulator.cpp)

# Adding Solver
set(SOLVER_BINARY_DIR ${CMAKE_BINARY_DIR}/Solver)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Solver ${SOLVER_BINARY_DIR})


# Adding opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries(Simulator Solver)
target_link_libraries(Simulator ${OpenCV_LIBS} )
