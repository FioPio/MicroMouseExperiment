add_subdirectory("Simulator")

add_executable(out main.cpp)
target_link_libraries(out Simulator)
