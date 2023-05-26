cmake_minimum_required(VERSION 3.4)

add_library(Simulator SHARED Simulator.cpp)

# Añadir Utils
#set(UTILS_BINARY_DIR ${CMAKE_BINARY_DIR}/Utils)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Utils ${UTILS_BINARY_DIR})

# Añadir Alsa                                    
#include_directories(${ALSA_INCLUDE_DIRS})    

#target_link_libraries(VoiceAnalizer vosk)
#target_link_libraries(VoiceAnalizer Utils)

#target_link_libraries (VoiceAnalizer ${ALSA_LIBRARIES}) 

# Adding opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries( Simulator ${OpenCV_LIBS} )