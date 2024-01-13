cmake_minimum_required(VERSION 3.27)
project(ASCII)

set(CMAKE_CXX_STANDARD 23)

add_executable(ASCII main.cpp)

# Find packages of OpenCV on your system
find_package(OpenCV REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Link libraries
target_link_libraries(ASCII ${OpenCV_LIBS})
