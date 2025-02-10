cmake_minimum_required(VERSION 3.30)
project(planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 设置release编译
set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include/)

file(GLOB SOURCES_DIR "src/*.cpp")

add_executable(planner main.cpp ${SOURCES_DIR})

target_link_libraries(planner ${OpenCV_LIBS})