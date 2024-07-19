cmake_minimum_required(VERSION 3.5)

project(mros_pico_bridge)

set(CMAKE_CXX_STANDARD 11)

find_library(PTHREAD pthread REQUIRED)

find_package(mros REQUIRED)

add_executable(mros_pico_bridge src/main.cpp)
target_include_directories(mros_pico_bridge PRIVATE ${MROS_INCLUDE_DIRS})
target_link_libraries(mros_pico_bridge ${MROS_LIBRARIES} ${PTHREAD})