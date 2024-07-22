cmake_minimum_required(VERSION 3.5)

project(mros_pico_bridge)

set(CMAKE_CXX_STANDARD 11)

find_library(PTHREAD pthread REQUIRED)

# Find ncurses
find_package(Curses REQUIRED)

find_package(mros REQUIRED)

add_executable(mros_pico_bridge src/main.cpp src/mbotSerialBridge.cpp)
target_include_directories(mros_pico_bridge PRIVATE include ${MROS_INCLUDE_DIRS})
target_link_libraries(mros_pico_bridge ${MROS_LIBRARIES} ${PTHREAD})

add_executable(listener_test src/listenerTest.cpp)
target_include_directories(listener_test PRIVATE include ${MROS_INCLUDE_DIRS})
target_link_libraries(listener_test ${MROS_LIBRARIES} ${PTHREAD})

add_executable(teleop_test src/teleopTest.cpp)
target_include_directories(teleop_test PRIVATE include ${MROS_INCLUDE_DIRS})
target_link_libraries(teleop_test ${MROS_LIBRARIES} ${PTHREAD} ${CURSES_LIBRARIES})