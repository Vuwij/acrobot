cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_NAME acrobot)

set(${PROJECT_NAME}_BOARD mkr1000)
project(${PROJECT_NAME} C CXX)
set_target_properties(hello PROPERTIES LINKER_LANGUAGE CXX)

include_directories(/home/vuwij/Arduino/libraries/Adafruit_BNO055)
add_executable(acrobot main/main.ino)
target_include_directories(acrobot /home/vuwij/Arduino/libraries/Adafruit_BNO055)
