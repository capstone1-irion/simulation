cmake_minimum_required(VERSION 3.16)
project(arion_motor_telemetry LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(ArionFixedMotorPlugin SHARED plugins/motor/FixedMotorState.cpp)
target_link_libraries(ArionFixedMotorPlugin PRIVATE
    gz-sim8::gz-sim8 gz-transport13::core gz-plugin2::register)
target_compile_options(ArionFixedMotorPlugin PRIVATE -Wall -Wextra -Werror)

enable_testing()
add_executable(motor_frame_test tests/motor_frame_test.cpp)
target_include_directories(motor_frame_test PRIVATE include)
target_compile_options(motor_frame_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME motor_frame_mapping COMMAND motor_frame_test)
add_test(NAME motor_frame_nonfinite COMMAND motor_frame_test nonfinite)
add_test(NAME motor_frame_rpm_range COMMAND motor_frame_test nonfinite range)
