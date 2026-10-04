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

# Opt-in only: existing motor-only builds and the custom_x500 model stay intact.
option(ARION_BUILD_BATTERY_COMPONENTS "Build fixed battery plugin and external MAVLink sender" OFF)
set(ARION_MAVLINK_INCLUDE_DIR "" CACHE PATH "Generated MAVLink C include root containing common/mavlink.h")
if(ARION_BUILD_BATTERY_COMPONENTS)
    if(NOT EXISTS "${ARION_MAVLINK_INCLUDE_DIR}/common/mavlink.h")
        message(FATAL_ERROR "Set ARION_MAVLINK_INCLUDE_DIR to the pinned PX4 generated mavlink directory; no automatic download or PX4 rebuild")
    endif()
    find_package(Threads REQUIRED)

    add_library(ArionFixedBatteryPlugin SHARED plugins/battery/FixedBatteryState.cpp)
    target_include_directories(ArionFixedBatteryPlugin PRIVATE include)
    target_link_libraries(ArionFixedBatteryPlugin PRIVATE
        gz-sim8::gz-sim8 gz-transport13::core gz-plugin2::register)
    target_compile_options(ArionFixedBatteryPlugin PRIVATE -Wall -Wextra -Werror)

    add_executable(arion_battery_sender tools/battery_sender.cpp)
    target_include_directories(arion_battery_sender PRIVATE include)
    target_include_directories(arion_battery_sender SYSTEM PRIVATE "${ARION_MAVLINK_INCLUDE_DIR}")
    target_link_libraries(arion_battery_sender PRIVATE gz-transport13::core Threads::Threads)
    target_compile_options(arion_battery_sender PRIVATE -Wall -Wextra -Werror)

    # static_assert only. Building this target does not execute any test process.
    add_library(battery_contract_compile_test OBJECT tests/battery_contract_compile_test.cpp)
    target_include_directories(battery_contract_compile_test PRIVATE include)
    target_compile_options(battery_contract_compile_test PRIVATE -Wall -Wextra -Werror)
endif()
