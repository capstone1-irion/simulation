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
option(ARION_BUILD_COUPLED_POWER "Build load-dependent motor/battery simulation" OFF)
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

if(ARION_BUILD_COUPLED_POWER)
    add_library(ArionCoupledPowerPlugin SHARED plugins/power/CoupledPowerState.cpp)
    target_include_directories(ArionCoupledPowerPlugin PRIVATE include)
    target_link_libraries(ArionCoupledPowerPlugin PRIVATE
        gz-sim8::gz-sim8 gz-transport13::core gz-plugin2::register)
    target_compile_options(ArionCoupledPowerPlugin PRIVATE -Wall -Wextra -Werror)
    add_executable(coupled_power_model_test tests/coupled_power_model_test.cpp)
    target_include_directories(coupled_power_model_test PRIVATE include)
    target_compile_options(coupled_power_model_test PRIVATE -Wall -Wextra -Werror)
    add_test(NAME coupled_power_model COMMAND coupled_power_model_test)
    add_executable(arion_power_observer tools/power_observer.cpp)
    target_link_libraries(arion_power_observer PRIVATE gz-transport13::core)
    target_compile_options(arion_power_observer PRIVATE -Wall -Wextra -Werror)

    # 운영 제어기: Plugin의 요청 수락과 실제 적용 revision을 구분한다.
    add_executable(arion_power_control tools/power_control.cpp)
    target_include_directories(arion_power_control PRIVATE include)
    target_link_libraries(arion_power_control PRIVATE gz-transport13::core)
    target_compile_options(arion_power_control PRIVATE -Wall -Wextra -Werror)

    add_executable(cmu_power_model_test tests/cmu_power_model_test.cpp)
    target_include_directories(cmu_power_model_test PRIVATE include)
    target_compile_options(cmu_power_model_test PRIVATE -Wall -Wextra -Werror)
    add_test(NAME cmu_power_model COMMAND cmu_power_model_test)

    # 선택형 제어의 재현 가능한 집중시험; 운영 서비스는 이 실행파일을 호출하지 않는다.
    find_package(Threads REQUIRED)
    add_executable(test_power_control tests/test_power_control.cpp)
    target_include_directories(test_power_control PRIVATE include)
    target_link_libraries(test_power_control PRIVATE Threads::Threads)
    target_compile_options(test_power_control PRIVATE -Wall -Wextra -Werror)
    add_test(NAME power_control_mailbox COMMAND test_power_control)

    add_executable(test_power_control_cli tests/test_power_control_cli.cpp)
    target_include_directories(test_power_control_cli PRIVATE include)
    target_link_libraries(test_power_control_cli PRIVATE gz-transport13::core)
    target_compile_options(test_power_control_cli PRIVATE -Wall -Wextra -Werror)
    add_test(NAME power_control_cli_contract COMMAND test_power_control_cli)

    add_executable(test_power_control_sdf tests/test_power_control_sdf.cpp)
    target_include_directories(test_power_control_sdf PRIVATE include)
    target_link_libraries(test_power_control_sdf PRIVATE ArionCoupledPowerPlugin gz-sim8::gz-sim8 gz-transport13::core)
    target_compile_options(test_power_control_sdf PRIVATE -Wall -Wextra -Werror)
    add_test(NAME power_control_sdf_callback COMMAND test_power_control_sdf)
endif()
