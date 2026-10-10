# 역할: V2 소유 중계 plugin/수신 어댑터만 빌드한다. PX4/원본 Gazebo는 빌드·수정하지 않는다.
# 입력: 설치된 Gazebo Harmonic 개발 패키지와 native/*.cpp.
# 출력: 기본 운영 빌드 폴더 hermes/build/native의 libMotorFault.so, gz_feed.
# 호출자: 최종 통합 검증의 cmake -S . -B hermes/build/native 및 cmake --build.
cmake_minimum_required(VERSION 3.16)
project(CapstoneNative LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(Threads REQUIRED)

add_library(MotorFault SHARED native/motor_fault.cpp)
target_compile_features(MotorFault PRIVATE cxx_std_17)
set_target_properties(MotorFault PROPERTIES CXX_EXTENSIONS OFF OUTPUT_NAME MotorFault)
target_link_libraries(MotorFault PRIVATE
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-plugin2::register
  Threads::Threads)

add_executable(gz_feed native/gz_feed.cpp)
target_compile_features(gz_feed PRIVATE cxx_std_17)
set_target_properties(gz_feed PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(gz_feed PRIVATE
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  Threads::Threads)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(MotorFault PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(gz_feed PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 실제 비행과 구별한 합성 경계 검사. 빌드만으로 실행하지 않는다.
add_executable(native_motor_test tests/native_motor_test.cpp)
target_compile_features(native_motor_test PRIVATE cxx_std_17)
set_target_properties(native_motor_test PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(native_motor_test PRIVATE
  gz-sim8::gz-sim8 gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10 gz-plugin2::register Threads::Threads)
