find_package(gz-transport13 REQUIRED)

px4_add_module(
    MODULE modules__arion_motor_bridge
    MAIN arion_motor_bridge
    SRCS ArionMotorBridge.cpp
    DEPENDS gz-transport13::core
    EXTERNAL
)
target_include_directories(modules__arion_motor_bridge PRIVATE
    ${EXTERNAL_MODULES_LOCATION}/../include)
