cmake_minimum_required(VERSION 3.0.2)
project(time_stamper_ros)

add_compile_options(-std=c++17 -Wdeprecated-declarations)

find_package(catkin_simple REQUIRED)
catkin_simple()


cs_add_library(${PROJECT_NAME}
        src/sysfs/SysfsPwm.cpp
        src/sysfs/SysfsGpio.cpp
        src/Node.cpp
        src/TimestampManager.cpp
        src/Filesystem.cpp
        )

cs_add_executable(${PROJECT_NAME}-node
        src/Main.cpp
        )

target_link_libraries(
        ${PROJECT_NAME}-node
        ${PROJECT_NAME}
)

include_directories(test/Mocks)

catkin_add_gtest(${PROJECT_NAME}-test
        test/Tests/test_entry_point.cc
        test/Tests/test_mock_pwm_subsystem.cc
        test/Tests/test_mock_filesystem.cc
        test/Tests/test_mock_gpio_subsystem.cc
        )

target_link_libraries(${PROJECT_NAME}-test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        gtest
        gmock_main
        )

cs_install()
cs_export()