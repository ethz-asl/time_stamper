cmake_minimum_required(VERSION 3.0.2)
project(time_stamper_ros)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED
        roscpp
        std_msgs
        message_generation
        )

add_message_files(FILES
        Timestamp.msg
        )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rpi_timestamp_generator
        CATKIN_DEPENDS roscpp message_runtime
)

set(CMAKE_MODULE_PATH
        ${CMAKE_MODULE_PATH}
        ${PROJECT_SOURCE_DIR}/cmake/
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/SysfsPwm.cpp
        src/Node.cpp
        src/TimestampManager.cpp include/TimestampManager.h)

add_executable(${PROJECT_NAME}-node
        src/Main.cpp
        )

target_link_libraries(
        ${PROJECT_NAME}-node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}

)

catkin_add_gtest(${PROJECT_NAME}-test
        test/test_entry_point.cc
        test/test_mock_pwm_subsystem.cc
        )

target_link_libraries(${PROJECT_NAME}-test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        gmock_main
        )