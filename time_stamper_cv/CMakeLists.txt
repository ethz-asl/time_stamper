cmake_minimum_required(VERSION 3.0.2)
project(time_stamper_cv)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
add_definitions(-Wall -Werror=return-type)
find_package(catkin_simple REQUIRED)

catkin_simple()

# Add manually for Ubuntu 18.04.
find_package(OpenCV REQUIRED)

cs_add_library(${PROJECT_NAME}
        src/led_detection_node.cpp
        src/trigonometry.cpp
        src/convex_shape.cpp
        src/image_processor.cpp
        src/led_state_parser.cpp
        src/keypoint_detector.cpp
        )
cs_add_executable(${PROJECT_NAME}-node src/main.cpp)

target_link_libraries(
        ${PROJECT_NAME}-node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}-test
        test/test_entry_point.cc
        test/test_trigonometry.cc
        test/test_convex_shape.cc
        test/test_led_parser.cc
        )

target_link_libraries(
        ${PROJECT_NAME}-test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        gmock_main
)

cs_install()
cs_export()
