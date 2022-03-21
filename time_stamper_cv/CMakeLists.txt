cmake_minimum_required(VERSION 3.0.2)
project(time_stamper_cv)

add_definitions(-Wall -Werror=return-type)
find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
        src/Node.cpp
        src/Trigonometry.cpp
        src/ShapeValidation.cpp
        )
cs_add_executable(${PROJECT_NAME}-node src/Main.cpp)

target_link_libraries(
        ${PROJECT_NAME}-node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

cs_add_executable(${PROJECT_NAME}-blobtest src/BlobDetectionTest.cpp)

target_link_libraries(
        ${PROJECT_NAME}-blobtest
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}-test
        test/test_entry_point.cc
        test/test_trigonometry.cc
        test/test_shape_validation.cc
        )

target_link_libraries(
        ${PROJECT_NAME}-test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        gmock_main
)

cs_install()
cs_export()