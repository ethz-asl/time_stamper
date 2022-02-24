#include "gtest/gtest.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_stamper_ros_tester");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
