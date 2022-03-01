#include "Node.h"

static const std::string OPENCV_WINDOW = "Image window";

Node::Node() {
  cv::namedWindow(OPENCV_WINDOW);
}

bool Node::Init() {
  img_pub_ = nh_.advertise<sensor_msgs::Image>("time_stamper_cv_image", 1);
  return true;
}

void Node::Start() {
  img_sub_ = nh_.subscribe("camera/image_raw", 1, &Node::CallbackRawImage, this);
}

void Node::CallbackRawImage(const sensor_msgs::Image &image) {
  cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvCopy(image);

  cv::imshow(OPENCV_WINDOW, cv_image->image);
  cv::waitKey(3);

  img_pub_.publish(cv_image->toImageMsg());
}

Node::~Node() {
  cv::destroyWindow(OPENCV_WINDOW);
}

