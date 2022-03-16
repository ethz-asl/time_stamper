#include "Node.h"

static const std::string OPENCV_WINDOW = "Image window";

Node::Node() {
  //cv::namedWindow(OPENCV_WINDOW);
}

bool Node::Init() {
  img_pub_ = nh_.advertise<sensor_msgs::Image>("time_stamper_cv_image", 1);
  return true;
}

void Node::Start() {
  img_sub_ = nh_.subscribe("output/image", 1, &Node::CallbackRawImage, this);
}

void Node::CallbackRawImage(const sensor_msgs::Image &image) {
  cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvCopy(image);
  cv::Mat mat = cv_image->image.clone();

  cv::SimpleBlobDetector::Params params;
  params.blobColor = 255;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 150;

  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.4;

  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.87;

  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.01;


  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::KeyPoint> edges;
  std::vector<cv::KeyPoint> leds;

  detector->detect(mat, keypoints);

  if (keypoints.empty()) {
    ROS_WARN("Keypoints empty");
  } else {
    std::cout << keypoints.size() << std::endl;
  }

  cv::Mat m_with_keypoints;
  cv::drawKeypoints(mat, keypoints, m_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  //cv::imshow(OPENCV_WINDOW, cv_image->image);
  //cv::waitKey(3);
  cv::imshow(OPENCV_WINDOW, m_with_keypoints);
  cv::waitKey(3);
  cv_bridge::CvImage out_msg;
  out_msg.header = cv_image->header;
  out_msg.encoding = cv_image->encoding;
  out_msg.image = m_with_keypoints;
  img_pub_.publish(out_msg.toImageMsg());
}

Node::~Node() {
  //cv::destroyWindow(OPENCV_WINDOW);
}

