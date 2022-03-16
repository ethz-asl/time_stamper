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
  cv::Mat input_mat = cv_image->image.clone();

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

  detector->detect(input_mat, keypoints);



  if (keypoints.empty()) {
    ROS_WARN("Keypoints empty");
  } else {
    std::cout << keypoints.size() << std::endl;
  }

  //cv::Mat m_with_leds;
  //cv::drawKeypoints(input_mat, keypoints, m_with_leds, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  for (const cv::KeyPoint &key_point : keypoints) {
    std::cout << "X: " << key_point.pt.x << " Y: " << key_point.pt.y << std::endl;
  }
  corner_leds corners = CalcCornerLeds(keypoints);

  edges.push_back(corners.upperLeft);
  edges.push_back(corners.upperRight);
  edges.push_back(corners.lowerLeft);
  edges.push_back(corners.lowerRight);

  cv::Mat m_with_leds;
  cv::drawKeypoints(input_mat, edges, m_with_leds, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


  //cv::Mat m_with_fixed_leds;
  //cv::drawKeypoints(m_with_leds, edges, m_with_fixed_leds, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


  //cv::imshow(OPENCV_WINDOW, cv_image->image);
  //cv::waitKey(3);
  cv::imshow(OPENCV_WINDOW, m_with_leds);
  cv::waitKey(3);
  cv_bridge::CvImage out_msg;
  out_msg.header = cv_image->header;
  out_msg.encoding = cv_image->encoding;
  out_msg.image = m_with_leds;
  img_pub_.publish(out_msg.toImageMsg());
}

Node::~Node() {
  //cv::destroyWindow(OPENCV_WINDOW);
}

point2 Node::GetLeftCornerLeds(const std::vector<cv::KeyPoint> &keypoints) {

  std::pair<cv::KeyPoint, cv::KeyPoint> point_2{};
  for (const cv::KeyPoint &key_point : keypoints) {

    if (key_point.pt.x < point_2.first.pt.x || point_2.first.pt.x == 0) {
      point_2.second = point_2.first;
      point_2.first = key_point;
      continue;
    }

    if (key_point.pt.x < point_2.second.pt.x  || point_2.first.pt.y == 0) {
      point_2.second = key_point;
      continue;
    }

  }
  return point_2;
}

corner_leds Node::CalcCornerLeds(const std::vector<cv::KeyPoint>& keypoints) {
  point2 point_2 = GetLeftCornerLeds(keypoints);

  corner_leds corners;

  cv::KeyPoint upperLeft{};
  cv::KeyPoint lowerLeft{};

  cv::KeyPoint upperRight{};
  cv::KeyPoint lowerRight{};

  std::vector<cv::KeyPoint> upperRow;
  std::vector<cv::KeyPoint> lowerRow;

  //Lower row has bigger y
  if (point_2.first.pt.y > point_2.second.pt.y) {
    lowerLeft = point_2.first;
    upperLeft = point_2.second;
  } else {
    upperLeft = point_2.first;
    lowerLeft = point_2.second;
  }

  float rowBoundary = (upperLeft.pt.y + lowerLeft.pt.y) / 2;

  for (const cv::KeyPoint& key_point : keypoints) {
    if (key_point.pt.y > rowBoundary) {
      lowerRow.push_back(key_point);
    } else {
      upperRow.push_back(key_point);
    }
  }


  for (const cv::KeyPoint& key_point : upperRow) {
      if (key_point.pt.x > upperRight.pt.x) {
        upperRight = key_point;
      }
  }

  for (const cv::KeyPoint& key_point : lowerRow) {
    if (key_point.pt.x > lowerRight.pt.x) {
      lowerRight = key_point;
    }
  }

  corners.lowerRight = lowerRight;
  corners.lowerLeft = lowerLeft;
  corners.upperRight = upperRight;
  corners.upperLeft = upperLeft;
  return corners;
}



