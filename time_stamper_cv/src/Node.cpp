#include "Node.h"
#include "Trigonometry.h"
#include "ShapeValidation.h"

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
  params.minArea = 50;

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

  detector->detect(input_mat, keypoints);

  if (keypoints.empty()) {
    if (!isKeypointsEmpty) {
      ROS_WARN("Keypoints empty");
    }
    isKeypointsEmpty = true;
    return;
  } else {
    if (isKeypointsEmpty) {
      ROS_INFO("Keypoints found");
    }
    isKeypointsEmpty = false;
    std::cout << keypoints.size() << std::endl;
  }

  std::vector<cv::Point> points{};

  for (const cv::KeyPoint &key_point : keypoints) {
    //std::cout << "X: " << key_point.pt.x << " Y: " << key_point.pt.y << std::endl;
    points.push_back(key_point.pt);
  }

  std::vector<cv::Point> hull{};
  cv::convexHull(points, hull, false);
  cv::polylines(input_mat, hull, true, cv::Scalar(255, 0, 0));

  if (hull.size() > 3) {

    cv::Point point_a = hull.at(hull.size() - 2);
    cv::Point point_b = hull.at(hull.size() - 1);

    std::vector<double> angles{};

    for (const auto &point_c : hull) {
      double angle = Trigonometry::CalcAngleCTriangle(point_a, point_c, point_b);

      if (angle < 170 || angle > 190) {
        std::cout << " A: " << angle << std::endl;
        cv::circle(input_mat, point_b, 30, cv::Scalar(255, 0, 0));
        cv::putText(input_mat, std::to_string(angle), cvPoint(point_b.x, point_b.y - 40),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250));
        angles.push_back(angle);
      }

      point_a = point_b;
      point_b = point_c;
    }

    if (ShapeValidation::rotateVector(&angles)) {
      ROS_INFO("Form valid");
    } else {
      ROS_WARN("Invalid shape");
    }
  }

  cv::imshow(OPENCV_WINDOW, input_mat);
  cv::waitKey(3);
  cv_bridge::CvImage out_msg;
  out_msg.header = cv_image->header;
  out_msg.encoding = cv_image->encoding;
  out_msg.image = input_mat;
  img_pub_.publish(out_msg.toImageMsg());
}

Node::~Node() {
  //cv::destroyWindow(OPENCV_WINDOW);
}

bool Node::filter(double min, double max, double value) {
  if (min > max) {
    abort();
  }
  return value >= min && value <= max;
}



