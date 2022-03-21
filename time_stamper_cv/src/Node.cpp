#include "Node.h"
#include "Trigonometry.h"
#include "ShapeValidation.h"
#include "map"

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
  }

  std::vector<cv::Point> points{};

  for (const cv::KeyPoint &key_point : keypoints) {
    //std::cout << "X: " << key_point.pt.x << " Y: " << key_point.pt.y << std::endl;
    points.push_back(key_point.pt);
  }

  std::vector<cv::Point> hull{};

  std::vector<PointAngle> angles;
  cv::convexHull(points, hull, false);
  cv::Mat visualization_mat = input_mat.clone();
  cv::polylines(visualization_mat, hull, true, cv::Scalar(255, 0, 0));
  cv::polylines(input_mat, hull, true, cv::Scalar(255, 0, 0));

  if (hull.size() > 3) {

    cv::Point point_a = hull.at(hull.size() - 2);
    cv::Point point_b = hull.at(hull.size() - 1);

    for (auto &point_c : hull) {
      double angle = Trigonometry::CalcAngleCTriangle(point_a, point_c, point_b);

      if (angle < 170 || angle > 190) {
        PointAngle point_angle{point_b, angle};
        angles.push_back(point_angle);
      }

      point_a = point_b;
      point_b = point_c;
    }

    if (ShapeValidation::rotateVector(&angles)) {
      if (!isShapeValid) {
        isShapeValid = true;
        ROS_INFO("Shape valid");
      }
      cv::circle(visualization_mat, angles.at(0).point, 30, cv::Scalar(255, 0, 0));
      cv::putText(visualization_mat, "Bottom Left", cvPoint(angles.at(0).point.x, angles.at(0).point.y - 40),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250));

      cv::circle(visualization_mat, angles.at(1).point, 30, cv::Scalar(255, 0, 0));
      cv::putText(visualization_mat, "Top Left", cvPoint(angles.at(1).point.x, angles.at(1).point.y - 40),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250));

      cv::circle(visualization_mat, angles.at(2).point, 30, cv::Scalar(255, 0, 0));
      cv::putText(visualization_mat, "Top Right", cvPoint(angles.at(2).point.x, angles.at(2).point.y - 40),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250));

      cv::circle(visualization_mat, angles.at(3).point, 30, cv::Scalar(255, 0, 0));
      cv::putText(visualization_mat, "Bottom Right", cvPoint(angles.at(3).point.x, angles.at(3).point.y - 40),
                  cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(200, 200, 250));

    } else if (isShapeValid) {
      isShapeValid = false;
      ROS_WARN("Shape invalid");
    }

    std::vector<cv::Point2f> a;
    for (const PointAngle &point_angle: angles) {
      a.push_back(point_angle.point);
    }

    if (a.size() == 4) {
      float multiplier = 5;
      std::vector<cv::Point2f> b{

          {0 * multiplier, 0 * multiplier},
          {0 * multiplier, 17 * multiplier},
          {66 * multiplier, 17 * multiplier},
          {102 * multiplier, 0 * multiplier},

      };

      std::vector<cv::Point3f> leds{};
      std::vector<cv::Point3f> leds_img{};
      for (int i = 1; i <= 16; i++) {
        leds.emplace_back(6.0f * (float) i * multiplier, 0, 1);
        leds_img.emplace_back(0, 0, 1);
      }

      cv::Mat homography = cv::findHomography(a, b, 0);
      cv::Mat result = cv::Mat::zeros(input_mat.size(), CV_8UC1);

      cv::warpPerspective(input_mat, result, homography, input_mat.size());

      cv::transform(leds, leds_img, homography.inv());

      int radius = 10;
      bool isrunning = false;
      cv::Mat cropped;
      for (const auto &led: leds_img) {
        if (isrunning) {
          continue;
        }


        if (led.x > 0 && led.y > 0) {

          std::cout << "Test test" << led.x << " " << led.y << std::endl;

          int row_begin = (int) ((led.x / led.z) - (radius / 2.0));
          int col_begin = (int) ((led.y / led.z) - (radius / 2.0));
          std::cout << row_begin << " " << col_begin << std::endl;

          cv::Rect led_rect(row_begin, col_begin, 16, 16);

          cropped = input_mat(led_rect);
          isrunning = true;
          cv::imshow(OPENCV_WINDOW + std::string(" cropped"), cropped);

        }
        cv::circle(input_mat, cv::Point2f(led.x / led.z, led.y / led.z), radius, cv::Scalar(255, 0, 0));
      }

      cv::imshow(OPENCV_WINDOW + std::string(" homography"), input_mat);
    }
  }

  cv::imshow(OPENCV_WINDOW + std::string(" Visualization"), visualization_mat);

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



