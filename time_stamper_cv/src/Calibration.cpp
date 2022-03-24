#include "Calibration.h"

Calibration::Calibration(CalibrationConfig cfg) {
  convex_shape_ = new ConvexShape(cfg.tolerance);
  detector_ = cv::SimpleBlobDetector::create(cfg.params);
}

cv_bridge::CvImage Calibration::ProcessImage(const sensor_msgs::Image &image) {
  cv_bridge::CvImage out_msg;
  out_msg.header = image_.header;
  out_msg.encoding = image_.encoding;
  image_ = image;
  cv::Mat input_mat = ConvertToCvImage();
  cv::Mat visualization_mat = input_mat.clone();
  detector_->detect(input_mat, keypoints_);

  SetKeypointStatus();
  if (isKeypointsEmpty) {
    out_msg.image = input_mat;
    return out_msg;
  }

  std::vector<cv::Point> points = ConvertKeyPoints();
  convex_shape_->Process(points);
  SetShapeStatus();

  int number = -1;
  if (convex_shape_->isShapeValid()) {
    number = GetLedCounter(input_mat, visualization_mat);
  }
  if (visualization_) {
    Visualize(visualization_mat, number);
  }

  cv::waitKey(3);
  out_msg.image = input_mat;
  return out_msg;
}

void Calibration::SetVisualization(bool visualization) {
  visualization_ = visualization;
}

cv::Mat Calibration::ConvertToCvImage() {
  cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvCopy(image_);
  cv::Mat input_mat = cv_image->image.clone();
  return input_mat;
}

std::vector<cv::Point> Calibration::ConvertKeyPoints() {
  std::vector<cv::Point> points{};
  points.reserve(keypoints_.size());

  std::transform(keypoints_.begin(),
                 keypoints_.end(),
                 std::back_inserter(points),
                 [](cv::KeyPoint &kp) { return kp.pt; });
  return points;
}

void Calibration::SetKeypointStatus() {
  if (keypoints_.empty()) {
    if (!isKeypointsEmpty) {
      ROS_WARN("Keypoints empty");
    }
    isKeypointsEmpty = true;
  } else {
    if (isKeypointsEmpty) {
      ROS_INFO("Keypoints found");
    }
    isKeypointsEmpty = false;
  }
}

void Calibration::SetShapeStatus() {
  if (convex_shape_->isShapeValid()) {
    if (!lastShapeValid) {
      lastShapeValid = true;
      ROS_INFO("Shape valid");
    }
  } else if (lastShapeValid) {
    lastShapeValid = false;
    ROS_WARN("Shape invalid");
  }
}

void Calibration::Visualize(const cv::Mat &visualization_mat, int number) {

  cv::polylines(visualization_mat, convex_shape_->getHull(), true, cv::Scalar(255, 0, 0));
  if (convex_shape_->isShapeValid()) {
    VisualizeCorners(visualization_mat, convex_shape_->getSortedPointAngles());
  }
  cv::Size s = visualization_mat.size();

  std::string shape_text("Shape: ");
  std::string counter_text("Counter: ");

  if (convex_shape_->isShapeValid()) {
    shape_text += "Valid";
    counter_text += std::to_string(number);
  } else {
    shape_text += "Invalid";
    counter_text += "---";
  }

  cv::putText(visualization_mat, shape_text, cv::Point(s.width * 0.05, s.height * 0.85),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 0));
  cv::putText(visualization_mat, counter_text, cv::Point(s.width * 0.05, s.height * 0.9),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 0));
  cv::imshow(OPENCV_WINDOW + std::string(" Visualization"), visualization_mat);
}

void Calibration::VisualizeCorners(cv::Mat visualization_mat, std::vector<PointAngle> corners) {
  for (int i = 0; i < corners.size(); i++) {
    cv::Scalar color_circle(255, 0, 0);
    cv::Scalar color_text(255, 0, 0);

    cv::Point corner = corners.at(i).point;
    cv::circle(visualization_mat, corner, 30, color_circle);
    cv::putText(visualization_mat, labels.at(i), cvPoint(corner.x, corner.y - 40),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color_text);
  }
}

std::vector<cv::Point3f> Calibration::GenerateLedRow(
    const cv::Point2f &first_led_pos, const cv::Vec2i &next_led, int amount, float multiplier) {
  std::vector<cv::Point3f> led_row;
  for (int i = 1; i <= amount; i++) {
    led_row.emplace_back((first_led_pos.x + (next_led.val[0] * (i * 1.0)) * multiplier),
                         (first_led_pos.y + (next_led.val[1] * (i * 1.0)) * multiplier),
                         1);
  }
  return led_row;
}

int Calibration::GetLedCounter(const cv::Mat &input_mat, cv::Mat visualization_mat) {

  int number = 0;
  float multiplier = 1;
  std::vector<cv::Point3f> leds_bottom_row = GenerateLedRow({0, 0}, {6, 0}, 16);
  std::vector<cv::Point3f> leds_img_bottom_row = GenerateLedRow({0, 0}, {0, 0}, 16);

  cv::Mat homography =
      cv::findHomography(convex_shape_->getPhysicalCorners(), convex_shape_->getVirtualCorners(multiplier), 0);

  cv::Mat result = cv::Mat::zeros(input_mat.size(), CV_8UC1);

  cv::warpPerspective(input_mat, result, homography, input_mat.size());

  cv::transform(leds_bottom_row, leds_img_bottom_row, homography.inv());

  float radius = 10.0f;

  for (int i = 0; i < leds_img_bottom_row.size(); i++) {

    cv::Point3_<float> led_img = leds_img_bottom_row.at(i);

    //Normalized led point
    cv::Point2f led_pos{led_img.x / led_img.z, led_img.y / led_img.z};

    int row_begin = (int) (led_pos.x - (radius / 2.0f));
    int col_begin = (int) (led_pos.y - (radius / 2.0f));

    //Create 16x16 image
    int size = 16;
    int half_size = size / 2;

    if (row_begin > 0 && col_begin > 0) {

      //TODO test if coordinates are in image
      cv::Rect led_rect(row_begin, col_begin, size, size);
      cv::Mat cropped = input_mat(led_rect);

      cv::Mat kernel = cv::Mat(size, size, CV_8UC1);
      cv::circle(kernel, cv::Point(half_size, half_size), half_size,
                 cv::Scalar(255, 255, 255), -1);

      cv::Mat kernel_normalized = kernel / 255;

      //Calculate average
      // todo: check if cv::sum(...) > 0
      double normalization = 1.0 / cv::sum(kernel_normalized)[0];
      cv::Scalar average = cv::sum(cropped.mul(kernel_normalized) * normalization);

      if (average.val[0] > 40) {
        number |= 1 << i;
      }

      //TODO Move to Visualize()
      cv::circle(visualization_mat, led_pos, (int) radius, cv::Scalar(255, 0, 0));

    }
  }
  return number;
}

Calibration::~Calibration() {
  delete convex_shape_;
}
