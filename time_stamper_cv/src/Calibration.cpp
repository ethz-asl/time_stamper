#include "Calibration.h"
#include "ros/ros.h"
#include "LedParser.h"

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

  PointVector points = ConvertKeyPoints();
  convex_shape_->Process(points);
  SetShapeStatus();

  int number = -1;
  if (convex_shape_->isShapeValid()) {
    number = GetLedCounter(input_mat, visualization_mat);
  }
  if (visualization_) {
    Visualize(visualization_mat, number);
  }
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

PointVector Calibration::ConvertKeyPoints() {
  PointVector points{};
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
    if (!isLastShapeValid) {
      isLastShapeValid = true;
      ROS_INFO("Shape valid");
    }
  } else if (isLastShapeValid) {
    isLastShapeValid = false;
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
  cv::waitKey(3);
}

void Calibration::VisualizeCorners(cv::Mat visualization_mat, PointAngleVector corners) {
  for (int i = 0; i < corners.size(); i++) {
    cv::Scalar color_circle(255, 0, 0);
    cv::Scalar color_text(255, 0, 0);

    cv::Point corner = corners.at(i).point;
    cv::circle(visualization_mat, corner, 30, color_circle);
    cv::putText(visualization_mat, labels.at(i), cvPoint(corner.x, corner.y - 40),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color_text);
  }
}

Point3fVector Calibration::GenerateLedRow(
    const cv::Point2f &first_led_pos, const cv::Vec2i &next_led, int amount, float multiplier) {
  Point3fVector led_row;
  for (int i = 1; i <= amount; i++) {
    led_row.emplace_back((first_led_pos.x + (next_led.val[0] * (i * 1.0)) * multiplier),
                         (first_led_pos.y + (next_led.val[1] * (i * 1.0)) * multiplier),
                         1);
  }
  return led_row;
}

int Calibration::GetLedCounter(const cv::Mat &input_mat, cv::Mat visualization_mat) {


  cv::Mat homography =
      cv::findHomography(convex_shape_->getPhysicalCorners(), convex_shape_->getVirtualCorners(1), 0);
  LedRowConfig led_row_config{{0, 0}, {6, 0}, 16};
  LedParser led_parser(led_row_config, input_mat);

  led_parser.TransformLedRow(homography.inv());
   Point3fVector led_row_transformed = led_parser.GetLedRow();

   for (const auto& led: led_parser.GetLedRow()) {
     //TODO Move to Visualize()
     cv::Point2f led_pos = LedParser::Normalize(led);
     cv::circle(visualization_mat, led_pos, (int) 10, cv::Scalar(255, 0, 0));
   }
  return led_parser.GetLedBinaryCounter();
}

Calibration::~Calibration() {
  delete convex_shape_;
}
