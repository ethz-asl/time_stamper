#include "Calibration.h"
#include "ros/ros.h"

Calibration::Calibration(const CalibrationConfig &cfg) {
  convex_shape_ = std::make_shared<ConvexShape>(ConvexShape(cfg.tolerance));
  detector_ = std::make_shared<Detector>(Detector(cfg.params));
  led_parser_ = std::make_shared<LedParser>(LedParser(cfg.led_row_config));
}

cv_bridge::CvImage Calibration::ProcessImage(const sensor_msgs::Image &image) {
  cv_bridge::CvImage out_msg;
  out_msg.header = image_.header;
  out_msg.encoding = image_.encoding;
  image_ = image;
  cv::Mat input_mat = ConvertToCvImage();
  cv::Mat visualization_mat = input_mat.clone();
  detector_->process(input_mat);
  detector_->pollKeyPointStatus(Calibration::Log);

  if (detector_->isKeypointsEmpty()) {
    out_msg.image = input_mat;
    return out_msg;
  }

  PointVector points(detector_->getKeyPoints());
  convex_shape_->Process(points);
  SetShapeStatus();

  int number = -1;
  if (convex_shape_->isShapeValid()) {
    led_parser_->ProcessImage(input_mat);
    cv::Mat inv_homography =
        cv::findHomography(convex_shape_->getPhysicalCorners(),
                           convex_shape_->getVirtualCorners(1),0).inv();

    led_parser_->TransformLedRow(inv_homography);
    Point3fVector led_row_transformed = led_parser_->GetLedRow();

    number = led_parser_->GetBinaryValue();
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

cv::Mat Calibration::ConvertToCvImage() const {
  cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvCopy(image_);
  cv::Mat input_mat = cv_image->image.clone();
  return input_mat;
}

void Calibration::SetShapeStatus() {
  if (convex_shape_->isShapeValid() != isLastShapeValid) {
    std::string v = convex_shape_->isShapeValid() ? "Valid" : "Invalid";
    ROS_INFO_STREAM("Shape " << v);
  }
}

void Calibration::Visualize(const cv::Mat &visualization_mat, int number) const {

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

    for (const auto &led: led_parser_->GetLedRow()) {
      cv::Point2f led_pos = LedParser::Normalize(led);
      cv::circle(visualization_mat, led_pos, (int) 10, cv::Scalar(255, 0, 0));
    }

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

void Calibration::VisualizeCorners(const cv::Mat &visualization_mat, PointAngleVector corners) const {
  for (int i = 0; i < corners.size(); i++) {
    cv::Scalar color_circle(255, 0, 0);
    cv::Scalar color_text(255, 0, 0);

    cv::Point corner = corners.at(i).point;
    cv::circle(visualization_mat, corner, 30, color_circle);
    cv::putText(visualization_mat, labels.at(i), cvPoint(corner.x, corner.y - 40),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color_text);
  }
}

void Calibration::Log(const std::string& message) {
  ROS_INFO("%s", message.c_str());
}
