#include "image_processor.h"
#include "ros/ros.h"

ImageProcessor::ImageProcessor(const ImageProcessorConfig &cfg) {
  convex_shape_ = std::make_shared<ConvexShape>(ConvexShape(cfg.tolerance));
  detector_ = std::make_shared<KeyPointDetector>(KeyPointDetector(cfg.params));
  led_parser_ = std::make_shared<LedStateParser>(LedStateParser(cfg.led_row_config));
}

cv_bridge::CvImage ImageProcessor::process(const sensor_msgs::Image &image) {
  cv_bridge::CvImage out_msg{image.header, image.encoding};
  cv::Mat input_mat = convertToCvImage(image);
  cv::Mat visualization_mat = input_mat.clone();

  detector_->process(input_mat);
  detector_->pollKeyPointStatus(ImageProcessor::log);

  if (detector_->isKeypointsEmpty()) {
    visualize(visualization_mat, -1);
    out_msg.image = input_mat;
    return out_msg;
  }

  PointVector points(detector_->getKeyPoints());
  convex_shape_->process(points);
  convex_shape_->pollShapeStatus(ImageProcessor::log);

  int number = -1;
  if (convex_shape_->isShapeValid()) {
    led_parser_->processImage(input_mat);

    /*
     * Get inverted homography, so we know the position of each LED even if it's turned off.
     */
    led_parser_->transformLedRow(convex_shape_->getInvHomography());

    number = led_parser_->getBinaryValue();
  }
  if (visualization_) {
    visualize(visualization_mat, number);
  }
  out_msg.image = input_mat;
  return out_msg;
}

void ImageProcessor::setVisualization(const bool visualization) {
  visualization_ = visualization;
}

cv::Mat ImageProcessor::convertToCvImage(const sensor_msgs::Image &image) {
  return cv_bridge::toCvCopy(image)->image.clone();
}

void ImageProcessor::visualize(const cv::Mat &visualization_mat, int number) const {

  cv::polylines(visualization_mat, convex_shape_->getHull(), true, cv::Scalar(255, 0, 0));
  if (convex_shape_->isShapeValid()) {
    visualizeCorners(visualization_mat, convex_shape_->getSortedPointAngles());
  }
  cv::Size s = visualization_mat.size();

  std::string shape_text("Shape: ");
  std::string counter_text("Counter: ");

  if (convex_shape_->isShapeValid()) {
    shape_text += "Valid";
    counter_text += std::to_string(number);

    for (const auto &led: led_parser_->getLedRow()) {
      cv::Point2f led_pos = LedStateParser::normalize(led);
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
  cv::imshow(OPENCV_WINDOW, visualization_mat);
  cv::waitKey(1);
}

void ImageProcessor::visualizeCorners(const cv::Mat &visualization_mat, PointAngleVector corners) const {
  for (int i = 0; i < corners.size(); i++) {
    cv::Scalar color_circle(255, 0, 0);
    cv::Scalar color_text(255, 0, 0);

    cv::Point corner = corners.at(i).point;
    cv::circle(visualization_mat, corner, 30, color_circle);
    cv::putText(visualization_mat, labels.at(i), cvPoint(corner.x, corner.y - 40),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, color_text);
  }
}

void ImageProcessor::log(const std::string& message) {
  ROS_INFO("%s", message.c_str());
}
