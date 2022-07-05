#include "ros/ros.h"

#include "image_processor.h"
#include <time_stamper_cv/Ledstate.h>

ImageProcessor::ImageProcessor(const ImageProcessorConfig &cfg) : cfg_(cfg) {
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
    if (visualization_) {
      visualize(visualization_mat);
    }
    out_msg.image = input_mat;
    return out_msg;
  }

  PointVector points(detector_->getKeyPoints());
  convex_shape_->process(points);
  convex_shape_->pollShapeStatus(ImageProcessor::log);

  if (convex_shape_->isShapeValid()) {
    led_parser_->processImage(input_mat);

    std::vector<std::string> rows = cfg_.rows();
    //Get inverted homography, so we know the position of each LED even if it's turned off.
    std::for_each(rows.begin(), rows.end(), [this] (const std::string& name) {
      led_parser_->transformLedRow(name, convex_shape_->getInvHomography());
    });

    /* When the first and last led in the top row are on, the state of the bottom row is changing.
       The frame has to be skipped. */
    int last_led_in_row = cfg_.led_row_config.at(ImageProcessorConfig::TOP_ROW).amount - 1;
    if (led_parser_->isLedOn(ImageProcessorConfig::TOP_ROW, 0)
        && led_parser_->isLedOn(ImageProcessorConfig::TOP_ROW, last_led_in_row)) {
      ROS_DEBUG("Skipped invalid LED pos");
    }
  }

  if (visualization_) {
    visualize(visualization_mat);
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

void ImageProcessor::visualize(const cv::Mat &visualization_mat) const {

  cv::polylines(visualization_mat, convex_shape_->getHull(), true, cv::Scalar(255, 0, 0));
  if (convex_shape_->isShapeValid()) {
    visualizeCorners(visualization_mat, convex_shape_->getSortedPointAngles());
  }
  cv::Size s = visualization_mat.size();

  std::string shape_text("Shape: ");
  std::string counter_text("Counter: ");

  if (convex_shape_->isShapeValid()) {
    int bottom_row_binary_value = led_parser_->getBinaryValue(ImageProcessorConfig::BOTTOM_ROW);
    shape_text += "Valid";
    counter_text += std::to_string(bottom_row_binary_value);

    std::vector<std::string> rows = cfg_.rows();
    std::for_each(rows.begin(), rows.end(), [this, &visualization_mat](const std::string& name) {
      for (const auto &led : led_parser_->getLedRow(name)) {
        cv::Point2f led_pos = LedStateParser::normalize(led);
        cv::circle(visualization_mat, led_pos, (int) 10, cv::Scalar(255, 0, 0));
      }
    });

  } else {
    shape_text += "Invalid";
    counter_text += "---";
  }

  //Print text on screen row by row
  double height_multiplier = 0.85;
  for (const auto &text: {shape_text, counter_text}) {
    cv::putText(visualization_mat, text, cv::Point((int) (s.width * 0.05), (int) (s.height * height_multiplier)),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 0));
    height_multiplier += 0.05;
  }
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

void ImageProcessor::log(const std::string &message) {
  ROS_INFO("%s", message.c_str());
}

void ImageProcessor::fillInLedStateMessage(time_stamper_cv::Ledstate* msg) {
  msg->counter = led_parser_->getBinaryValue(ImageProcessorConfig::BOTTOM_ROW);
  msg->is_valid = convex_shape_->isShapeValid();

  std::vector<std::string> rows = cfg_.rows();
  std::for_each(rows.begin(), rows.end(), [this, &msg] (const std::string& name) {
    for (int i = 0; i < led_parser_->getLedRow(name).size(); i++) {
      double brightness = led_parser_->getLedBrightness(name, i);
      msg->intensity.push_back(brightness);
      msg->binary_state.push_back(led_parser_->isLedOn(name, i));
    }
  });
}
