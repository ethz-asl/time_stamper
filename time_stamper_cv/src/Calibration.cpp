#include "Calibration.h"

Calibration::Calibration(CalibrationConfig cfg) {
    tolerance_ = cfg.tolerance;
    detector_ = cv::SimpleBlobDetector::create(cfg.params);
}

cv_bridge::CvImage Calibration::ProcessImage(const sensor_msgs::Image& image) {
  image_ = image;
  cv::Mat input_mat = ConvertToCvImage();
  cv::Mat visualization_mat = input_mat.clone();
  detector_->detect(input_mat, keypoints_);

  SetKeypointStatus();
  if (isKeypointsEmpty) {
    cv_bridge::CvImage out_msg;
    out_msg.header = image_.header;
    out_msg.encoding = image_.encoding;
    out_msg.image = input_mat;
    return out_msg;
  }

  std::vector<cv::Point> points = ConvertKeyPoints();
  ConvexShape convex_shape(points);

  cv::polylines(visualization_mat, convex_shape.getHull(), true, cv::Scalar(255, 0, 0));

  if (convex_shape.isShapeValid()) {
    if (!lastShapeValid) {
      lastShapeValid = true;
      ROS_INFO("Shape valid");
    }
    VisualizeCorners(visualization_mat, convex_shape.getRotatedPointAngles());

  } else if (lastShapeValid) {
    lastShapeValid = false;
    ROS_WARN("Shape invalid");
  }



  unsigned long number = 0;

  if (convex_shape.isShapeValid()) {
    float multiplier = 5;

    std::vector<cv::Point3f> leds_bottom_row{};
    std::vector<cv::Point3f> leds_img_bottom_row{};
    for (int i = 1; i <= 16; i++) {
      leds_bottom_row.emplace_back(6.0f * (float) i * multiplier, 0, 1);
      leds_img_bottom_row.emplace_back(0, 0, 1);
    }

    cv::Mat homography = cv::findHomography(convex_shape.getPhysicalCorners(), convex_shape.getVirtualCorners(multiplier), 0);
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

        cv::Rect led_rect(row_begin, col_begin, size, size);
        cv::Mat cropped = input_mat(led_rect);

        cv::Mat kernel = cv::Mat(size, size, CV_8UC1);
        cv::circle(kernel, cv::Point(half_size, half_size), half_size,
                   cv::Scalar(255, 255, 255), -1);

        cv::Mat kernel_normalized = kernel / 255;

        //Calculate average
        cv::Scalar scalar = cv::sum(cropped.mul(kernel_normalized) / cv::sum(kernel_normalized));

        double average_brightness = scalar.val[0];
        if (average_brightness > 40) {
          number |= 1UL << i;
        }

        cv::circle(input_mat, led_pos, (int) radius, cv::Scalar(255, 0, 0));
      }
    }
  }

  cv::Size s = visualization_mat.size();

  std::string shape_status = lastShapeValid ? "Valid" : "Invalid";
  std::string shape_text = "Shape: " + shape_status;
  std::string counter_text = "counter: " + std::to_string(number);

  if (!lastShapeValid || number == 0) {
    counter_text = "counter: " + std::string(" ---");
  }

  cv::putText(visualization_mat, shape_text, cv::Point(s.width * 0.05, s.height * 0.85),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 0));
  cv::putText(visualization_mat, counter_text, cv::Point(s.width * 0.05, s.height * 0.9),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 0));

  cv::imshow(OPENCV_WINDOW + std::string(" homography"), input_mat);

  cv::imshow(OPENCV_WINDOW + std::string(" Visualization"), visualization_mat);

  cv::waitKey(3);

  cv_bridge::CvImage out_msg;
  out_msg.header = image_.header;
  out_msg.encoding = image_.encoding;
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

void Calibration::Visualize(cv::Mat input_mat) {

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





