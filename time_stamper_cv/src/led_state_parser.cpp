#include "led_state_parser.h"

#include <utility>

LedStateParser::LedStateParser(LedRowConfig led_row_config, int image_crop_size)
    : led_row_config_(std::move(led_row_config)), size_(image_crop_size) {

  cv::Mat kernel = cv::Mat(image_crop_size, image_crop_size, CV_8UC1);

  int half_size = image_crop_size / 2;
  cv::circle(kernel, cv::Point(half_size, half_size), half_size,
             cv::Scalar(255, 255, 255), -1);

  kernel_normalized_ = 1.0 / (kernel / 255);
}

void LedStateParser::processImage(const cv::Mat &image) {
  led_row_.clear();
  led_row_ = generateLedRow(led_row_config_);
  image_ = image;
}

Point3fVector LedStateParser::generateLedRow(const LedRowConfig &cfg) {

  Point3fVector led_row;
  cv::Point2f led_pos = cfg.first_led_pos;

  for (int i = 1; i <= cfg.amount; i++) {
    led_row.emplace_back((led_pos.x + ((float) cfg.next_led.val[0] * (float) i) * (float) cfg.multiplier),
                         (led_pos.y + ((float) cfg.next_led.val[1] * (float) i) * (float) cfg.multiplier),
                         1);
  }
  return led_row;
}

void LedStateParser::transformLedRow(const cv::Mat &homography) {
  Point3fVector empty_row;
  cv::transform(led_row_, empty_row, homography);
  led_row_ = empty_row;
}

double LedStateParser::getLedBrightness(int index) const {
  cv::Point3_<float> led_transformed = led_row_.at(index);

  cv::Point2f led_pos = normalize(led_transformed);

  cv::Point begin(
      (int) (led_pos.x - (size_ / 2.0)),
      (int) (led_pos.y - (size_ / 2.0)));

  if ((begin.y + size_) > image_.rows || begin.x + size_ > image_.cols ||
      (begin.x < 0 && begin.y < 0)) {
    return -1;
  }

  cv::Rect led_rect(begin, cv::Point(begin.x + size_, begin.y + size_));
  cv::Mat cropped = image_(led_rect);

  //average - cv::sum(...) > 0 is always true
  cv::Scalar average = cv::sum(cropped.mul(kernel_normalized_));

  return average.val[0];
}

bool LedStateParser::isLedOn(const int index, const float min_brightness) const {
  if (min_brightness > 255) {
    return false;
  }
  double a = getLedBrightness(index);
  return a > min_brightness;
}

const Point3fVector &LedStateParser::getLedRow() const {
  return led_row_;
}

int LedStateParser::getBinaryValue() const {
  int count = 0;
  for (int i = 0; i < led_row_.size(); i++) {
    if (isLedOn(i)) {
      count |= 1 << i;
    }
  }
  return count;
}

cv::Point2f LedStateParser::normalize(const cv::Point3_<float> &pt) {
  return {pt.x / pt.z, pt.y / pt.z};
}



