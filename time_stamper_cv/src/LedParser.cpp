#include "LedParser.h"

#include <utility>

LedParser::LedParser(LedRowConfig  led_row_config, int image_crop_size)
: led_row_config_(std::move(led_row_config)), size_(image_crop_size) {

  cv::Mat kernel = cv::Mat(image_crop_size, image_crop_size, CV_8UC1);

  int half_size = image_crop_size / 2;
  cv::circle(kernel, cv::Point(half_size, half_size), half_size,
             cv::Scalar(255, 255, 255), -1);

 kernel_normalized_ = kernel / 255;
}

void LedParser::ProcessImage(cv::Mat image) {
  led_row_.clear();
  led_row_ = GenerateLedRow(led_row_config_);
  image_ = std::move(image);
}

Point3fVector LedParser::GenerateLedRow(const LedRowConfig& cfg) {

  Point3fVector led_row;
  cv::Point2f led_pos = cfg.first_led_pos;

  for (int i = 1; i <= cfg.amount; i++) {
    led_row.emplace_back((led_pos.x + (cfg.next_led.val[0] * (i * 1.0)) * cfg.multiplier),
                         (led_pos.y + (cfg.next_led.val[1] * (i * 1.0)) * cfg.multiplier),
                         1);
  }
  return led_row;
}

void LedParser::TransformLedRow(const cv::Mat& homography) {
  Point3fVector empty_row;
  cv::transform(led_row_, empty_row, homography);
  led_row_ = empty_row;
}

double LedParser::GetLedBrightness(int index, float radius) {
  cv::Point3_<float> led_transformed = led_row_.at(index);

  //Normalized led point
  cv::Point2f led_pos{led_transformed.x / led_transformed.z, led_transformed.y / led_transformed.z};
  int row_begin = (int) (led_pos.x - (radius / 2.0f));
  int col_begin = (int) (led_pos.y - (radius / 2.0f));

  if ((col_begin + size_) > image_.rows || row_begin + size_ > image_.cols) {
    return -1;
  }

  if (row_begin > 0 && col_begin > 0) {// &&row_begin + size_ < image_.rows && col_begin + size_ < image_.cols) {

    cv::Rect led_rect(row_begin, col_begin, size_, size_);
    cv::Mat cropped = image_(led_rect);

    //average - cv::sum(...) > 0 is always true
    double normalization = 1.0 / cv::sum(kernel_normalized_)[0];
    cv::Scalar average = cv::sum(cropped.mul(kernel_normalized_) * normalization);

    return average.val[0];
  }
  return -1;
}

bool LedParser::isLedOn(int index, float radius, int min_brightness) {
  double a = GetLedBrightness(index, radius);
  return a > min_brightness;
}

const Point3fVector &LedParser::GetLedRow() const {
  return led_row_;
}

int LedParser::GetLedBinaryCounter() {
  int count = 0;
  for (int i = 0; i < led_row_.size(); i++) {
    if (isLedOn(i)) {
      count |= 1 << i;
    }
  }
  return count;
}
cv::Point2f LedParser::Normalize(const cv::Point3_<float>& pt) {
  return {pt.x/pt.z, pt.y/pt.z};
}



