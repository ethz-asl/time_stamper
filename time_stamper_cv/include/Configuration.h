#pragma once

struct LedRowConfig {
  cv::Point2f first_led_pos;
  cv::Vec2i next_led;
  int amount;
  int multiplier = 1;
};

struct CalibrationConfig {
  int tolerance{};
  cv::SimpleBlobDetector::Params params;
  LedRowConfig led_row_config{{0, 0}, {6, 0}, 16};
};