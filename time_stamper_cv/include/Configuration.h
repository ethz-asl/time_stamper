#pragma once

struct CalibrationConfig {
  int tolerance{};
  cv::SimpleBlobDetector::Params params;
};