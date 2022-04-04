#pragma once
#include "detector.h"
#include "opencv2/opencv.hpp"
class detector {
 public:
  explicit detector(cv::SimpleBlobDetector::Params params);
  void process(const cv::Mat& input_mat);
  void pollKeyPointStatus(const std::function<void(std::string)> &function);
  std::vector<cv::KeyPoint> getKeyPoints();
  bool isKeypointsEmpty();

 private:
  cv::Ptr<cv::SimpleBlobDetector> detector_{};
  std::vector<cv::KeyPoint> keypoints_{};
  bool is_last_keypoints_empty_{false};
};

