#pragma once

#include <functional>

#include "opencv2/opencv.hpp"


class KeyPointDetector {
 public:
  explicit KeyPointDetector(cv::SimpleBlobDetector::Params params);
  void process(const cv::Mat& input_mat);
  void pollKeyPointStatus(const std::function<void(std::string)> &function);
  std::vector<cv::KeyPoint> getKeyPoints();
  bool isKeypointsEmpty();

 private:
  cv::Ptr<cv::SimpleBlobDetector> detector_{};
  std::vector<cv::KeyPoint> keypoints_{};
  bool is_last_keypoints_empty_{false};
};

