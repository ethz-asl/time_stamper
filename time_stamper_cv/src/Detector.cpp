#include "Detector.h"
#include "ros/ros.h"

Detector::Detector(cv::SimpleBlobDetector::Params params) {
  detector_ = cv::SimpleBlobDetector::create(params);
}

void Detector::process(const cv::Mat &input_mat) {
  detector_->detect(input_mat, keypoints_);
}

bool Detector::isKeypointsEmpty() {
  return keypoints_.empty();
}

void Detector::pollKeyPointStatus(const std::function<void(std::string)>& function) {
  if (keypoints_.empty() != is_last_keypoints_empty_) {
    std::string v = keypoints_.empty() ? "empty" : "found";
    is_last_keypoints_empty_ = keypoints_.empty();
    function("Shape " + v);
  }
}

std::vector<cv::KeyPoint> Detector::getKeyPoints() {
  return keypoints_;
}

