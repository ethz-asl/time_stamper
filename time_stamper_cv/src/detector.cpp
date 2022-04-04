#include "detector.h"
#include "ros/ros.h"

detector::detector(cv::SimpleBlobDetector::Params params) {
  detector_ = cv::SimpleBlobDetector::create(params);
}

void detector::process(const cv::Mat &input_mat) {
  detector_->detect(input_mat, keypoints_);
}

bool detector::isKeypointsEmpty() {
  return keypoints_.empty();
}

void detector::pollKeyPointStatus(const std::function<void(std::string)>& function) {
  if (keypoints_.empty() != is_last_keypoints_empty_) {
    std::string v = keypoints_.empty() ? "empty" : "found";
    is_last_keypoints_empty_ = keypoints_.empty();
    function("Keypoints " + v);
  }
}

std::vector<cv::KeyPoint> detector::getKeyPoints() {
  return keypoints_;
}

