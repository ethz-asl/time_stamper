#pragma once
#include "opencv2/opencv.hpp"

class Trigonometry {
 public:

  /**
   * Calculates distance between two points.
   * @param a
   * @param b
   * @return distance
   */
  static double CalcDistance(const cv::Point &a, const cv::Point &b);

  /**
   * Calculates inner angle of point target
   * @param a
   * @param b
   * @param target
   * @return Inner angle in degrees
   */
  static double CalcInnerAngle(const cv::Point &a, const cv::Point &b, const cv::Point &target);

 private:
  static double LawOfCos(double a, double b, double c);
};
