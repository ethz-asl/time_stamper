#pragma once
#include <vector>
#include <functional>

#include <opencv2/core/types.hpp>

#include "common.h"

/**
 * This class is used to detect a given convex shape. First, a convex hull is calculated out of all keypoints.
 * Thereupon, it calculates if the convex hull match the given convex shape.
 *
 * At the moment only hardcoded shape (@var virtualCorners) is supported.
 * ConvexShape should support all polygons in future.
 */

class ConvexShape {
 public:
  /**
   * Creates ConvexShape object
   * @param tolerance is used for isWithinTolerance().
   */
  explicit ConvexShape(int tolerance);

  /**
   * Calculates a convex shape from given points.
   *
   * @param raw_points
   */
  void process(const PointVector& raw_points);

  /**
   * Get all points which define the convex shape.
   * @return
   */
  PointVector getHull() const;

  /**
   * Checks if hull is valid.
   * @return true if hull has expected size, otherwise false.
   */
  bool isHullValid() const;

  /**
   * Used to get scaled virtualCorners.
   * @param scaling
   * @return Scaled virtualCorners
   */
  Point2fVector getVirtualCorners(float scaling = 1.0f);

  /**
   * Get points which define convex shape. Starting with bottom left clockwise.
   * @return Points if IsShapeValid() returns true, otherwise an empty vector is returned.
   */
  PointAngleVector getSortedPointAngles() const;

  /**
   * Get all points in image coordinates
   * @return
   */
  Point2fVector getPhysicalCorners();

  /**
   * Checks if shape on image is expected size and checks whether the inner angles are in tolerance.
   * @return true if expected size and the inner angles are in tolerance, otherwise false.
   */
  bool isShapeValid() const;

  /**
   * Checks whether a value is in tolerance.
   * @param value1
   * @param value2
   * @return true if in tolerance, otherwise false.
   */

  bool isWithinTolerance(double value1, double value2) const;

  /**
   * Check shape status change. Useful for logging
   * @param function is called when the status change.
   */
  void pollShapeStatus(const std::function<void(std::string)> &function);

  /**
   * Get inverted homography
   * @return
   */
  cv::Mat getInvHomography();

  /**
   * Default destructor
   */
  ~ConvexShape() = default;

  // For testing
  friend class ConvexShapeTest;

 private:
  void calculateHull();
  PointAngleVector calculatePointAngles();
  bool calculateSortedPointAngles();

  int tolerance_{10};
  PointVector raw_points_{};
  PointVector hull_{};
  PointAngleVector point_angles_sorted_{};
  bool is_last_shape_valid_{false};

  Point2fVector virtualCorners {
      {0, 0},
      {0, 17},
      {66, 17},
      {102, 0},
  };
};
