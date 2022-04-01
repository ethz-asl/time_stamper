#pragma once
#include <vector>
#include <opencv2/core/types.hpp>
#include "Common.h"

/**
 * At the moment only hardcoded shape (@var virtualCorners) is supported.
 * ConvexShape should support all polygons in future.
 */

class ConvexShape {
 public:
  /**
   * Creates ConvexShape object
   * @param tolerance is used for ToleranceFilter().
   */
  explicit ConvexShape(int tolerance);

  /**
   * Calculates a convex shape from given points.
   *
   * @param raw_points
   */
  void Process(PointVector raw_points);

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
   * @param multiplier
   * @return Scaled virtualCorners
   */
  Point2fVector getVirtualCorners(int multiplier = 1);

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
   * @param expected_value
   * @param actual_value
   * @return true if in tolerance, otherwise false.
   */

  bool ToleranceFilter(int expected_value, double actual_value) const;

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
  static bool Filter(double min, double max, double value);

  int tolerance_;
  PointVector raw_points_{};
  PointVector hull_{};
  PointAngleVector point_angles_sorted_{};

  Point2fVector virtualCorners{
    {0 , 0},
    {0, 17},
    {66, 17},
    {102, 0},
    };
};
