#include "led_detection_node.h"
#include "trigonometry.h"

LedDetectionNode::LedDetectionNode() {
  calibration_ = std::make_shared<ImageProcessor>(ImageProcessor(getConfiguration()));
}

void LedDetectionNode::init() {
  img_pub_ = nh_.advertise<sensor_msgs::Image>("time_stamper_cv_image", 1);
  calibration_->setVisualization(nh_private_.param("show_visualization", true));
}

void LedDetectionNode::start() {
  img_sub_ = nh_.subscribe("output/image", 1, &LedDetectionNode::callbackRawImage, this);
  ROS_INFO("Node started");
}

void LedDetectionNode::callbackRawImage(const sensor_msgs::Image &image) const {
  cv_bridge::CvImage out_msg = calibration_->process(image);
  img_pub_.publish(out_msg.toImageMsg());
}

ImageProcessorConfig LedDetectionNode::getConfiguration() const {
  ImageProcessorConfig cfg;
  cv::SimpleBlobDetector::Params params;

  params.blobColor = nh_private_.param("blob_color", 255);

  // Filter by Area
  params.filterByArea = nh_private_.param("filter_by_area", true);
  params.minArea = nh_private_.param("min_area", 80.0f);

  // Filter by Circularity
  params.filterByCircularity = nh_private_.param("filter_by_circularity", true);
  params.minCircularity = nh_private_.param("min_convexity", 0.4f);

  // Filter by Convexity
  params.filterByConvexity = nh_private_.param("filter_by_convexity", false);
  params.minConvexity = nh_private_.param("min_convexity", 0.87f);

  // Filter by Inertia
  params.filterByInertia = nh_private_.param("filter_by_inertia", false);
  params.minInertiaRatio = nh_private_.param("min_inertia_ratio", 0.01f);

  cfg.tolerance = nh_private_.param("tolerance", 10);
  cfg.params = params;
  return cfg;
}


