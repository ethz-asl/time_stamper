#include <armadillo>
#include <pwd.h>
#include "opencv2/opencv.hpp"
int main() {
  using namespace cv;
  struct passwd *pw = getpwuid(getuid());
  const char *homedir = pw->pw_dir;

  std::string path_from_homedir_to_img("/catkin_ws/src/time_stamper/time_stamper_cv/img/");
  std::string path_to_img = homedir + path_from_homedir_to_img;
  // Read image
  Mat im = imread(path_to_img + "frame0000.jpg", IMREAD_GRAYSCALE);

  SimpleBlobDetector::Params params;
  // Change thresholds
  //params.minThreshold = 127;
  //params.maxThreshold = 128;
  //params.thresholdStep = 50;
  params.blobColor = 255;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 150;

  // Filter by Circularity
  params.filterByCircularity = true;
  params.minCircularity = 0.4;

  // Filter by Convexity
  params.filterByConvexity = false;
  params.minConvexity = 0.87;

  // Filter by Inertia
  params.filterByInertia = false;
  params.minInertiaRatio = 0.01;

  // Set up the detector with default parameters.
  Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
  // Detect blobs
  std::vector<KeyPoint> keypoints;
  detector->detect(im, keypoints);
  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
  Mat im_with_keypoints;
  drawKeypoints(im, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  // Show blobs
  imshow("keypoints", im_with_keypoints);


  for (const auto &key_point : keypoints) {
    //std::cout << key_point.size << std::endl;
    std::cout << "X: " << key_point.pt.x << " Y: " << key_point.pt.y << std::endl;
  }
  Mat inverted_image;
  bitwise_not(im, inverted_image);

  Mat inverted_image_with_keypoints;
  drawKeypoints(inverted_image, keypoints, inverted_image_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  imshow("Inverted keypoints", inverted_image_with_keypoints);

  waitKey(0);
}

