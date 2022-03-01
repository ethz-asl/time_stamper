#include "opencv2/opencv.hpp"
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include "ros/ros.h"
using namespace cv;

int main() {

  struct passwd *pw = getpwuid(getuid());
  const char *homedir = pw->pw_dir;

  std::string path_from_homedir_to_img("/catkin_ws/src/time_stamper/time_stamper_cv/img/");
  std::string path_to_img = homedir + path_from_homedir_to_img;

  Mat img_grayscale = imread(path_to_img + "example.jpg", IMREAD_UNCHANGED);

  // Display the image.
  //imshow("grayscale image", img_grayscale);

  Mat imageLine = img_grayscale.clone();
  Point pointA(200, 80);
  Point pointB(450, 80);
  line(imageLine, pointA, pointB, Scalar(255, 255, 0), 3, 8, 0);
  imshow("Lined image", imageLine);
  // Wait for a keystroke.
  waitKey(0);
  // Destroys all the windows created
  destroyAllWindows();
  // Write the image in the same directory
  imwrite(path_to_img + "grayscale.jpg", img_grayscale);
}