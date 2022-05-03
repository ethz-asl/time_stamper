# time_stamper
*This project is currently in development*

## Timestamping
- Clock frequency: 1 - 10'526'315 hz ~95 nanoseconds
- Every 640 cycles a Timestamp is generated (64 * 10)
- If clock is e.g. 10 hz, it takes 64s to generate a timestamp.

## Packages
| Package          |   Installed on   |                                 Description                                 |
|:-----------------|:----------------:|:---------------------------------------------------------------------------:|
| kernel module    | Raspberry Pi CM4 | Used for hardware timestamping. See [kernel module](kernelmodule/README.md) |
| time_stamper_ros | Raspberry Pi CM4 |               Used for led control and timestamp generation.                |
| time_stamper_cv  |       Host       |               Used to detect real time led state with OpenCV.               |

## Installation
### Prerequisites
- Host PC and Raspberry Pi Compute Module 4 with ROS installed.
- Requires custom TimeStamper board with CM4 mounted.
- ROS Camera driver
- Works with ROS Melodic and newer

### Raspberry Pi CM4 installation
- Go to catkin workspace `cd catkin_ws/src`
- Clone this repo with `git clone https://github.com/ethz-asl/time_stamper`
- [Setup](kernelmodule/README.md#time_stamperc-setup), [compile](kernelmodule/README.md#compile) and [start](kernelmodule/README.md#usage) the kernel module
- `catkin build time_stamper_ros`
- Start ROS node with `roslaunch time_stamper_ros time_stamper_default.launch`
- Optional: time_stamper_cv can be removed

### Host installation
- Go to catkin workspace `cd catkin_ws/src`
- Clone this repo with `git clone https://github.com/ethz-asl/time_stamper`
- Build with `catkin build time_stamper_cv`
- Source ros env with `source ~/catkin_ws/devel/setup.bash`
- Start ROS Node with `roslaunch time_stamper_cv time_stamper_cv.launch`
- Optional: The kernel module and time_stamper_ros can be removed
