# time_stamper

## Features
- Kernel module for hardware timestamping. See [kernel module](kernelmodule/README.md)
- ROS node to publish generated timestamps

## Installation
- Works with ROS Melodic and newer
- Go to catkin workspace `cd catkin_ws/src`
- Clone this repo with `git clone https://github.com/ethz-asl/time_stamper`
- [Setup](kernelmodule/README.md#time_stamperc-setup), [compile](kernelmodule/README.md#compile) and [start](kernelmodule/README.md#usage) kernel module
- `catkin build time_stamper_ros`
- Start ROS Node with `roslaunch time_stamper_ros time_stamper_default.launch`

## Timestamping 
- Clock frequency: 1 - 10'526'315 hz ~95 nanoseconds
- Every 640 cycles a Timestamp is generated (64 * 10)
- If clock is e.g. 10 hz, it takes 64s to generate a timestamp.

## Known Issues
- If too many file descriptors are open system-wide node does silently stop publishing timestamps.
