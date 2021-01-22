# Time2Collision

This Project is skill test for Siera.Ai

## Installation

This whole code repository is tested on ubuntu 18.04 ROS-melodic. It can also work on ROS-kinetic but haven't been tested yet.


The following script installs ros melodic and a temp catkin workspace in your system

```sh
# to set file permission
chmod +x install.sh
source install.sh
```


## Code Usage

To launch final launch file please run the following:

```sh
roslaunch siera time2collision.launch

# to change time threshold add
roslaunch siera time2collision.launch ttc:=<time(in s)>
# for example
roslaunch siera time2collision.launch ttc:=3
```


## Code setup

This whole process flow for code is referred in Code documentation along with the mail.

This code adds the followinf additional topics while running:

1. /time2collision : This topic emits final calculated time to collision in String format

2. /min_distance : This topic publishes minimum pedestrian distance from robot in String format.

3. /person_coordinates : This topic publishes bbox coordinated of pedestrians in frame

4. /sync_depth_image : This topic publishes buffered depth image to sync with colored output image.

## Video Link

Please click here for demo: (<https://drive.google.com/file/d/1MOi1JifhZhercmvjGv_RIK95SM2iWm98/view?usp=sharing>)
