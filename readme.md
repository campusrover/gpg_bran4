# gpg_bran - GoPiGo4 Base Software

* This repo should be placed on the gopigo3 as well as on your remote computer
* The code here is mostly from other places
* Where it is unchanged, we use submodules
* Where it has been changed we copied from the source

## Content

* Copied from [Dexter's Gopigo3 node and additional nodes](https://github.com/ros-gopigo/gopigo3_node) and then **changed** here.
* **Unchanged, and included as a submodule**:  [YDlidar's node and additional related nodes](https://github.com/YDLIDAR/ydlidar_ros)

## Use of git submodules

* A less well known feature which allows one repo to contain essentially a reference to another repo
* It's important to understand this so that the reference can be maintained
* Here's where I learned all about it: <https://gist.github.com/gitaarik/8735255>

## References

* <https://github.com/ros-gopigo3/gopigo3-pi-code>
* <http://therobotacademy.com/512_Getting-started-virtual-GoPiGo3/>


## Use

* Clone this repo under catlin_ws/src on the GoPiGo3's pi as well as on the remote computer
* `cd gpg_bran4` ;`git submodule update --init`

## How to create this from scratch
* To add a submodule do this: 
  * `git submodule add https://github.com/ros-gopigo/gopigo3_node.git`
  * `git submodule add https://github.com/YDLIDAR/ydlidar_ros.git`

## Handy Commands
* `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## Preparing a GoPiGo3 Robot
* Start with the original [Bernardo microSD image](https://sourceforge.net/projects/dexter-raspbian-for-robots/files/GoPiGo3_Ubuntu18.04-ROS_Melodic.tar.gz/download)
* `mkdir -p catkin_ws/src`
* `cd catkin_ws/src`
* `git clone (this repo)`
* `cd ..`
* `catkin_make`
* `source devel/setup.bash`

* Build raspicam_node from sources, following instructions in the readme: [Raspicam_node](https://github.com/UbiquityRobotics/raspicam_node)

