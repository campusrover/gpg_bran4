# Use of git submodules

* A less well known feature which allows one repo to contain essentially a reference to another repo
* It's important to understand this so that the reference can be maintained
* Here's where I learned all about it: <https://gist.github.com/gitaarik/8735255>

## References

* <https://github.com/ros-gopigo3/gopigo3-pi-code>
* <http://therobotacademy.com/512_Getting-started-virtual-GoPiGo3/>

## Base modules


* [Dexter's Gopigo3 node and additional nodes](https://github.com/ros-gopigo/gopigo3_node)
* [YDlidar's node and additional related nodes](https://github.com/YDLIDAR/ydlidar_ros)


## Use

* Clone this repo under catlin_ws/src on the GoPiGo3's pi as well as on the remote computer
* `cd gpg_bran3` ;`git submodule update --init`

## How to create this from scratch
* To add a submodule do this: 
  * `git submodule add https://github.com/ros-gopigo/gopigo3_node.git`
  * `git submodule add https://github.com/YDLIDAR/ydlidar_ros.git`

## Handy Commands
* `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
