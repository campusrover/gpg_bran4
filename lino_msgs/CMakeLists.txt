cmake_minimum_required(VERSION 2.8.3)
project(lino_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  Velocities.msg
  PID.msg
  Diag.msg
  Imu.msg
  Inst.msg
  ArmMsg.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
