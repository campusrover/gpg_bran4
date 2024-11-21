# BranArm Instructions

## Basics

There are four servos that control the arm. The Shoulder (100), Elbow (101), Wrist(102) and Claw(103).  Notice that they can not all take on any position at the same time, because they will hit the robot or other parts of the arm. This is bad and will damage the arm.

## Control

We control the robot by publishing a topic to /diag. If you check you will see that diag has two string parameters, command and subcommand. For arm motion the command is always "arm". There are several subcommands and many of them are "secret" that is, not to be used by you.

Servos are controlled by setting the desired angle. Not all angles are valid. Here are the min and  max for each one:

```
#define SH_MAX_DEG 254      //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define SH_MIN_DEG 0        //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define EL_MAX_DEG 185      //max  degrees   2.5 deg/pulselen
#define EL_MIN_DEG 0       //min  degrees   (puls/2.5)-43
#define WR_MAX_DEG 240      //max wrist degrees
#define WR_MIN_DEG -9       //min wrist degrees
#define CL_MAX_DEG 21
#define CL_MIN_DEG 5
```

## Debug Mode

To allow for safe experimentation, the arm (currently) comes up in debug mode which means that no matter what the commands are, the arm will not move.

## Commands

Remember all commands are given by publishing to topic /diag

* Park command: `rostopic pub /diag lino_msgs/Diag "{command: 'arm', subcommand: 'park', arg1: 0.0, arg2: 0.0, arg3: 0.0}"`
* Enable debug mode: `rostopic pub /diag lino_msgs/Diag "{command: 'arm', subcommand: 'diag', arg1: 99, arg2: 0.0, arg3: 0.0}"`
* Disable debug mode: `rostopic pub /diag lino_msgs/Diag "{command: 'arm', subcommand: 'diag', arg1: 88, arg2: 0.0, arg3: 0.0}"`
* Get arm status: `rostopic pub /diag lino_msgs/Diag "{command: 'arm', subcommand: 'diag', arg1: 1, arg2: 0.0, arg3: 0.0}"`

## Servos

