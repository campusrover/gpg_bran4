#ifndef BRANARMCONSTANTS_H
#define BRANARMCONSTANTS_H

#define SHOULDER 15   // servo number
#define ELBOW 14      // servo number
#define WRIST 13      // servo number
#define CLAW 12       // servo number

#define EL_DEGOFFSET 43
#define EL_DEGSCALE 2.5
#define SH_DEGOFFSET 124
#define SH_DEGSCALE 1.77
#define WR_DEGOFFSET 75
#define WR_DEGSCALE 1.77
#define CL_DEGOFFSET 43
#define CL_DEGSCALE 2.5

// Degree Maxs and Mins
#define SH_MAX_DEG 254      //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define SH_MIN_DEG 0        //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define EL_MAX_DEG 185      //max  degrees   2.5 deg/pulselen
#define EL_MIN_DEG -50      //min  degrees   (puls/2.5)-43
#define WR_MAX_DEG 300      //max wrist degrees
#define WR_MIN_DEG 0      //min wrist degrees
#define CL_MAX_DEG 21
#define CL_MIN_DEG 6

// Notes:
// shoulder
//  0 means parallel to the ground
//  90 means straight up
// wrist
// elbow
//  90 means inline with upper arm
//  0 means 90 degrees from upper arm

// Named standard arm positions
// Park
#define SH_PARK_DEG 0       // parallel to ground
#define EL_PARK_DEG 90
#define WR_PARK_DEG 300     //park degrees

// To the floor
#define SH_FLOOR_DOWN_DEG 196    //
#define EL_FLOOR_DOWN_DEG 0
#define WR_FLOOR_DOWN_DEG -9

// Floor up, same as floor down except tip is offset
// vertically 
#define SH_FLOOR_UP_DEG 51
#define EL_FLOOR_UP_DEG 25
#define WR_FLOOR_UP_DEG 153

// Floor out, same as floor down but reaching as far as possible
// vertically 
#define SH_FLOOR_OUT_DEG 200
#define EL_FLOOR_OUT_DEG 110
#define WR_FLOOR_OUT_DEG 65

// Straight up 
#define SH_STRAIGHTUP_DEG 90
#define EL_STRAIGHTUP_DEG 95
#define WR_STRAIGHTUP_DEG 90

// Straight up with wrist parallel to ground
#define SH_VERT_HORIZ_HAND 90
#define EL_VERT_HORIZ_HAND 90
#define WR_VERT_HORIZ_HAND 180

// All pointing backward
#define SH_ALL_BACKWARD_DEG 0
#define EL_ALL_BACKWARD_DEG 90
#define WR_ALL_BACKWARD_DEG 90

// All pointing forward
#define SH_ALL_FORWARD_DEG 180
#define EL_ALL_FORWARD_DEG 90
#define WR_ALL_FORWARD_DEG 90

// Claw
#define CL_PARK_DEG 21
#define CL_OPEN_DEG 6
#define CL_CLOSED_DEG 21
#define CL_CAN_CLOSED_DEG 12

#endif

