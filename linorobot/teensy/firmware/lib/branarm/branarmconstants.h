#ifndef BRANARMCONSTANTS_H
#define BRANARMCONSTANTS_H

#define SERVOMIN 100  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 160  // this is the 'maximum' pulse length count (out of 4096)
#define SHOULDER 15   // servo number
#define ELBOW 14      // servo number
#define WRIST 13      // servo number
#define CLAW 12       // servo number

// Convert pulse -> degrees (pulse + 43) * 2.5
#define CLAWPARK 160  // claw closed
#define CLAWMIN 160   // claw closed
#define CLAWMAX 120   // claw open
#define CLAWOPEN 120
#define CLAWCLOSED 160
#define CLAWPARKDEG 21  
#define CLAWMAXDEG 5
#define CLAWMINDEG 21
#define CLAWOPENDEG 5
#define CLAWCLOSEDDEG 21

#define EL_DEGOFFSET 43
#define EL_DEGSCALE 2.5
#define SH_DEGOFFSET 67.8
#define SH_DEGSCALE 1.77
#define WR_DEGOFFSET 75
#define WR_DEGSCALE 1.77
#define CL_DEGOFFSET 43
#define CL_DEGSCALE 2.5


//  wrist pulse length  = (degrees + 75)*1.77
// wrist degrees = pulselenth /1.77 - 75
#define WRISTPARK 570        // rotated toward front of elbow
#define WRISTVERTICAL 272    // parallel to elbow
#define WRISTHORIZONTAL 460  // right angle to elbow
#define WRISTMIN 100         //113???
#define WRISTMAX 570         //
#define WRISTMAXDEG 250      //max wrist degrees
#define WRISTMINDEG 0        //min wrist degrees
//90 parallel to elbow degrees
// 180 right angle to elbow towards front
//  0 left angle to elbow towards back of Platform
// 245  park

// elbow pulse length  = (degrees + 65.3)*2.83
// elbow degrees = pulselenth /2.83 - 65.3
#define ELBOWPARK 340        // parallel to ground old 340
#define ELBOWVERTICAL 340    // parallel to shoulder old 340
#define ELBOWHORIZONTAL 570  // perpendicular to shoulder old 570
#define ELBOWMIN 185         // *******  this is the 'minimum' pulse length count (out of 4096) old 185
#define ELBOWMAX 570         // old 570
#define ELBOWMAXDEG 185      //max  degrees   2.5 deg/pulselen
#define ELBOWMINDEG 31       //min  degrees   (puls/2.5)-43

//  shoulder pulse length  = (degrees + 67.8)*1.77
// shoulder degrees = pulselenth /1.77 - 67.8
#define SHOULDERPARK 120        // parallel to ground
#define SHOULDERMIN 120         // this is the 'minimum' pulse length count (out of 4096)
#define SHOULDERMAX 570         // this is the 'maximum' pulse length count (out of 4096)
#define SHOULDERHORIZONTAL 445  // parallel to ground
#define SHOULDERVERTICAL 285    // perpendicular to ground
#define SHOULDERMAXDEG 254      //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define SHOULDERMINDEG 0        //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define SHOULDERVERTICALDEG 90  // perpendicular to ground
#define SHOULDERFORWARDEG 180   // horizontal  180

// Standard arm positions, aka armlocs

// Park
#define SH_PARK_DEG 0       // parallel to ground
#define EL_PARK_DEG 90
#define WR_PARK_DEG 245     //park degrees

// To the floor
#define SH_FLOOR_DOWN_DEG 196    //
#define EL_FLOOR_DOWN_DEG 0
#define WR_FLOOR_DOWN_DEG -9     //touching flooor degrees*************** should be -10

// Straight up 
#define SH_STRAIGHTUP 90
#define EL_STRAIGHTUP 90
#define WR_STRAIGHTUP 90

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

// Floor up, same as floor down except tip is offset
// vertically 
#define SH_FLOOR_UP_DEG 51
#define EL_FLOOR_UP_DEG 25
#define WR_FLOOR_UP_DEG 153

#endif
