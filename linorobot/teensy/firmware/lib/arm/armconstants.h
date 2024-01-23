#ifndef ARMCONSTANTS_H
#define ARMCONSTANTS_H

#define SERVOMIN 100  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 160  // this is the 'maximum' pulse length count (out of 4096)
#define SHOULDER 15   // servo number
#define ELBOW 14      // servo number
#define WRIST 13      // servo number
#define CLAW 12       // servo number

#define CLAWPARK 160  // claw closed
#define CLAWMIN 160   // claw closed
#define CLAWMAX 120   // claw open
#define CLAWOPEN 120
#define CLAWPARKDEG 21  // claw closed deglen = (deg + 43) * 2.5;
#define CLAWCLOSED 160

#define EL_DEGOFFSET 43
#define EL_DEGSCALE 2.5
#define SH_DEGOFFSET 67.8
#define SH_DEGSCALE 1.77
#define WR_DEGOFFSET 75
#define WR_DEGSCALE 1.77


//  wrist pulse length  = (degrees + 75)*1.77
// wrist degrees = pulselenth /1.77 - 75
#define WRISTPARK 570        // rotated toward front of elbow
#define WRISTVERTICAL 272    // parallel to elbow
#define WRISTHORIZONTAL 460  // right angle to elbow
#define WRISTMIN 100         //113???
#define WRISTMAX 570         //
#define WRISTMAXDEG 250      //max wrist degrees
#define WRISTMINDEG 0        //min wrist degrees
#define WR_PARK_DEG 245     //park degrees
#define WR_FLOOR_DEG -9     //touching flooor degrees*************** should be -10

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
#define EL_FLOOR_DEG 0
#define EL_PARK_DEG 90

//  shoulder pulse length  = (degrees + 67.8)*1.77
// shoulder degrees = pulselenth /1.77 - 67.8
#define SHOULDERPARK 120        // parallel to ground
#define SHOULDERMIN 120         // this is the 'minimum' pulse length count (out of 4096)
#define SHOULDERMAX 570         // this is the 'maximum' pulse length count (out of 4096)
#define SHOULDERHORIZONTAL 445  // parallel to ground
#define SHOULDERVERTICAL 285    // perpendicular to ground
#define SHOULDERMAXDEG 254      //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define SHOULDERMINDEG 0        //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define SH_PARK_DEG 0       // parallel to ground
#define SHOULDERVERTICALDEG 90  // perpendicular to ground
#define SHOULDERFORWARDEG 180   // horizontal  180
#define SH_FLOOR_DEG 196    //

// Waypoints

// Straight up 
#define SH_STRAIGHTUP 90
#define EL_STRAIGHTUP 90
#define WR_STRAIGHTUP 90

// Straight up with wrist parallel to ground
#define SH_VERT_HORIZ_HAND 90
#define EL_VERT_HORIZ_HAND 90
#define WR_VERT_HORIZ_HAND 180

// All pointing forward
#define SH_ALL_FORWARD 0
#define EL_ALL_FORWARD 90
#define WR_ALL_FORWARD 90

// Vertical just above floor position
#define SH_ALL_VERTABOVEPARK 21.51
#define EL_ALL_VERTABOVEPARK -21.51
#define WR_ALL_VERTABOVEPARK 136.98

struct ServoData {
    int shoulder;
    int elbow;
    int wrist;
};

enum Waypoint {
    PARK,
    FLOOR,
    STRAIGHTUP,
    VERT_HORIZ_HAND,
    ALL_FORWARD,
    ALL_VERTABOVEPARK
};

ServoData waypoints[] = {
    {SH_PARK_DEG, EL_PARK_DEG, WR_PARK_DEG},
    {SH_FLOOR_DEG, EL_FLOOR_DEG, WR_FLOOR_DEG},
    {SH_STRAIGHTUP, EL_STRAIGHTUP, WR_STRAIGHTUP},
    {SH_VERT_HORIZ_HAND, EL_VERT_HORIZ_HAND, WR_VERT_HORIZ_HAND},
    {SH_ALL_FORWARD, EL_ALL_FORWARD, WR_ALL_FORWARD},
    {SH_ALL_VERTABOVEPARK, EL_ALL_VERTABOVEPARK, WR_ALL_VERTABOVEPARK}
};



#endif
