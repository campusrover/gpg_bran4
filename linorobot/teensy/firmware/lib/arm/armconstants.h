#ifndef ARMCONSTANTS_H
#define ARM_H

#define SERVOMIN 100  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 160  // this is the 'maximum' pulse length count (out of 4096)
#define Shoulder 15   // servo number
#define Elbow 14      // servo number
#define Wrist 13      // servo number
#define Claw 12       // servo number

#define ClawPark 160  // claw closed
#define ClawMIN 160   // claw closed
#define ClawMAX 120   // claw open
#define ClawOpen 120
#define ClawParkdeg 21  // claw closed deglen = (deg + 43) * 2.5;
#define ClawClosed 160


//  wrist pulse length  = (degrees + 75)*1.77
// wrist degrees = pulselenth /1.77 - 75
#define WristPark 570        // rotated toward front of elbow
#define WristVertical 272    // parallel to elbow
#define WristHorizontal 460  // right angle to elbow
#define WristMIN 100         //113???
#define WristMAX 570         //
#define WristMAXdeg 250      //max wrist degrees
#define WristMINdeg 0        //min wrist degrees
#define WristParkdeg 245     //park degrees
#define WristFloordeg -9     //touching flooor degrees*************** should be -10

//90 parallel to elbow degrees
// 180 right angle to elbow towards front
//  0 left angle to elbow towards back of Platform
// 245  park

//  elbow pulse length  = (degrees + 65.3)*2.83
// elbow degrees = pulselenth /2.83 - 65.3
#define ElbowPark 340        // parallel to ground old 340
#define ElbowVertical 340    // parallel to shoulder old 340
#define ElbowHorizontal 570  // perpendicular to shoulder old 570
#define ElbowMIN 185         // *******  this is the 'minimum' pulse length count (out of 4096) old 185
#define ElbowMAX 570         // old 570
#define ElbowMAXdeg 185      //max  degrees   2.5 deg/pulselen
#define ElbowMINdeg 31       //min  degrees   (puls/2.5)-43
#define ElbowFloordeg 0
#define ElbowParkdeg 90

//  shoulder pulse length  = (degrees + 67.8)*1.77
// shoulder degrees = pulselenth /1.77 - 67.8
#define ShoulderPark 120        // parallel to ground
#define ShoulderMIN 120         // this is the 'minimum' pulse length count (out of 4096)
#define ShoulderMAX 570         // this is the 'maximum' pulse length count (out of 4096)
#define ShoulderHorizontal 445  // parallel to ground
#define ShoulderVertical 285    // perpendicular to ground
#define ShoulderMAXdeg 254      //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define ShoulderMINdeg 0        //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define ShoulderParkdeg 0       // parallel to ground
#define ShoulderVerticaldeg 90  // perpendicular to ground
#define ShoulderForwardeg 180   // horizontal  180
#define ShoulderFloordeg 196    //

#endif
