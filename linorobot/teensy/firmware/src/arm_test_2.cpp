/*
  Brandeis Platform Autonomous Robot 

  Arm exerciser using degree input

  All angles realitive:  Platform front to right
  
  Uses Adafruit_PWMServoDriver.h library

 
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver ARM = Adafruit_PWMServoDriver(0x40);

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

#define ClawClosed 160

//   int deglen = ((-deg +180 +63.5)*1.77 );  //rev4
//  wrist pulse length  = (degrees + 75)*1.77
// wrist degrees = pulselenth /1.77 - 75  
#define WristPark 570      // rotated toward front of elbow
#define WristVertical 272  // parallel to elbow
#define WristHorizontal 460   // right angle to elbow
#define WristMIN 100     //113???
#define WristMAX 570     // 
#define WristMAXdeg 250  //max wrist degrees   
#define WristMINdeg 0  //min wrist degrees  
#define WristParkdeg 245  //min wrist degrees  
#define WristFloordeg -38  //touching flooor degrees
 //90 parallel to elbow degrees   
// 180 right angle to elbow towards front
//  0 left angle to elbow towards back of Platform
// 245  park                    




//  elbow pulse length  = (degrees + 65.3)*2.83
// elbow degrees = pulselenth /2.83 - 65.3 
#define ElbowPark 340      // parallel to ground old 340
#define ElbowVertical 340  // parallel to shoulder old 340
#define ElbowHorizontal 570  // perpendicular to shoulder old 570
#define ElbowMIN 185     // *******  this is the 'minimum' pulse length count (out of 4096) old 185
#define ElbowMAX 570     // old 570
#define ElbowMAXdeg 185  //max  degrees   2.5 deg/pulselen 
#define ElbowMINdeg 0  //min  degrees   (puls/2.5)-43
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
#define ShoulderMINdeg 0      //max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
#define ShoulderParkdeg 0        // parallel to ground
#define ShoulderVerticaldeg 90    // perpendicular to ground
#define ShoulderForwardeg 180   // horizontal  180
#define ShoulderFloordeg 228 //



int CurrentClaw = ClawPark;  // current position
int CurrentWrist = WristPark;
int CurrentElbow = ElbowPark;
int CurrentShoulder = ShoulderPark;

char c;     // close claw
char o;     // open claw
char w;     //wrist   note wrist has sign and 3 digits!
char e;     //elbow
char s;     //shoulder
int degree;


// Set for servo channel to test
int servonum = Claw;

//serial interface
const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data
char rcvchar1;
char rcvchar2;
char rcvchar3;
char rcvchar4;
char rcvchar5;


////////////////////////////////////////////////////////////////////////////////////////////////
// Serial Commands

/*

xxx is number only 0-360   must be 3 digits  within the limits end marker is new line (cr for those of us that know what a carriage is)

Commands  o  claw open
          c  claw closed
          wxxx  move wrist to xxx degrees 
                000+ = claw perpendicular to elbow left
                090+ = claw pointing parallel to elbow 
                180+ = claw perpendicular to elbow right
                              037- = claw on floor

          exxx  move elbow to xxx degrees
          sxxx  move shoulder to xxx degrees

          */



boolean newData = false;
//===============================================================================
//  Initialization
//===============================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Adruno is ready");
Serial.println("Supply the desired angle integer 3 digits");
  ARM.begin();
  ARM.setPWMFreq(60);
  delay(10);



  ARM.setPWM(Claw, 0, ClawPark);
  ARM.setPWM(Wrist, 0, WristPark);
  ARM.setPWM(Elbow, 0, ElbowPark);
  ARM.setPWM(Shoulder, 0, ShoulderPark);

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);  // turn the Left wheel off by making the voltage LOW
}
//===============================================================================
//  Main
//===============================================================================
void loop() {


  //Serial input

  recvWithEndMarker();



  if (newData == true) {
    rcvchar1 = receivedChars[0];
    rcvchar2 = receivedChars[1];
    rcvchar3 = receivedChars[2];
    rcvchar4 = receivedChars[3];
    rcvchar5 = receivedChars[4];

    degree = (rcvchar2 - '0') * 100 + (rcvchar3 - '0') * 10 + (rcvchar4 - '0');


    if (rcvchar1 == 'o') openclaw();

    if (rcvchar1 == 'c') closeclaw();

    Serial.print(degree);
    if (rcvchar1 == 'w') wrist(degree);

    if (rcvchar1 == 'e') elbow(degree);

    if (rcvchar1 == 's') shoulder(degree);
  }


  showNewData();
}

//===============================================================================
//  Park 
//===============================================================================

// Park();

void openclaw() {  // Claw   MIN is closed   MAX is open

  if (CurrentClaw >= ClawOpen) {
    for (int pulselen = CurrentClaw; pulselen > ClawOpen; pulselen--) {
      ARM.setPWM(Claw, 0, ClawOpen);

      delay(20);
    }
  }
  CurrentClaw = ClawOpen;
}

//===============================================================================
//  Close the claw
//===============================================================================

void closeclaw() {  // Claw MIN is closed   MAX is open

  if (CurrentClaw <= ClawClosed) {
    for (int pulselen = CurrentClaw; pulselen < ClawClosed; pulselen++) {
      ARM.setPWM(Claw, 0, ClawClosed);

      delay(20);
    }
  }
  CurrentClaw = ClawClosed;
}


////////////////////////////////////////////////////////////////////////////////
//===============================================================================
//  Move  Wrist
//===============================================================================



void wrist(int deg) {
  
  int deglen = (deg + 75 )* 1.77;    // pulselen of commanded degrees  Rev 1
 //int deglen = ((-deg +180 +63.5)*1.77 );  //rev 4
  if (CurrentWrist <= deglen) {
    //Serial.println("wrist");
    for (int pulselen = CurrentWrist; pulselen < deglen; pulselen++) {
      ARM.setPWM(Wrist, 0, pulselen);
      //Serial.println(deglen);
      delay(20);
    }
  } else {
    for (int pulselen = CurrentWrist; pulselen > deglen; pulselen--) {
      ARM.setPWM(Wrist, 0, pulselen);
      delay(20);
    }
  }

  CurrentWrist = deglen;

} 

//===============================================================================
//  Move elbow
//===============================================================================

void elbow(int deg) {
  int deglen = (deg + 43 )* 2.5;    // pulselen of commanded degrees

  if (CurrentElbow <= deglen) {
    //Serial.println("Elbow");
    for (int pulselen = CurrentElbow; pulselen < deglen; pulselen++) {
      ARM.setPWM(Elbow, 0, pulselen);
     // Serial.println(deglen);
      delay(20);
    }
  } else {
    for (int pulselen = CurrentElbow; pulselen > deglen; pulselen--) {
      ARM.setPWM(Elbow, 0, pulselen);
      delay(20);
    }
  }

  CurrentElbow = deglen;

} 



//===============================================================================
//  Move shoulder
//===============================================================================

void shoulder(int deg) {
 
 int deglen = (deg + 67.8 )* 1.77;    // pulselen of commanded degrees

  if (CurrentShoulder <= deglen) {
    //Serial.println("shoulder");
    for (int pulselen = CurrentShoulder; pulselen < deglen; pulselen++) {
      ARM.setPWM(Shoulder, 0, pulselen);
     // Serial.println(deglen);
      delay(20);
    }
  } else {
    for (int pulselen = CurrentShoulder; pulselen > deglen; pulselen--) {
      ARM.setPWM(Shoulder, 0, pulselen);
      delay(20);
    }
  }

  CurrentShoulder = deglen;

} 

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    Serial.print("This just in ... ");
    Serial.println(receivedChars);
    newData = false;
  }
}