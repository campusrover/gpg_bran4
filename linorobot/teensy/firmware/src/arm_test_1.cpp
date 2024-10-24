


/*
  Brandeis Platform Autonomous Robot 

  Move arm from  park to floor
  
  Uses Adafruit_PWMServoDriver.h library

 
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver ARM = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 100  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 160  // this is the 'maximum' pulse length count (out of 4096)
#define Shoulder 15   // servo number
#define Elbow 14      // servo number
#define Wrist 12      // servo number
#define Claw 13      // servo number

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



int destination_shoulder;
int destination_wrist;
int destination_elbow;


float wristposition;
float elbowposition;
float shoulderposition;




float CurrentClaw = ClawParkdeg;  // current position
float CurrentWrist = WristParkdeg;
float CurrentElbow = ElbowParkdeg;
float CurrentShoulder = ShoulderParkdeg;
float iterations = 0;

int shouldercnt = 0;
int elbowcnt = 0;
int wristcnt = 0;
float shoulderdelta = 0;
float elbowdelta = 0;
float wristdelta = 0;




char p;  // park arm
char f;  // move arm to floor position
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
boolean newData = false;

// case states
static const int waitInput = 1;
static const int setStartFinish = 2;
static const int setIterationsDelta = 3;
static const int moveSF = 4;
int currentCase = waitInput;

float iteration_time = millis();
float iteration_interval = 20;  //20 milli second


////////////////////////////////////////////////////////////////////////////////
//===============================================================================
//  Move  Wrist
//===============================================================================



void wrist(float deg) {

  int deglen = (deg + 75) * 1.77;  // pulselen of commanded degrees
  ARM.setPWM(Wrist, 0, deglen);
  // delay(20);
  //CurrentWrist = deg;
}

//===============================================================================
//  Move elbow
//===============================================================================

void elbow(float deg) {
  int deglen = (deg + 43) * 2.5;  // pulselen of commanded degrees
  ARM.setPWM(Elbow, 0, deglen);
  // delay(20);


  //CurrentElbow = deg;
}



//===============================================================================
//  Move shoulder
//===============================================================================

void shoulder(float deg) {

  int deglen = (deg + 67.8) * 1.77;  // pulselen of commanded degrees
  ARM.setPWM(Shoulder, 0, deglen);

  // delay(20);


  //CurrentShoulder = deg;
}




////////////////////////////////////////////////////////////////////////////////////////////////
// Serial Commands

/*

xxx is number only 0-360   must be 3 digits (sign + 4 digits for writs) within the limits end marker is new line (cr for those of us that know what a carriage is)

Commands  o  claw open
          c  claw closed
          wxxxs  move wrist to xxxs degrees so claw is realitive to ground where s is the sign  must be included!
                000+ = claw parallel to ground
                090- = claw pointing to ground
                090+ = claw pointing to sky
          exxx  move elbow to xxx degrees
          sxxx  move shoulder to xxx degrees
          p     move arm to park position
          f     move arm to floor position

          */

/////////////////////////////////////////////////////////////////

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

  if (millis() > iteration_time + iteration_interval)  // if time is up  execute case et al

  // just loop if time is not up
  {

    iteration_time = millis();  // update time








    switch (currentCase)  //note with switch statement put code in brackets  {}
    {
      case waitInput:  //set_Waypoint  set the waypoint from the waypoint array set stop
        {

          recvWithEndMarker();



          if (newData == true) {
            rcvchar1 = receivedChars[0];
            rcvchar2 = receivedChars[1];
            rcvchar3 = receivedChars[2];
            rcvchar4 = receivedChars[3];
            rcvchar5 = receivedChars[4];




            if (rcvchar1 == 'p') {
              destination_shoulder = ShoulderParkdeg;
              destination_wrist = WristParkdeg;
              destination_elbow = ElbowParkdeg;
            }


            if (rcvchar1 == 'f') {
              destination_shoulder = ShoulderFloordeg;
              destination_wrist = WristFloordeg;
              destination_elbow = ElbowFloordeg;
            }


            showNewData();

            currentCase = setStartFinish;
          } else {
            currentCase = waitInput;
          }
          break;
        }


      case setStartFinish:


        {
          currentCase = setIterationsDelta;
          break;
        }

      case setIterationsDelta:  //determine joint deltas and number of iterations (1 iteration = 1 degree of the longest joint degree travel)
        {
          Serial.println("set Iterations Delta");
          shouldercnt = abs(destination_shoulder - CurrentShoulder);
          elbowcnt = abs(destination_elbow - CurrentElbow);
          wristcnt = abs(destination_wrist - CurrentWrist);  //note wrist is backwards

          if (shouldercnt > elbowcnt && shouldercnt > wristcnt) iterations = shouldercnt;
          if (elbowcnt > shouldercnt && elbowcnt > wristcnt) iterations = elbowcnt;
          if (wristcnt > shouldercnt && wristcnt > elbowcnt) iterations = wristcnt;

          shouldercnt = destination_shoulder - CurrentShoulder;
          elbowcnt = destination_elbow - CurrentElbow;
          wristcnt = destination_wrist - CurrentWrist;  //note wrist is backwards
          iterations = iterations / 2;
          shoulderdelta = shouldercnt / iterations;
          elbowdelta = elbowcnt / iterations;
          wristdelta = wristcnt / iterations;
          Serial.println(iterations);
          Serial.println(wristcnt);
          Serial.println(wristdelta);

          currentCase = moveSF;
          Serial.println("Move Park or Floor");
          break;
        }

      case moveSF:  //move arm joint delta

        {


          CurrentWrist = CurrentWrist + wristdelta;
          wrist(CurrentWrist);

          CurrentElbow = CurrentElbow + elbowdelta;
          elbow(CurrentElbow);

          CurrentShoulder = CurrentShoulder + shoulderdelta;
          shoulder(CurrentShoulder);

          Serial.println(shoulderdelta);
          Serial.println(iterations);
          Serial.println(CurrentShoulder);
          iterations = iterations - 1;

          currentCase = moveSF;
          if (iterations < 1) currentCase = waitInput;  //jump out after iterations are done

          break;
        }



      default:
        // if nothing else matches, do the default

        break;
    }
  }  // end time delay
}



