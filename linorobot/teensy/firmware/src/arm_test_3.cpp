/*
  Brandeis Platform Autonomous Robot

  Arm exerciser using degree input

  All angles realitive:  Platform front to right

  Uses Adafruit_PWMServoDriver.h library


*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include "branarmconstants.h"

Adafruit_PWMServoDriver ARM = Adafruit_PWMServoDriver(0x40);

// #define SERVOMIN 100 // this is the 'minimum' pulse length count (out of 4096)
// #define SERVOMAX 160 // this is the 'maximum' pulse length count (out of 4096)
// #define Shoulder 15  // servo number
// #define Elbow 14     // servo number
// #define Wrist 13     // servo number
// #define Claw 12      // servo number

// #define ClawPark 160 // claw closed
// #define ClawMIN 160  // claw closed
// #define ClawMAX 120  // claw open
// #define ClawOpen 120

// #define ClawClosed 160

// //   int deglen = ((-deg +180 +63.5)*1.77 );  //rev4
// //  wrist pulse length  = (degrees + 75)*1.77
// // wrist degrees = pulselenth /1.77 - 75
// #define WristPark 570       // rotated toward front of elbow
// #define WristVertical 272   // parallel to elbow
// #define WristHorizontal 460 // right angle to elbow
// #define WristMIN 100        // 113???
// #define WristMAX 570        //
// #define WristMAXdeg 250     // max wrist degrees
// #define WristMINdeg 0       // min wrist degrees
// #define WristParkdeg 245    // min wrist degrees
// #define WristFloordeg -38   // touching flooor degrees
// // 90 parallel to elbow degrees
// // 180 right angle to elbow towards front
// //  0 left angle to elbow towards back of Platform
// // 245  park

// //  elbow pulse length  = (degrees + 65.3)*2.83
// // elbow degrees = pulselenth /2.83 - 65.3
// #define ElbowPark 340       // parallel to ground old 340
// #define ElbowVertical 340   // parallel to shoulder old 340
// #define ElbowHorizontal 570 // perpendicular to shoulder old 570
// #define ElbowMIN 185        // *******  this is the 'minimum' pulse length count (out of 4096) old 185
// #define ElbowMAX 570        // old 570
// #define ElbowMAXdeg 185     // max  degrees   2.5 deg/pulselen
// #define ElbowMINdeg 0       // min  degrees   (puls/2.5)-43
// #define ElbowFloordeg 0
// #define ElbowParkdeg 90

// //  shoulder pulse length  = (degrees + 67.8)*1.77
// // shoulder degrees = pulselenth /1.77 - 67.8
// #define ShoulderPark 120       // parallel to ground
// #define ShoulderMIN 120        // this is the 'minimum' pulse length count (out of 4096)
// #define ShoulderMAX 570        // this is the 'maximum' pulse length count (out of 4096)
// #define ShoulderHorizontal 445 // parallel to ground
// #define ShoulderVertical 285   // perpendicular to ground
// #define ShoulderMAXdeg 254     // max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
// #define ShoulderMINdeg 0       // max  degrees   1.77 deg/pulselen (ie degree/1.77 = pulselen)
// #define ShoulderParkdeg 0      // parallel to ground
// #define ShoulderVerticaldeg 90 // perpendicular to ground
// #define ShoulderForwardeg 180  // horizontal  180
// #define ShoulderFloordeg 228   //

int CL_PARK = (CL_PARK_DEG + CL_DEGOFFSET) * CL_DEGSCALE;
int currentClaw = CL_PARK; 
int WR_PARK = (WR_PARK_DEG + WR_DEGOFFSET) * WR_DEGSCALE;
int currentWrist = WR_PARK;
int EL_PARK = (EL_PARK_DEG + EL_DEGOFFSET) * EL_DEGSCALE;
int currentElbow = EL_PARK;
int SH_PARK = (SH_PARK_DEG + SH_DEGOFFSET) * SH_DEGSCALE;;
int currentShoulder = SH_PARK;
int CL_CLOSED = (CL_CLOSED_DEG + CL_DEGOFFSET) * CL_DEGSCALE;;
int CL_OPEN = (CL_OPEN_DEG + CL_DEGOFFSET) * CL_DEGSCALE;;


// int deglen = (deg + convert_offset) * convert_scale;


//===============================================================================
//  Initialization
//===============================================================================
void park();

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ; // Wait for serial port to connect
  Serial.println("setup()");
  Serial.println("Parking the arm");
  ARM.begin();
  ARM.setPWMFreq(60);
  delay(10);
  String input = "";

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW); // turn the Left wheel off by making the voltage LOW
  park();
}

// BEGIN GENERATED

// Forward declarations
void openClaw();
void closeClaw();
void shoulder(int position);
void park();
// Forward declarations
void openClaw();
void closeClaw();
void shoulder(int position);
void park();
void wrist(int position);

String currentCommand = "";
bool isExecutingCommand = false;

String promptForText(const char* prompt) {
    Serial.println(prompt);  // Print prompt on new line
    Serial.println("Available commands:");
    Serial.println("- co (claw open)");
    Serial.println("- cc (claw close)");
    Serial.println("- sv (shoulder vertical)");
    Serial.println("- wv (wrist vertical)");
    Serial.println("- pk (park)");
    String input = "";
    char c;
    
    while (true) {
        if (Serial.available()) {
            c = Serial.read();
            
            if ((c == '\n' || c == '\r') && input.length() > 0) {
                Serial.println();
                break;
            }
            else if (c >= 32 && c <= 126) {
                input += c;
                Serial.print(c);
            }
            else if (c == 8 || c == 127) {
                if (input.length() > 0) {
                    input.remove(input.length() - 1);
                    Serial.print("\b \b");
                }
            }
        }
    }
    
    input.trim();
    return input;
}

void executeCommand(String command) {
    command.toLowerCase();
    
    if (command == "co") {
        openClaw();
        isExecutingCommand = false;
    }
    else if (command == "cc") {
        closeClaw();
        isExecutingCommand = false;
    }
    else if (command == "sv") {
        shoulder(SH_STRAIGHTUP);
        isExecutingCommand = false;
    }
    else if (command == "pk") {
        park();
        isExecutingCommand = false;
    }
    else if (command == "wv") {
        wrist(WR_STRAIGHTUP);
        isExecutingCommand = false;
    }
}

bool isValidCommand(String command) {
    command.toLowerCase();
    return (command == "co" || 
            command == "cc" || 
            command == "sv" ||
            command == "pk" ||
            command == "wv");
}

void loop() {
    if (isExecutingCommand) {
        executeCommand(currentCommand);
    } 
    else {
        String command = promptForText("Enter command: ");
        
        if (isValidCommand(command)) {
            currentCommand = command;
            isExecutingCommand = true;
            executeCommand(command);
        } 
        else {
            Serial.println("Unrecognized command. Valid commands are:");
            Serial.println("- co (claw open)");
            Serial.println("- cc (claw close)");
            Serial.println("- sv (shoulder vertical)");
            Serial.println("- wv (wrist vertical)");
            Serial.println("- pk (park)");
        }
    }
    
    delay(50);
}
// END GENERATED

// Low Level Operations
void park()
{
  ARM.setPWM(CLAW, 0, CL_PARK);
  ARM.setPWM(WRIST, 0, WR_PARK);
  ARM.setPWM(ELBOW, 0, EL_PARK);
  ARM.setPWM(SHOULDER, 0, SH_PARK);

  currentClaw = CL_PARK; // current position
  currentWrist = WR_PARK;
  currentElbow = EL_PARK;
  currentShoulder = SH_PARK;
}

void openClaw()
{ // Claw   MIN is closed   MAX is open

  if (currentClaw >= CL_OPEN)
  {
    for (int pulselen = currentClaw; pulselen > CL_OPEN; pulselen--)
    {
      ARM.setPWM(CLAW, 0, CL_OPEN);

      delay(20);
    }
  }
  currentClaw = CL_OPEN;
}

void closeClaw()
{ // Claw MIN is closed   MAX is open

  if (currentClaw <= CL_CLOSED)
  {
    for (int pulselen = currentClaw; pulselen < CL_CLOSED; pulselen++)
    {
      ARM.setPWM(CLAW, 0, pulselen);

      delay(20);
    }
  }
  currentClaw = CL_CLOSED;
}

void wrist(int deg)
{

  int deglen = (deg + 75) * 1.77; // pulselen of commanded degrees  Rev 1
                                  // int deglen = ((-deg +180 +63.5)*1.77 );  //rev 4
  // if (CurrentWrist <= deglen)
  // {
  //   // Serial.println("wrist");
  //   for (int pulselen = CurrentWrist; pulselen < deglen; pulselen++)
  //   {
  //     ARM.setPWM(Wrist, 0, pulselen);
  //     // Serial.println(deglen);
  //     delay(20);
  //   }
  // }
  // else
  // {
  //   for (int pulselen = CurrentWrist; pulselen > deglen; pulselen--)
  //   {
  //     ARM.setPWM(Wrist, 0, pulselen);
  //     delay(20);
  //   }
  // }
    ARM.setPWM(WRIST, 0, deglen);
    Serial.println("Wrist: " + String(deglen));
    delay(20);
    currentWrist = deglen;
}
void elbow(int deg)
{
  int deglen = (deg + 43) * 2.5; // pulselen of commanded degrees

  if (currentElbow <= deglen)
  {
    // Serial.println("Elbow");
    for (int pulselen = currentElbow; pulselen < deglen; pulselen++)
    {
      ARM.setPWM(ELBOW, 0, pulselen);
      // Serial.println(deglen);
      delay(20);
    }
  }
  else
  {
    for (int pulselen = currentElbow; pulselen > deglen; pulselen--)
    {
      ARM.setPWM(ELBOW, 0, pulselen);
      delay(20);
    }
  }

  currentElbow = deglen;
}

void shoulder(int deg)
{

  int deglen = (deg + 67.8) * 1.77; // pulselen of commanded degrees
  ARM.setPWM(SHOULDER, 0, deglen);
  Serial.println("sh def " + String(deglen));
  delay(20);
  currentShoulder = deglen;



  // if (CurrentShoulder <= deglen)
  // {
  //   // Serial.println("shoulder");
  //   for (int pulselen = CurrentShoulder; pulselen < deglen; pulselen++)
  //   {
  //     ARM.setPWM(Shoulder, 0, pulselen);
  //     Serial.println("sh<deg " + String(pulselen));
  //     delay(20);
  //   }
  // }
  // else
  // {
  //   for (int pulselen = CurrentShoulder; pulselen > deglen; pulselen--)
  //   {
  //     ARM.setPWM(Shoulder, 0, pulselen);
  //     Serial.println("sh>deg " + String(pulselen));
  //     delay(20);
  //   }
  }