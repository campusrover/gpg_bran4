/*
  Brandeis Platform Autonomous Robot

  Arm exerciser using degree input

  All angles realitive:  Platform front to right

  Uses Adafruit_PWMServoDriver.h library


*/
#include <ros.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include "branarmconstants.h"
#include "branutils.h"

Adafruit_PWMServoDriver ARM = Adafruit_PWMServoDriver(0x40);

int CL_PARK = (CL_PARK_DEG + CL_DEGOFFSET) * CL_DEGSCALE;
int currentClaw = CL_PARK;
int WR_PARK = (WR_PARK_DEG + WR_DEGOFFSET) * WR_DEGSCALE;
int currentWrist = WR_PARK;
int EL_PARK = (EL_PARK_DEG + EL_DEGOFFSET) * EL_DEGSCALE;
int currentElbow = EL_PARK;
int SH_PARK = (SH_PARK_DEG + SH_DEGOFFSET) * SH_DEGSCALE;
;
int currentShoulder = SH_PARK;
int CL_CLOSED = (CL_CLOSED_DEG + CL_DEGOFFSET) * CL_DEGSCALE;
;
int CL_OPEN = (CL_OPEN_DEG + CL_DEGOFFSET) * CL_DEGSCALE;
;

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
void wrist(int position);
void elbow(int position);

// Command types
enum CommandType
{
  SIMPLE,       // No parameters
  FIXED_PARAM,  // Fixed parameter value
  RUNTIME_PARAM // Parameter provided at runtime
};

// Command table structure
struct Command
{
  const char *shortCmd;    // Two-letter command
  const char *description; // Description for help
  CommandType type;        // Type of command
  union
  {
    void (*simple)(); // For SIMPLE commands
    struct
    { // For FIXED_PARAM commands
      void (*func)(int);
      int value;
    } fixed;
    struct
    { // For RUNTIME_PARAM commands
      void (*func)(int);
      int minValue;
      int maxValue;
    } runtime;
  } function;
};

// Command table
const Command commands[] = {
    {"co", "claw open", SIMPLE, {.simple = openClaw}},
    {"cc", "claw close", SIMPLE, {.simple = closeClaw}},
    {"sv", "shoulder vertical", FIXED_PARAM, {.fixed = {shoulder, SH_STRAIGHTUP_DEG}}},
    {"wv", "wrist vertical", FIXED_PARAM, {.fixed = {wrist, WR_STRAIGHTUP_DEG}}},
    {"ev", "elbow vertical", FIXED_PARAM, {.fixed = {elbow, EL_STRAIGHTUP_DEG}}},
    {"pk", "park", SIMPLE, {.simple = park}},
    {"sh", "shoulder <angle>", RUNTIME_PARAM, {.runtime = {shoulder, SH_MIN_DEG, SH_MAX_DEG}}},
    {"el", "elbow <angle>", RUNTIME_PARAM, {.runtime = {elbow, EL_MIN_DEG, EL_MAX_DEG}}}
  };

const int numCommands = sizeof(commands) / sizeof(commands[0]);

String currentCommand = "";
bool isExecutingCommand = false;

void printAvailableCommands()
{
  Serial.println("Available commands:");
  for (int i = 0; i < numCommands; i++)
  {
    Serial.print("- ");
    Serial.print(commands[i].shortCmd);
    Serial.print(" (");
    Serial.print(commands[i].description);
    if (commands[i].type == RUNTIME_PARAM)
    {
      Serial.print(" [");
      Serial.print(commands[i].function.runtime.minValue);
      Serial.print("..");
      Serial.print(commands[i].function.runtime.maxValue);
      Serial.print("]");
    }
    Serial.println(")");
  }
}

// Parse integer parameter from string
bool parseParameter(String input, String cmd, int &param)
{
  input.trim();
  if (input.startsWith(cmd + " "))
  {
    String paramStr = input.substring(cmd.length() + 1);
    paramStr.trim();
    if (paramStr.length() > 0)
    {
      param = paramStr.toInt();
      return true;
    }
  }
  return false;
}

String promptForText(const char *prompt)
{
  Serial.println(prompt);
  printAvailableCommands();
  String input = "";
  char c;

  while (true)
  {
    if (Serial.available())
    {
      c = Serial.read();

      if ((c == '\n' || c == '\r') && input.length() > 0)
      {
        Serial.println();
        break;
      }
      else if (c >= 32 && c <= 126)
      {
        input += c;
        Serial.print(c);
      }
      else if (c == 8 || c == 127)
      {
        if (input.length() > 0)
        {
          input.remove(input.length() - 1);
          Serial.print("\b \b");
        }
      }
    }
  }

  input.trim();
  return input;
}

void executeCommand(String input)
{
  String cmd = input.toLowerCase();
  for (int i = 0; i < numCommands; i++)
  {
    if (cmd.startsWith(commands[i].shortCmd))
    {
      switch (commands[i].type)
      {
      case SIMPLE:
        commands[i].function.simple();
        break;

      case FIXED_PARAM:
        commands[i].function.fixed.func(commands[i].function.fixed.value);
        break;

      case RUNTIME_PARAM:
    
        int param;
        if (parseParameter(input, commands[i].shortCmd, param))
        {
          if (param >= commands[i].function.runtime.minValue &&
              param <= commands[i].function.runtime.maxValue)
          {
            commands[i].function.runtime.func(param);
          }
          else
          {
            Serial.print("Parameter must be between ");
            Serial.print(commands[i].function.runtime.minValue);
            Serial.print(" and ");
            Serial.println(commands[i].function.runtime.maxValue);
          }
        }
        else
        {
          Serial.print("Usage: ");
          Serial.print(commands[i].shortCmd);
          Serial.println(" <value>");
        }
        break;
      }
      isExecutingCommand = false;
      return;
    }
  }
}

bool isValidCommand(String input)
{
  String cmd = input.toLowerCase();
  for (int i = 0; i < numCommands; i++)
  {
    if (commands[i].type == RUNTIME_PARAM)
    {
      // For runtime parameter commands, check if it starts with the command
      if (cmd.startsWith(String(commands[i].shortCmd) + " "))
      {

        int param;
        if (parseParameter(input, commands[i].shortCmd, param))
        {
          return param >= commands[i].function.runtime.minValue &&
                 param <= commands[i].function.runtime.maxValue;
        }
        return false;
      }
    }
    else if (cmd == commands[i].shortCmd)
    {
      return true;
    }
  }
  return false;
}

void loop()
{
  if (isExecutingCommand)
  {
    executeCommand(currentCommand);
  }
  else
  {
    String command = promptForText("Enter command: ");

    if (isValidCommand(command))
    {
      currentCommand = command;
      isExecutingCommand = true;
      executeCommand(command);
    }
    else
    {
      Serial.println("Invalid command or parameter.");
      printAvailableCommands();
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
  ARM.setPWM(ELBOW, 0, deglen);
  Serial.println("elbow pwm " + String(deglen) + " and elbow deg " + String(deg));
  delay(20);

  // if (currentElbow <= deglen)
  // {
  //   // Serial.println("Elbow");
  //   for (int pulselen = currentElbow; pulselen < deglen; pulselen++)
  //   {
  //     ARM.setPWM(ELBOW, 0, pulselen);
  //     // Serial.println(deglen);
  //     delay(20);
  //   }
  // }
  // else
  // {
  //   for (int pulselen = currentElbow; pulselen > deglen; pulselen--)
  //   {
  //     ARM.setPWM(ELBOW, 0, pulselen);
  //     delay(20);
  //   }
  // }

  currentElbow = deglen;
}

void shoulder(int deg)
{
  int deglen = (deg + 67.8) * 1.77; // pulselen of commanded degrees
  ARM.setPWM(SHOULDER, 0, deglen);
  Serial.println("sholder pwm " + String(deglen) + " and shoulder deg " + String(deg));
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