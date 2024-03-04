#include <Wire.h>
#include <Adafruit_MCP23X17.h>
 
#define LEFT_LED_PIN 1      // MCP23XXX pin Left LED is attached to
#define RIGHT_LED_PIN 2     // MCP23XXX pin Right LED is attached to

Adafruit_MCP23X17 mcp;
 
void setup() {
  Serial.begin(9600);
  //while (!Serial);
  Serial.println("MCP23xxx Combo Test!");
 
  if (!mcp.begin_I2C(0x58)) {
      Serial.println("Error.");
      while (1);
  }
 
  // configure LED pin for output
  mcp.pinMode(RIGHT_LED_PIN, OUTPUT);
  mcp.pinMode(LEFT_LED_PIN, OUTPUT);
 
}
 
void loop() {
  mcp.digitalWrite(LEFT_LED_PIN, HIGH);
  mcp.digitalWrite(RIGHT_LED_PIN, LOW);
  delay(500);
  mcp.digitalWrite(LEFT_LED_PIN, LOW);
  mcp.digitalWrite(RIGHT_LED_PIN, HIGH);
  delay(500);
}
 