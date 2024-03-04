#ifndef BRANLED_H
#define BRANLED_H

#include <Arduino.h>
#include <ros.h>

class BrandeisLED {
public:
    BrandeisLED();
    void disconnectedLoop();
    
private:
    Adafruit_MCP23X17 mcp;
}

#endif
