
#define PINCERMIN  170
#define PINCERMAX  300 
#define CLAWMIN 212
#define CLAWMAX 360
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum_pincer = 0;
uint16_t pulselen_pincer = PINCERMIN;

uint8_t servonum_claw = 1;
uint16_t pulselen_claw = CLAWMIN;

