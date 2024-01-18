
void LEDCallback(const lino_msgs::Led& led_msg)
{   
    if (led_msg.wire == 1){
        LED_on1 = led_msg.on;
        LED_blink1 = led_msg.blink;
    } else if (led_msg.wire == 2){
        LED_on2 = led_msg.on;
        LED_blink2 = led_msg.blink;
    } else if (led_msg.wire == 3){
        LED_on3 = led_msg.on;
        LED_blink3 = led_msg.blink;
    }
    
}


void LEDUpdate()
{
    if (LED_on1 && !LED_blink1)
    {
        aw.digitalWrite(1, LOW);
    } else if (LED_on1 && LED_blink1)
    {   
        if (LED_blink_on1){
            aw.digitalWrite(1, LOW);
            LED_blink_on1 = false;
        } else{
            aw.digitalWrite(1, HIGH);
            LED_blink_on1 = true;
        }
    } else if (!LED_on1){
        aw.digitalWrite(1, HIGH);
    }


    if (LED_on2 && !LED_blink2)
    {
        aw.digitalWrite(2, LOW);
    } else if (LED_on2 && LED_blink2)
    {   
        if (LED_blink_on2){
            aw.digitalWrite(2, LOW);
            LED_blink_on2 = false;
        } else{
            aw.digitalWrite(2, HIGH);
            LED_blink_on2 = true;
        }
    } else if (!LED_on2){
        aw.digitalWrite(2, HIGH);
    }


    if (LED_on3 && !LED_blink3)
    {
        aw.digitalWrite(3, LOW);
    } else if (LED_on3 && LED_blink3)
    {   
        if (LED_blink_on3){
            aw.digitalWrite(3, LOW);
            LED_blink_on3 = false;
        } else{
            aw.digitalWrite(3, HIGH);
            LED_blink_on3 = true;
        }
    } else if (!LED_on3){
        aw.digitalWrite(3, HIGH);
    }
}


//Header file for subscribing for LED
#include "lino_msgs/Led.h"

// ros::Subscriber<lino_msgs::Led> led_sub("led", LEDCallback);
