#include "crinstrument.h"
#include "lino_msgs/Inst.h"
#include "std_msgs/UInt16.h"

lino_msgs::Inst inst_msg;
ros::Publisher inst_pub("inst", &inst_msg);

class Instrument
{

    Instrument() {};
    
    void setup(NodeHandle nh);

    
}