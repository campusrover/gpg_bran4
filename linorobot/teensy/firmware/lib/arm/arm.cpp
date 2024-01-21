#include "arm.h"

Arm::Arm() : nodeHandle(nullptr)
{
    ARM = Adafruit_PWMServoDriver(0x40);
    iteration_time = millis();
    iteration_interval = 20; // 20 milli second
    state = "idle";
};

void Arm::setup(ros::NodeHandle &nh)
{
    nodeHandle = &nh;
    ARM.begin();
    ARM.setPWMFreq(60);
    ARM.setPWM(CLAW, 0, CLAWPARK);
    ARM.setPWM(WRIST, 0, WRISTPARK);
    ARM.setPWM(ELBOW, 0, ELBOWPARK);
    ARM.setPWM(SHOULDER, 0, SHOULDERPARK);
    nodeHandle->loginfo("Arm setup complete");
}

void Arm::loop()
{
    if (millis() < iteration_time + iteration_interval || state == "idle")
        return;
    iteration_time = millis();
    if (state == "move")
        move();
}

void Arm::elbow(float deg)
{
    int deglen = (deg + EL_DEGOFFSET) * EL_DEGSCALE; // pulselen of commanded degrees
    ARM.setPWM(ELBOW, 0, deglen);
}

void Arm::shoulder(float deg)
{
    int deglen = (deg + SH_DEGOFFSET) * SH_DEGSCALE; // pulselen of commanded degrees
    ARM.setPWM(SHOULDER, 0, deglen);
}

void Arm::wrist(float deg)
{
    int deglen = (deg + WR_DEGOFFSET) * WR_DEGSCALE; // pulselen of commanded degrees
    ARM.setPWM(WRIST, 0, deglen);
}

void Arm::openClaw()
{ // Claw   MIN is closed   MAX is open

    if (currentClaw >= CLAWOPEN)
    {
        for (int pulselen = currentClaw; pulselen > CLAWOPEN; pulselen--)
        {
            ARM.setPWM(CLAW, 0, CLAWOPEN);
            delay(20);
        }
    }
    currentClaw = CLAWOPEN;
}

void Arm::closeClaw()
{ // Claw MIN is closed   MAX is open

    if (currentClaw <= CLAWCLOSED)
    {
        for (int pulselen = currentClaw; pulselen < CLAWCLOSED; pulselen++)
        {
            ARM.setPWM(CLAW, 0, CLAWCLOSED);
            delay(20);
        }
    }
    currentClaw = CLAWCLOSED;
}

String Arm::getState()
{
    return state;
}

void Arm::calculateIterationDeltas()
{
    int shouldercnt = 0;
    int elbowcnt = 0;
    int wristcnt = 0;
    float shoulderdelta = 0;
    float elbowdelta = 0;
    float wristdelta = 0;

    shouldercnt = abs(destination_shoulder - currentShoulder);
    elbowcnt = abs(destination_elbow - currentElbow);
    wristcnt = abs(destination_wrist - currentWrist); // note wrist is backwards

    if (shouldercnt > elbowcnt && shouldercnt > wristcnt)
        iterations = shouldercnt;
    if (elbowcnt > shouldercnt && elbowcnt > wristcnt)
        iterations = elbowcnt;
    if (wristcnt > shouldercnt && wristcnt > elbowcnt)
        iterations = wristcnt;

    shouldercnt = destination_shoulder - currentShoulder;
    elbowcnt = destination_elbow - currentElbow;
    wristcnt = destination_wrist - currentWrist; // note wrist is backwards
    iterations = iterations / 2;
    if (iterations == 0)
    {
        shoulderdelta = 0;
        elbowdelta = 0;
        wristdelta = 0;
    }
    else
    {
        shoulderdelta = shouldercnt / iterations;
        elbowdelta = elbowcnt / iterations;
        wristdelta = wristcnt / iterations;
    }
}

int Arm::move()
{
    if (iterations <= 0 || state != "move")
    {
        state = "idle";
        return 0;
    }
    currentWrist = currentWrist + wristdelta;
    wrist(currentWrist);
    currentElbow = currentElbow + elbowdelta;
    elbow(currentElbow);
    currentShoulder = currentShoulder + shoulderdelta;
    shoulder(currentShoulder);
    iterations = iterations - 1;
    return iterations;
}

void Arm::armCommand(String command)
{
    if (command == "park")
    {
        destination_shoulder = SHOULDERPARKDEG;
        destination_wrist = WRISTPARKDEG;
        destination_elbow = ELBOWPARKDEG;
        calculateIterationDeltas();
        state = "move";
        return;
    }
    if (command == "floor")
    {
        destination_shoulder = SHOULDERFLOORDEG;
        destination_wrist = WRISTFLOORDEG;
        destination_elbow = ELBOWFLOORDEG;
        calculateIterationDeltas();
        state = "move";
    }
}

void Arm::armCommand(String command, float arg)
{
    if (command == "wrist")
    {
        destination_shoulder = currentShoulder;
        destination_wrist = (int)arg;
        destination_elbow = currentElbow
        calculateIterationDeltas();
        state = "move";
    }
    if (command == "elbow")
    {
        destination_shoulder = currentShoulder;
        destination_wrist = currentWrist
        destination_elbow = (int)arg;
        calculateIterationDeltas();
        state = "move";
    }

    if (command == "shoulder")
    {
        destination_shoulder = (int)arg;
        destination_wrist = currentWrist
        destination_elbow = currentElbow
        state = "move";
        calculateIterationDeltas();
    }
}

// void Arm::wrist(int deg)
// {
//     int deglen = (deg + 75) * 1.77; // pulselen of commanded degrees  Rev 1
//                                     // int deglen = ((-deg +180 +63.5)*1.77 );  //rev 4
//     if (currentWrist <= deglen)
//     {
//         for (int pulselen = currentWrist; pulselen < deglen; pulselen++)
//         {
//             ARM.setPWM(WRIST, 0, pulselen);
//             delay(20);
//         }
//     }
//     else
//     {
//         for (int pulselen = currentWrist; pulselen > deglen; pulselen--)
//         {
//             ARM.setPWM(WRIST, 0, pulselen);
//             delay(20);
//         }
//     }
//     currentWrist = deglen;
// }

// void Arm::elbow(int deg)
// {
//     int deglen = (deg + 43) * 2.5; // pulselen of commanded degrees

//     if (currentElbow <= deglen)
//     {
//         for (int pulselen = currentElbow; pulselen < deglen; pulselen++)
//         {
//             ARM.setPWM(ELBOW, 0, pulselen);
//             delay(20);
//         }
//     }
//     else
//     {
//         for (int pulselen = currentElbow; pulselen > deglen; pulselen--)
//         {
//             ARM.setPWM(ELBOW, 0, pulselen);
//             delay(20);
//         }
//     }

//     currentElbow = deglen;
// }

// void Arm::shoulder(int deg)
// {
//     int deglen = (deg + 67.8) * 1.77; // pulselen of commanded degrees

//     if (currentShoulder <= deglen)
//     {
//         for (int pulselen = currentShoulder; pulselen < deglen; pulselen++)
//         {
//             ARM.setPWM(SHOULDER, 0, pulselen);
//             delay(20);
//         }
//     }
//     else
//     {
//         for (int pulselen = currentShoulder; pulselen > deglen; pulselen--)
//         {
//             ARM.setPWM(SHOULDER, 0, pulselen);
//             delay(20);
//         }
//     }
//     currentShoulder = deglen;
// }