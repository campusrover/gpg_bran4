#include "arm.h"

Arm::Arm() : nodeHandle(nullptr)
{
    ARM = Adafruit_PWMServoDriver(0x40);
    iteration_time = millis();
    iteration_interval = 20; // 20 milli second
    state = "idle";
};

void Arm::setup(ros::NodeHandle& nh) 
{
    nodeHandle = &nh;
    ARM.begin();
    ARM.setPWMFreq(60);
    ARM.setPWM(CLAW, 0, CLAWPARK);
    ARM.setPWM(WRIST, 0, WRISTPARK);
    ARM.setPWM(ELBOW, 0, ELBOWPARK);
    ARM.setPWM(SHOULDER, 0, SHOULDERPARK);
    // pinMode(5, OUTPUT);
    // digitalWrite(5, LOW); // turn the Left wheel off by making the voltage LOW
    nodeHandle->loginfo("Arm setup complete");
}

void Arm::loop()
{
    if (millis() < iteration_time + iteration_interval || state == "idle")
        return;
    iteration_time = millis();
    if (state == "move") move();
    return;
}

void Arm::elbow(float deg)
{
    int deglen = (deg + 43) * 2.5; // pulselen of commanded degrees
    ARM.setPWM(ELBOW, 0, deglen);
    // delay(20);
    // CurrentElbow = deg;
}

void Arm::shoulder(float deg)
{
    int deglen = (deg + 67.8) * 1.77; // pulselen of commanded degrees
    ARM.setPWM(SHOULDER, 0, deglen);
    // delay(20);
    // CurrentShoulder = deg;
}

void Arm::wrist(float deg)
{
    int deglen = (deg + 75) * 1.77; // pulselen of commanded degrees
    ARM.setPWM(WRIST, 0, deglen);
    // delay(20);
    // CurrentWrist = deg;
}

String Arm::getState()
{
    return state;
}

void Arm::calculateIterationDeltas()
{
    // nh.loginfo("calculateIterationDeltas");

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
    if (iterations == 0) {
        shoulderdelta = 0;
        elbowdelta = 0;
        wristdelta = 0;
    } else {
        shoulderdelta = shouldercnt / iterations;
        elbowdelta = elbowcnt / iterations;
        wristdelta = wristcnt / iterations;
    }
}

int Arm::move()
{
    if (iterations <= 0 || state != "move") {
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
    