#include "arm.h"

Arm::Arm() : node_handle(nullptr)
{
    ARM = Adafruit_PWMServoDriver(0x40);
    iteration_time = millis();
    iteration_interval = 20; // 20 milli second
    state = "idle";
};

void Arm::setup(ros::NodeHandle &nh)
{
    node_handle = &nh;
    ARM.begin();
    ARM.setPWMFreq(60);
    ARM.setPWM(CLAW, 0, CLAWPARK);
    ARM.setPWM(WRIST, 0, WRISTPARK);
    ARM.setPWM(ELBOW, 0, ELBOWPARK);
    ARM.setPWM(SHOULDER, 0, SHOULDERPARK);
<<<<<<< HEAD
    node_handle->loginfo("Arm setup complete");
=======
    currentClaw = CLAWPARKDEG;
    currentWrist = WRISTPARKDEG;
    currentElbow = ELBOWPARKDEG;
    currentShoulder = SHOULDERPARKDEG;
    Arm::traceOut("setup");
}

void Arm::traceOut(String msg)
{
    char buffer[100];
    sprintf(buffer, "msg: %s, curSh %f, curEl %f,  curWr %f, curCl %f", msg.c_str(), currentShoulder, currentElbow, currentWrist, currentClaw);
    nodeHandle->loginfo(buffer);
>>>>>>> f5ae87f68c69d9e8117f016557061c6fa3309c26
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
// true = open; false = close
void Arm::claw(bool open_close)
{
    if (open_close)
        if (current_claw >= CLAWOPEN)
        {
            for (int pulse_len = current_claw; pulse_len > CLAWOPEN; pulse_len--)
            {
                ARM.setPWM(CLAW, 0, pulse_len);
                delay(20);
            }

            current_claw = CLAWOPEN;
        }
        else
        {
            if (current_claw <= CLAWCLOSED)
            {
                for (int pulse_len = current_claw; pulse_len < CLAWCLOSED; pulse_len++)
                {
                    ARM.setPWM(CLAW, 0, pulse_len);
                    delay(20);
                }
            }
            current_claw = CLAWCLOSED;
        }
}

void Arm::open_claw()
{ // Claw   MIN is closed   MAX is open

    if (current_claw >= CLAWOPEN)
    {
        for (int pulselen = current_claw; pulselen > CLAWOPEN; pulselen--)
        {
            ARM.setPWM(CLAW, 0, CLAWOPEN);
            delay(20);
        }
    }
    current_claw = CLAWOPEN;
}

void Arm::close_claw()
{ // Claw MIN is closed   MAX is open

    if (current_claw <= CLAWCLOSED)
    {
        for (int pulselen = current_claw; pulselen < CLAWCLOSED; pulselen++)
        {
            ARM.setPWM(CLAW, 0, CLAWCLOSED);
            delay(20);
        }
    }
    current_claw = CLAWCLOSED;
}

String Arm::getState()
{
    return state;
}

void Arm::calculate_iteration_deltas()
{
    int shouldercnt = 0;
    int elbowcnt = 0;
    int wristcnt = 0;

    shouldercnt = abs(destination_shoulder - current_shoulder);
    elbowcnt = abs(destination_elbow - current_elbow);
    wristcnt = abs(destination_wrist - current_wrist); // note wrist is backwards

    if (shouldercnt > elbowcnt && shouldercnt > wristcnt)
        iterations = shouldercnt;
    if (elbowcnt > shouldercnt && elbowcnt > wristcnt)
        iterations = elbowcnt;
    if (wristcnt > shouldercnt && wristcnt > elbowcnt)
        iterations = wristcnt;

    shouldercnt = destination_shoulder - current_shoulder;
    elbowcnt = destination_elbow - current_elbow;
    wristcnt = destination_wrist - current_wrist; // note wrist is backwards
    iterations = iterations / 2;
    if (iterations == 0)
    {
        shoulderDelta = 0;
        elbowDelta = 0;
        wristDelta = 0;
    }
    else
    {
        shoulderDelta = shouldercnt / iterations;
        elbowDelta = elbowcnt / iterations;
        wristDelta = wristcnt / iterations;
    }
}

int Arm::move()
{
    if (iterations <= 0 || state != "move")
    {
        state = "idle";
        return 0;
    }
<<<<<<< HEAD
    current_wrist = current_wrist + wristdelta;
    wrist(current_wrist);
    current_elbow = current_elbow + elbowdelta;
    elbow(current_elbow);
    current_shoulder = current_shoulder + shoulderdelta;
    shoulder(current_shoulder);
=======
    currentWrist = currentWrist + wristDelta;
    wrist(currentWrist);
    currentElbow = currentElbow + elbowDelta;
    elbow(currentElbow);
    currentShoulder = currentShoulder + shoulderDelta;
    shoulder(currentShoulder);
>>>>>>> f5ae87f68c69d9e8117f016557061c6fa3309c26
    iterations = iterations - 1;
    return iterations;
}

void Arm::arm_command(String command)
{
    if (command == "park")
    {
        destination_shoulder = SH_PARK_DEG;
        destination_wrist = WR_PARK_DEG;
        destination_elbow = EL_PARK_DEG;
        calculate_iteration_deltas();
        state = "move";
        return;
    }
    if (command == "floor")
    {
        destination_shoulder = SH_FLOOR_DEG;
        destination_wrist = WR_FLOOR_DEG;
        destination_elbow = EL_FLOOR_DEG;
        calculate_iteration_deltas();
        state = "move";
    }
    if (command == "straightup")
    {
        destination_shoulder = SH_STRAIGHTUP;
        destination_wrist = WR_STRAIGHTUP;
        destination_elbow = EL_STRAIGHTUP;
        calculate_iteration_deltas();
        state = "move";
    }
    if (command == "verthorizhand")
    {
        destination_shoulder = SH_VERT_HORIZ_HAND;
        destination_wrist = WR_VERT_HORIZ_HAND;
        destination_elbow = EL_VERT_HORIZ_HAND;
        calculate_iteration_deltas();
        state = "move";
    }
    if (command == "allforward")
    {
        destination_shoulder = SH_ALL_FORWARD;
        destination_wrist = WR_ALL_FORWARD;
        destination_elbow = EL_ALL_FORWARD;
        calculate_iteration_deltas();
        state = "move";
    }
    if (command == "allvertabovepark")
    {
        destination_shoulder = SH_ALL_VERTABOVEPARK;
        destination_wrist = WR_ALL_VERTABOVEPARK;
        destination_elbow = EL_ALL_VERTABOVEPARK;
        calculate_iteration_deltas();
        state = "move";
    }
}

void Arm::arm_command(String command, float arg)
{
    Arm::traceOut("armCommand");

    if (command == "wrist")
    {
        destination_shoulder = current_shoulder;
        destination_wrist = (int)arg;
<<<<<<< HEAD
        destination_elbow = current_elbow
=======
        destination_elbow = currentElbow;
>>>>>>> f5ae87f68c69d9e8117f016557061c6fa3309c26
        calculateIterationDeltas();
        state = "move";
    }
    if (command == "elbow")
    {
<<<<<<< HEAD
        destination_shoulder = current_shoulder;
        destination_wrist = current_wrist
            destination_elbow = (int)arg;
        calculate_iteration_deltas();
=======
        destination_shoulder = currentShoulder;
        destination_wrist = currentWrist;
        destination_elbow = (int)arg;
        calculateIterationDeltas();
>>>>>>> f5ae87f68c69d9e8117f016557061c6fa3309c26
        state = "move";
    }

    if (command == "shoulder")
    {
        destination_shoulder = (int)arg;
<<<<<<< HEAD
        destination_wrist = current_wrist
            destination_elbow = currentElbow
                state = "move";
        calculate_iteration_deltas();
    }
}
=======
        destination_wrist = currentWrist;
        destination_elbow = currentElbow;
        state = "move";
        calculateIterationDeltas();
    }
}
>>>>>>> f5ae87f68c69d9e8117f016557061c6fa3309c26
