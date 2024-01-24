#include "branarm.h"


ServoCoords arm_locs[] = {
    {SH_PARK_DEG, EL_PARK_DEG, WR_PARK_DEG},
    {SH_FLOOR_DOWN_DEG, EL_FLOOR_DOWN_DEG, WR_FLOOR_DOWN_DEG},
    {SH_STRAIGHTUP, EL_STRAIGHTUP, WR_STRAIGHTUP},
    {SH_VERT_HORIZ_HAND, EL_VERT_HORIZ_HAND, WR_VERT_HORIZ_HAND},
    {SH_ALL_BACKWARD_DEG, EL_ALL_BACKWARD_DEG, WR_ALL_BACKWARD_DEG},
    {SH_FLOOR_UP_DEG, EL_FLOOR_UP_DEG, WR_FLOOR_UP_DEG}};

ServoInfo servos = {BrandeisServo(), BrandeisServo(), BrandeisServo(), BrandeisServo()};

BrandeisArm::BrandeisArm() : node_handle(nullptr)
{
    ARM = Adafruit_PWMServoDriver(0x40);
    iteration_time = millis();
    iteration_interval = 20; // 20 milli second
    state = "idle";
};

void BrandeisArm::setup(ros::NodeHandle &nh)
{
    node_handle = &nh;
    ARM.begin();
    ARM.setPWMFreq(60);
    ARM.setPWM(CLAW, 0, CLAWPARK);
    ARM.setPWM(WRIST, 0, WRISTPARK);
    ARM.setPWM(ELBOW, 0, ELBOWPARK);
    ARM.setPWM(SHOULDER, 0, SHOULDERPARK);

    node_handle->loginfo("Arm setup complete");

    current_claw = CLAWPARKDEG;
    current_wrist = WR_PARK_DEG;
    current_elbow = EL_PARK_DEG;
    current_shoulder = SH_PARK_DEG;
    BrandeisArm::traceOut("setup");

    servos.shoulder.current = SH_PARK_DEG;
    servos.elbow.current = EL_PARK_DEG;
    servos.wrist.current = WR_PARK_DEG;
    servos.claw.current = CLAWPARKDEG;
}

void BrandeisArm::traceOut(String msg)
{
    char buffer[100];
    sprintf(buffer, "msg: %s, curSh %f, curEl %f,  curWr %f, curCl %f", msg.c_str(), current_shoulder, current_elbow, current_wrist, current_claw);
    node_handle->loginfo(buffer);
}

void BrandeisArm::loop()
{
    if (millis() < iteration_time + iteration_interval || state == "idle")
        return;
    iteration_time = millis();
    if (state == "move")
        move();
    else if (state == "movex")
    {
        movex();
    }
}

void BrandeisArm::calculate_iteration_deltas()
{
    int shouldercnt;
    int elbowcnt;
    int wristcnt;

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

void BrandeisArm::configure_ease_algorithm(int duration_in_ms)
{
    servos.shoulder.destination = destination_shoulder;
    servos.elbow.destination = destination_elbow;
    servos.wrist.destination = destination_wrist;
    servos.shoulder.duration_ms = duration_in_ms;
    servos.elbow.duration_ms = duration_in_ms;
    servos.wrist.duration_ms = duration_in_ms;
}

int BrandeisArm::move()
{
    current_wrist = current_wrist + wristDelta;
    wrist(current_wrist);
    current_elbow = current_elbow + elbowDelta;
    elbow(current_elbow);
    current_shoulder = current_shoulder + shoulderDelta;
    shoulder(current_shoulder);
    iterations = iterations - 1;
    return iterations;
}

void BrandeisArm::movex()
{
    servos.shoulder.calc_elapsed(millis());
    servos.elbow.calc_elapsed(millis());
    servos.wrist.calc_elapsed(millis());
    servos.claw.calc_elapsed(millis());
    servos.shoulder.update_current();
    servos.elbow.update_current();
    servos.wrist.update_current();
    servos.claw.update_current();
}

void BrandeisArm::elbow(float deg)
{
    int deglen = (deg + EL_DEGOFFSET) * EL_DEGSCALE; // pulselen of commanded degrees
    ARM.setPWM(ELBOW, 0, deglen);
}

void BrandeisArm::shoulder(float deg)
{
    int deglen = (deg + SH_DEGOFFSET) * SH_DEGSCALE; // pulselen of commanded degrees
    ARM.setPWM(SHOULDER, 0, deglen);
}

void BrandeisArm::wrist(float deg)
{
    int deglen = (deg + WR_DEGOFFSET) * WR_DEGSCALE; // pulselen of commanded degrees
    ARM.setPWM(WRIST, 0, deglen);
}
// true = open; false = close
void BrandeisArm::claw(bool open_close)
{
    if (open_close)
    {
        if (current_claw >= CLAWOPEN)
        {
            for (int pulse_len = current_claw; pulse_len > CLAWOPEN; pulse_len--)
            {
                ARM.setPWM(CLAW, 0, pulse_len);
                delay(20);
            }

            current_claw = CLAWOPEN;
        }
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

void BrandeisArm::open_claw()
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

void BrandeisArm::close_claw()
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

String BrandeisArm::getState()
{
    return state;
}

void BrandeisArm::arm_command(String command)
{
    if (command == "park")
    {
        destination_shoulder = SH_PARK_DEG;
        destination_wrist = WR_PARK_DEG;
        destination_elbow = EL_PARK_DEG;
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
    if (command == "allback")
    {
        destination_shoulder = SH_ALL_BACKWARD_DEG;
        destination_wrist = WR_ALL_BACKWARD_DEG;
        destination_elbow = EL_ALL_BACKWARD_DEG;
        calculate_iteration_deltas();
        state = "move";
    }
    if (command == "allforward")
    {
        destination_shoulder = SH_ALL_FORWARD_DEG;
        destination_wrist = WR_ALL_FORWARD_DEG;
        destination_elbow = EL_ALL_FORWARD_DEG;
        calculate_iteration_deltas();
        state = "move";
    }
    if (command == "floorup")
    {
        destination_shoulder = SH_FLOOR_UP_DEG;
        destination_wrist = WR_FLOOR_UP_DEG;
        destination_elbow = EL_FLOOR_UP_DEG;
        calculate_iteration_deltas();
        state = "move";
    }
    if (command == "floordown")
    {
        destination_shoulder = SH_FLOOR_DOWN_DEG;
        destination_wrist = WR_FLOOR_DOWN_DEG;
        destination_elbow = EL_FLOOR_DOWN_DEG;
        calculate_iteration_deltas();
        state = "move";
    }

    BrandeisArm::traceOut("arm_command");
}

void BrandeisArm::arm_command(String command, float arg)
{
    BrandeisArm::traceOut("armCommand");

    if (command == "wrist")
    {
        destination_shoulder = current_shoulder;
        destination_wrist = (int)arg;
        destination_elbow = current_elbow;
        calculate_iteration_deltas();
        state = "move";
    }
    if (command == "elbow")
    {
        destination_shoulder = current_shoulder;
        destination_wrist = current_wrist;
        destination_elbow = (int)arg;
        calculate_iteration_deltas();
        state = "move";
    }

    if (command == "shoulder")
    {
        destination_shoulder = (int)arg;
        destination_wrist = current_wrist;
        destination_elbow = current_elbow;
        state = "movex";
        configure_ease_algorithm(6000);
        // calculate_iteration_deltas();
    }
}
