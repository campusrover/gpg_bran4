#include "branarm.h"

ArmPositions arm_locs[] = {
    {"park", SH_PARK_DEG, EL_PARK_DEG, WR_PARK_DEG, 5000},
    {"floordown", SH_FLOOR_DOWN_DEG, EL_FLOOR_DOWN_DEG, WR_FLOOR_DOWN_DEG,
     5000},
    {"straightup", SH_STRAIGHTUP, EL_STRAIGHTUP, WR_STRAIGHTUP, 5000},
    {"vert1", SH_VERT_HORIZ_HAND, EL_VERT_HORIZ_HAND, WR_VERT_HORIZ_HAND, 5000},
    {"straightback", SH_ALL_BACKWARD_DEG, EL_ALL_BACKWARD_DEG,
     WR_ALL_BACKWARD_DEG, 5000},
    {"floorup", SH_FLOOR_UP_DEG, EL_FLOOR_UP_DEG, WR_FLOOR_UP_DEG, 5000}};

ServoInfo servos = {BrandeisServo(), BrandeisServo(), BrandeisServo(),
                    BrandeisServo()};
int log_counter = 0;
int shoulder_counter = 0;
int skipped_loop_counter = 0;

BrandeisArm::BrandeisArm() : node_handle(nullptr) {
  servo_driver = Adafruit_PWMServoDriver(0x40);
  iteration_time = millis();
  iteration_interval = 20; // 20 milli second
  state = "idle";
};

void BrandeisArm::setup(ros::NodeHandle &nh) {
  node_handle = &nh;
  servo_driver.begin();
  servo_driver.setPWMFreq(60);

  servos.shoulder.setup("shoulder", *node_handle, servo_driver, SHOULDER,
                        SHOULDERMAXDEG, SHOULDERMINDEG, SH_DEGOFFSET,
                        SH_DEGSCALE, SH_PARK_DEG);
  servos.elbow.setup("elbow", *node_handle, servo_driver, ELBOW, ELBOWMAXDEG,
                     ELBOWMINDEG, EL_DEGOFFSET, EL_DEGSCALE, EL_PARK_DEG);
  servos.wrist.setup("wrist", *node_handle, servo_driver, WRIST, WRISTMAXDEG,
                     WRISTMINDEG, WR_DEGOFFSET, WR_DEGSCALE, WR_PARK_DEG);
  servos.claw.setup("claw", *node_handle, servo_driver, CLAW, CLAWMAXDEG,
                    CLAWMINDEG, CL_DEGOFFSET, CL_DEGSCALE, CLAWPARKDEG);
}

void BrandeisArm::loop() {
  if (millis() <= iteration_time + iteration_interval) {
    return;
  }
  iteration_time = millis();
  if (state == "idle") {
    return;
  } else if (state == "movex") {
    movex();
    if (arm_motion_stopped()) {
      state = "idle";
    }
  }
}

bool BrandeisArm::arm_motion_stopped(void) {
  boolean stopped =
      !(servos.shoulder.moving || servos.elbow.moving || servos.wrist.moving);
  return stopped;
}

void BrandeisArm::movex() {
  LOG_INFO("movex: state: %s", state.c_str());
  if (servos.shoulder.moving) {
    double new_angle = servos.shoulder.compute_next_increment(millis());
    servos.shoulder.move(new_angle);
  }
  if (servos.elbow.moving) {
    double new_angle = servos.elbow.compute_next_increment(millis());
    servos.elbow.move(new_angle);
  }
  if (servos.wrist.moving) {
    double new_angle = servos.wrist.compute_next_increment(millis());
    servos.wrist.move(new_angle);
  }
}

String BrandeisArm::getState() { return state; }

void BrandeisArm::arm_command(String command, float arg) {
  if (command == "wrist") {
    double arg_double = (double)arg;
    LOG_INFO("destination wrist: %f", arg_double);
    servos.wrist.setup_ease(arg_double, millis(), 2000);
    state = "movex";
  }

  if (command == "elbow") {
    double arg_double = (double)arg;
    LOG_INFO("destination elbow: %f", arg_double);
    servos.elbow.setup_ease(arg_double, millis(), 2000);
    state = "movex";
  }

  if (command == "shoulder") {
    double arg_double = (double)arg;
    LOG_INFO("destination shoulder: %f", arg_double);
    servos.shoulder.setup_ease(arg_double, millis(), 2000);
    state = "movex";
  }
  if (command == "combo") {
    double arg_double = (double)arg;
    LOG_INFO("combined: %f", arg_double);
    servos.shoulder.setup_ease(45, millis(), 2000);
    servos.wrist.setup_ease(90, millis(), 2000);
    state = "movex";
  }
  if (command == "test") {
    if (arg == 1) {
      servos.shoulder.test_mode = true;
      servos.elbow.test_mode = true;
      servos.wrist.test_mode = true;
      servos.claw.test_mode = true;
      LOG_INFO("test mode off. ARM NO MOTION", state.c_str());

    } else if (arg == 0) {
      servos.shoulder.test_mode = false;
      servos.elbow.test_mode = false;
      servos.wrist.test_mode = false;
      servos.claw.test_mode = false;
      LOG_INFO("test mode off. ARM WILL MOVE", state.c_str());
    }
  }
  for (ArmPositions pos : arm_locs) {
    if (command == pos.name) {
      LOG_INFO("arm_command: %s", command.c_str());
      servos.shoulder.setup_ease(pos.shoulder, millis(), pos.duration);
      servos.elbow.setup_ease(pos.shoulder, millis(), pos.duration);
      servos.wrist.setup_ease(pos.shoulder, millis(), pos.duration);
      state = "movex";
    }
  }
}
