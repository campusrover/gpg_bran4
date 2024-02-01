#include "branarm.h"

ArmPositions arm_locs[] = {
    {"park", SH_PARK_DEG, EL_PARK_DEG, WR_PARK_DEG},
    {"floordown", SH_FLOOR_DOWN_DEG, EL_FLOOR_DOWN_DEG, WR_FLOOR_DOWN_DEG},
    {"straightup", SH_STRAIGHTUP, EL_STRAIGHTUP, WR_STRAIGHTUP},
    {"vert1", SH_VERT_HORIZ_HAND, EL_VERT_HORIZ_HAND, WR_VERT_HORIZ_HAND},
    {"straightback", SH_ALL_BACKWARD_DEG, EL_ALL_BACKWARD_DEG,
     WR_ALL_BACKWARD_DEG},
    {"floorup", SH_FLOOR_UP_DEG, EL_FLOOR_UP_DEG, WR_FLOOR_UP_DEG}};

int log_counter = 0;
int shoulder_counter = 0;
int skipped_loop_counter = 0;

BrandeisServo shoulder_servo;
BrandeisServo elbow_servo;
BrandeisServo wrist_servo;
BrandeisServo claw_servo;

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

  shoulder_servo.setup(100, *node_handle, servo_driver, SHOULDER,
                       SH_MAX_DEG, SH_MIN_DEG, SH_DEGOFFSET,
                       SH_DEGSCALE, SH_PARK_DEG);
  elbow_servo.setup(101, *node_handle, servo_driver, ELBOW, EL_MAX_DEG,
                    EL_MIN_DEG, EL_DEGOFFSET, EL_DEGSCALE, EL_PARK_DEG);
  wrist_servo.setup(102, *node_handle, servo_driver, WRIST, WR_MAX_DEG,
                    WR_MIN_DEG, WR_DEGOFFSET, WR_DEGSCALE, WR_PARK_DEG);
  claw_servo.setup(103, *node_handle, servo_driver, CLAW, CL_MAX_DEG,
                   CL_MIN_DEG, CL_DEGOFFSET, CL_DEGSCALE, CL_PARK_DEG);
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
      !(shoulder_servo.moving || elbow_servo.moving || wrist_servo.moving || claw_servo.moving);
  return stopped;
}

void BrandeisArm::movex() {
  if (shoulder_servo.moving) {
    double new_angle = shoulder_servo.compute_next_increment(millis());
    shoulder_servo.move(new_angle);
  }
  if (elbow_servo.moving) {
    double new_angle = elbow_servo.compute_next_increment(millis());
    elbow_servo.move(new_angle);
  }
  if (wrist_servo.moving) {
    double new_angle = wrist_servo.compute_next_increment(millis());
    wrist_servo.move(new_angle);
  }
  if (claw_servo.moving) {
    double new_angle = claw_servo.compute_next_increment(millis());
    claw_servo.move(new_angle);
  }
}

String BrandeisArm::getState() { return state; }

void BrandeisArm::arm_command(String command, float arg) {
  if (command == "wrist") {
    double arg_double = (double)arg;
    LOG_INFO("destination wrist: %f", arg_double);
    wrist_servo.setup_ease(arg_double);
    state = "movex";
  }

  if (command == "elbow") {
    double arg_double = (double)arg;
    LOG_INFO("destination elbow: %f", arg_double);
    elbow_servo.setup_ease(arg_double);
    state = "movex";
  }

  if (command == "shoulder") {
    double arg_double = (double)arg;
    LOG_INFO("destination shoulder: %f", arg_double);
    shoulder_servo.setup_ease(arg_double);
    state = "movex";
  }
  if (command == "claw") {
    double arg_double = (double)arg;
    LOG_INFO("destination claw: %f", arg_double);
    claw_servo.setup_ease(arg_double);
    state = "movex";
  }

  if (command == "reset") {
    double arg_double = (double)arg;
    LOG_INFO("reset: %f", arg_double);
    shoulder_servo.move(SH_PARK_DEG);
    elbow_servo.move(EL_PARK_DEG);
    wrist_servo.move(WR_PARK_DEG);
    claw_servo.move(CL_PARK_DEG);
  }
  if (command == "test") {
    if (arg == 1) {
      shoulder_servo.test_mode = true;
      elbow_servo.test_mode = true;
      wrist_servo.test_mode = true;
      claw_servo.test_mode = true;
      LOG_INFO("test mode off. ARM NO MOTION: %s", state.c_str());

    } else if (arg == 0) {
      shoulder_servo.test_mode = false;
      elbow_servo.test_mode = false;
      wrist_servo.test_mode = false;
      claw_servo.test_mode = false;
      LOG_INFO("test mode off. ARM WILL MOVE: %s", state.c_str());
    }
  }
  for (ArmPositions pos : arm_locs) {
    if (command == pos.name) {
      LOG_INFO("arm_command: %s", command.c_str());
      shoulder_servo.setup_ease(pos.shoulder);
      elbow_servo.setup_ease(pos.elbow);
      wrist_servo.setup_ease(pos.wrist);
      state = "movex";
    }
  }
}
