#include "branarm.h"
#include "branutils.h"

  #define SH_POS1 140
  #define EL_POS1 -5
  #define WR_POS1 70
  #define CL_POS1 21



ArmPositions arm_locs[] = {
    {"park", SH_PARK_DEG, EL_PARK_DEG, WR_PARK_DEG},
    {"floordown", SH_FLOOR_DOWN_DEG, EL_FLOOR_DOWN_DEG, WR_FLOOR_DOWN_DEG},
    {"straightup", SH_STRAIGHTUP_DEG, EL_STRAIGHTUP_DEG, WR_STRAIGHTUP_DEG},
    {"vert1", SH_VERT_HORIZ_HAND, EL_VERT_HORIZ_HAND, WR_VERT_HORIZ_HAND},
    {"straightback", SH_ALL_BACKWARD_DEG, EL_ALL_BACKWARD_DEG,
     WR_ALL_BACKWARD_DEG},
    {"floorup", SH_FLOOR_UP_DEG, EL_FLOOR_UP_DEG, WR_FLOOR_UP_DEG},
    {"floorout", SH_FLOOR_OUT_DEG, EL_FLOOR_OUT_DEG, WR_FLOOR_OUT_DEG}};

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
  iteration_interval = 100; // 20 milli second
  state = "idle";
  arm_prog_pc = -1;
};

void BrandeisArm::setup(ros::NodeHandle &nh) {
  node_handle = &nh;
  servo_driver.begin();
  servo_driver.setPWMFreq(60);

  shoulder_servo.setup(100, *node_handle, servo_driver, SHOULDER, SH_MAX_DEG,
                       SH_MIN_DEG, SH_DEGOFFSET, SH_DEGSCALE, SH_PARK_DEG);
  elbow_servo.setup(101, *node_handle, servo_driver, ELBOW, EL_MAX_DEG,
                    EL_MIN_DEG, EL_DEGOFFSET, EL_DEGSCALE, EL_PARK_DEG);
  wrist_servo.setup(102, *node_handle, servo_driver, WRIST, WR_MAX_DEG,
                    WR_MIN_DEG, WR_DEGOFFSET, WR_DEGSCALE, WR_PARK_DEG);
  claw_servo.setup(103, *node_handle, servo_driver, CLAW, CL_MAX_DEG,
                   CL_MIN_DEG, CL_DEGOFFSET, CL_DEGSCALE, CL_PARK_DEG);
  LOG_ERROR("Arm Setup Complete.");
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
    } else if (state == "moveprog") {
      moveprog();
    }
  }
}

bool BrandeisArm::arm_motion_stopped(void) {
  boolean stopped = !(shoulder_servo.moving || elbow_servo.moving ||
                      wrist_servo.moving || claw_servo.moving);
  return stopped;
}

void BrandeisArm::movex() {
  delay(50);
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

void BrandeisArm::moveprog() {
  delay(50);

}

// heuristic to generate time for a compounde move.
long BrandeisArm::move_duration_heuristic(long new_shoulder, long new_elbow, long new_wrist) {
  long shoulder_travel = abs(new_shoulder - shoulder_servo.current_angle);
  return max(1000, (shoulder_travel * 1000) / 30.0);
}

void BrandeisArm::arm_command(String command, float arg) {
  double arg_double = (double)arg;  
  if (command == "wrist") {
//    long duration = (arg_double * 1000) / 30.0; // 30 degrees per second
    long duration = 5000;
    LOG_INFO("command wrist: %.1f %ld", arg_double, duration);
    wrist_servo.setup_ease(arg_double, duration);
    state = "movex";
  }

  if (command == "elbow") {
//    long duration = (arg_double * 1000) / 30.0; // 30 degrees per second
    long duration = 5000;
    LOG_INFO("command elbow: %f %ld", arg_double, duration);
    elbow_servo.setup_ease(arg_double, duration);
    state = "movex";
  }

  if (command == "shoulder") {
    // long duration = (arg_double * 1000) / 30.0; // 30 degrees per second
    long duration = 5000;
    LOG_INFO("command shoulder: %.1f %ld", arg_double, duration);
    shoulder_servo.setup_ease(arg_double, duration);
    state = "movex";
  }

  if (command == "claw") {
    LOG_INFO("command claw: %f", arg_double);
    claw_servo.setup_ease(arg_double, 1000);
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

  if (command == "diag") {
    LOG_INFO("diag: %f", arg);
    if (arg == 99) {
      shoulder_servo.test_mode = true;
      elbow_servo.test_mode = true;
      wrist_servo.test_mode = true;
      claw_servo.test_mode = true;
      LOG_INFO("test mode ON. ARM WILL NOT MOVE: %s", state.c_str());
    } else if (arg == 88) {
      shoulder_servo.test_mode = false;
      elbow_servo.test_mode = false;
      wrist_servo.test_mode = false;
      claw_servo.test_mode = false;
      LOG_INFO("test mode OFF. ARM WILL MOVE. state = %s", state.c_str());
    } else if (arg == 1) {
      LOG_INFO("Current arm status; state = %s", state.c_str());
      shoulder_servo.status();
      elbow_servo.status();
      wrist_servo.status();
      claw_servo.status();
    }
  }

  if (command == "prog1") {
    arm_prog_servo[0] = 100;
    arm_prog_angle[0] = SH_POS1;
    arm_prog_pc = 0;
    LOG_INFO("prog1 %d %d %d", arm_prog_servo[0], arm_prog_angle[0], arm_prog_pc);
    state = "moveprog";
  }      

  
  for (ArmPositions pos : arm_locs) {
    if (command == pos.name) {
      long duration = move_duration_heuristic(pos.shoulder, pos.elbow, pos.wrist);
      LOG_INFO("arm_command: %s, time: %ld", command.c_str(), duration);
      shoulder_servo.setup_ease(pos.shoulder, duration);
      state = "movex";
      if (!wait_for_servo()) {
        return;
      }
      elbow_servo.setup_ease(pos.elbow, duration);
      state = "movex";
      if (!wait_for_servo()) {
        return;
      }
      wrist_servo.setup_ease(pos.wrist, duration);
      state = "movex";
      if (!wait_for_servo()) {
        return;
      }
    }
  }
}

void BrandeisArm::general_setup_ease(int servo_id, int angle, long duration) {
  if (servo_id == SHOULDER) {
    shoulder_servo.setup_ease(angle, duration);
  } else if (servo_id == ELBOW) {
    elbow_servo.setup_ease(angle, duration);
  } else if (servo_id == WRIST) {
    wrist_servo.setup_ease(angle, duration);
  } else if (servo_id == CLAW) {
    wrist_servo.setup_ease(angle, duration);
  } else {
    LOG_ERROR("ERROR in general_setup_ease");
  }
}

// Method will return true once the arm is no longer moving. 
// If after 30 seconds the arm is still moving, the method will return false.
bool BrandeisArm::wait_for_servo() {
  int wait_timeout_count = 5;
  while (shoulder_servo.moving || elbow_servo.moving || wrist_servo.moving) {
    wait_timeout_count--;
    if (wait_timeout_count == 0) {
      LOG_ERROR("Wait timeout");
      return false;
    }
    delay(1); // Delay 1 millisecond
  }
  return true;
} 

// Method will return true once the arm is no longer moving. 
// If after 30 seconds the arm is still moving, the method will return false.
bool BrandeisArm::wait_for_servo2() {
  int wait_timeout_count = 5;
  while (!arm_motion_stopped())
  {
    wait_timeout_count--;
    if (wait_timeout_count == 0) {
      LOG_ERROR("Wait timeout");
      return false;
    }
    delay(100); // Delay in millisecond
  }
  return true;
}