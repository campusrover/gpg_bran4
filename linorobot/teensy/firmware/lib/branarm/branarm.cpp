#include "branarm.h"

ServoCoords arm_locs[] = {
    {SH_PARK_DEG, EL_PARK_DEG, WR_PARK_DEG},
    {SH_FLOOR_DOWN_DEG, EL_FLOOR_DOWN_DEG, WR_FLOOR_DOWN_DEG},
    {SH_STRAIGHTUP, EL_STRAIGHTUP, WR_STRAIGHTUP},
    {SH_VERT_HORIZ_HAND, EL_VERT_HORIZ_HAND, WR_VERT_HORIZ_HAND},
    {SH_ALL_BACKWARD_DEG, EL_ALL_BACKWARD_DEG, WR_ALL_BACKWARD_DEG},
    {SH_FLOOR_UP_DEG, EL_FLOOR_UP_DEG, WR_FLOOR_UP_DEG}};

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

  servos.shoulder.setup(*node_handle, servo_driver, SHOULDER, SHOULDERMAXDEG,
                        SHOULDERMINDEG, SH_DEGOFFSET, SH_DEGSCALE, SH_PARK_DEG);
  servos.elbow.setup(*node_handle, servo_driver, ELBOW, ELBOWMAXDEG,
                        ELBOWMINDEG, EL_DEGOFFSET, EL_DEGSCALE, EL_PARK_DEG);
  servos.wrist.setup(*node_handle, servo_driver, WRIST, WRISTMAXDEG,
                        WRISTMINDEG, WR_DEGOFFSET, WR_DEGSCALE, WR_PARK_DEG);
  servos.claw.setup(*node_handle, servo_driver, CLAW, CLAWMAXDEG, 
                        CLAWMINDEG, CL_DEGOFFSET, CL_DEGSCALE, CLAWPARKDEG);


  current_claw = CLAWPARKDEG;
  current_wrist = WR_PARK_DEG;
  current_elbow = EL_PARK_DEG;
  current_shoulder = SH_PARK_DEG;
}

void BrandeisArm::traceOut(String msg) {
  char buffer[100];
  sprintf(buffer, "%s, %.1f %.1f %.1f %.1f ", msg.c_str(), iterations, current_shoulder, current_elbow, current_wrist);
  node_handle->loginfo(buffer);
}

void BrandeisArm::loop() {
  if (millis() <= iteration_time + iteration_interval) {
    return;
  }
  iteration_time = millis();
  if (state == "idle") {
    return;
  } else if (state == "move") {
    move();
    if (iterations < 1)
      state = "idle";
  } else if (state == "movex") {
    movex();
    if (arm_motion_stopped()) {
      state = "idle";
    }
  }
}

void BrandeisArm::calculate_iteration_deltas() {
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
  if (iterations == 0) {
    shoulderDelta = 0;
    elbowDelta = 0;
    wristDelta = 0;
  } else {
    shoulderDelta = shouldercnt / iterations;
    elbowDelta = elbowcnt / iterations;
    wristDelta = wristcnt / iterations;
  }
}

// void BrandeisArm::traceOut2(String msg, int mod = 1) {
//   char buffer[400];
//   if ((++log_counter % mod) != 0)
//     return;
//   snprintf(buffer, sizeof(buffer),
//            "  s, %.1f, %.1f, %.1f, %lu, %lu"
//            "  e, %.1f, %.1f, %.1f, %lu, %lu"
//            "  w, %.1f, %.1f, %.1f, %lu, %lu",
//            servos.shoulder.current_angle, servos.shoulder.target_angle,
//            servos.shoulder.change_in_value_angle,
//            servos.shoulder.duration_ms, servos.shoulder.elapsed_time_ms,
//            servos.elbow.current_angle, servos.elbow.target_angle,
//            servos.elbow.change_in_value_angle, servos.elbow.duration_ms,
//            servos.elbow.elapsed_time_ms, servos.wrist.current_angle,
//            servos.wrist.target_angle, servos.wrist.change_in_value_angle,
//            servos.wrist.duration_ms, servos.wrist.elapsed_time_ms);
//   node_handle->loginfo(buffer);
// }

int BrandeisArm::move() {

  traceOut("move call");
  current_wrist = current_wrist + wristDelta;
  wrist(current_wrist);
  current_elbow = current_elbow + elbowDelta;
  elbow(current_elbow);
  current_shoulder = current_shoulder + shoulderDelta;
  shoulder(current_shoulder);
  iterations = iterations - 1;
  return iterations;
}

bool BrandeisArm::arm_motion_stopped(void) {
  // elapsed_time_ms >= duration_ms means that motion is complete because
  // more time has passed than what we were going for
  boolean stopped =
      !(servos.shoulder.moving || servos.elbow.moving || servos.wrist.moving);
  char buffer[100];
  sprintf(buffer, "arm_motion_stopped %d %d %d %d", stopped,
          servos.shoulder.moving, servos.elbow.moving, servos.wrist.moving);
  node_handle->loginfo(buffer);
  return stopped;
}

void BrandeisArm::movex() {
  char buffer[100];
  sprintf(buffer, "movex: state: %s", state.c_str());
  node_handle->loginfo(buffer);
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

void BrandeisArm::elbow(float deg) {
  int deglen =
      (deg + EL_DEGOFFSET) * EL_DEGSCALE; // pulselen of commanded degrees
  servo_driver.setPWM(ELBOW, 0, deglen);
}

void BrandeisArm::shoulder(float deg) {
  shoulder_counter++;
  int deglen =
      (deg + SH_DEGOFFSET) * SH_DEGSCALE; // pulselen of commanded degrees
  servo_driver.setPWM(SHOULDER, 0, deglen);
}

void BrandeisArm::wrist(float deg) {
  int deglen =
      (deg + WR_DEGOFFSET) * WR_DEGSCALE; // pulselen of commanded degrees
  servo_driver.setPWM(WRIST, 0, deglen);
}
// true = open; false = close
void BrandeisArm::claw(bool open_close) {
  if (open_close) {
    if (current_claw >= CLAWOPEN) {
      for (int pulse_len = current_claw; pulse_len > CLAWOPEN; pulse_len--) {
        servo_driver.setPWM(CLAW, 0, pulse_len);
        delay(20);
      }

      current_claw = CLAWOPEN;
    }
  } else {
    if (current_claw <= CLAWCLOSED) {
      for (int pulse_len = current_claw; pulse_len < CLAWCLOSED; pulse_len++) {
        servo_driver.setPWM(CLAW, 0, pulse_len);
        delay(20);
      }
    }
    current_claw = CLAWCLOSED;
  }
}

void BrandeisArm::open_claw() { // Claw   MIN is closed   MAX is open

  if (current_claw >= CLAWOPEN) {
    for (int pulselen = current_claw; pulselen > CLAWOPEN; pulselen--) {
      servo_driver.setPWM(CLAW, 0, CLAWOPEN);
      delay(20);
    }
  }
  current_claw = CLAWOPEN;
}

void BrandeisArm::close_claw() { // Claw MIN is closed   MAX is open
  if (current_claw <= CLAWCLOSED) {
    for (int pulselen = current_claw; pulselen < CLAWCLOSED; pulselen++) {
      servo_driver.setPWM(CLAW, 0, CLAWCLOSED);
      delay(20);
    }
  }
  current_claw = CLAWCLOSED;
}

String BrandeisArm::getState() { return state; }

void BrandeisArm::arm_command(String command) {
  if (command == "park") {
    destination_shoulder = SH_PARK_DEG;
    destination_wrist = WR_PARK_DEG;
    destination_elbow = EL_PARK_DEG;
    calculate_iteration_deltas();
    state = "move";
  }
  if (command == "straightup") {
    destination_shoulder = SH_STRAIGHTUP;
    destination_wrist = WR_STRAIGHTUP;
    destination_elbow = EL_STRAIGHTUP;
    calculate_iteration_deltas();
    state = "move";
  }
  if (command == "verthorizhand") {
    destination_shoulder = SH_VERT_HORIZ_HAND;
    destination_wrist = WR_VERT_HORIZ_HAND;
    destination_elbow = EL_VERT_HORIZ_HAND;
    calculate_iteration_deltas();
    state = "move";
  }
  if (command == "allback") {
    destination_shoulder = SH_ALL_BACKWARD_DEG;
    destination_wrist = WR_ALL_BACKWARD_DEG;
    destination_elbow = EL_ALL_BACKWARD_DEG;
    calculate_iteration_deltas();
    state = "move";
  }
  if (command == "allforward") {
    destination_shoulder = SH_ALL_FORWARD_DEG;
    destination_wrist = WR_ALL_FORWARD_DEG;
    destination_elbow = EL_ALL_FORWARD_DEG;
    calculate_iteration_deltas();
    state = "move";
  }
  if (command == "floorup") {
    destination_shoulder = SH_FLOOR_UP_DEG;
    destination_wrist = WR_FLOOR_UP_DEG;
    destination_elbow = EL_FLOOR_UP_DEG;
    calculate_iteration_deltas();
    state = "move";
  }
  if (command == "floordown") {
    destination_shoulder = SH_FLOOR_DOWN_DEG;
    destination_wrist = WR_FLOOR_DOWN_DEG;
    destination_elbow = EL_FLOOR_DOWN_DEG;
    calculate_iteration_deltas();
    state = "move";
  }

  // BrandeisArm::traceOut2("arm_command");
}

void BrandeisArm::arm_command(String command, float arg) {
  BrandeisArm::traceOut("armCommand");

  if (command == "wrist") {
    double arg_double = (double) arg;
    char buffer[100];
    sprintf(buffer, "destination wrist: %f", arg_double);
    node_handle->loginfo(buffer);
    servos.wrist.setup_ease(arg_double, millis(), 2000);
    state = "movex";
  }

  if (command == "elbow") {
    double arg_double = (double) arg;
    char buffer[100];
    sprintf(buffer, "destination elbow: %f", arg_double);
    node_handle->loginfo(buffer);
    servos.elbow.setup_ease(arg_double, millis(), 2000);
    state = "movex";
  }

  if (command == "shoulder") {
    double arg_double = (double) arg;
    char buffer[100];
    sprintf(buffer, "destination shoulder: %f", arg_double); node_handle->loginfo(buffer);
    servos.shoulder.setup_ease(arg_double, millis(), 2000);
    state = "movex";
  }
    if (command == "combo") {
      double arg_double = (double) arg;
      char buffer[100];
      sprintf(buffer, "combined: %f", arg_double); node_handle->loginfo(buffer);
      servos.shoulder.setup_ease(45, millis(), 2000);
      servos.wrist.setup_ease(90, millis(), 2000);
      state = "movex";
    }

    if (command == "test") {
      char buffer[100];
      if (arg == 1) {
        servos.shoulder.test_mode = true;
        servos.elbow.test_mode = true;
        servos.wrist.test_mode = true;
        servos.claw.test_mode = true;
        sprintf(buffer, "test mode on. no motion"); node_handle->loginfo(buffer);

      } else if (arg == 0) {
        servos.shoulder.test_mode = false;
        servos.elbow.test_mode = false;
        servos.wrist.test_mode = false;
        servos.claw.test_mode = false;
        sprintf(buffer, "test mode off. arm WILL MOVE"); node_handle->loginfo(buffer);

      }
    }

}
