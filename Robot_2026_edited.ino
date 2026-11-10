#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <I2Cdev.h>
#include <Servo.h> 
//libraries for MPU servos and I2C communication

// ===== Button & Limit Switch =====
#define BTN_PIN 11
const int limitSwitch = 31; 
//did we even use this?? start button connected to pin 11, limit switch to 31

// ===== NEW: 5 IR Sensor Pins =====
const int IR_LL_PIN = 35; // Far Left
const int IR_L_PIN  = 37; // Mid Left
const int IR_C_PIN  = 39; // Center
const int IR_R_PIN  = 41; // Mid Right
const int IR_RR_PIN = 43; // Far Right
//IR sensor pins

// ===== Servo Motors =====
Servo rightArmServo;
Servo leftArmServo; 
const int RIGHT_SERVO_PIN = 44; 
const int LEFT_SERVO_PIN = 42;  
const int SERVO_UP_POS = 0;     
const int SERVO_DOWN_POS = 120;  

//defining servo pins and up down positions

// ===== MPU6050 gyroscope =====
MPU6050 mpu; 
float gyroZ_offset = 0; //in case gyroscope has a zero offset
float totalGyroZ = 0; //accumulated gyro reading
bool trackingRotation = false; //whether it's tracking
unsigned long lastTime = 0; //time difference for calculating rotation

// ===== CNC Shield motor control pins =====
const byte enablePin = 8;
const int StepX = 2, DirX = 5; 
const int StepY = 3, DirY = 6; 
const int StepZ = 4, DirZ = 7; 
const int StepA = 12, DirA = 13; 

//activates the stepper motors, steps tells them how far, direction tells them which way to turn

// ===== Motion constants =====
const float STEPS_PER_MM = 8.67;  //how far the motors turn to go one mm
const float STEPS_PER_DEGREE = 10; //same with above
const float TURN_CORRECTION_K = 0.98930;  
const float FWD_TRIM = 0.94;     //to allow one side to run slightly faster 

const int STEER_CORRECTION_FACTOR = 6; //amplifies steering when robot is off course

enum ChassisDirection { FORWARD, BACKWARD, LEFT, RIGHT }; 
int current_half_period = 500;  //current speed its running at

// ===== IMU data streaming =====
unsigned long lastImuUpdate = 0;
const unsigned long IMU_UPDATE_INTERVAL = 200; 

long totalStepsX = 0;
long totalStepsY = 0;
int currentMoveDirection = FORWARD;

// ====== Inertial detection logic ======  
const float MOTION_THRESHOLD = 1.0;    //how the robot considers itself motionless
unsigned long stationary_time   = 0;
bool was_stationary             = false;
long  biasAccumulatorRaw = 0;  
int   biasCountRaw       = 0;

//These accumulate gyro measurements while the robot is stationary so that you can dynamically update the gyro bias.

void calibrateGyroZ(int samples = 200) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += mpu.getRotationZ();
    delay(5);
  }
  gyroZ_offset = sum / (float)samples;
  Serial.print("gyroZ_offset = ");
  Serial.println(gyroZ_offset);
}
//calculating the zero offset

void setup() {
  Serial.begin((115200));
  Wire.begin();
  Wire.setClock(400000);
  while (!Serial);  
  while (Serial.available()) Serial.read();  

  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected");
    calibrateGyroZ();
  } else {
    Serial.println("MPU6050 DEAD");
  }

  pinMode(enablePin, OUTPUT); digitalWrite(enablePin, LOW);
  pinMode(StepX, OUTPUT); pinMode(DirX, OUTPUT);
  pinMode(StepY, OUTPUT); pinMode(DirY, OUTPUT);
  pinMode(StepZ, OUTPUT); pinMode(DirZ, OUTPUT);
  pinMode(StepA, OUTPUT); pinMode(DirA, OUTPUT);
  
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(limitSwitch, INPUT_PULLUP);

  pinMode(IR_LL_PIN, INPUT);
  pinMode(IR_L_PIN, INPUT);
  pinMode(IR_C_PIN, INPUT);
  pinMode(IR_R_PIN, INPUT);
  pinMode(IR_RR_PIN, INPUT);

  rightArmServo.attach(RIGHT_SERVO_PIN);
  rightArmServo.write(SERVO_UP_POS); 
  
  leftArmServo.attach(LEFT_SERVO_PIN); 
  leftArmServo.write(SERVO_DOWN_POS);  

  trackRotationStart();
}

bool checkStop() {
  if (Serial.available() > 0) {
    char c = Serial.peek();
    if (c == '!') {
      Serial.read(); 
      digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
      digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);
      Serial.println("HALTED");
      return true;
    }
  }
  return false;
}

void loop() {
  static String input = "";
  trackRotationUpdate();
  
  if (digitalRead(BTN_PIN) == LOW) {
    delay(50); 
    if (digitalRead(BTN_PIN) == LOW) {
      Serial.println("Button Pressed! Starting Course...");
      
      processChassisCommand("CUSTOM");
      
      while(digitalRead(BTN_PIN) == LOW); 
    }
  }

  while (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == '!') {
      digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
      digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);
      Serial.println("HALTED");
      input = "";
    }
    else if (ch == '\n' || ch == '\r') {
      input.trim();
      if (input.length() > 0) {
        processChassisCommand(input);
        input = "";
      }
    } else {
      input += ch;
    }
  }
}

void trackRotationStart() {
  totalGyroZ = 0;
  lastTime = millis();
  trackingRotation = true;
}

void trackRotationUpdate() {
  if (!trackingRotation) return;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  int16_t gz_raw = mpu.getRotationZ();
  float rate = (gz_raw - gyroZ_offset) / 131.0;

  if (abs(rate) < MOTION_THRESHOLD) {
    if (!was_stationary) {
      was_stationary  = true;
      stationary_time = now;
      biasAccumulatorRaw = 0;
      biasCountRaw       = 0;
    }
    biasAccumulatorRaw += gz_raw;
    biasCountRaw++;

    if (now - stationary_time > 3000 && biasCountRaw > 0) {
      gyroZ_offset = biasAccumulatorRaw / biasCountRaw;
      stationary_time = now;
      biasAccumulatorRaw = 0;
      biasCountRaw       = 0;
    }
    return;  
  }
  was_stationary = false;
  totalGyroZ += rate * dt;
}

void processChassisCommand(String cmd) {
  if (cmd.startsWith("V")) {
    current_half_period = cmd.substring(1).toInt();
    Serial.println("Speed set.");
  }
  else if (cmd.startsWith("F") && !cmd.startsWith("TF")) {
    moveChassisDir(FORWARD, cmd.substring(1).toInt(), current_half_period);
  }
  else if (cmd.startsWith("B") && !cmd.startsWith("TB")) {
    moveChassisDir(BACKWARD, cmd.substring(1).toInt(), current_half_period);
  }
  else if (cmd.startsWith("TF")) {
    moveDifferentialDistance(FORWARD, cmd.substring(2).toInt(), current_half_period);
  }
  else if (cmd.startsWith("TB")) {
    moveDifferentialDistance(BACKWARD, cmd.substring(2).toInt(), current_half_period);
  }
  else if (cmd.startsWith("TS")) {
    int commaIndex = cmd.indexOf(',');
    if (commaIndex > 0) {
      float min_mm = cmd.substring(2, commaIndex).toFloat();
      float max_mm = cmd.substring(commaIndex + 1).toFloat();
      followLineUntilStop(min_mm, max_mm, current_half_period);
    } else {
      followLineUntilStop(20.0, cmd.substring(2).toFloat(), current_half_period);
    }
  }
  else if (cmd.startsWith("TI")) {
    String params = cmd.substring(2);
    int commaIdx = params.indexOf(',');
    float blind_mm = 50.0, max_mm;
    if (commaIdx >= 0) {
      blind_mm = params.substring(0, commaIdx).toFloat();
      max_mm = params.substring(commaIdx + 1).toFloat();
    } else {
      max_mm = params.toFloat();
    }
    followLineUntilIntersection(blind_mm, max_mm, current_half_period);
  }
  else if (cmd == "UR") {
    performUTurn(RIGHT, current_half_period);
  }
  else if (cmd == "UL") {
    performUTurn(LEFT, current_half_period);
  }
  else if (cmd == "SNAKE") {
    Serial.println("Initiating Standalone Snake Pattern...");
    runStandardCourse(current_half_period);
  }
  else if (cmd == "CUSTOM") {
    Serial.println("Initiating Standalone Custom Pattern...");
    runCustomCourse(current_half_period);
  }
  else if (cmd == "TRT") {
    sequentialRightTurn();
    Serial.println("DONE");
  }
  else if (cmd == "SWL") {
    sweepTurnUntilLine(LEFT);
    Serial.println("DONE");
  }
  else if (cmd == "SWR") {
    sweepTurnUntilLine(RIGHT);
    Serial.println("DONE");
  }
  else if (cmd == "U180") {
    turn180AndCatchLine();
    Serial.println("DONE");
  }
  else if (cmd == "RU") {
    rightArmServo.write(SERVO_UP_POS);
    delay(400); Serial.println("DONE");
  }
  else if (cmd == "RD") {
    rightArmServo.write(SERVO_DOWN_POS);
    delay(400); Serial.println("DONE");
  }
  else if (cmd == "LU") {
    leftArmServo.write(SERVO_UP_POS);
    delay(400); Serial.println("DONE");
  }
  else if (cmd == "LD") {
    leftArmServo.write(SERVO_DOWN_POS);
    delay(400); Serial.println("DONE");
  }
  else if (cmd == "S") {
    Serial.println("DONE");
  }
  else {
    Serial.println("?");
  }
}

float normalizeAngle(float angle) {
  angle = fmod(angle, 360.0);
  if (angle > 180.0)  angle -= 360.0;
  if (angle <= -180.0) angle += 360.0;
  return angle;
}

void trackRotationStop() { trackingRotation = false; }
void resetYaw() { totalGyroZ = 0; }
float getYaw() { return -normalizeAngle(totalGyroZ); }

// =====================================================
// 5-SENSOR TRACKING LOGIC
// =====================================================

void followLineUntilStop(float blind_mm, float max_mm, int base_half_period) {
  long targetSteps = round(max_mm * STEPS_PER_MM);
  long blindSteps = round(blind_mm * STEPS_PER_MM); 
  long stepsL = 0, stepsR = 0;
  
  int lineConfirmCount = 0;
  int required_confirm_steps = round(10.0 * STEPS_PER_MM); 

  setChassisMovementDirection(FORWARD);

  unsigned long lastStepMicrosL = micros();
  unsigned long lastStepMicrosR = micros();
  bool stepStateL = LOW, stepStateR = LOW;

  while (stepsL < targetSteps && stepsR < targetSteps) {
    if (checkStop()) return; 

    int irL  = digitalRead(IR_L_PIN);
    int irLL = digitalRead(IR_LL_PIN);
    int irC  = digitalRead(IR_C_PIN);
    int irR  = digitalRead(IR_R_PIN);
    int irRR = digitalRead(IR_RR_PIN);

    if (stepsL > blindSteps || stepsR > blindSteps) {
        //Serial.println("I CAN SEE AGAIN");
        int lineCount = 0;
        if (irL == LOW) lineCount++;
        if (irC == LOW) lineCount++;
        if (irR == LOW) lineCount++;
        if (irRR == LOW) lineCount++;
        if (irLL == LOW) lineCount++;

        if (lineCount >= 3) {
          //Serial.println(lineCount);
          lineConfirmCount++; 
          if (lineConfirmCount > required_confirm_steps) {
            //Serial.print("REQUIRED CONFIRM STEPS:");
            //Serial.println(lineConfirmCount);
            digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
            digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);

            Serial.println("MSG: True Intersection Confirmed!");
            Serial.println("DONE:HIT");
            return; 
          }
        } else {
          lineConfirmCount = 0; 
        }
    }

    int delayL = base_half_period;
    int delayR = base_half_period;

    if (irL == HIGH && irR == LOW) delayL = base_half_period * STEER_CORRECTION_FACTOR; 
    else if (irR == HIGH && irL == LOW) delayR = base_half_period * STEER_CORRECTION_FACTOR; 

    unsigned long now = micros();
    if (now - lastStepMicrosL >= delayL) {
      stepStateL = !stepStateL;
      digitalWrite(StepY, stepStateL); digitalWrite(StepA, stepStateL);
      lastStepMicrosL = now;
      if (stepStateL == LOW) stepsL++; 
    }
    if (now - lastStepMicrosR >= delayR) {
      stepStateR = !stepStateR;
      digitalWrite(StepX, stepStateR); digitalWrite(StepZ, stepStateR);
      lastStepMicrosR = now;
      if (stepStateR == LOW) stepsR++; 
    }
  }
  Serial.println("DONE:TIMEOUT");
}

int followLineUntilIntersection(float blind_mm, float max_mm, int base_half_period) {
  long targetSteps = round(max_mm * STEPS_PER_MM);
  long blindSteps = round(blind_mm * STEPS_PER_MM);
  long stepsL = 0, stepsR = 0;

  setChassisMovementDirection(FORWARD);

  unsigned long lastStepMicrosL = micros();
  unsigned long lastStepMicrosR = micros();
  bool stepStateL = LOW, stepStateR = LOW;
  int lineConfirmCount = 0;

  while (stepsL < targetSteps && stepsR < targetSteps) {
    if (checkStop()) return 2;

    int irL  = digitalRead(IR_L_PIN);
    int irC  = digitalRead(IR_C_PIN);
    int irR  = digitalRead(IR_R_PIN);

    if (stepsL > blindSteps || stepsR > blindSteps) {
        int lineCount = 0;
        if (irL == HIGH) lineCount++;
        if (irC == HIGH) lineCount++;
        if (irR == HIGH) lineCount++;

        if (lineCount >= 2) {
          lineConfirmCount++;
          if (lineConfirmCount > 10) {
            digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
            digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);
            Serial.println("DONE:HIT");
            return 0;
          }
        } else {
          lineConfirmCount = 0; 
        }
    }

    int delayL = base_half_period;
    int delayR = base_half_period;

    if (irL == HIGH && irR == LOW) delayL = base_half_period * STEER_CORRECTION_FACTOR; 
    else if (irR == HIGH && irL == LOW) delayR = base_half_period * STEER_CORRECTION_FACTOR; 

    unsigned long now = micros();
    if (now - lastStepMicrosL >= delayL) {
      stepStateL = !stepStateL;
      digitalWrite(StepY, stepStateL); digitalWrite(StepA, stepStateL);
      lastStepMicrosL = now;
      if (stepStateL == LOW) stepsL++;
    }
    if (now - lastStepMicrosR >= delayR) {
      stepStateR = !stepStateR;
      digitalWrite(StepX, stepStateR); digitalWrite(StepZ, stepStateR);
      lastStepMicrosR = now;
      if (stepStateR == LOW) stepsR++;
    }
  }
  Serial.println("DONE:TIMEOUT");
  return 1;
}

void moveDifferentialDistance(int dir, float mm, int base_half_period) {
  long targetSteps = round(mm * STEPS_PER_MM);
  long stepsL = 0, stepsR = 0;
  setChassisMovementDirection(dir);

  unsigned long lastStepMicrosL = micros();
  unsigned long lastStepMicrosR = micros();
  bool stepStateL = LOW, stepStateR = LOW;

  while (stepsL < targetSteps && stepsR < targetSteps) {
    if (checkStop()) return; 

    int irL  = digitalRead(IR_L_PIN);
    int irR  = digitalRead(IR_R_PIN);

    int delayL = round(base_half_period * FWD_TRIM); 
    int delayR = base_half_period;

    if (irL == HIGH && irR == LOW) delayL = base_half_period * STEER_CORRECTION_FACTOR; 
    else if (irR == HIGH && irL == LOW) delayR = base_half_period * STEER_CORRECTION_FACTOR; 
    
    unsigned long now = micros();
    if (now - lastStepMicrosL >= delayL) {
      stepStateL = !stepStateL;
      digitalWrite(StepY, stepStateL); digitalWrite(StepA, stepStateL);
      lastStepMicrosL = now;
      if (stepStateL == LOW) stepsL++; 
    }
    if (now - lastStepMicrosR >= delayR) {
      stepStateR = !stepStateR;
      digitalWrite(StepX, stepStateR); digitalWrite(StepZ, stepStateR);
      lastStepMicrosR = now;
      if (stepStateR == LOW) stepsR++; 
    }
  }
  Serial.println("DONE");
}

bool moveChassisDir(int dir, float mm, int half_period_target) {
  int steps = round(mm * STEPS_PER_MM);
  int accel_steps = max((int)(steps * 0.05), 10);
  int decel_start = steps - accel_steps;
  int start_period = half_period_target * 2;
  int current_period;

  setChassisMovementDirection(dir);

  for (int i = 0; i < steps; i++) {
    if (checkStop()) return true; 

    if (i < accel_steps) current_period = start_period - (start_period - half_period_target) * i / accel_steps;
    else if (i >= decel_start) current_period = half_period_target + (start_period - half_period_target) * (i - decel_start) / accel_steps;
    else current_period = half_period_target;

    digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
    delayMicroseconds(current_period);
    digitalWrite(StepX, LOW);  digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW);  digitalWrite(StepA, LOW);
    delayMicroseconds(current_period);

    if (digitalRead(limitSwitch) == LOW) {
      Serial.println("BUMP detected! Stopping and retreating.");
      
      if (dir == FORWARD) setChassisMovementDirection(BACKWARD);
      else setChassisMovementDirection(FORWARD);
      
      int retreatSteps = round(100 * STEPS_PER_MM);
      for (int k = 0; k < retreatSteps; k++) {
        digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);
        digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
        delayMicroseconds(half_period_target);
        digitalWrite(StepX, LOW);  digitalWrite(StepY, LOW);
        digitalWrite(StepZ, LOW);  digitalWrite(StepA, LOW);
        delayMicroseconds(half_period_target);
      }
      return true; 
    }
  }
  Serial.println("DONE");
  return false; 
}

void setChassisMovementDirection(int dir) {
  currentMoveDirection = dir;
  switch (dir) {
    case FORWARD:
      digitalWrite(DirX, LOW); digitalWrite(DirY, HIGH);
      digitalWrite(DirZ, LOW); digitalWrite(DirA, HIGH); break;
    case BACKWARD:
      digitalWrite(DirX, HIGH); digitalWrite(DirY, LOW);
      digitalWrite(DirZ, HIGH); digitalWrite(DirA, LOW); break;
    case LEFT:
      digitalWrite(DirX, HIGH); digitalWrite(DirY, HIGH);
      digitalWrite(DirZ, LOW); digitalWrite(DirA, LOW); break;
    case RIGHT:
      digitalWrite(DirX, LOW); digitalWrite(DirY, LOW);
      digitalWrite(DirZ, HIGH); digitalWrite(DirA, HIGH); break;
  }
}

// ===== SEQUENTIAL TURNING LOGIC ======

void sequentialRightTurn() {
  digitalWrite(DirX, LOW); digitalWrite(DirY, LOW);
  digitalWrite(DirZ, LOW); digitalWrite(DirA, LOW);

  int turnState = 0;
  long stepsTaken = 0;
  long blindSteps = 45.0 * STEPS_PER_DEGREE; 

  while (turnState < 3) {
    if (checkStop()) return; 

    digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);  // CHANGEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
    digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
    delayMicroseconds(400); 
    digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);
    delayMicroseconds(400);

    stepsTaken++;

    if (stepsTaken > blindSteps) {
      int irC  = digitalRead(IR_C_PIN);
      int irR  = digitalRead(IR_R_PIN);
      int irRR = digitalRead(IR_RR_PIN);

      if (turnState == 0 && irRR == HIGH) {
        turnState = 1; 
      }
      else if (turnState == 1 && irR == HIGH) {
        turnState = 2; 
      }
      else if (turnState == 2 && irC == HIGH) {
        turnState = 3; 
      }
    }
  }
}

// Fixed for 5-Sensor sweep
void sweepTurnUntilLine(int turnDir) {
  trackRotationStart();
  if (turnDir == RIGHT) { 
    digitalWrite(DirX, LOW); digitalWrite(DirY, LOW);
    digitalWrite(DirZ, LOW); digitalWrite(DirA, LOW);
  } else {
    digitalWrite(DirX, HIGH); digitalWrite(DirY, HIGH);
    digitalWrite(DirZ, HIGH); digitalWrite(DirA, HIGH);
  }

  while (abs(totalGyroZ) < 45.0) {
    if (checkStop()) { trackRotationStop(); return; } 
    digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
    delayMicroseconds(150);
    digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);
    delayMicroseconds(150);
    trackRotationUpdate();
  }

  int turnState = 0;
  while (true) { 
    if (checkStop()) { trackRotationStop(); return; } 
    
    int irLL = digitalRead(IR_LL_PIN);
    int irL  = digitalRead(IR_L_PIN);
    int irC  = digitalRead(IR_C_PIN);
    int irR  = digitalRead(IR_R_PIN);
    int irRR = digitalRead(IR_RR_PIN);

    // Sequential Catch Logic 
    if (turnDir == RIGHT) {
      if (turnState == 0 && (irRR == HIGH || irR == HIGH)) turnState = 1;
      if (turnState == 1 && irC == HIGH) turnState = 2;
    } else {
      if (turnState == 0 && (irLL == HIGH || irL == HIGH)) turnState = 1;
      if (turnState == 1 && irC == HIGH) turnState = 2;
    }

    if (turnState == 2) break; 

    digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
    delayMicroseconds(150);
    digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);
    delayMicroseconds(150);
    trackRotationUpdate();
  }
  trackRotationStop();
}

// Fixed for 5-Sensor sweep
void turn180AndCatchLine() {
  trackRotationStart();
  digitalWrite(DirX, HIGH); digitalWrite(DirY, HIGH); 
  digitalWrite(DirZ, HIGH); digitalWrite(DirA, HIGH);

  while (abs(totalGyroZ) < 160.0) {
    if (checkStop()) { trackRotationStop(); return; } 
    digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
    delayMicroseconds(150);
    digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);
    delayMicroseconds(150);
    trackRotationUpdate();
  }

  int turnState = 0;
  while (true) { 
    if (checkStop()) { trackRotationStop(); return; } 
    
    int irC  = digitalRead(IR_C_PIN);
    int irR  = digitalRead(IR_R_PIN);
    int irRR = digitalRead(IR_RR_PIN);

    // Turn 180 acts as a RIGHT turn
    if (turnState == 0 && (irRR == HIGH || irR == HIGH)) turnState = 1;
    if (turnState == 1 && irC == HIGH) turnState = 2;

    if (turnState == 2) break; 

    digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
    delayMicroseconds(150);
    digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);
    delayMicroseconds(150);
    trackRotationUpdate();
  }
  trackRotationStop();
}

// Fixed for 5-Sensor sweep
bool turnUntilLineOrMax(int turnDir, float blind_degrees, float max_degrees) {
  const int PIVOT_STEP_DELAY_US = 220; 

  trackRotationStart();
  if (turnDir == RIGHT) { 
    digitalWrite(DirX, LOW); digitalWrite(DirY, LOW);
    digitalWrite(DirZ, LOW); digitalWrite(DirA, LOW);
  } else {
    digitalWrite(DirX, HIGH); digitalWrite(DirY, HIGH);
    digitalWrite(DirZ, HIGH); digitalWrite(DirA, HIGH);
  }

  bool found = false;
  int turnState = 0;
  
  while (abs(totalGyroZ) < max_degrees) {
    digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
    delayMicroseconds(PIVOT_STEP_DELAY_US);
    digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);
    delayMicroseconds(PIVOT_STEP_DELAY_US);
    trackRotationUpdate();

    if (abs(totalGyroZ) >= blind_degrees) {
      
      int irLL = digitalRead(IR_LL_PIN);
      int irL  = digitalRead(IR_L_PIN);
      int irC  = digitalRead(IR_C_PIN);
      int irR  = digitalRead(IR_R_PIN);
      int irRR = digitalRead(IR_RR_PIN);

      // Sequential Catch Logic
      if (turnDir == RIGHT) {
        if (turnState == 0 && (irRR == HIGH || irR == HIGH)) turnState = 1;
        if (turnState == 1 && irC == HIGH) turnState = 2;
      } else {
        if (turnState == 0 && (irLL == HIGH || irL == HIGH)) turnState = 1;
        if (turnState == 1 && irC == HIGH) turnState = 2;
      }

      if (turnState == 2) {
        found = true;
        break;
      }
    }
  }
  trackRotationStop();
  return found;
}

int performUTurn(int turnDir, int speed) {
  float offset1_mm = (turnDir == RIGHT) ? 100.0 : 130.0; //blind move forward at 1st intersection
  float offset2_mm = (turnDir == LEFT) ? 200.0 : 335.3; //blind move forward at 2nd intersection

  float blind_degrees, max_degrees;
  if (turnDir == RIGHT) {
    blind_degrees = 90.0;
    max_degrees = 170.0;  
  } else {
    blind_degrees = 80.0;
    max_degrees = 170.0;
  }

  float step3_blind_mm, step3_max_mm; //finding second intersection line
  if (turnDir == RIGHT) {
    step3_blind_mm = 80.0;
    step3_max_mm = 350.0;
  } else {
    step3_blind_mm = 150.0;
    step3_max_mm = 350.0;
  }

  if (moveChassisDir(FORWARD, offset1_mm, speed)) return 1;
  if (!turnUntilLineOrMax(turnDir, blind_degrees, max_degrees)) return 2;
  if (followLineUntilIntersection(step3_blind_mm, step3_max_mm, speed) == 2) return 1; 
  if (moveChassisDir(FORWARD, offset2_mm, speed)) return 1;
  if (!turnUntilLineOrMax(turnDir, blind_degrees, max_degrees)) return 3;

  return 0;
}

void runStandardCourse(int speed) {
  int turnSequence[] = {RIGHT, LEFT, RIGHT, LEFT, RIGHT, LEFT};
  const int NUM_LINES = 6;
  const int NODES_PER_LINE = 6;

  int rowSpeed = speed - 100;
  if (rowSpeed < 100) rowSpeed = 100;
  
  int gapSpeed = speed + 300; 

  for (int i = 0; i < NUM_LINES; i++) {
    Serial.print("--- Driving Line "); Serial.print(i + 1); Serial.println(" ---");

    
    for (int node = 1; node <= NODES_PER_LINE; node++) {
        Serial.println(node);
        float blind = 150.0;
        processChassisCommand("TS" + String(blind) + ",500"); 
        delay(1000); 
    }
    
    if (i < NUM_LINES - 1) { 
        Serial.println("Passed 6 lines. Tracking 230mm gap...");
        current_half_period = gapSpeed; 
        processChassisCommand("TF230");
        
        Serial.println("Looking for turn intersection...");
        processChassisCommand("TI50,400");
        
        //going forward a little so the robot turns on center at intersection
        processChassisCommand("F100");

        current_half_period = speed; 
        Serial.print("Turning "); Serial.println(turnSequence[i] == RIGHT ? "RIGHT" : "LEFT");
        if (turnSequence[i] == RIGHT) processChassisCommand("UR");
        else processChassisCommand("UL");
        
    } else {
        Serial.println("Driving to final corner...");
        current_half_period = gapSpeed;
        processChassisCommand("TI50,330");
        current_half_period = speed;
    }
  }
  
  Serial.println("COURSE COMPLETE");
  Serial.println("DONE");
  processChassisCommand("TF350");
}

void runCustomCourse(int speed) {
  Serial.println("--- Starting Custom Course ---");
  
  // 1. Forward 28 cm (280 mm)
  Serial.println("1. Moving Forward 28 cm...");
  processChassisCommand("TF280");
  delay(500); 
  
  // 2. Turn Right Sequential
  Serial.println("2. Turning Right (Checking Outer -> Mid -> Center Sensors)...");
  processChassisCommand("TRT"); 
  delay(500);
  
  // 3. Forward 38 cm (380 mm)
  Serial.println("3. Moving Forward 38 cm...");
  processChassisCommand("TF380");
  delay(500);
  
  // 4. Turn Right Sequential
  Serial.println("4. Turning Right (Checking Outer -> Mid -> Center Sensors)...");
  processChassisCommand("TRT"); 
  
  Serial.println("--- Custom Course Complete ---");
  Serial.println("DONE");
}