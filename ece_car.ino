#include <ECE3.h>

/**
   TODO:

   Implement mechanism to prevent crossfiring IR sensors causing a donut
   Test whether bounds are necessary for the PID output
   Implement algorithm that detects whether we are off the path
   Implements algorithm that searches for a path
*/
const int TURN_SPEED = 120;          // 0-255
const int BLACK_THRESHOLD = 2200;   // Lowest value from 0-2500 that corresponds to being on top of a black line
const int BLACK_COUNT_THRESHOLD = 6;// 0-8 needed number of sensors with black threshold

const int FIRST_TIME = 1800;
const int FIRST_TIME_END = 3200;
const int SECOND_TIME = 1000;
const int SECOND_TIME_END = 2400;

/**
   PID Constants
*/

const float FAST_KP = 0.067;
const float FAST_KD = 0.32;

/**
   General Constants
*/

const int BASE_SPEED = 40;          // 0-255
const int FAST_SPEED = 80;          // 0-255

const float BASE_KP = 0.034;
const float BASE_KD = 0.24;

const int INVERTED = 1;
const int BOUND = BASE_SPEED / 2;

/**
   Fusion Constants
*/

const int IR_MIN_0 = 549;
const int IR_MIN_1 = 573;
const int IR_MIN_2 = 643;
const int IR_MIN_3 = 643;
const int IR_MIN_4 = 715;
const int IR_MIN_5 = 619;
const int IR_MIN_6 = 549;
const int IR_MIN_7 = 619;

/**
  const int IR_MIN_0 = 618;
  const int IR_MIN_1 = 664;
  const int IR_MIN_2 = 734;
  const int IR_MIN_3 = 757;
  const int IR_MIN_4 = 805;
  const int IR_MIN_5 = 781;
  const int IR_MIN_6 = 711;
  const int IR_MIN_7 = 805;
*/


/**
  const int IR_MIN_0 = 784;
  const int IR_MIN_1 = 683;
  const int IR_MIN_2 = 752;
  const int IR_MIN_3 = 683;
  const int IR_MIN_4 = 660;
  const int IR_MIN_5 = 660;
  const int IR_MIN_6 = 798;
  const int IR_MIN_7 = 735;
*/

const float IR_FACTOR_0 = 0.5709065997;
const float IR_FACTOR_1 = 0.5503577325;
const float IR_FACTOR_2 = 0.5720823799;
const float IR_FACTOR_3 = 0.5503577325;
const float IR_FACTOR_4 = 0.5434782609;
const float IR_FACTOR_5 = 0.5434782609;
const float IR_FACTOR_6 = 0.5875440658;
const float IR_FACTOR_7 = 0.5664438654;

const int IR_WEIGHT[] = {8, 4, 2, 1};
const int IR_DIVISOR = 4;

/**
   Pins and Physical constants
*/

const int L_NSLP_PIN = 31; // nslp ==> awake & ready for PWM
const int L_DIR_PIN = 29;
const int L_PWM_PIN = 40;
const int R_NSLP_PIN = 11; // nslp ==> awake & ready for PWM
const int R_DIR_PIN = 30;
const int R_PWM_PIN = 39;

const int sensorCount = 8;

const int uturnCounts = 330;

/**
   Global variables
*/
uint16_t sensorValues[sensorCount]; // right -> left, 0 -> 7
int prevError;
int baseSpeed;
bool hasUturned;

enum states {
  INITIAL,
  NORMAL,
  UTURN,
  STOP,
  END,
} state;

/**
   Helper function to update sensorValues using ECE3 library
*/
void updateValues() {
  ECE3_read_IR(sensorValues);
}

/**
   Returns the calculated fusion output of the sensorValues array

   DOES NOT UPDATE IT

   TODO:

   Implement prevention against crosspiece (2500s across all sensors)
*/
int getFusionOutput() {
  return ((- IR_WEIGHT[0] * (IR_FACTOR_0 * (sensorValues[0] - IR_MIN_0))
           - IR_WEIGHT[1] * (IR_FACTOR_1 * (sensorValues[1] - IR_MIN_1))
           - IR_WEIGHT[2] * (IR_FACTOR_2 * (sensorValues[2] - IR_MIN_2))
           - IR_WEIGHT[3] * (IR_FACTOR_3 * (sensorValues[3] - IR_MIN_3))
           + IR_WEIGHT[3] * (IR_FACTOR_4 * (sensorValues[4] - IR_MIN_4))
           + IR_WEIGHT[2] * (IR_FACTOR_5 * (sensorValues[5] - IR_MIN_5))
           + IR_WEIGHT[1] * (IR_FACTOR_6 * (sensorValues[6] - IR_MIN_6))
           + IR_WEIGHT[0] * (IR_FACTOR_7 * (sensorValues[7] - IR_MIN_7)))
          / IR_DIVISOR);
}

/**
   Returns true when the IR sensors returns a value consistent with being on the horizontal line
   indicating stop/turnaround
*/
bool onSolidLine() {
  // If sensor reads black underneath them, increment count and then check if the count is greater than threshold
  int count = 0;
  for (int i = 0; i < sensorCount; i++) {
    if (sensorValues[i] > BLACK_THRESHOLD) {
      count++;
    }
  }
  return count >= BLACK_COUNT_THRESHOLD;
}

/**
   Returns offset to add and minus from BASE_SPEED for the left and right pwn signal respectively

   TODO:

   Implement bounds to prevent negative values (May not be necessary and may be harmful by increasing delay)
*/
int getBasePIDOutput(int error) {
  int ret = BASE_KP * INVERTED * error + BASE_KD * INVERTED * (error - prevError);
  prevError = error;
  return ret;
}

int getFastPIDOutput(int error) {
  int ret = FAST_KP * INVERTED * error + FAST_KD * INVERTED * (error - prevError);
  prevError = error;
  return ret;
}

/**
   Updates the PWM pins with a new speed. (- steers to the left and + steers to the right)
*/
void writeWheels(int offset) {

  analogWrite(L_PWM_PIN, baseSpeed - offset);
  analogWrite(R_PWM_PIN, baseSpeed + offset);
}

/*
  This function changes the car speed gradually (in about 30 ms) from initial
  speed to final speed. This non-instantaneous speed change reduces the load
  on the plastic geartrain, and reduces the failure rate of the motors.
*/
void ChangeBaseSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd) {
  int diffLeft = finalLeftSpd - initialLeftSpd;
  int diffRight = finalRightSpd - initialRightSpd;
  int stepIncrement = 20;
  int numStepsLeft = abs(diffLeft) / stepIncrement;
  int numStepsRight = abs(diffRight) / stepIncrement;
  int numSteps = max(numStepsLeft, numStepsRight);
  int pwmLeftVal = initialLeftSpd; // initialize left wheel speed
  int pwmRightVal = initialRightSpd; // initialize right wheel speed
  int deltaLeft = (diffLeft) / numSteps; // left in(de)crement
  int deltaRight = (diffRight) / numSteps; // right in(de)crement

  for (int k = 0; k < numSteps; k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(L_PWM_PIN, pwmLeftVal);
    analogWrite(R_PWM_PIN, pwmRightVal);
    delay(30);
  } // end for int k

  analogWrite(L_PWM_PIN, finalLeftSpd);
  analogWrite(R_PWM_PIN, finalRightSpd);
}

void setup() {
  ECE3_Init();

  // Pin Setup

  // Wheel Pins
  pinMode(L_NSLP_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);

  pinMode(R_NSLP_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);

  digitalWrite(L_DIR_PIN, LOW);
  digitalWrite(L_NSLP_PIN, HIGH);

  digitalWrite(R_DIR_PIN, LOW);
  digitalWrite(R_NSLP_PIN, HIGH);

  // Set Initial Previous Error
  updateValues();
  prevError = getFusionOutput();
  hasUturned = false;
  state = INITIAL;
  baseSpeed = FAST_SPEED;
}

int getAverageTicks() {
  return (getEncoderCount_left() + getEncoderCount_right()) / 2;
}


/**
   Primary execution loop
*/
void loop() {
  updateValues();
  switch (state) {
    case INITIAL:
      ChangeBaseSpeeds(0, BASE_SPEED, 0, BASE_SPEED);
      prevError = getFusionOutput();
      resetEncoderCount_left();
      state = NORMAL;
      break;
    case NORMAL:
      if (onSolidLine()) {
        if (hasUturned) {
          state = STOP;
        } else {
          // Stop car, reset encoders, change direction, and speed up in uturn
          ChangeBaseSpeeds(BASE_SPEED, 0, BASE_SPEED, 0);
          delay(50);
          resetEncoderCount_left();
          resetEncoderCount_right();
          digitalWrite(L_DIR_PIN, HIGH);
          ChangeBaseSpeeds(0, TURN_SPEED, 0, TURN_SPEED);
          state = UTURN;
        }
        break;
      }
      if (hasUturned) {
        if (getEncoderCount_left() > SECOND_TIME && getEncoderCount_left() < SECOND_TIME_END) {
          if (baseSpeed != FAST_SPEED) {
            ChangeBaseSpeeds(BASE_SPEED, FAST_SPEED, BASE_SPEED, FAST_SPEED);
          }
          baseSpeed = FAST_SPEED;
          writeWheels(getFastPIDOutput(getFusionOutput()));
        } else {
          if (baseSpeed != BASE_SPEED) {
            ChangeBaseSpeeds(FAST_SPEED, BASE_SPEED, FAST_SPEED, BASE_SPEED);
          }
          baseSpeed = BASE_SPEED;
          writeWheels(getBasePIDOutput(getFusionOutput()));
        }
      } else {
        if (getEncoderCount_left() > FIRST_TIME && getEncoderCount_left() < FIRST_TIME_END) {
          if (baseSpeed != FAST_SPEED) {
            ChangeBaseSpeeds(BASE_SPEED, FAST_SPEED, BASE_SPEED, FAST_SPEED);
          }
          baseSpeed = FAST_SPEED;
          writeWheels(getFastPIDOutput(getFusionOutput()));
        } else {
          if (baseSpeed != BASE_SPEED) {
            ChangeBaseSpeeds(FAST_SPEED, BASE_SPEED, FAST_SPEED, BASE_SPEED);
          }
          baseSpeed = BASE_SPEED;
          writeWheels(getBasePIDOutput(getFusionOutput()));
        }
      }

      break;
    case UTURN:
      if (getAverageTicks() > uturnCounts) {
        // stop car, set to straight, accelerate, and delay to prevent stopping on the same line
        ChangeBaseSpeeds(TURN_SPEED, 0, TURN_SPEED, 0);
        delay(50);
        digitalWrite(L_DIR_PIN, LOW);
        ChangeBaseSpeeds(0, BASE_SPEED, 0, BASE_SPEED);
        delay(50);
        hasUturned = true;
        resetEncoderCount_left();
        state = NORMAL;
      }
      break;
    case STOP:
      ChangeBaseSpeeds(BASE_SPEED, 0, BASE_SPEED, 0);
      state = END;
      break;
    case END:
      break;
  }
}
