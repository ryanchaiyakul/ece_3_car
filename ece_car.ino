#include <ECE3.h>

/**
 * TODO:
 * 
 * Implement mechanism to prevent crossfiring IR sensors causing a donut
 * Test whether bounds are necessary for the PID output
 * Implement algorithm that detects whether we are off the path
 * Implements algorithm that searches for a path
 */

/**
 * General Constants
 */

const int BASE_SPEED = 50;          // 0-255 straighish speed
const int TWISTY_SPEED = 50;        // 0-255 when offset is greater than twisty threshold 
const int SEARCH_SPEED = 10;        // 0-255 speed when track is lost
const int UTURN_SPEED = 10;         // 0-255 uturn speed

const int BLACK_THRESHOLD = 2000;   // Loest value from 0-2500 that corresponds to being on top of a black line

const int INACTIVE_DELAY = 1000;    // # ms before accelerating from the start
const int ACCEL_DELAY = 2;          // # ms per change in speed

const int TWISTY_OFFSET_THRESHOLD = 5;
const int SAFE_OFFSET_THRESHOLD = 3;
const int OFFSET_COUNT_THRESHOLD = 100; // # of safe cycles before accelerating again

/**
 * PID Constants
 */

const float STRAIGHT_KP = 0.02;
const float STRAIGHT_KD = 0.08;
const float TWISTY_KP = 0.02;
const float TWISTY_KD = 0.08;

const int INVERTED = 1;
const int BOUND = BASE_SPEED / 2;

/**
 * Fusion Constants
*/

const int IR_MIN_0 = 784;
const int IR_MIN_1 = 683;
const int IR_MIN_2 = 752;
const int IR_MIN_3 = 683;
const int IR_MIN_4 = 660;
const int IR_MIN_5 = 660;
const int IR_MIN_6 = 798;
const int IR_MIN_7 = 735;

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

const int FUSION_OFFSET = 0;

/**
 * Pins and Physical constants
 */

const int L_NSLP_PIN = 31; // nslp ==> awake & ready for PWM
const int L_DIR_PIN = 29;
const int L_PWM_PIN = 40;
const int R_NSLP_PIN = 11; // nslp ==> awake & ready for PWM
const int R_DIR_PIN = 30;
const int R_PWM_PIN = 39;

const int sensorCount = 8;

/**
 * Global variables (embedded programming)
*/

uint16_t sensorValues[sensorCount]; // right -> left, 0 -> 7

int fusionOutput;
int offset;
float kp, kd;
int prevError;

int lSpeed;
int rSpeed;

int increment;
int cyclesLeft;

int lowOffsetCount;

enum states {
  INACTIVE,
  STRAIGHT,
  TWISTY,
  LOST,
  ACCEL,
  UTURN,
} state, postAccel;

enum uturn_states {
    SPINUP,
    STABLE,
    SPINDOWN,
} uturn_state;

/**
 * Private global variables
 */

int __curSpeed;         // Used by writeWheels
bool __prevSolidLine;   // Used by onSolidLines to remember previous state

/**
 * Helper function to update sensorValues using ECE3 library
 */
void updateValues() {
  ECE3_read_IR(sensorValues);
}

/**
 * Returns true a singular time if a solid line is entered and will not change until it leaves and re enters a solid line
 */
bool onSolidLine() {
    // If all of the sensors read black underneath them, it indicates the horizontal line
    for (int i = 0; i < sensorCount; i++) {
        if (sensorValues[i] < BLACK_THRESHOLD) {
            __prevSolidLine = false;
            return false;
        }
    }
    // Latch the output
    if (__prevSolidLine) {
      return false;
    }
    __prevSolidLine = true;
    return true;
}

/**
 * Set fusionOutput based of constants and current sensorValues readings
 */
void setFusionOutput() {
    fusionOutput = ((- IR_WEIGHT[0] * (IR_FACTOR_0 * (sensorValues[0]-IR_MIN_0))
                     - IR_WEIGHT[1] * (IR_FACTOR_1 * (sensorValues[1]-IR_MIN_1))
                     - IR_WEIGHT[2] * (IR_FACTOR_2 * (sensorValues[2]-IR_MIN_2))
                     - IR_WEIGHT[3] * (IR_FACTOR_3 * (sensorValues[3]-IR_MIN_3))
                     + IR_WEIGHT[3] * (IR_FACTOR_4 * (sensorValues[4]-IR_MIN_4))
                     + IR_WEIGHT[2] * (IR_FACTOR_5 * (sensorValues[5]-IR_MIN_5))
                     + IR_WEIGHT[1] * (IR_FACTOR_6 * (sensorValues[6]-IR_MIN_6))
                     + IR_WEIGHT[0] * (IR_FACTOR_7 * (sensorValues[7]-IR_MIN_7)))
                     / IR_DIVISOR) - FUSION_OFFSET;
}

/**
 * Sets offset based off of current values of global Kp, Kd, and fusionOutput
 */
void setPIDOffset() {
    offset = kp * INVERTED * fusionOutput + kd * INVERTED * (fusionOutput - prevError);
    prevError = fusionOutput;
}

/**
 * Updates the PWM pins with lSpeed and rSpeed
 */
void writeWheels() {
    analogWrite(L_PWM_PIN, lSpeed);
    analogWrite(R_PWM_PIN, rSpeed);
}

/**
 * Updates lSpeed and rSpeed based of offset and __curSpeed
 */
void setSpeeds() {
  lSpeed = __curSpeed - offset;
  rSpeed = __curSpeed + offset;
}

void setup() {
    ECE3_Init();

    // Pin Setup

    // Wheel Pins
    pinMode(L_NSLP_PIN,OUTPUT);
    pinMode(L_DIR_PIN,OUTPUT);
    pinMode(L_PWM_PIN,OUTPUT);

    pinMode(R_NSLP_PIN,OUTPUT);
    pinMode(R_DIR_PIN,OUTPUT);
    pinMode(R_PWM_PIN,OUTPUT);

    digitalWrite(L_DIR_PIN,LOW);
    digitalWrite(L_NSLP_PIN,HIGH);

    digitalWrite(R_DIR_PIN,LOW);
    digitalWrite(R_NSLP_PIN,HIGH);

    // Set default values for global variables
    state = INACTIVE;
    uturn_state = SPINUP;
    offset = 0;
    lowOffsetCount = 0;

    __curSpeed = 0;
    __prevSolidLine = false;
    
    //Serial.begin(9600);
}

/**
 * Legacy code to print out sensorValues while testing
 * 
 * REMOVE DURING FINAL VERSION
 */
void printSensorValues() {
    for (int i = 0; i < sensorCount; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(" ");
    }
    Serial.print("\n");
}


/**
 * Primary execution loop
 */
void loop() {
    // Update sensorValues
    updateValues();
    setFusionOutput();
    
    switch (state) {
        INACTIVE:
            // Accelerate once placed on start line
            if (onSolidLine()) {
                offset = 0;
                cyclesLeft = BASE_SPEED;
                increment = 1;
                postAccel = STRAIGHT;
                state = ACCEL;
            }
            // Wait before starting (so the person can let go of the car)
            delay(INACTIVE_DELAY);
            break;
        STRAIGHT:
            // Decelerate before uturning
            if (onSolidLine()) {
                offset = 0;
                cyclesLeft = BASE_SPEED;
                increment = -1;
                postAccel = UTURN;
                state = ACCEL;
                break;
            }

            setPIDOffset();

            // Switch to twisty speed if the offset is too large
            if (offset > TWISTY_OFFSET_THRESHOLD) {
                kp = TWISTY_KP;
                kd = TWISTY_KD;

                offset = 0;
                cyclesLeft = BASE_SPEED - TWISTY_SPEED;
                increment = -1;
                postAccel = TWISTY;
                state = ACCEL;
            }
            break;
        TWISTY:
            // Decelerate before uturning
            if (onSolidLine()) {
                offset = 0;
                cyclesLeft = TWISTY_SPEED;
                increment = -1;
                postAccel = UTURN;
                state = ACCEL;
                break;
            }

            setPIDOffset();

            // Reset counter if offset is still too high (still curving)
            if (offset > SAFE_OFFSET_THRESHOLD) {
                lowOffsetCount = 0;
                break;
            }
            lowOffsetCount++;

            // Once the path has been straight for a while, accelerate again
            if (lowOffsetCount == OFFSET_COUNT_THRESHOLD) {
                lowOffsetCount = 0;
                kp = STRAIGHT_KP;
                kd = STRAIGHT_KD;

                offset = 0;
                cyclesLeft = BASE_SPEED - TWISTY_SPEED;
                increment = 1;
                postAccel = STRAIGHT;
                state = ACCEL;
            }
            break;
        LOST:
                break;
        ACCEL:
            __curSpeed += increment;
            if (cyclesLeft == 0) {
            // Update prevError before entering PID controlled state if necessary
            prevError = fusionOutput;
            state = postAccel;
            break;
            }
            cyclesLeft--;
            delay(ACCEL_DELAY);
        break;
        UTURN:
            switch (uturn_state) {
                SPINUP:
                    if (offset == UTURN_SPEED) {
                        uturn_state = STABLE;
                    } else {
                        offset++;
                    }
                    break;
                STABLE:
                    // Include encoder based break as well
                    if (onSolidLine()) {
                        uturn_state = SPINDOWN;
                    }
                    break;
                SPINDOWN:
                    if (offset == 0) {
                        uturn_state = SPINUP;

                        // offset = 0; already implied
                        cyclesLeft = BASE_SPEED;
                        increment = 1;
                        postAccel = STRAIGHT;
                        state = ACCEL;
                    } else {
                        offset--;
                    }
                    break;
            }
            break;
    }
    setSpeeds();
    writeWheels();
}
