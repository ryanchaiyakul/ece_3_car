#include <ECE3.h>

/**
 * General Constants
 */

const int BASE_SPEED = 50;  // 0-255

/**
 * PID Constants
 */

const float KP = 0.02;
const float KD = 0.08;

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

const int FUSION_OFFSET = 80;

/**
 * Pins
 */

const int L_NSLP_PIN = 31; // nslp ==> awake & ready for PWM
const int L_DIR_PIN = 29;
const int L_PWM_PIN = 40;
const int R_NSLP_PIN = 11; // nslp ==> awake & ready for PWM
const int R_DIR_PIN = 30;
const int R_PWM_PIN = 39;

/**
 * Values
*/
const int sensorCount = 8;
uint16_t sensorValues[sensorCount]; // right -> left, 0 -> 7
int prevError;

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

    // Set Previous Error
    ECE3_read_IR(sensorValues);
    prevError = getFusionOutput();
    
    //Serial.begin(9600);
}

void printSensorValues() {
    for (int i = 0; i < sensorCount; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(" ");
    }
    Serial.print("\n");
}

int getFusionOutput() {
    return ((- IR_WEIGHT[0] * (IR_FACTOR_0 * (sensorValues[0]-IR_MIN_0))
            - IR_WEIGHT[1] * (IR_FACTOR_1 * (sensorValues[1]-IR_MIN_1))
            - IR_WEIGHT[2] * (IR_FACTOR_2 * (sensorValues[2]-IR_MIN_2))
            - IR_WEIGHT[3] * (IR_FACTOR_3 * (sensorValues[3]-IR_MIN_3))
            + IR_WEIGHT[3] * (IR_FACTOR_4 * (sensorValues[4]-IR_MIN_4))
            + IR_WEIGHT[2] * (IR_FACTOR_5 * (sensorValues[5]-IR_MIN_5))
            + IR_WEIGHT[1] * (IR_FACTOR_6 * (sensorValues[6]-IR_MIN_6))
            + IR_WEIGHT[0] * (IR_FACTOR_7 * (sensorValues[7]-IR_MIN_7)))
            /IR_DIVISOR) - FUSION_OFFSET;
}

int getPIDOutput() {
    int error = getFusionOutput();
    //Serial.println(error);
    int ret = KP * INVERTED * error + KD * INVERTED * (error - prevError);
    prevError = error;
    return ret;
}

void writeWheels() {
    int offset = getPIDOutput();
    //Serial.println(offset);
    analogWrite(L_PWM_PIN, BASE_SPEED - offset);
    analogWrite(R_PWM_PIN, BASE_SPEED + offset);
}

void loop() {
  ECE3_read_IR(sensorValues);
  //Serial.println(getFusionOutput());
  writeWheels();
  //delay(500);
}
