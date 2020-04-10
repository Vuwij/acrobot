#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Timer.h"

#include "gen/calc_B.h"
#include "gen/calc_C.h"
#include "gen/calc_D.h"
#include "gen/calc_De.h"
#include "gen/calc_dUde.h"
#include "gen/calc_E.h"
#include "gen/calc_EE.h"
#include "gen/calc_J.h"
#include "gen/calc_P.h"
#include "gen/calc_qd.h"
#include "gen/calc_rend.h"

Timer t;
double ts = 0.02;
double tplot = 0.04;
double steps_per_rotation = 2797;
double leg_length = 0.3480;
int timeout = 0;

// Sensors
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, 0x29);

// Information
bool A_set = false;
bool B_set = false;
volatile double encoderPos = 0;
volatile int stepCount = 0;
volatile double X[4] = {M_PI_2, 0, 0, 0};
volatile double dist_to_floor = 1;
imu::Vector<3> imu_data1;
imu::Vector<3> imu_data2;

void doEncoderA(){
    A_set = digitalRead(0) == HIGH;
    encoderPos += (A_set != B_set) ? +1 : -1;
}
void doEncoderB(){
    B_set = digitalRead(1) == HIGH;
    encoderPos += (A_set == B_set) ? +1 : -1;
}

void calculateState() {

    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    sensors_event_t bno1_orientation, bno2_orientation;

    double qm = 0;
    double yaw = 0;
    imu_data1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu_data2 = bno2.getVector(Adafruit_BNO055::VECTOR_EULER);

    if (stepCount % 2 == 1) {
        yaw = imu_data1(2) / 180 * M_PI;
        qm = -encoderPos / steps_per_rotation * 2 * M_PI;
    } else {
        yaw = imu_data2(2) / 180 * M_PI;
        qm = encoderPos / steps_per_rotation * 2 * M_PI;
        if (yaw > 0) {
            yaw = -yaw + M_PI;
        } else {
            yaw = -yaw - M_PI;
        }
        yaw = yaw + M_PI_2;
    }

    double q1 = yaw;
    double q1_dot = (q1 - X[0])/ts;
    double q2 = qm;
    double q2_dot = (q2 - X[1])/ts;

    dist_to_floor = sin(q1+q2) * leg_length + sin(q1) * leg_length;
    if (dist_to_floor < 0 && timeout == 0) {
        stepCount = stepCount + 1;
        timeout = 100;
    } else if(timeout > 0) {
        timeout --;
    }

    X[0] = q1;
    X[1] = q2;
    X[2] = q1_dot;
    X[3] = q2_dot;
}

void plot() {
    Serial.println();
    Serial.print(imu_data1(2));
    Serial.print(" ");
    Serial.print(imu_data2(2));
    Serial.print(" ");
    Serial.print(X[0]);
    Serial.print(" ");
    Serial.print(X[1]);
    Serial.print(" ");
    Serial.print(X[2]);
    Serial.print(" ");
    Serial.print(X[3]);
    Serial.print(" ");
    Serial.print(dist_to_floor);
}

void setup(void) {
    Serial.begin(115200);

    attachInterrupt(0, doEncoderA, CHANGE);
    attachInterrupt(1, doEncoderB, CHANGE);
    // Check if sensors exists
    if (!bno1.begin()) {
        Serial.print("Ooops, no BNO055 0 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    if (!bno2.begin()) {
        Serial.print("Ooops, no BNO055 1 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    t.every(ts*1000,calculateState);
    t.every(tplot*1000,plot);

}

void loop(void) {
    t.update();
}
