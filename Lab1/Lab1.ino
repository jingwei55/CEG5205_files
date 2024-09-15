// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// // class default I2C address is 0x68
// // specific I2C addresses may be passed as a parameter here
// // AD0 low = 0x68 (default for InvenSense evaluation board)
// // AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz;
// float ax_mss, ay_mss, az_mss;  // Acceleration in m/s²
// float vx = 0, vy = 0, vz = 0;  // Velocity in m/s
// float dx = 0, dy = 0, dz = 0;  // Displacement in meters
// float gyroXrate, gyroYrate, gyroZrate;
// float roll = 0, pitch = 0, yaw = 0;  // Orientation angles
// unsigned long previousTime, currentTime;
// float dt;  // Delta time in seconds

// // Define filter constant (higher value = more smoothing)
// #define ALPHA 0.5
// #define ACCEL_THRESHOLD 0.05  // m/s², ADJUST THIS based on your noise level

// #define ACCEL_SCALE 16384.0  // Sensitivity for ±2g (LSB/g)
// #define GYRO_SCALE 131.0  // Sensitivity for ±250°/s (LSB/°/s)
// #define GRAVITY 9.81  // Acceleration due to gravity (m/s²)
// #define STATIONARY_TIME_THRESHOLD 2000 // Stationary time threshold in milliseconds, ADJUST THIS

// // Variables to store filtered acceleration values
// float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    // previousTime = millis();
}

void loop() {
    // read raw accel/gyro measurements from device
    // accelgyro.getAcceleration(&ax, &ay, &az);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // display tab-separated accel x/y/z values
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    // Serial.print(gz); Serial.print(",");
    Serial.println("");

    delay(10);

    // float axReal = ax / 16384.0;  // Assuming ±2g range
    // float ayReal = ay / 16384.0;
    // float azReal = az / 16384.0;

    // float gxReal = gx / 131.0;    // Assuming ±250°/s range
    // float gyReal = gy / 131.0;
    // float gzReal = gz / 131.0;

    // Buffers to hold formatted strings
    // char formattedAx[10];
    // char formattedAy[10];
    // char formattedAz[10];
    // char formattedGx[10];
    // char formattedGy[10];
    // char formattedGz[10];

    // // Format to 3 decimal places without specifying minimum width
    // dtostrf(axReal, 0, 3, formattedAx); // 0 minimum width, 3 decimal places
    // dtostrf(ayReal, 0, 3, formattedAy); // 0 minimum width, 3 decimal places
    // dtostrf(azReal, 0, 3, formattedAz); // 0 minimum width, 3 decimal places

    // dtostrf(gxReal, 0, 3, formattedGx); // 0 minimum width, 3 decimal places
    // dtostrf(gyReal, 0, 3, formattedGy); // 0 minimum width, 3 decimal places
    // dtostrf(gzReal, 0, 3, formattedGz); // 0 minimum width, 3 decimal places

    // // Display formatted data as comma-separated values
    // Serial.print(formattedAx); Serial.print(",");
    // Serial.print(formattedAy); Serial.print(",");
    // Serial.print(formattedAz); Serial.print(",");
    // Serial.print(formattedGx); Serial.print(",");
    // Serial.print(formattedGy); Serial.print(",");
    // Serial.print(formattedGz); Serial.println("");

    // // Convert raw accelerations to m/s²
    // ax_mss = (ax / ACCEL_SCALE) * GRAVITY;
    // ay_mss = (ay / ACCEL_SCALE) * GRAVITY;
    // az_mss = (az / ACCEL_SCALE) * GRAVITY;

    // // Convert raw gyroscope values to degrees/second
    // gyroXrate = gx / GYRO_SCALE;
    // gyroYrate = gy / GYRO_SCALE;
    // gyroZrate = gz / GYRO_SCALE;

    // // Calculate delta time (dt) in seconds
    // currentTime = millis();
    // dt = (currentTime - previousTime) / 1000.0;  // Convert ms to s
    // previousTime = currentTime;

    // // Integrate gyroscope rates to get orientation (roll, pitch, yaw)
    // roll += gyroXrate * dt;
    // pitch += gyroYrate * dt;
    // yaw += gyroZrate * dt;

    // // Apply complementary filter to combine accelerometer and gyroscope for pitch/roll
    // float accelRoll = atan2(ay_mss, az_mss) * 180 / 3.14159265359; // Roll from accel
    // float accelPitch = atan2(-ax_mss, sqrt(ay_mss * ay_mss + az_mss * az_mss)) * 180 / 3.14159265359; // Pitch from accel
    
    // // Complementary filter combines short-term gyro and long-term accel data
    // roll = ALPHA * (roll + gyroXrate * dt) + (1 - ALPHA) * accelRoll;
    // pitch = ALPHA * (pitch + gyroYrate * dt) + (1 - ALPHA) * accelPitch;

    // // Correct acceleration for orientation (compensating for tilt)
    // // Roll and pitch are in degrees, so convert to radians for trigonometry
    // float rollRad = roll * 3.14159265359 / 180;
    // float pitchRad = pitch * 3.14159265359 / 180;

    // // Rotate acceleration into the horizontal plane
    // float ax_corrected = ax_mss - GRAVITY * sin(pitchRad);
    // float ay_corrected = ay_mss - GRAVITY * sin(rollRad);
    // float az_corrected = az_mss - GRAVITY * cos(rollRad) * cos(pitchRad);

    // // Integrate corrected acceleration to get velocity
    // vx += ax_corrected * dt;
    // vy += ay_corrected * dt;
    // vz += az_corrected * dt;

    // // Integrate velocity to get displacement
    // dx += vx * dt;
    // dy += vy * dt;
    // dz += vz * dt;

    // // Check if IMU is stationary based on acceleration and gyroscope thresholds
    // if (abs(ax_corrected) < ACCEL_THRESHOLD &&
    //     abs(ay_corrected) < ACCEL_THRESHOLD &&
    //     abs(az_corrected) < ACCEL_THRESHOLD &&
    //     abs(gyroXrate) < ACCEL_THRESHOLD &&
    //     abs(gyroYrate) < ACCEL_THRESHOLD &&
    //     abs(gyroZrate) < ACCEL_THRESHOLD) {
        
    //     if (!isStationary) {
    //         stationaryStartTime = millis();
    //         isStationary = true;
    //     } else {
    //         // If stationary for longer than the threshold time, zero velocity and displacement
    //         if (millis() - stationaryStartTime > STATIONARY_TIME_THRESHOLD) {
    //             vx = vy = vz = 0;  // Zero velocity
    //             dx = dy = dz = 0;  // Zero displacement
    //             Serial.println("Zeroing displacement due to prolonged stationary state");
    //         }
    //     }
    // } else {
    //     isStationary = false;  // Reset stationary state if movement is detected
    // }

    // // Output displacement values
    // Serial.print("Displacement (m): ");
    // Serial.print("X: "); Serial.print(dx); Serial.print(", ");
    // Serial.print("Y: "); Serial.print(dy); Serial.print(", ");
    // Serial.print("Z: "); Serial.println(dz);

    // blink LED to indicate activity
    // blinkState = !blinkState;
    // digitalWrite(LED_PIN, blinkState);
}
