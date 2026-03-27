/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
 
#include "ICM45605S.h"

/* 
  The Arduino Serial Plotter can display a maximum of eight values or curves.
  So you can print one sensor data except algo output
*/
//#define PRINT_ACCEL
//#define PRINT_GYRO
//#define PRINT_MAG
//#define PRINT_BIAS

// Instantiate an ICM456XX with LSB address set to 0
ICM456xx IMU(Wire, 0);

uint8_t irq_received = 0;
uint8_t algo = ALGO_GRV;

void irq_handler(void) {
  irq_received = 1;
}

void setup() {
  int ret;
  Serial.begin(115200);
  while (!Serial) {}

  // Initializing the ICM456XX
  ret = IMU.begin();
  if (ret != 0) {
    Serial.print("ICM456xx initialization failed: ");
    Serial.println(ret);
    while (1)
      ;
  }
  // Start GAF algo with interrupt on pin 2
  // ALGO_GRV, enable GRV when enable 6-axis(AG)
  // ALGO_GMRV, enable GMRV when enable 6-axis(AM)
  // ALGO_RV, enable RV when enable 9-axis(AGM)
  algo = ALGO_GRV;
  ret = IMU.startGaf(2, irq_handler, algo);
  if (ret != 0) {
    Serial.print("GAF failed: ");
    Serial.println(ret);
    while (1)
      ;
  }

  Serial.println("CLEARDATA");
}

void loop() {
  // Wait for interrupt to read data from fifo
  if (irq_received) {
    irq_received = 0;

    float W, X, Y, Z, accuracy;

    if (algo == ALGO_GRV)
    {
      IMU.getGaf_GRVData(W, X, Y, Z);
      Serial.print("GRV ");
    } else if (algo == ALGO_GMRV)
    {
      IMU.getGaf_GMRVData(W, X, Y, Z, accuracy);
      Serial.print("GMRV ");
    } else if (algo == ALGO_RV)
    {
      IMU.getGaf_RVData(W, X, Y, Z, accuracy);
      Serial.print("RV ");
    }

    Serial.print("W:");
    Serial.print(W);
    Serial.print(",");
    Serial.print("X:");
    Serial.print(X);
    Serial.print(",");
    Serial.print("Y:");
    Serial.print(Y);
    Serial.print(",");
    Serial.print("Z:");
    Serial.print(Z); 
    if (algo != ALGO_GRV){
      Serial.print(",");
      Serial.print("Accuracy:");
      Serial.print(accuracy);
    }
    Serial.print(" ");
    
#ifdef PRINT_MAG
    if (algo == ALGO_GMRV || algo == ALGO_RV) {
      IMU.getGaf_RMData(X, Y, Z);
      Serial.print("MagX:");
      Serial.print(X);
      Serial.print(",");
      Serial.print("MagY:");
      Serial.print(Y);
      Serial.print(",");
      Serial.print("MagZ:");
      Serial.print(Z);
      Serial.print(" ");
    }
#endif

#ifdef PRINT_BIAS
    int bx,by,bz,gaf_acc;
    // Read bias(q16) and accuracy for gyro(GYRO), mag(MAG)
    IMU.getGaf_BiasData(GYRO, bx, by, bz, gaf_acc);

    Serial.print("Bias_X:");
    Serial.print(bx);
    Serial.print(",");
    Serial.print("Bias_Y:");      
    Serial.print(by);
    Serial.print(",");
    Serial.print("Bias_Z:");      
    Serial.print(bz);
    Serial.print(",");
    Serial.print("Accuracy:");
    Serial.print(gaf_acc);
    Serial.print(" ");
#endif

    inv_imu_sensor_data_t imu_data;
    // Read registers
    IMU.getDataFromRegisters(imu_data);

#ifdef PRINT_ACCEL
    Serial.print("AccelX:");
    Serial.print(imu_data.accel_data[0]);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(imu_data.accel_data[1]);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(imu_data.accel_data[2]);
    Serial.print(" ");
#endif

#ifdef PRINT_GYRO
    if (algo != ALGO_GMRV) {
      Serial.print("GyroX:");
      Serial.print(imu_data.gyro_data[0]);
      Serial.print(",");
      Serial.print("GyroY:");      
      Serial.print(imu_data.gyro_data[1]);
      Serial.print(",");
      Serial.print("GyroZ:");      
      Serial.print(imu_data.gyro_data[2]);
    }
#endif
    Serial.println("");
  }
}
