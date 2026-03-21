// MatrixLidar.h — DFRobot 8x8 Matrix ToF sensor
//
// Top-level API (called from the main .ino):
//   setupMatrixLidar()                  — call once from setup()
//   loopMatrixLidar()                   — call every loop() iteration
//   writeMatrixLidarData(wireMode)      — call from data_loop(); returns PacketField
//   handleMatrixLidar()                 — call from request_custom() to respond to 'M' command
//
// Requires StormNetCommon.h to be included before this file.

#include "DFRobot_MatrixLidar.h"

#define MATRIX_LIDAR_ADDRESS   0x33
#define MATRIX_LIDAR_BUFFER_SIZE 64
#define MATRIX_LIDAR_BAD_VALUE 4000
#define MATRIX_LIDAR_NUM_RESULTS 1

DFRobot_MatrixLidar_I2C tof(MATRIX_LIDAR_ADDRESS);
uint16_t matrixLidarBuffer[MATRIX_LIDAR_BUFFER_SIZE];
long matrixLidarResults[MATRIX_LIDAR_NUM_RESULTS] = {0};

// --- Internal helpers ---

void getMatrixLidarData() {
  tof.getAllData(matrixLidarBuffer);
}

long getMatrixLidarAverage() {
  int count = 0;
  long sum = 0;

  for (uint8_t i = 0; i < 8; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      long value = matrixLidarBuffer[i * 8 + j];
      if (value != MATRIX_LIDAR_BAD_VALUE) {
        sum += value;
        count++;
      }
    }
  }

  return (count == 0) ? -1 : sum / count;
}

// --- Top-level API ---

void setupMatrixLidar() {
  while (tof.begin() != 0) {
    Serial.println("begin error !!!!!");
  }
  Serial.println("begin success");
  while (tof.setRangingMode(eMatrix_8X8) != 0) {
    Serial.println("init error !!!!!");
    delay(1000);
  }
  Serial.println("init success");
}

void loopMatrixLidar() {
  getMatrixLidarData();
  matrixLidarResults[0] = getMatrixLidarAverage();
}

PacketField writeMatrixLidarData(wireMode mode) {
  if (mode == serialMode) {
    Serial.print("matrix lidar: ");
  }
  writeLongs(matrixLidarResults, MATRIX_LIDAR_NUM_RESULTS, mode);
  return {"M", "Matrix Lidar", MATRIX_LIDAR_NUM_RESULTS * (int)sizeof(long)};
}

void handleMatrixLidar() {
  writeMatrixLidarData(g_talkMode);
}
