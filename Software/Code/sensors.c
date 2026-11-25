#ifndef SENSORS_H
#define SENSORS_H

#include "board_config.h"

typedef struct {
    float accel[3];
    float gyro[3];
    float gyro_bias[3];
    float baro_alt;
    float baro_base_alt;
    double gps_lat;
    double gps_lon;
    double gps_base_lat;
    double gps_base_lon;
    float gps_pos_xy[2];
    float gps_vel_xy[2];
    bool gps_fix;
} SensorData_t;

void Sensors_Init(SensorData_t *data);
void Sensors_Calibrate(SensorData_t *data);
void Sensors_ReadIMU(SensorData_t *data);
void Sensors_ReadBaro(SensorData_t *data);
void Sensors_ParseGPS_Byte(SensorData_t *data, uint8_t byte);

#endif