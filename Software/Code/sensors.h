#include "sensors.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define ICM_REG_DEVICE_CONFIG   0x11
#define ICM_REG_PWR_MGMT0       0x4E
#define ICM_REG_GYRO_CONFIG0    0x4F
#define ICM_REG_ACCEL_CONFIG0   0x50
#define ICM_REG_ACCEL_DATA_X1   0x1F

static void IMU_WriteReg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg & 0x7F, val};
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(IMU_SPI_HANDLE, tx, 2, 10);
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
}

static void IMU_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t addr = reg | 0x80;
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(IMU_SPI_HANDLE, &addr, 1, 10);
    HAL_SPI_Receive(IMU_SPI_HANDLE, buf, len, 10);
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
}

void Sensors_Init(SensorData_t *data) {
    memset(data, 0, sizeof(SensorData_t));
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    IMU_WriteReg(ICM_REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(10);
    IMU_WriteReg(ICM_REG_PWR_MGMT0, 0x0F);
    HAL_Delay(50);
    IMU_WriteReg(ICM_REG_GYRO_CONFIG0, 0x06);
    IMU_WriteReg(ICM_REG_ACCEL_CONFIG0, 0x06);

    uint8_t dps_cfg[2] = {0x06, 0x26};
    HAL_I2C_Master_Transmit(BARO_I2C_HANDLE, DPS310_I2C_ADDR, dps_cfg, 2, 10);
    dps_cfg[0] = 0x07; dps_cfg[1] = 0x26;
    HAL_I2C_Master_Transmit(BARO_I2C_HANDLE, DPS310_I2C_ADDR, dps_cfg, 2, 10);
    dps_cfg[0] = 0x08; dps_cfg[1] = 0x07;
    HAL_I2C_Master_Transmit(BARO_I2C_HANDLE, DPS310_I2C_ADDR, dps_cfg, 2, 10);
}

void Sensors_Calibrate(SensorData_t *data) {
    float gx_sum = 0, gy_sum = 0, gz_sum = 0, alt_sum = 0;
    for (int i = 0; i < 500; i++) {
        Sensors_ReadIMU(data);
        gx_sum += data->gyro[0];
        gy_sum += data->gyro[1];
        gz_sum += data->gyro[2];
        Sensors_ReadBaro(data);
        alt_sum += data->baro_alt;
        HAL_Delay(2);
    }
    data->gyro_bias[0] = gx_sum / 500.0f;
    data->gyro_bias[1] = gy_sum / 500.0f;
    data->gyro_bias[2] = gz_sum / 500.0f;
    data->baro_base_alt = alt_sum / 500.0f;
}

void Sensors_ReadIMU(SensorData_t *data) {
    uint8_t raw[12];
    IMU_ReadRegs(ICM_REG_ACCEL_DATA_X1, raw, 12);

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t gx = (int16_t)((raw[6] << 8) | raw[7]);
    int16_t gy = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gz = (int16_t)((raw[10] << 8) | raw[11]);

    const float accel_scale = (16.0f * 9.80665f) / 32768.0f;
    const float gyro_scale  = (2000.0f * 0.0174532925f) / 32768.0f;

    float raw_ax = ax * accel_scale;
    float raw_ay = ay * accel_scale;
    float raw_az = az * accel_scale;

    float raw_gx = (gx * gyro_scale) - data->gyro_bias[0];
    float raw_gy = (gy * gyro_scale) - data->gyro_bias[1];
    float raw_gz = (gz * gyro_scale) - data->gyro_bias[2];

    const float lpf = 0.25f;
    data->accel[0] += lpf * (raw_ax - data->accel[0]);
    data->accel[1] += lpf * (raw_ay - data->accel[1]);
    data->accel[2] += lpf * (raw_az - data->accel[2]);

    data->gyro[0] += lpf * (raw_gx - data->gyro[0]);
    data->gyro[1] += lpf * (raw_gy - data->gyro[1]);
    data->gyro[2] += lpf * (raw_gz - data->gyro[2]);
}

void Sensors_ReadBaro(SensorData_t *data) {
    uint8_t reg = 0x00;
    uint8_t buf[3];
    HAL_I2C_Master_Transmit(BARO_I2C_HANDLE, DPS310_I2C_ADDR, &reg, 1, 5);
    if (HAL_I2C_Master_Receive(BARO_I2C_HANDLE, DPS310_I2C_ADDR, buf, 3, 5) == HAL_OK) {
        int32_t raw_p = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
        if (raw_p & 0x800000) raw_p -= 16777216;
        float pressure_pa = 101325.0f + ((float)raw_p / 524288.0f);
        float alt = 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.1903f));
        data->baro_alt = alt - data->baro_base_alt;
    }
}

void Sensors_ParseGPS_Byte(SensorData_t *data, uint8_t byte) {
    static char nmea_buf[128];
    static uint8_t idx = 0;

    if (byte == '$') {
        idx = 0;
    }
    if (idx < sizeof(nmea_buf) - 1) {
        nmea_buf[idx++] = (char)byte;
    }
    if (byte == '\n') {
        nmea_buf[idx] = '\0';
        if (strncmp(nmea_buf, "$GNGGA", 6) == 0 || strncmp(nmea_buf, "$GPGGA", 6) == 0) {
            char *tok = strtok(nmea_buf, ",");
            int field = 0;
            double lat = 0, lon = 0;
            char ns = 'N', ew = 'E';
            int fix = 0;

            while (tok != NULL) {
                if (field == 2) lat = atof(tok);
                else if (field == 3) ns = tok[0];
                else if (field == 4) lon = atof(tok);
                else if (field == 5) ew = tok[0];
                else if (field == 6) fix = atoi(tok);
                tok = strtok(NULL, ",");
                field++;
            }

            if (fix > 0) {
                int lat_deg = (int)(lat / 100.0);
                double lat_min = lat - (lat_deg * 100.0);
                double dec_lat = lat_deg + (lat_min / 60.0);
                if (ns == 'S') dec_lat = -dec_lat;

                int lon_deg = (int)(lon / 100.0);
                double lon_min = lon - (lon_deg * 100.0);
                double dec_lon = lon_deg + (lon_min / 60.0);
                if (ew == 'W') dec_lon = -dec_lon;

                if (!data->gps_fix) {
                    data->gps_base_lat = dec_lat;
                    data->gps_base_lon = dec_lon;
                    data->gps_fix = true;
                }

                float new_x = (float)((dec_lon - data->gps_base_lon) * 111320.0 * cos(dec_lat * 0.0174532925));
                float new_y = (float)((dec_lat - data->gps_base_lat) * 110540.0);

                data->gps_vel_xy[0] = (new_x - data->gps_pos_xy[0]) * 10.0f;
                data->gps_vel_xy[1] = (new_y - data->gps_pos_xy[1]) * 10.0f;
                data->gps_pos_xy[0] = new_x;
                data->gps_pos_xy[1] = new_y;
            }
        }
        idx = 0;
    }
}