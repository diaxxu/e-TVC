#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

#define IMU_SPI_HANDLE          (&hspi1)
#define IMU_CS_PORT             GPIOA
#define IMU_CS_PIN              GPIO_PIN_4

#define BARO_I2C_HANDLE         (&hi2c1)
#define DPS310_I2C_ADDR         (0x77 << 1)

#define ELRS_UART_HANDLE        (&huart1)
#define GPS_UART_HANDLE         (&huart2)

#define PWM_TIM_HANDLE          (&htim1)
#define ESC_MAIN_CHANNEL        TIM_CHANNEL_1
#define SERVO_TVC_X_CHANNEL     TIM_CHANNEL_2
#define SERVO_TVC_Y_CHANNEL     TIM_CHANNEL_3
#define SERVO_CHUTE_CHANNEL     TIM_CHANNEL_4

#define LOOP_TIM_HANDLE         (&htim6)

#endif