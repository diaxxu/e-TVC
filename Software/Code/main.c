#include "board_config.h"
#include "sensors.h"
#include "estimator.h"
#include "crsf.h"
#include "flight_control.h"

static SensorData_t sensor_data;
static VehicleState_t vehicle_state;
static RC_Input_t rc_input;

static uint8_t elrs_rx_byte;
static uint8_t gps_rx_byte;
static volatile bool loop_200hz_flag = false;

void App_Init(void) {
    Sensors_Init(&sensor_data);
    Sensors_Calibrate(&sensor_data);
    Estimator_Init(&vehicle_state);
    FlightControl_Init();

    HAL_TIM_PWM_Start(PWM_TIM_HANDLE, ESC_MAIN_CHANNEL);
    HAL_TIM_PWM_Start(PWM_TIM_HANDLE, SERVO_TVC_X_CHANNEL);
    HAL_TIM_PWM_Start(PWM_TIM_HANDLE, SERVO_TVC_Y_CHANNEL);
    HAL_TIM_PWM_Start(PWM_TIM_HANDLE, SERVO_CHUTE_CHANNEL);

    __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, ESC_MAIN_CHANNEL, 1000);
    __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, SERVO_TVC_X_CHANNEL, 1500);
    __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, SERVO_TVC_Y_CHANNEL, 1500);
    __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, SERVO_CHUTE_CHANNEL, 1000);

    HAL_UART_Receive_IT(ELRS_UART_HANDLE, &elrs_rx_byte, 1);
    HAL_UART_Receive_IT(GPS_UART_HANDLE, &gps_rx_byte, 1);
    HAL_TIM_Base_Start_IT(LOOP_TIM_HANDLE);
}

void App_Loop(void) {
    static uint8_t baro_div = 0;

    if (loop_200hz_flag) {
        loop_200hz_flag = false;
        const float dt = 0.005f;
        uint32_t now = HAL_GetTick();

        Sensors_ReadIMU(&sensor_data);
        if (++baro_div >= 4) {
            baro_div = 0;
            Sensors_ReadBaro(&sensor_data);
        }

        Estimator_Update(&vehicle_state, &sensor_data, dt);

        uint16_t esc_pwm = 1000, sx_pwm = 1500, sy_pwm = 1500, chute_pwm = 1000;
        FlightControl_Update200Hz(dt, &vehicle_state, &rc_input, now,
                                  &esc_pwm, &sx_pwm, &sy_pwm, &chute_pwm);

        __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, ESC_MAIN_CHANNEL, esc_pwm);
        __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, SERVO_TVC_X_CHANNEL, sx_pwm);
        __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, SERVO_TVC_Y_CHANNEL, sy_pwm);
        __HAL_TIM_SET_COMPARE(PWM_TIM_HANDLE, SERVO_CHUTE_CHANNEL, chute_pwm);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == LOOP_TIM_HANDLE) {
        loop_200hz_flag = true;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == ELRS_UART_HANDLE) {
        CRSF_ParseByte(&rc_input, elrs_rx_byte, HAL_GetTick());
        HAL_UART_Receive_IT(ELRS_UART_HANDLE, &elrs_rx_byte, 1);
    } else if (huart == GPS_UART_HANDLE) {
        Sensors_ParseGPS_Byte(&sensor_data, gps_rx_byte);
        HAL_UART_Receive_IT(GPS_UART_HANDLE, &gps_rx_byte, 1);
    }
}