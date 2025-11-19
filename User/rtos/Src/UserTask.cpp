//
// Created by lychee on 2025/11/1.
//
#include "../Inc/UserTask.h"
#include "../../device/Inc/motor.h"
#include "cmsis_os2.h"
#include "can.h"
#include "imu.h"
#include "RC.h"

uint8_t tx_data[8];
uint8_t stop_data[8] = { 0 };
CAN_TxHeaderTypeDef tx_header = {
    .StdId = 0x1fe,
    .ExtId = 0,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};
extern M6020_Motor Motor_yaw, Motor_pitch;
uint32_t can_tx_mail_box_;
extern int stop_flag;

int16_t yaw_intensity = 0;
int16_t pitch_intensity = 0;

[[noreturn]] void motor_task(void*) {
    while (true) {
        const auto tick = osKernelGetTickCount();

        yaw_intensity = Motor_yaw.handle();
        tx_data[0] = (uint8_t)(yaw_intensity >> 8);
        tx_data[1] = (uint8_t)(yaw_intensity & 0xFF);
        Motor_pitch.FeedforwardIntensityCalc();
        pitch_intensity = Motor_pitch.handle();
        tx_data[6] = (uint8_t)(pitch_intensity >> 8);
        tx_data[7] = (uint8_t)(pitch_intensity & 0xFF);
        if (stop_flag)
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, stop_data, &can_tx_mail_box_);
        else
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
        osDelayUntil(tick + 1);
    }
}

osThreadId_t motor_task_handle;
constexpr osThreadAttr_t motor_task_attributes = {
    .name = "motor_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityNormal),
};

float R_imu[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
float gyro_bias[3] = { 0, 0, 0 };
IMU my_imu(0.001f, 0.5f, 0.5f, R_imu, gyro_bias);

[[noreturn]] void imu_task(void*) {
    while (true) {
        const auto tick = osKernelGetTickCount();
        my_imu.readSensor();
        my_imu.update();
        osDelayUntil(tick + 1);
    }
}

osThreadId_t imu_task_handle;
constexpr osThreadAttr_t imu_task_attributes = {
    .name = "imu_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityLow),
};

RemoteControl my_rc;

[[noreturn]] void rc_task(void*) {
    while (true) {
        const auto tick = osKernelGetTickCount();
        my_rc.handle();
        osDelayUntil(tick + 1);
    }
}

osThreadId_t rc_task_handle;
constexpr osThreadAttr_t rc_task_attributes = {
    .name = "rc_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityHigh),
};

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart == &huart3) {
        my_rc.uartRxCallback(Size);
    }
		
}

float pitch_target,yaw_target;
[[noreturn]] void control_task(void*) {
    while (true) {
        const auto tick = osKernelGetTickCount();

        switch (my_rc.s2) {
            case RemoteControl::DOWN:
                HAL_CAN_AddTxMessage(&hcan1, &tx_header, stop_data, &can_tx_mail_box_);
                break;
            case RemoteControl::UP:
                pitch_target = 80.0f + 20.0f * my_rc.ch1;
                yaw_target = 0.0f - 180.0f * my_rc.ch0;
                Motor_pitch.SetPosition(pitch_target,0,0);
                Motor_yaw.SetPosition(yaw_target,0,0);
default:
break;
        }
        osDelayUntil(tick + 1);
    }
}

osThreadId_t control_task_handle;
constexpr osThreadAttr_t control_task_attributes = {
    .name = "control_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityHigh),
};

void user_tasks_init() {
    motor_task_handle = osThreadNew(motor_task, nullptr, &motor_task_attributes);
    imu_task_handle = osThreadNew(imu_task, nullptr, &imu_task_attributes);
	 
    rc_task_handle = osThreadNew(rc_task, nullptr, &rc_task_attributes);
    control_task_handle = osThreadNew(control_task, nullptr, &rc_task_attributes);
}
