//
// Created by lychee on 2025/11/1.
//
#include "../Inc/UserTask.h"
#include "../../device/Inc/motor.h"
#include "cmsis_os2.h"
#include "can.h"

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
        yaw_intensity = Motor_yaw.handle();
        tx_data[0] = (uint8_t)(yaw_intensity >> 8);
        tx_data[1] = (uint8_t)(yaw_intensity & 0xFF);
        pitch_intensity = Motor_pitch.handle();
        tx_data[6] = (uint8_t)(pitch_intensity >> 8);
        tx_data[7] = (uint8_t)(pitch_intensity & 0xFF);
        if (stop_flag)
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, stop_data, &can_tx_mail_box_);
        else
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &can_tx_mail_box_);
    }
}

osThreadId_t motor_task_handle;
constexpr osThreadAttr_t motor_task_attributes = {
    .name = "motor_task",
    .stack_size = 128 * 4,
    .priority = (osPriorityNormal),
};

void user_tasks_init() {
    motor_task_handle = osThreadNew(motor_task, nullptr, &motor_task_attributes);
}
