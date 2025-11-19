//
// Created by lychee on 2025/10/26.
//
#include "RC.h"
#include "usart.h"
#include "string.h"
#include "mymath.h"

RemoteControl::RemoteControl() {}

void RemoteControl::init() {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, 32);
    tick = HAL_GetTick();
}

bool RemoteControl::updateStatus() {
    // uint32_t tick_now = HAL_GetTick();
    if ((HAL_GetTick() - tick) < 500) {
        status = true;
    } else {
        status = false;
    }
    return status;
}

bool RemoteControl::checkStatus() {
    return status;
}


//解析rx_data中的数据，存入ch0-3、s1-2
void RemoteControl::handle() {
    ch0_ori = ((int16_t)rx_data[0] | ((int16_t)rx_data[1] << 8)) & 0x07FF;
    ch1_ori = ((int16_t)rx_data[1] >> 3 | ((int16_t)rx_data[2] << 5)) & 0x07FF;
    ch2_ori = (((int16_t)rx_data[2] >> 6) | ((int16_t)rx_data[3] << 2) | ((int16_t)rx_data[4] << 10)) & 0x07FF;
    ch3_ori = (((int16_t)rx_data[4] >> 1) | ((int16_t)rx_data[5] << 7)) & 0x07FF;
    ch0 = linearMapping(ch0_ori, 364, 1684, -1, 1);
    ch1 = linearMapping(ch1_ori, 364, 1684, -1, 1);
    ch2 = linearMapping(ch2_ori, 364, 1684, -1, 1);
    ch3 = linearMapping(ch3_ori, 364, 1684, -1, 1);
    s1 = (SwitchPos)(((rx_data[5] >> 4) & 0x0C) >> 2);
    s2 = (SwitchPos)((rx_data[5] >> 4) & 0x03);
}

void RemoteControl::uartRxCallback(uint16_t Size) {
        memcpy(rx_data, rx_buf, Size);
        updateStatus();
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, Size);
}

