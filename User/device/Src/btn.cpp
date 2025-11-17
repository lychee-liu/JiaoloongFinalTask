//
// Created by lychee on 2025/11/17.
//
#include "../Inc/btn.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == KEY_Pin) {
        stop_flag = !stop_flag;
    }
}