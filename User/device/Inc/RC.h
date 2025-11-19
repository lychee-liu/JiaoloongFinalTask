//
// Created by lychee on 2025/10/26.
//

#ifndef INC_1025REMOTECTRL_RC_H
#define INC_1025REMOTECTRL_RC_H

#ifdef __cplusplus
extern "C" {
    #endif

#include "stm32f4xx_hal.h"
#include "main.h"

class RemoteControl {
private:
    uint32_t tick;
    bool status = true;
    uint8_t rx_buf[32] = { 0 };
    uint8_t rx_data[32] = { 0 };

    int16_t ch0_ori = 0;
    int16_t ch1_ori = 0;
    int16_t ch2_ori = 0;
    int16_t ch3_ori = 0;
public:
    enum SwitchPos {
        RESET,
        UP,
        DOWN,
        MID
    };



    float ch0 = 0.0f;
    float ch1 = 0.0f;
    float ch2 = 0.0f;
    float ch3 = 0.0f;
    SwitchPos s1 = RESET, s2 = RESET;
    RemoteControl();
    void init();
    bool updateStatus();
    bool checkStatus();
    void handle();
    void uartRxCallback(uint16_t Size);
};

    #ifdef __cplusplus
}
#endif

#endif //INC_1025REMOTECTRL_RC_H