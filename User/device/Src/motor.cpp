#include "../Inc/motor.h"
#include <cmath>
#include "../../Core/Inc/can.h"
//#include "algorithm.h"


void M6020_Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    last_ecd_angle_ = ecd_angle_;
    last_rotate_speed_ = rotate_speed_;
    int16_t ecd_angle = ((int16_t)rx_data[0] << 8) | (int16_t)rx_data[1];
    ecd_angle_ = linearMapping(ecd_angle, 0, 8191, 0.0, 360.0);
    delta_ecd_angle_ = float(ecd_angle_ - last_ecd_angle_);
    if (delta_ecd_angle_ > 180)
        delta_ecd_angle_ -= 360;
    else if (delta_ecd_angle_ < -180)
        delta_ecd_angle_ += 360;
    rotate_speed_ = static_cast<float>((int16_t)(rx_data[2] << 8 | rx_data[3]));
    current_ = static_cast<float>(((int16_t)rx_data[4] << 8) | (int16_t)rx_data[5]);
    temp_ = rx_data[6];
    delta_angle_ = delta_ecd_angle_ / ratio_;
    angle_ += delta_angle_;

    fdb_angle_ = angle_;
    fdb_speed_ = rotate_speed_;
    //若反馈转速不稳定
    //fdb_speed_ = rotate_speed_ * 0.1 + last_rotate_speed_ * 0.9;
    // if (fdb_speed_ > 200)
    //     fdb_speed_ = 200;
    // else if (fdb_speed_ < -200)
    //     fdb_speed_ = -200;
}

void M6020_Motor::SetIntensity(float intensity) {
    output_intensity_ = intensity;
    control_method_ = TORQUE;
}

void M6020_Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
    control_method_ = SPEED;
}

void M6020_Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
    control_method_ = POSITION_SPEED;
}

int16_t MAX_INTENSITY = 10000;

int16_t M6020_Motor::handle() {
    switch (control_method_) {
        case TORQUE:
            break;
        case SPEED:
            FeedforwardIntensityCalc(angle_);
            output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
            break;
        case POSITION_SPEED:
            FeedforwardIntensityCalc(angle_);
            target_speed_ = ppid_.calc(target_angle_, fdb_angle_) + feedforward_speed_;
            output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
            break;
    }

    int16_t intensity = (int16_t)(output_intensity_ * 16384.0f / 3.0f);
    if (intensity > MAX_INTENSITY) {
        intensity = MAX_INTENSITY;
    } else if (intensity < -MAX_INTENSITY) {
        intensity = -MAX_INTENSITY;
    }
    return intensity;
}

float M6020_Motor::FeedforwardIntensityCalc(float current_angle) {
    //feedforward_intensity_ = 0.5 * 9.8 * 0.05524 * sinf(current_angle * 3.14 / 180.0) / 0.3f;
    return 0;
}

M6020_Motor Motor_yaw(1,
                      PID(0.006, 0, 0, 0, 1.5, 0.2),
                      PID(10, 0, 5, 0, 400, 0.05),
                      M6020_Motor::POSITION_SPEED);
M6020_Motor Motor_pitch(1,
                        PID(0.006, 0, 0, 0, 1.5, 0.2),
                        PID(10, 0, 5, 0, 400, 0.05),
                        M6020_Motor::TORQUE);

uint8_t rx_data[8];
CAN_RxHeaderTypeDef rx_header;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0, &rx_header, rx_data);
        if (rx_header.StdId == 0x205) {
            Motor_yaw.canRxMsgCallback(rx_data);
        }
        if (rx_header.StdId == 0x208) {
            Motor_pitch.canRxMsgCallback(rx_data);
        }
    }
}