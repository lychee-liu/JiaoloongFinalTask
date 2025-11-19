//
// Created by lychee on 2025/11/19.
//
#include "imu.h"
#include "imu.h"
#include <cmath>

// 常量定义
#define PI 3.14
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float MG_TO_MS2 = 9.8f / 1000.0f; // mg转换为m/s²

IMU::IMU(const float& dt,
         const float& kg,
         const float& g_thres,
         const float R_imu[3][3],
         const float gyro_bias[3]):
    mahony_(dt, kg, g_thres) {
    // 复制传感器安装方向矩阵
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_imu_[i][j] = R_imu[i][j];
        }
    }

    // 复制陀螺仪零偏
    for (int i = 0; i < 3; i++) {
        gyro_bias_[i] = gyro_bias[i];
    }
}

void IMU::init(EulerAngle_t euler_deg_init) {
    // 将初始欧拉角转换为弧度
    float yaw_rad = euler_deg_init.yaw * DEG_TO_RAD;
    float pitch_rad = euler_deg_init.pitch * DEG_TO_RAD;
    float roll_rad = euler_deg_init.roll * DEG_TO_RAD;

    // 初始化数据结构
    euler_rad_ = EulerAngle_t(yaw_rad, pitch_rad, roll_rad);
    euler_deg_ = euler_deg_init;

    // 清零其他变量
    for (int i = 0; i < 3; i++) {
        gyro_sensor_[i] = 0.0f;
        gyro_world_[i] = 0.0f;
        gyro_sensor_dps_[i] = 0.0f;
        gyro_world_dps_[i] = 0.0f;
        accel_sensor_[i] = 0.0f;
        accel_world_[i] = 0.0f;
    }
}

uint8_t rx_acc_data[6], rx_gyro_data[6];

void IMU::readSensor() {
    // 读取加速度计和陀螺仪数据
    // 1. 设置/读取acc0x41寄存器中的量程range参数，并换算为量程系数
    uint8_t raw_range_accel;
    bmi088_accel_read_reg(0x41, &raw_range_accel, 1);
    float kAccel = (float)((1 << (raw_range_accel + 1)) * 1.5 * 1000);

    // 2. 读取acc0x12寄存器中的6位acc数据
    bmi088_accel_read_reg(0x12, rx_acc_data, 6);

    // 3. 用量程系数将原始数据转换为常用单位
    raw_data_.accel[0] = (int16_t)(rx_acc_data[1] << 8 | rx_acc_data[0]);
    raw_data_.accel[1] = (int16_t)(rx_acc_data[3] << 8 | rx_acc_data[2]);
    raw_data_.accel[2] = (int16_t)(rx_acc_data[5] << 8 | rx_acc_data[4]);

    accel_sensor_[0] = (float)raw_data_.accel[0] * kAccel / 32768 * MG_TO_MS2;
    accel_sensor_[1] = (float)raw_data_.accel[1] * kAccel / 32768 * MG_TO_MS2;
    accel_sensor_[2] = (float)raw_data_.accel[2] * kAccel / 32768 * MG_TO_MS2;

    // 1. 设置/读取gyro0x0F寄存器中的量程range参数，并换算为量程系数
    uint8_t raw_range_gyro;
    bmi088_gyro_read_reg(0x0F, &raw_range_gyro, 1);
    float kGyro = (float)(125 * (1 << (4 - raw_range_gyro)));
    // 2. 读取gyro0x02寄存器中的6位gyro数据
    bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);
    // 3. 用量程系数将原始数据转换为常用单位
    raw_data_.gyro[0] = (int16_t)(rx_gyro_data[1] << 8 | rx_gyro_data[0]);
    raw_data_.gyro[1] = (int16_t)(rx_gyro_data[3] << 8 | rx_gyro_data[2]);
    raw_data_.gyro[2] = (int16_t)(rx_gyro_data[5] << 8 | rx_gyro_data[4]);

    gyro_sensor_dps_[0] = (float)raw_data_.gyro[0] * kGyro / 32768;
    gyro_sensor_dps_[1] = (float)raw_data_.gyro[1] * kGyro / 32768;
    gyro_sensor_dps_[2] = (float)raw_data_.gyro[2] * kGyro / 32768;

    gyro_sensor_[0] = gyro_sensor_dps_[0] * DEG_TO_RAD;
    gyro_sensor_[1] = gyro_sensor_dps_[1] * DEG_TO_RAD;
    gyro_sensor_[2] = gyro_sensor_dps_[2] * DEG_TO_RAD;

    // 应用传感器安装方向变换
    float temp_accel[3] = { 0 };
    float temp_gyro[3] = { 0 };

    for (int i = 0; i < 3; i++) {
        temp_accel[i] = 0.0f;
        temp_gyro[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            temp_accel[i] += R_imu_[i][j] * accel_sensor_[j];
            temp_gyro[i] += R_imu_[i][j] * gyro_sensor_[j];
        }
    }

    // 应用陀螺仪零偏补偿
    for (int i = 0; i < 3; i++) {
        accel_world_[i] = temp_accel[i];
        gyro_world_[i] = temp_gyro[i] - gyro_bias_[i];
        gyro_world_dps_[i] = gyro_world_[i] * RAD_TO_DEG;
    }
}

void IMU::update(void) {
    // 使用Mahony算法更新姿态
    mahony_.update(q_, gyro_sensor_, accel_sensor_);

    // 从四元数计算欧拉角
    // 横滚角 (x-axis rotation)
    float sinr_cosp = 2.0f * (q_[0] * q_[1] + q_[2] * q_[3]);
    float cosr_cosp = 1.0f - 2.0f * (q_[1] * q_[1] + q_[2] * q_[2]);
    euler_rad_.roll = atan2f(sinr_cosp, cosr_cosp);

    // 俯仰角 (y-axis rotation)
    float sinp = 2.0f * (q_[0] * q_[2] - q_[3] * q_[1]);
    if (fabsf(sinp) >= 1.0f) {
        euler_rad_.pitch = copysignf(PI / 2.0f, sinp);
    } else {
        euler_rad_.pitch = asinf(sinp);
    }

    // 偏航角 (z-axis rotation)
    float siny_cosp = 2.0f * (q_[0] * q_[3] + q_[1] * q_[2]);
    float cosy_cosp = 1.0f - 2.0f * (q_[2] * q_[2] + q_[3] * q_[3]);
    euler_rad_.yaw = atan2f(siny_cosp, cosy_cosp);

    // 转换为角度制
    euler_deg_.roll = euler_rad_.roll * RAD_TO_DEG;
    euler_deg_.pitch = euler_rad_.pitch * RAD_TO_DEG;
    euler_deg_.yaw = euler_rad_.yaw * RAD_TO_DEG;
}




