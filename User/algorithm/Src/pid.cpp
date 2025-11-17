//
// Created by lychee on 2025/10/18.
//
#include "../Inc/pid.h"

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k):
    kp_(kp),
    ki_(ki),
    kd_(kd),
    i_max_(i_max),
    out_max_(out_max),
    d_filter_k_(d_filter_k) {}

void PID::reset() {
    ref_ = 0.0f;
    fdb_ = 0.0f;
    err_ = 0.0f;
    err_sum_ = 0.0f;
    last_err_ = 0.0f;
    pout_ = 0.0f;
    iout_ = 0.0f;
    dout_ = 0.0f;
    last_dout_ = 0.0f;
    output_ = 0.0f;
}

float PID::calc(float ref, float fdb) {
    ref_ = ref;
    fdb_ = fdb;
    last_err_ = err_;
    last_dout_ = dout_;
    err_ = ref_ - fdb_;
    err_sum_ += err_;
    pout_ = kp_ * err_;
    iout_ = ki_ * err_sum_;
    if (iout_ > i_max_)
        iout_ = i_max_;
    else if (iout_ < -i_max_)
        iout_ = -i_max_;
    dout_ = kd_ * (err_ - last_err_);
    dout_ = (1 - d_filter_k_) * dout_ + d_filter_k_ * last_dout_;
    output_ = pout_ + dout_ + iout_;
    if (output_ > out_max_)
        output_ = out_max_;
    else if (output_ < -out_max_)
        output_ = -out_max_;
    return output_;
}

