//
// Created by lychee on 2025/10/3.
//
#include "main.h"
#include "../../algorithm/Inc/pid.h"
#include "../../algorithm/Inc/mymath.h"

class M6020_Motor {
public:
    enum ControlMethod {
        TORQUE,
        SPEED,
        POSITION_SPEED
    };

    explicit M6020_Motor(const float ratio, PID spid, PID ppid, ControlMethod control_method = TORQUE):
        ratio_(ratio),
        spid_(spid),
        ppid_(ppid),
        control_method_(control_method) {
        spid_.reset();
        ppid_.reset();
    };
    void canRxMsgCallback(const uint8_t rx_data[8]);

    float GetAngle() const {
        return angle_;
    }

    float GetSpeed() const {
        return rotate_speed_;
    }

    float GetCurrent() const {
        return current_;
    }

    float GetTemperature() const {
        return temp_;
    }

    ControlMethod GetControlMethod() const {
        return control_method_;
    }

    void SetControlMethod(ControlMethod method) {
        control_method_ = method;
    }

    void SetPosition(float target_position, float feedforward_speed = 0, float feedforward_intensity = 0);
    void SetSpeed(float target_speed, float feedforward_intensity = 0);
    void SetIntensity(float intensity);
    int16_t handle(void);
    float FeedforwardIntensityCalc(float current_angle);

private:
    const float ratio_;

    float angle_ = 0.f;
    float delta_angle_ = 0.f;
    float ecd_angle_ = 0.f; //
    float last_ecd_angle_ = 0.f; //
    float delta_ecd_angle_ = 0.f; //
    float last_rotate_speed_ = 0.f; //
    float rotate_speed_ = 0.f; //
    float current_ = 0.f; //
    float temp_ = 0.f; //

    PID spid_ = PID(1.0f, 1.0f, 0.0f, 10.0f, 10.0f, 1);
    PID ppid_ = PID(1.0f, 1.0f, 0.0f, 10.0f, 10.0f, 1);
    float target_angle_ = 0.f, fdb_angle_ = 0.f;
    float target_speed_ = 0.f, fdb_speed_ = 0.f, feedforward_speed_ = 0.f;
    float feedforward_intensity_ = 0.f, output_intensity_ = 0.f;
    ControlMethod control_method_ = TORQUE;
};
