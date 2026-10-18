#ifndef STEP_MOTOR_HPP
#define STEP_MOTOR_HPP

#include <stdint.h>
#include "line_follow_config.hpp"

struct MotorFeedback
{
    float rpm; // 驱动器返回的带符号实际转速，单位转/分钟，并非目标速度估算值
    uint32_t tick_ms;
    uint32_t sequence;
};

class StepMotor
{
public:
    StepMotor(uint8_t id, uint8_t forward_direction);
    bool appendVelocity(float wheel_mps, uint8_t acceleration, bool invert_direction) const;
    bool appendRelativePosition(float distance_m, float speed_mps, uint8_t acceleration) const;

private:
    uint8_t id_;
    uint8_t forward_direction_;
};

// 发送接口仅允许底盘任务 ChassisTask 调用；接收解析在串口回调中执行。
class StepMotorBus
{
public:
    StepMotorBus();
    bool setTwist(float speed_mps, float omega_rad_s,
                  uint8_t acceleration = LineConfig::MotorAcceleration,
                  bool invert_direction = LineConfig::VelocityDirectionInverted);
    // 最近成功提交给串口的左右轮命令，不代表电机已经达到此转速。
    float sentLeftRpm() const { return sent_left_rpm_; }
    float sentRightRpm() const { return sent_right_rpm_; }
    bool stop();
    bool moveStraight(float distance_m, float speed_mps);
    bool requestVelocity(uint8_t id);
    void onRx(const uint8_t *bytes, uint16_t size, uint32_t tick_ms);
    MotorFeedback feedback(uint8_t id) const;

    volatile uint32_t sent_count = 0;
    volatile uint32_t send_fail_count = 0;
    volatile uint32_t ack_count = 0;
    volatile uint32_t reject_count = 0;

private:
    bool sendVelocities(float left_mps, float right_mps, uint8_t acceleration, bool invert_direction);
    float sent_left_rpm_ = 0.0f;
    float sent_right_rpm_ = 0.0f;
    StepMotor motors_[4];
    uint8_t rx_[6] = {};
    uint8_t rx_size_ = 0;
    uint8_t rx_expected_ = 0;
    uint32_t last_rx_ms_ = 0;
    volatile float rpm_[4] = {};
    volatile uint32_t feedback_ms_[4] = {};
    volatile uint32_t feedback_sequence_[4] = {};
};

#endif
