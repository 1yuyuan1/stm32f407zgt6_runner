#include "step_motor.hpp"
#include "line_follow_config.hpp"
#include "Emm_V5.h"
#include "bsp_uart.h"
#include <math.h>

namespace
{
constexpr float kWheelDiameterM = LineConfig::WheelDiameterM;
constexpr float kTrackWidthM = LineConfig::TrackWidthM;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kPulsesPerRev = LineConfig::PulsesPerRev;
constexpr uint8_t kAcceleration = LineConfig::MotorAcceleration;
static_assert(LineConfig::MotorCommandScale == 1.0f || LineConfig::MotorCommandScale == 10.0f,
              "MotorCommandScale must be 1 or 10");

uint16_t toRpm(float wheel_mps)
{
    return (uint16_t)(fabsf(wheel_mps) * 60.0f * LineConfig::MotorCommandScale /
                      (kPi * kWheelDiameterM) + 0.5f);
}
}

StepMotor::StepMotor(uint8_t id, uint8_t forward_direction)
    : id_(id), forward_direction_(forward_direction) {}

bool StepMotor::appendVelocity(float wheel_mps, uint8_t acceleration) const
{
    if (!isfinite(wheel_mps) || fabsf(wheel_mps) * 60.0f /
        (kPi * kWheelDiameterM) > LineConfig::MaxMotorRpm + 0.01f) return false;
    const uint16_t rpm = toRpm(wheel_mps);
    if (rpm > LineConfig::MaxMotorRpm * LineConfig::MotorCommandScale ||
        MMCL_count + 8 > MMCL_LEN - 5) return false;
    const uint8_t direction = wheel_mps < 0.0f ? (uint8_t)!forward_direction_ : forward_direction_;
    Emm_V5_MMCL_Vel_Control(id_, direction, rpm, acceleration, false);
    return true;
}

bool StepMotor::appendRelativePosition(float distance_m, float speed_mps, uint8_t acceleration) const
{
    if (!isfinite(distance_m) || !isfinite(speed_mps) || fabsf(distance_m) > 100.0f ||
        speed_mps <= 0.0f || speed_mps * 60.0f / (kPi * kWheelDiameterM) >
        LineConfig::MaxMotorRpm + 0.01f) return false;
    const uint16_t rpm = toRpm(speed_mps);
    if (rpm == 0 || rpm > LineConfig::MaxMotorRpm * LineConfig::MotorCommandScale ||
        MMCL_count + 13 > MMCL_LEN - 5) return false;
    const uint32_t pulses = (uint32_t)(fabsf(distance_m) * kPulsesPerRev /
                                      (kPi * kWheelDiameterM) + 0.5f);
    const uint8_t direction = distance_m < 0.0f ? (uint8_t)!forward_direction_ : forward_direction_;
    Emm_V5_MMCL_Pos_Control(id_, direction, rpm, acceleration, pulses, false, false);
    return true;
}

StepMotorBus::StepMotorBus()
    : motors_{StepMotor(1, 1), StepMotor(2, 0), StepMotor(3, 1), StepMotor(4, 0)} {}

bool StepMotorBus::sendVelocities(float left_mps, float right_mps, uint8_t acceleration)
{
    // 每个控制周期重新组装最新目标，避免旧命令不断累积。
    MMCL_count = 0;
    const bool encoded = motors_[0].appendVelocity(left_mps, acceleration) &&
                         motors_[1].appendVelocity(right_mps, acceleration) &&
                         motors_[2].appendVelocity(left_mps, acceleration) &&
                         motors_[3].appendVelocity(right_mps, acceleration);
    const bool sent = encoded && Emm_V5_Multi_Motor_Cmd(0);
    MMCL_count = 0;
    if (sent)
    {
        ++sent_count;
        sent_left_rpm_ = (left_mps < 0.0f ? -1.0f : 1.0f) * toRpm(left_mps) / LineConfig::MotorCommandScale;
        sent_right_rpm_ = (right_mps < 0.0f ? -1.0f : 1.0f) * toRpm(right_mps) / LineConfig::MotorCommandScale;
    }
    else ++send_fail_count;
    return sent;
}

bool StepMotorBus::setTwist(float speed_mps, float omega_rad_s, uint8_t acceleration)
{
    if (!isfinite(speed_mps) || !isfinite(omega_rad_s)) return false;
    float left = speed_mps - 0.5f * kTrackWidthM * omega_rad_s;
    float right = speed_mps + 0.5f * kTrackWidthM * omega_rad_s;
    if (!isfinite(left) || !isfinite(right)) return false;
    const float maximum = fmaxf(fabsf(left), fabsf(right));
    const float limit_mps = LineConfig::MaxMotorRpm * kPi * kWheelDiameterM / 60.0f;
    if (maximum > limit_mps)
    {
        // 左右轮速度同比例缩放，保持原来的转弯曲率。
        const float scale = limit_mps / maximum;
        left *= scale;
        right *= scale;
    }
    return sendVelocities(left, right, acceleration);
}

bool StepMotorBus::stop()
{
    MMCL_count = 0;
    for (uint8_t id = 1; id <= 4; ++id) Emm_V5_MMCL_Stop_Now(id, false);
    const bool sent = Emm_V5_Multi_Motor_Cmd(0);
    MMCL_count = 0;
    if (sent)
    {
        ++sent_count;
        sent_left_rpm_ = sent_right_rpm_ = 0.0f;
    }
    else ++send_fail_count;
    return sent;
}

bool StepMotorBus::moveStraight(float distance_m, float speed_mps)
{
    MMCL_count = 0;
    bool encoded = true;
    for (uint8_t i = 0; i < 4; ++i)
        encoded = encoded && motors_[i].appendRelativePosition(distance_m, speed_mps, kAcceleration);
    const bool sent = encoded && Emm_V5_Multi_Motor_Cmd(0);
    MMCL_count = 0;
    return sent;
}

bool StepMotorBus::requestVelocity(uint8_t id)
{
    if (id < 1 || id > 4) return false;
    const uint8_t request[] = {id, 0x35, 0x6B};
    return BspUart_Transmit(uart2_inst, request, sizeof(request)) == HAL_OK;
}

void StepMotorBus::onRx(const uint8_t *bytes, uint16_t size, uint32_t tick_ms)
{
    if (tick_ms - last_rx_ms_ > 4U) rx_size_ = 0;
    last_rx_ms_ = tick_ms;
    for (uint16_t i = 0; i < size; ++i)
    {
        const uint8_t byte = bytes[i];
        if (rx_size_ == 0)
        {
            if (byte >= 1 && byte <= 4) rx_[rx_size_++] = byte;
            continue;
        }
        rx_[rx_size_++] = byte;
        if (rx_size_ == 2)
        {
            if (byte == 0x35) rx_expected_ = 6;
            else if (byte == 0xAA || byte == 0xF6 || byte == 0xFD || byte == 0xFE || byte == 0xF3)
                rx_expected_ = 4;
            else
            {
                rx_size_ = 0;
                continue;
            }
        }
        if (rx_size_ == 4 && rx_[1] == 0x35 && byte == 0x6B &&
            (rx_[2] == 0xE2 || rx_[2] == 0xEE))
        {
            ++reject_count;
            rx_size_ = 0;
            continue;
        }
        if (rx_size_ == rx_expected_)
        {
            if (byte == 0x6B)
            {
                const uint8_t index = (uint8_t)(rx_[0] - 1);
                if (rx_[1] == 0x35 && rx_[2] <= 1)
                {
                    const uint16_t magnitude = ((uint16_t)rx_[3] << 8) | rx_[4];
                    if (magnitude <= 5000)
                    {
                        rpm_[index] = (rx_[2] == 0) ? (float)magnitude : -(float)magnitude;
                        feedback_ms_[index] = tick_ms;
                        ++feedback_sequence_[index];
                    }
                }
                else if (rx_[2] == 0x02) ++ack_count;
                else if (rx_[2] == 0xE2 || rx_[2] == 0xEE) ++reject_count;
            }
            rx_size_ = 0;
            rx_expected_ = 0;
        }
    }
}

MotorFeedback StepMotorBus::feedback(uint8_t id) const
{
    if (id < 1 || id > 4) return {0.0f, 0, 0};
    const uint8_t index = (uint8_t)(id - 1);
    return {rpm_[index], feedback_ms_[index], feedback_sequence_[index]};
}
