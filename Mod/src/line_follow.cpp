#include "line_follow.hpp"
#include <math.h>

namespace
{
constexpr float kPi = 3.14159265358979323846f;
using namespace LineConfig;

float clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

float wrapPi(float angle)
{
    while (angle > kPi) angle -= 2.0f * kPi;
    while (angle < -kPi) angle += 2.0f * kPi;
    return angle;
}
}

void LineFollower::reset()
{
    had_line_ = false;
    previous_error_m_ = 0.0f;
    filtered_error_m_ = 0.0f;
    last_line_yaw_rad_ = 0.0f;
    gap_distance_m_ = 0.0f;
    gap_time_s_ = 0.0f;
    last_speed_mps_ = 0.0f;
}

LineCommand LineFollower::step(const LineInput &input, const ChassisControl &control)
{
    if (!Chassis_ParametersValid(&control))
        return {0.0f, 0.0f, filtered_error_m_, LineStatus::InvalidParameter};

    // 基础转速换算为车体线速度；弯道降速由实际转向需求连续决定。
    const float base_speed = control.speed_rpm * (kPi * WheelDiameterM) / 60.0f;
    if (!input.gray_fresh || !input.imu_fresh)
        return {0.0f, 0.0f, filtered_error_m_, LineStatus::NoSensor};

    const uint16_t active = WhiteLineIsOne ? input.gray_bits : (uint16_t)~input.gray_bits;
    uint8_t active_count = 0;
    bool found = false;
    float best_error_m = 0.0f;
    float best_distance = 1000.0f;
    uint8_t best_width = 0;

    for (uint8_t i = 0; i < 16;)
    {
        if ((active & (1U << i)) == 0)
        {
            ++i;
            continue;
        }

        const uint8_t first = i;
        while (i < 16 && (active & (1U << i)) != 0)
        {
            ++active_count;
            ++i;
        }
        const uint8_t last = (uint8_t)(i - 1);
        const uint8_t width = (uint8_t)(last - first + 1);
        const float physical_index = ((float)first + (float)last) * 0.5f - 7.5f;
        const float candidate = physical_index * SensorPitchM * (Channel1OnLeft ? 1.0f : -1.0f);
        const float distance = fabsf(candidate - (had_line_ ? filtered_error_m_ : 0.0f));
        if (!found || distance < best_distance)
        {
            found = true;
            best_distance = distance;
            best_error_m = candidate;
            best_width = width;
        }
    }

    if (active_count > 8 || best_width > 6)
        return {0.0f, 0.0f, filtered_error_m_, LineStatus::Ambiguous};

    const float dt = clamp(input.dt_s, 0.001f, 0.05f);
    if (found)
    {
        filtered_error_m_ = had_line_ ? (0.65f * filtered_error_m_ + 0.35f * best_error_m)
                                      : best_error_m;
        const float derivative = had_line_ ? (filtered_error_m_ - previous_error_m_) / dt : 0.0f;
        previous_error_m_ = filtered_error_m_;
        last_line_yaw_rad_ = input.yaw_rad;
        gap_distance_m_ = 0.0f;
        gap_time_s_ = 0.0f;
        had_line_ = true;

        // 前置约 20cm 的单排传感器，其测得的偏差同时受车体横向偏移和航向影响。
        const float omega = clamp(-control.kp * filtered_error_m_ - control.kd * derivative
                                  - YawRateDamping * input.yaw_rate_rad_s,
                                  -control.max_omega, control.max_omega);
        const float turn_demand = clamp(fabsf(omega) / control.max_omega, 0.0f, 1.0f);
        const float speed = base_speed * (1.0f - (1.0f - control.turn_speed_ratio) * turn_demand);
        last_speed_mps_ = speed;
        return {speed, speed > 0.0f ? omega : 0.0f, filtered_error_m_, LineStatus::Tracking};
    }

    if (!had_line_)
        return {0.0f, 0.0f, 0.0f, LineStatus::NoLine};

    // 仅对疑似直线虚线缺口短时保持航向；转弯偏出白线时不继续盲走。
    if (fabsf(filtered_error_m_) > 0.025f || fabsf(input.yaw_rate_rad_s) > 0.6f)
        return {0.0f, 0.0f, filtered_error_m_, LineStatus::NoLine};

    gap_distance_m_ += last_speed_mps_ * dt;
    gap_time_s_ += dt;
    if (gap_distance_m_ > MaxGapM || gap_time_s_ > MaxGapS)
        return {0.0f, 0.0f, filtered_error_m_, LineStatus::NoLine};

    const float heading_error = wrapPi(last_line_yaw_rad_ - input.yaw_rad);
    const float omega = clamp(GapHeadingKp * heading_error,
                              -control.max_omega, control.max_omega);
    const float gap_speed = base_speed * control.turn_speed_ratio;
    // 降低基础转速立即限幅，尤其是写入零速时不能继续沿用上一周期的速度。
    last_speed_mps_ = clamp(last_speed_mps_ * 0.8f + gap_speed * 0.2f, 0.0f, base_speed);
    return {last_speed_mps_, last_speed_mps_ > 0.0f ? omega : 0.0f,
            filtered_error_m_, LineStatus::ShortGap};
}
