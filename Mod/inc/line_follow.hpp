#ifndef LINE_FOLLOW_HPP
#define LINE_FOLLOW_HPP

#include <stdint.h>
#include "line_follow_config.hpp"
#include "chassis_app.h"

enum class LineStatus : uint8_t
{
    Tracking,
    ShortGap,
    NoSensor,
    NoLine,
    Ambiguous,
    InvalidParameter // 在线参数非法
};

struct LineInput
{
    uint16_t gray_bits;
    bool gray_fresh;
    float yaw_rad;
    float yaw_rate_rad_s;
    bool imu_fresh;
    float dt_s;
};

struct LineCommand
{
    float speed_mps;
    float omega_rad_s;   // 正值表示逆时针转动，即左转
    float line_error_m;  // 正值表示白线位于传感器中心右侧
    LineStatus status;
};

class LineFollower
{
public:
    void reset();
    // 参数由底盘任务每周期取快照，算法不直接访问调试器全局变量。
    LineCommand step(const LineInput &input, const ChassisControl &control);

private:
    bool had_line_ = false;
    float previous_error_m_ = 0.0f;
    float filtered_error_m_ = 0.0f;
    float last_line_yaw_rad_ = 0.0f;
    float gap_distance_m_ = 0.0f;
    float gap_time_s_ = 0.0f;
    float last_speed_mps_ = 0.0f;
};

#endif
