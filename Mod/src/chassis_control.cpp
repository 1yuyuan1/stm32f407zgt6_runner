#include "chassis_app.h"
#include "line_follow_config.hpp"
#include <math.h>

// 调试器和上层使用同一份参数；复位会恢复这里的默认值，不写入闪存。
volatile ChassisControl g_chassis_control = {
    0, 0, LineConfig::DefaultSpeedRpm,
    LineConfig::Kp, LineConfig::Kd, LineConfig::DefaultMaxOmega,
    LineConfig::DefaultTurnSpeedRatio
};
volatile ChassisStatus g_chassis_status = {};

extern "C" void Chassis_Start(void) { g_chassis_control.start = 1; }
extern "C" void Chassis_Stop(void) { g_chassis_control.stop = 1; }

extern "C" int Chassis_SetSpeedRpm(float rpm)
{
    if (!isfinite(rpm) || rpm < 0.0f || rpm > LineConfig::MaxMotorRpm) return 0;
    g_chassis_control.speed_rpm = rpm;
    return 1;
}

extern "C" int Chassis_ParametersValid(const ChassisControl *control)
{
    if (control == nullptr) return 0;
    const float values[] = {control->speed_rpm, control->kp, control->kd,
                            control->max_omega, control->turn_speed_ratio};
    for (float value : values)
        if (!isfinite(value) || value < 0.0f) return 0;
    // 增益上限用于捕获调试器误写，避免中间计算溢出；不代表推荐调参范围。
    return control->speed_rpm <= LineConfig::MaxMotorRpm && control->kp <= 10000.0f &&
           control->kd <= 1000.0f && control->max_omega >= 0.1f &&
           control->max_omega <= 10.0f && control->turn_speed_ratio >= 0.1f &&
           control->turn_speed_ratio <= 1.0f;
}
