#ifndef LINE_FOLLOW_CONFIG_HPP
#define LINE_FOLLOW_CONFIG_HPP

#include <stdint.h>

// 硬件常量与上电默认参数；在线调参入口见 chassis_app.h。
// 已确认的硬件与传感器配置集中固定；只把常用调试量放在底盘控制结构体中。
namespace LineConfig
{
constexpr uint8_t GrayId = 0x01; // 灰度模块的串口设备地址
constexpr bool WhiteLineIsOne = false; // 当前实测绿色背景为 0xFFFF；默认用低电平表示白线
constexpr bool Channel1OnLeft = true; // 以车辆前进方向为准，第 1 路位于左侧时设为 true
constexpr float YawDirection = 1.0f;  // 车体逆时针转动时偏航角应增大，方向相反则改为 -1
constexpr float WheelDiameterM = 0.100f; // 车轮直径，单位米
constexpr float TrackWidthM = 0.300f; // 左右车轮中心间距，单位米
constexpr float PulsesPerRev = 3200.0f; // 直接驱动、1.8 度步距角、16 细分时每圈的脉冲数
constexpr uint8_t MotorAcceleration = 200; // Emm 协议的加速度档位，并非转/分钟每秒
constexpr float MotorCommandScale = 1.0f; // 仅在驱动启用 S_Vel 的 0.1 转/分钟输入模式时设为 10
constexpr float MaxMotorRpm = 5000.0f; // 单轮物理转速上限，单位转/分钟
constexpr bool VelocityDirectionInverted = true; // 现象：速度模式前进指令实际后退，默认反转四轮速度方向
constexpr float SensorPitchM = 0.0126f; // 相邻灰度探头的中心间距，单位米
constexpr float DefaultSpeedRpm = 3000.0f; // 直线基础RPM；Watch运行时可调
constexpr float DefaultTurnSpeedRatio = 0.25f; // 最大转向需求时保留的速度比例
constexpr float DefaultMaxOmega = 1.2f; // 车体最大转向角速度，单位弧度/秒
constexpr float MaxGapM = 0.07f; // 按速度指令积分的缺口距离上限，并非实测地面行驶距离
constexpr float MaxGapS = 0.55f; // 允许短断线持续的最长时间，单位秒
constexpr float Kp = 12.0f; // 横向偏差的比例增益
constexpr float Kd = 0.08f; // 横向偏差变化率的微分增益
constexpr float YawRateDamping = 0.15f; // 偏航角速度的阻尼增益
constexpr float GapHeadingKp = 2.5f; // 短断线时保持航向的比例增益
constexpr uint32_t GrayPeriodMs = 5; // 灰度查询任务周期，单位毫秒
constexpr uint32_t ControlPeriodMs = 10; // 底盘控制周期，单位毫秒
constexpr uint32_t GrayTimeoutMs = 20; // 灰度有效数据的超时阈值，单位毫秒
constexpr uint32_t ImuTimeoutMs = 200; // 姿态有效数据的超时阈值，单位毫秒
constexpr uint32_t MotorFeedbackTimeoutMs = 200; // 单台电机转速反馈的超时阈值，单位毫秒
}
#endif
