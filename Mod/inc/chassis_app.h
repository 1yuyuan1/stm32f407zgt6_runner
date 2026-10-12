#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CHASSIS_IDLE = 0,       // 待机，持续采集数据
    CHASSIS_RUNNING = 1,    // 正在巡线
    CHASSIS_FAULT = 2       // 故障锁定，排除原因后重新启动
} ChassisState;

typedef enum {
    CHASSIS_FAULT_NONE = 0,
    CHASSIS_FAULT_SENSOR = 1,   // 灰度或姿态数据过期
    CHASSIS_FAULT_LINE = 2,     // 丢线或白线范围异常
    CHASSIS_FAULT_FEEDBACK = 3, // 电机反馈过期
    CHASSIS_FAULT_TX = 4,       // 连续发送失败
    CHASSIS_FAULT_REJECT = 5,   // 电机拒绝控制指令
    CHASSIS_FAULT_PARAMETER = 6 // 调试参数超出范围或不是有限数值
} ChassisFault;

// Keil Watch 可写入口：参数在下一控制周期读取；多个参数建议在待机时修改。
// 启动后直线、弯道和短虚线均使用同一个基础转速，不再按偏差自动降速。
typedef struct {
    uint32_t start;        // 写 1 请求启动，任务处理后清零
    uint32_t stop;         // 写 1 请求停止，优先于启动，处理后清零
    float speed_rpm;      // 基础转速，0～5000RPM；0 表示保持运行状态但四轮零速
    float kp;             // 横向偏差比例增益，输入米，输出弧度/秒
    float kd;             // 横向偏差微分增益
    float yaw_damping;    // 偏航角速度阻尼增益
    float gap_heading_kp; // 短断线时的航向保持增益
    float max_omega;      // 目标角速度上限，单位弧度/秒
    uint32_t acceleration; // Emm 加速度档位，0～255；0 为直接启动
} ChassisControl;

// Keil Watch 只读入口：按车辆前进方向统一命令符号，反馈保留驱动器原始符号。
typedef struct {
    ChassisState state;
    ChassisFault fault;
    uint32_t line_status;       // 0 巡线，1 短断线，2 数据过期，3 丢线，4 歧义，5 参数非法
    uint32_t ready;             // 灰度、姿态和四轮反馈都新鲜且参数合法时为 1
    uint32_t stop_pending;      // 停车帧尚未被发送接口接受时为 1
    uint32_t control_count;     // 底盘控制周期计数
    float applied_speed_rpm;    // 本周期通过检查的基础转速
    float command_speed_mps;   // 算法目标前进速度，尚未经过单轮限速
    float command_omega_rad_s; // 算法目标角速度，左转为正
    float line_error_m;         // 白线偏差，右侧为正
    float sent_left_rpm;        // 最近成功提交发送的左轮指令转速
    float sent_right_rpm;       // 最近成功提交发送的右轮指令转速
    float motor_feedback_rpm[4]; // ID 1～4 的实测转速，保留驱动器符号
    uint32_t motor_feedback_count[4]; // 各电机有效转速帧数，停车时也应增长
    uint16_t gray_raw_bits;     // 原始 16 路灰度数据
    uint32_t gray_frame_count;  // 有效灰度帧累计次数
    uint32_t gray_timeout_count;
    float imu_yaw_rad;
    float imu_yaw_deg;
    uint32_t imu_frame_count;   // 通过校验的欧拉角帧累计次数
    uint32_t motor_tx_busy_count;
    uint32_t motor_ack_count;
    uint32_t motor_reject_count;
    uint32_t chassis_overrun_count; // 控制间隔超过 12ms 的次数
} ChassisStatus;

extern volatile ChassisControl g_chassis_control;
extern volatile ChassisStatus g_chassis_status;

// 在调度器已经运行后初始化一次；仅 cpp_main 调用。
void Chassis_Init(void);
// 以下接口供上层任务使用，只发布请求，不在调用者任务中发送电机指令。
void Chassis_Start(void);
void Chassis_Stop(void);
// 转速合法返回 1；越界、负数、无穷或非数返回 0，保留原设置。
int Chassis_SetSpeedRpm(float rpm);
// 上层任务获取一致状态快照；调试器可直接展开 g_chassis_status。
void Chassis_GetStatus(ChassisStatus *status);
// 参数检查用于底盘任务和本机测试；空指针或非法参数返回 0。
int Chassis_ParametersValid(const ChassisControl *control);

#ifdef __cplusplus
}
#endif
#endif
