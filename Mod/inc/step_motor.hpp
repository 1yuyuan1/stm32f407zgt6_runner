#ifndef STEP_MOTOR_HPP
#define STEP_MOTOR_HPP

#include "Emm_V5.h"
#include "bsp_uart.h"

class StepMotor
{
public:
    StepMotor();
    ~StepMotor();
    double _wheel_diameter = 0.080 ; // 轮子直径，单位为米
    void SetLinearPos(float target_pos,float target_vel); // 设置线性位置，单位为米
    void SetLinearVel(float target_vel); // 设置线性速度，单位为米/秒
private:
    uint8_t id; // 电机ID
    uint8_t mode = 0; // 控制模式，0-位置控制，1-速度控制，2-电流控制
    uint8_t dir = 0; // 方向，0-正转，1-反转
    uint8_t acc = 0; // 加速度,0-255,0-表示立马启动
    uint32_t clk;//脉冲数（16细分下发送3200个脉冲电机转一圈）
    uint8_t raf = 0; // 相位/绝对标志，false为相对运动，true为绝对值运动
    uint8_t snf = 0;//多机同步标志 ，false为不启用，true为启用
    
};



#endif

