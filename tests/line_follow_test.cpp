#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include "gray16.hpp"
#include "line_follow.hpp"
#include "step_motor.hpp"
#include "ch040.h"
#include "bsp_uart.h"
#include "Emm_V5.h"

static uint32_t tick_ms;
static uint8_t *dma_bytes;
static uint16_t dma_size;
static UART_HandleTypeDef *dma_uart;
extern "C" uint32_t HAL_GetTick(void) { return tick_ms; }
extern "C" void Error_Handler(void) { std::abort(); }
extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *bytes, uint16_t size)
{
    if (uart->gState != HAL_UART_STATE_READY) return HAL_BUSY;
    uart->gState = HAL_UART_STATE_BUSY_TX;
    dma_bytes = bytes;
    dma_size = size;
    dma_uart = uart;
    return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{ return HAL_UART_Transmit_DMA(u, b, n); }
extern "C" HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *b, uint16_t n, uint32_t)
{ return HAL_UART_Transmit_DMA(u, b, n); }
extern "C" HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *, uint16_t)
{ u->RxState = HAL_UART_STATE_BUSY_RX; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{ return HAL_UARTEx_ReceiveToIdle_DMA(u,b,n); }
extern "C" HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{ u->gState = HAL_UART_STATE_READY; return HAL_OK; }

static void completeTx()
{
    dma_uart->gState = HAL_UART_STATE_READY;
    HAL_UART_TxCpltCallback(dma_uart);
}

static void testGrayAndImu()
{
    Gray16Parser gray;
    const uint8_t reply[] = {0x75, 0x80, 0x01, 0x03};
    assert(gray.feed(reply, 2, 1) == 0);
    assert(gray.feed(reply + 2, 2, 2) == 1);
    assert(gray.sample().bits == 0x0180 && gray.sample().sequence == 1);
    const uint8_t bad[] = {0x75, 0x00, 0x00, 0x04};
    gray.feed(bad, sizeof bad, 3);
    assert(gray.sample().sequence == 1 && gray.sample().tick_ms == 2);
    const uint8_t headerPayload[] = {0x75, 0x75, 0x03, 0x03};
    gray.feed(headerPayload, sizeof headerPayload, 4);
    assert(gray.sample().bits == 0x0375);
    const uint8_t corruptThenGood[] = {0x75, 0xFF, 0x75, 0x80, 0x01, 0x03};
    gray.feed(corruptThenGood, sizeof corruptThenGood, 5);
    assert(gray.sample().bits == 0x0180 && gray.sample().sequence == 3);
    gray.feed(reply, 2, 6);
    gray.feed(reply, sizeof reply, 30); // 丢弃超时的残缺帧
    assert(gray.sample().sequence == 4);

    ch040_t imu;
    uint8_t angle[] = {0x55,0x53,0,0,0,0,0,0x40,0,0,0};
    for (unsigned i=0; i<10; ++i) angle[10] += angle[i];
    imu.analyze_data(angle, 4);
    assert(imu.validFrameCount() == 0);
    imu.analyze_data(angle+4, 7);
    assert(imu.validFrameCount() == 1 && std::fabs(imu.getYawDeg()-90.0f) < 0.01f);
    angle[10] ^= 1;
    imu.analyze_data(angle, 11);
    assert(imu.validFrameCount() == 1); // 校验错误的帧不得刷新有效数据时间戳
}

static void testLine()
{
    LineFollower follower;
    ChassisControl control = {0, 0, 48.0f, LineConfig::Kp, LineConfig::Kd,
                              LineConfig::YawRateDamping, LineConfig::GapHeadingKp,
                              LineConfig::MaxOmega, LineConfig::MotorAcceleration};
    LineInput input = {0x0180, true, 0.0f, 0.0f, true, 0.01f};
    auto command = follower.step(input, control);
    assert(command.status == LineStatus::Tracking && std::fabs(command.omega_rad_s) < 0.001f);
    assert(command.speed_mps > 0.0f);
    input.gray_bits = 0x0030; // 白线位于左侧
    follower.reset();
    command = follower.step(input, control);
    assert(command.omega_rad_s > 0.0f);
    input.gray_bits = 0x0C00; // 白线位于右侧
    follower.reset();
    assert(follower.step(input, control).omega_rad_s < 0.0f);
    follower.channel_1_on_left = false;
    follower.reset();
    assert(follower.step(input, control).omega_rad_s > 0.0f);
    follower.channel_1_on_left = true;
    follower.line_bit_is_one = false;
    input.gray_bits = (uint16_t)~0x0180;
    follower.reset();
    assert(std::fabs(follower.step(input, control).line_error_m) < 0.001f);
    follower.line_bit_is_one = true;
    input.gray_bits = 0x0180;
    follower.reset();
    follower.step(input, control);
    input.gray_bits = 0;
    input.yaw_rad = 0.05f;
    command = follower.step(input, control);
    assert(command.status == LineStatus::ShortGap && command.omega_rad_s < 0);
    for (unsigned i=0; i<60; ++i) command = follower.step(input, control);
    assert(command.status == LineStatus::NoLine && command.speed_mps == 0);
    input.gray_bits = 0xFFFF;
    assert(follower.step(input, control).status == LineStatus::Ambiguous);
    input.gray_fresh = false;
    assert(follower.step(input, control).status == LineStatus::NoSensor);
    input.gray_fresh = true;
    input.imu_fresh = false;
    assert(follower.step(input, control).status == LineStatus::NoSensor);
    input.imu_fresh = true;
    follower.reset();
    input.gray_bits = 0;
    assert(follower.step(input, control).status == LineStatus::NoLine);
    input.gray_bits = 0x0180;
    follower.step(input, control);
    input.gray_bits = 0;
    input.yaw_rate_rad_s = 1.0f;
    assert(follower.step(input, control).status == LineStatus::NoLine); // 转弯丢线时不得盲目跨越缺口
}

// 验证统一转速、在线调参与非法值处理，不依赖单片机硬件。
static void testRuntimeControl()
{
    assert(g_chassis_control.start == 0 && g_chassis_control.stop == 0);
    assert(g_chassis_control.speed_rpm == 3000.0f);
    assert(Chassis_SetSpeedRpm(5000));
    assert(!Chassis_SetSpeedRpm(5001));
    assert(!Chassis_SetSpeedRpm(-1));
    assert(!Chassis_SetSpeedRpm(std::numeric_limits<float>::infinity()));
    assert(!Chassis_SetSpeedRpm(std::numeric_limits<float>::quiet_NaN()));
    assert(g_chassis_control.speed_rpm == 5000);
    assert(Chassis_SetSpeedRpm(3000));
    Chassis_Start();
    Chassis_Stop();
    assert(g_chassis_control.start == 1 && g_chassis_control.stop == 1);
    g_chassis_control.start = g_chassis_control.stop = 0;

    ChassisControl control = {0, 0, 3000.0f, 12, 0.08f, 0.15f, 2.5f, 1.2f, 200};
    assert(Chassis_ParametersValid(&control));
    LineFollower follower;
    LineInput input = {0x0180, true, 0, 0, true, 0.01f};
    const float expected = 3000.0f * 3.14159265358979323846f * 0.1f / 60.0f;
    auto straight = follower.step(input, control);
    assert(std::fabs(straight.speed_mps - expected) < 0.0001f);
    input.gray_bits = 0x0030;
    auto curve = follower.step(input, control);
    assert(curve.speed_mps == straight.speed_mps && curve.omega_rad_s > 0);

    // 速度变更在下一次算法调用生效；短缺口也使用同一速度。
    control.speed_rpm = 100;
    follower.reset();
    input.gray_bits = 0x0180;
    straight = follower.step(input, control);
    input.gray_bits = 0;
    auto gap = follower.step(input, control);
    assert(gap.status == LineStatus::ShortGap && gap.speed_mps == straight.speed_mps);
    follower.reset();
    control.speed_rpm = 3000;
    input.gray_bits = 0x0180;
    follower.step(input, control);
    input.gray_bits = 0;
    // 高速下一个周期就可能超过 7cm，不能为了维持高速而绕过缺口限制。
    assert(follower.step(input, control).status == LineStatus::NoLine);

    control.speed_rpm = 0;
    input.gray_bits = 0x0030;
    follower.reset();
    auto zero = follower.step(input, control);
    assert(zero.speed_mps == 0 && zero.omega_rad_s == 0);
    control.speed_rpm = 5001;
    assert(follower.step(input, control).status == LineStatus::InvalidParameter);
    control.speed_rpm = 100;
    control.kp = std::numeric_limits<float>::quiet_NaN();
    assert(!Chassis_ParametersValid(&control));
    control.kp = 12;
    control.acceleration = 256;
    assert(!Chassis_ParametersValid(&control));
    assert(!Chassis_ParametersValid(nullptr));
}

static void testMotorAndDma()
{
    DMA_HandleTypeDef dma = {};
    UART_HandleTypeDef uart = {HAL_UART_STATE_READY,HAL_UART_STATE_READY,&dma};
    BspUart_InstRegist(&uart2_inst, &uart, 64, BspUartType_DMA, BspUartType_DMA, nullptr);
    StepMotorBus bus;
    for (int n=0; n<100; ++n)
    {
        assert(bus.setTwist(0.25f, 0));
        assert(dma_size == 37 && MMCL_count == 0); // 重复发送不得累积旧命令
        assert(dma_bytes[0] == 0 && dma_bytes[1] == 0xAA && dma_bytes[3] == 37);
        for (unsigned motor=0; motor<4; ++motor)
        {
            const unsigned offset = 4 + motor*8;
            assert(dma_bytes[offset] == motor+1 && dma_bytes[offset+1] == 0xF6);
            assert(dma_bytes[offset+2] == (motor%2 == 0 ? 1 : 0));
            assert(dma_bytes[offset+4] == 48);
        }
        uint8_t preserved[37];
        std::memcpy(preserved, dma_bytes, 37);
        assert(!bus.setTwist(-0.25f,0)); // 串口忙时不得覆盖尚在发送的旧帧
        assert(std::memcmp(preserved, dma_bytes, 37) == 0);
        completeTx();
        tick_ms += 10;
    }
    assert(bus.setTwist(0.25f, 1.0f));
    assert(dma_bytes[8] < dma_bytes[16]); // 逆时针左转时，左轮速度应低于右轮
    completeTx();
    assert(bus.stop() && dma_size == 25);
    for (unsigned m=0;m<4;++m) assert(dma_bytes[5+m*5] == 0xFE);
    completeTx();
    assert(bus.moveStraight(0.314159265f,0.25f) && dma_size == 57);
    assert(dma_bytes[12] == 0x0C && dma_bytes[13] == 0x80); // 每圈 3200 个脉冲
    completeTx();
    // 检查线上实际编码，确保 3000/5000RPM 未被旧限速或整数截断。
    const float rpm_to_mps = 3.14159265358979323846f * 0.1f / 60.0f;
    assert(bus.setTwist(3000 * rpm_to_mps, 0, 123));
    for (unsigned m = 0; m < 4; ++m)
    {
        unsigned offset = 4 + 8 * m;
        assert(((dma_bytes[offset+3] << 8) | dma_bytes[offset+4]) == 3000);
        assert(dma_bytes[offset+5] == 123);
    }
    assert(bus.sentLeftRpm() == 3000 && bus.sentRightRpm() == 3000);
    completeTx();
    assert(bus.setTwist(5000 * rpm_to_mps, 0));
    assert(bus.sentLeftRpm() == 5000 && bus.sentRightRpm() == 5000);
    completeTx();
    assert(bus.setTwist(5000 * rpm_to_mps, 10));
    assert(bus.sentLeftRpm() < bus.sentRightRpm() && bus.sentRightRpm() == 5000);
    for (unsigned m = 0; m < 4; ++m)
    {
        unsigned offset = 4 + 8 * m;
        assert(((dma_bytes[offset+3] << 8) | dma_bytes[offset+4]) <= 5000);
    }
    completeTx();
    assert(!bus.setTwist(std::numeric_limits<float>::infinity(), 0));
    assert(bus.moveStraight(0.314159265f, 5000 * rpm_to_mps));
    assert(((dma_bytes[7] << 8) | dma_bytes[8]) == 5000);
    completeTx();
    assert(bus.stop());
    assert(bus.sentLeftRpm() == 0 && bus.sentRightRpm() == 0);
    completeTx();
    const uint8_t speed[] = {1,0x35,1,0,48,0x6B};
    bus.onRx(speed,3,100);
    bus.onRx(speed+3,3,101);
    assert(bus.feedback(1).rpm == -48 && bus.feedback(1).sequence == 1);
    const uint8_t reject[] = {1,0xF6,0xE2,0x6B};
    bus.onRx(reject,4,110);
    assert(bus.reject_count == 1);
    const uint8_t queryReject[] = {2,0x35,0xE2,0x6B};
    bus.onRx(queryReject,4,120);
    assert(bus.reject_count == 2);

    for (int n=0;n<40;++n) Emm_V5_MMCL_Vel_Control(1,1,48,0,0);
    assert(Emm_V5_Multi_Motor_Cmd(0) && dma_size == 325); // 发送长度不得因使用八位整数而截断
    assert(dma_bytes[2] == 1 && dma_bytes[3] == 69);
    completeTx();
    for (int n=0;n<100;++n) Emm_V5_MMCL_Vel_Control(1,1,48,0,0);
    assert(!Emm_V5_Multi_Motor_Cmd(0) && MMCL_count == 0); // 组帧溢出时应拒绝发送
    assert(bus.setTwist(0.25f,0));
    tick_ms += 31;
    BspUart_Service(&uart2_inst); // 恢复卡住的 DMA，避免停车帧一直无法发送
    assert(bus.stop());
    completeTx();
}

int main()
{
    testGrayAndImu();
    testLine();
    testRuntimeControl();
    testMotorAndDma();
    std::puts("PASS: sensor framing, freshness, steering/gap control, motor packets and DMA ownership");
}
