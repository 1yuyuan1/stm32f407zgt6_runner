#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include "chassis_app.h"
#include "bsp_uart.h"
#include "usart.h"
extern "C" {
#include "cmsis_os2.h"
#include "task.h"
}

static DMA_HandleTypeDef dma;
UART_HandleTypeDef huart2 = {0, 0, &dma}, huart3 = {0, 0, &dma}, huart6 = {0, 0, &dma};
static uint32_t tick_ms;
static void (*chassis_task)(void *);
static uint8_t last_motion[64];
static unsigned last_motion_size;
struct TestComplete {};

extern "C" uint32_t HAL_GetTick(void) { return tick_ms; }
extern "C" void Error_Handler(void) { std::abort(); }
extern "C" TickType_t xTaskGetTickCount(void) { return tick_ms; }
extern "C" osThreadId_t osThreadNew(void (*entry)(void *), void *, const osThreadAttr_t *attr)
{
    if (std::strcmp(attr->name, "ChassisTask") == 0) chassis_task = entry;
    return reinterpret_cast<void *>(1);
}
extern "C" HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
    if (u->gState != HAL_UART_STATE_READY) return HAL_BUSY;
    u->gState = HAL_UART_STATE_BUSY_TX;
    if (u == &huart2 && n > 4)
    {
        assert(n <= sizeof(last_motion));
        std::memcpy(last_motion, b, n);
        last_motion_size = n;
    }
    return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{ return HAL_UART_Transmit_DMA(u, b, n); }
extern "C" HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u, uint8_t *b, uint16_t n, uint32_t)
{ return HAL_UART_Transmit_DMA(u, b, n); }
extern "C" HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *, uint16_t)
{ u->RxState = HAL_UART_STATE_BUSY_RX; return HAL_OK; }
extern "C" HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{ return HAL_UARTEx_ReceiveToIdle_DMA(u, b, n); }
extern "C" HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{ u->gState = HAL_UART_STATE_READY; return HAL_OK; }

static void feedSensors()
{
    uint8_t gray[] = {0x75, 0x7F, 0xFE, 0x03};
    uart3_gray_inst.rx_callback(&huart3, gray, sizeof gray);
    uint8_t imu[] = {0x55, 0x53, 0, 0, 0, 0, 0, 0, 0, 0, 0xA8};
    uart6_imu_inst.rx_callback(&huart6, imu, sizeof imu);
    for (uint8_t id = 1; id <= 4; ++id)
    {
        uint8_t reply[] = {id, 0x35, 0, 0, 0, 0x6B};
        uart2_inst.rx_callback(&huart2, reply, sizeof reply);
    }
}

// 在每个控制周期发布状态后检查结果，继续同一个底盘任务验证状态转换。
extern "C" void vTaskDelayUntil(TickType_t *wake, TickType_t increment)
{
    for (auto u : {&huart2, &huart3, &huart6})
        if (u->gState == HAL_UART_STATE_BUSY_TX)
        {
            u->gState = HAL_UART_STATE_READY;
            HAL_UART_TxCpltCallback(u);
        }
    *wake += increment;
    tick_ms = *wake;
    if (tick_ms % 10 == 5)
    {
        ChassisStatus s;
        Chassis_GetStatus(&s);
        switch (s.control_count)
        {
        case 1:
            assert(s.state == CHASSIS_IDLE && last_motion_size == 25);
            assert(s.ready == 1 && s.gray_frame_count > 0 && s.imu_frame_count > 0);
            assert(s.gray_raw_bits == 0xFE7F && s.gray_active_bits == 0x0180);
            assert(g_chassis_control.speed_rpm == 3000.0f);
            Chassis_Start();
            break;
        case 2:
            assert(s.state == CHASSIS_RUNNING && g_chassis_control.start == 0);
            assert(s.sent_left_rpm == 3000 && s.sent_right_rpm == 3000);
            assert(last_motion_size == 37 && last_motion[7] == 0x0B && last_motion[8] == 0xB8);
            assert(last_motion[6] == 0 && last_motion[14] == 1);
            assert(Chassis_SetSpeedRpm(100));
            break;
        case 3:
            assert(s.sent_left_rpm == 100 && s.applied_speed_rpm == 100);
            g_chassis_control.speed_rpm = 5001; // 模拟调试器直接写入非法值
            break;
        case 4:
            assert(s.state == CHASSIS_FAULT && s.fault == CHASSIS_FAULT_PARAMETER);
            assert(last_motion_size == 25 && s.sent_left_rpm == 0);
            Chassis_Start();
            break;
        case 5:
            assert(s.state == CHASSIS_FAULT && s.fault == CHASSIS_FAULT_PARAMETER);
            assert(Chassis_SetSpeedRpm(100));
            break;
        case 6:
            assert(s.state == CHASSIS_FAULT); // 参数恢复不能自动起步
            Chassis_Start();
            break;
        case 7:
            assert(s.state == CHASSIS_RUNNING && s.sent_left_rpm == 100);
            Chassis_Start();
            Chassis_Stop();
            break;
        case 8:
            assert(s.state == CHASSIS_IDLE && s.sent_left_rpm == 0);
            assert(g_chassis_control.start == 0 && g_chassis_control.stop == 0);
            Chassis_Start();
            break;
        case 9:
            assert(s.state == CHASSIS_RUNNING);
            assert(Chassis_SetSpeedRpm(0));
            break;
        case 10:
            assert(s.state == CHASSIS_RUNNING && s.sent_left_rpm == 0 && s.sent_right_rpm == 0);
            break;
        case 13:
            assert(s.state == CHASSIS_FAULT && s.fault == CHASSIS_FAULT_SENSOR);
            assert(last_motion_size == 25);
            throw TestComplete{};
        default: break;
        }
    }
    if (g_chassis_status.control_count < 10) feedSensors();
    assert(tick_ms < 200);
}

int main()
{
    Chassis_Init();
    assert(chassis_task != nullptr);
    feedSensors();
    try { chassis_task(nullptr); }
    catch (const TestComplete &) {
        std::puts("PASS: real chassis task, live RPM, stop priority, fault latch and sensor timeout");
        return 0;
    }
    return 1;
}
