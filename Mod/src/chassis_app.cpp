#include "chassis_app.h"
#include "bsp_uart.h"
#include "ch040.h"
#include "gray16.hpp"
#include "line_follow.hpp"
#include "step_motor.hpp"
#include "usart.h"

extern "C" {
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
}

namespace
{
constexpr uint32_t kGrayPeriodMs = LineConfig::GrayPeriodMs;
constexpr uint32_t kControlPeriodMs = LineConfig::ControlPeriodMs;
constexpr uint32_t kGrayTimeoutMs = LineConfig::GrayTimeoutMs;
constexpr uint32_t kImuTimeoutMs = LineConfig::ImuTimeoutMs;
constexpr uint32_t kMotorFeedbackTimeoutMs = LineConfig::MotorFeedbackTimeoutMs;
constexpr uint8_t kGraySensorId = LineConfig::GrayId;

struct ImuSnapshot
{
    float yaw_rad;
    uint32_t tick_ms;
    uint32_t sequence;
};

Gray16Parser gray_parser;
StepMotorBus motor_bus;
LineFollower line_follower;
ImuSnapshot imu_snapshot = {0.0f, 0, 0};
volatile bool gray_query_pending = false;
volatile uint32_t gray_timeout_count = 0;
uint32_t gray_query_tick_ms = 0;

osThreadId_t gray_task_handle;
osThreadId_t chassis_task_handle;
// 状态只由底盘任务整理后统一发布，串口回调仅更新各自的数据快照。
ChassisStatus status = {};

template <typename T>
T readShared(const volatile T &source)
{
    T result;
    taskENTER_CRITICAL();
    const volatile uint8_t *from = reinterpret_cast<const volatile uint8_t *>(&source);
    uint8_t *to = reinterpret_cast<uint8_t *>(&result);
    for (unsigned i = 0; i < sizeof(T); ++i) to[i] = from[i];
    taskEXIT_CRITICAL();
    return result;
}

void publishStatus()
{
    taskENTER_CRITICAL();
    volatile uint8_t *to = reinterpret_cast<volatile uint8_t *>(&g_chassis_status);
    const uint8_t *from = reinterpret_cast<const uint8_t *>(&status);
    for (unsigned i = 0; i < sizeof(status); ++i) to[i] = from[i];
    taskEXIT_CRITICAL();
}

float wrapPi(float angle)
{
    while (angle > WIT_PI) angle -= 2.0f * WIT_PI;
    while (angle < -WIT_PI) angle += 2.0f * WIT_PI;
    return angle;
}

bool olderThan(uint32_t now, uint32_t sample_time, uint32_t timeout)
{
    // 任务取得当前时间后，中断可能发布比该时间晚一个节拍的新数据。
    return (int32_t)(now - sample_time) > (int32_t)timeout;
}

Gray16Sample readGray()
{
    taskENTER_CRITICAL();
    Gray16Sample sample = gray_parser.sample();
    taskEXIT_CRITICAL();
    return sample;
}

ImuSnapshot readImu()
{
    taskENTER_CRITICAL();
    ImuSnapshot sample = imu_snapshot;
    taskEXIT_CRITICAL();
    return sample;
}

bool feedbackFresh(uint32_t now_ms)
{
    taskENTER_CRITICAL();
    bool fresh = true;
    for (uint8_t id = 1; id <= 4; ++id)
    {
        const MotorFeedback feedback = motor_bus.feedback(id);
        if (feedback.sequence == 0 || olderThan(now_ms, feedback.tick_ms, kMotorFeedbackTimeoutMs))
            fresh = false;
    }
    taskEXIT_CRITICAL();
    return fresh;
}

void motorRxCallback(UART_HandleTypeDef *huart, uint8_t *bytes, uint8_t size)
{
    if (huart == &huart2) motor_bus.onRx(bytes, size, HAL_GetTick());
}

void grayRxCallback(UART_HandleTypeDef *huart, uint8_t *bytes, uint8_t size)
{
    if (huart == &huart3 && gray_parser.feed(bytes, size, HAL_GetTick()) != 0)
        gray_query_pending = false;
}

void imuRxCallback(UART_HandleTypeDef *huart, uint8_t *bytes, uint8_t size)
{
    if (huart != &huart6) return;
    const uint32_t previous = ch040.validFrameCount();
    ch040.analyze_data(bytes, size);
    if (ch040.validFrameCount() == previous) return;

    imu_snapshot.yaw_rad = ch040.getYaw();
    imu_snapshot.tick_ms = HAL_GetTick();
    imu_snapshot.sequence = ch040.validFrameCount();
}

void GrayTask(void *)
{
    TickType_t wake_tick = xTaskGetTickCount();
    const uint8_t request[] = {0x57, kGraySensorId};

    for (;;)
    {
        BspUart_Service(&uart3_gray_inst);
        const uint32_t now_ms = HAL_GetTick();
        if (gray_query_pending && now_ms - gray_query_tick_ms >= 2 * kGrayPeriodMs)
        {
            gray_query_pending = false;
            ++gray_timeout_count;
        }
        if (!gray_query_pending)
        {
            gray_query_pending = true;
            if (BspUart_Transmit(uart3_gray_inst, request, sizeof(request)) == HAL_OK)
            {
                gray_query_tick_ms = now_ms;
            }
            else
            {
                gray_query_pending = false;
            }
        }
        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(kGrayPeriodMs));
    }
}

void ChassisTask(void *)
{
    TickType_t wake_tick = xTaskGetTickCount();
    uint32_t previous_tick_ms = HAL_GetTick();
    uint32_t previous_imu_tick_ms = 0;
    uint32_t previous_imu_sequence = 0;
    float previous_yaw_rad = 0.0f;
    float filtered_yaw_rate = 0.0f;
    uint8_t next_motor_id = 1;
    uint8_t consecutive_tx_failures = 0;
    uint32_t observed_reject_count = 0;
    bool was_running = false;
    bool stop_pending = true; // 主控复位后，先停止电机可能仍在执行的运动。

    for (;;)
    {
        BspUart_Service(&uart2_inst);
        BspUart_Service(&uart6_imu_inst);
        const uint32_t now_ms = HAL_GetTick();
        const float dt_s = (float)(now_ms - previous_tick_ms) * 0.001f;
        if (now_ms - previous_tick_ms > kControlPeriodMs + 2U) ++status.chassis_overrun_count;
        previous_tick_ms = now_ms;
        const Gray16Sample gray = readGray();
        const ImuSnapshot imu = readImu();

        const ChassisControl control = readShared(g_chassis_control);
        const bool parameters_valid = Chassis_ParametersValid(&control) != 0;
        const bool sensors_ready = gray.sequence != 0 && !olderThan(now_ms, gray.tick_ms, kGrayTimeoutMs) &&
                                   imu.sequence != 0 && !olderThan(now_ms, imu.tick_ms, kImuTimeoutMs);
        const bool motors_ready = feedbackFresh(now_ms);
        status.ready = parameters_valid && sensors_ready && motors_ready;
        status.applied_speed_rpm = parameters_valid ? control.speed_rpm : 0.0f;
        ++status.control_count;

        // 停止优先，同一周期同时请求启动和停止时只执行停止。
        if (control.stop != 0)
        {
            taskENTER_CRITICAL();
            g_chassis_control.stop = 0;
            g_chassis_control.start = 0;
            taskEXIT_CRITICAL();
            status.state = CHASSIS_IDLE;
            status.fault = CHASSIS_FAULT_NONE;
            stop_pending = true;
        }
        else if (control.start != 0 && !stop_pending)
        {
            g_chassis_control.start = 0;
            if (status.state != CHASSIS_RUNNING)
            {
                status.fault = !parameters_valid ? CHASSIS_FAULT_PARAMETER :
                               !sensors_ready ? CHASSIS_FAULT_SENSOR :
                               !motors_ready ? CHASSIS_FAULT_FEEDBACK : CHASSIS_FAULT_NONE;
                status.state = status.fault == CHASSIS_FAULT_NONE ? CHASSIS_RUNNING : CHASSIS_FAULT;
            }
        }

        status.gray_raw_bits = gray.bits;
        status.gray_frame_count = gray.sequence;

        if (imu.sequence != 0 && imu.sequence != previous_imu_sequence)
        {
            if (previous_imu_sequence != 0)
            {
                const uint32_t elapsed = imu.tick_ms - previous_imu_tick_ms;
                if (elapsed > 0 && elapsed < kImuTimeoutMs)
                {
                    const float rate = wrapPi(imu.yaw_rad - previous_yaw_rad) * 1000.0f /
                                       (float)elapsed;
                    filtered_yaw_rate = 0.7f * filtered_yaw_rate + 0.3f * rate;
                }
            }
            previous_yaw_rad = imu.yaw_rad;
            previous_imu_tick_ms = imu.tick_ms;
            previous_imu_sequence = imu.sequence;
        }

        if (status.state == CHASSIS_RUNNING)
        {
            if (!was_running)
            {
                line_follower.reset();
                observed_reject_count = motor_bus.reject_count;
                consecutive_tx_failures = 0;
            }
            was_running = true;
            LineInput input = {
                gray.bits,
                gray.sequence != 0 && !olderThan(now_ms, gray.tick_ms, kGrayTimeoutMs),
                imu.yaw_rad,
                filtered_yaw_rate,
                imu.sequence != 0 && !olderThan(now_ms, imu.tick_ms, kImuTimeoutMs),
                dt_s
            };

            LineCommand command = line_follower.step(input, control);
            status.line_error_m = command.line_error_m;
            status.line_status = (uint8_t)command.status;
            status.command_speed_mps = command.speed_mps;
            status.command_omega_rad_s = command.omega_rad_s;

            if (!parameters_valid)
            {
                status.fault = CHASSIS_FAULT_PARAMETER;
                status.state = CHASSIS_FAULT;
                stop_pending = true;
            }
            else if (motor_bus.reject_count != observed_reject_count)
            {
                observed_reject_count = motor_bus.reject_count;
                status.fault = CHASSIS_FAULT_REJECT;
                status.state = CHASSIS_FAULT;
                stop_pending = true;
            }
            else if (!motors_ready)
            {
                status.fault = CHASSIS_FAULT_FEEDBACK;
                status.state = CHASSIS_FAULT;
                stop_pending = true;
            }
            else if (command.status != LineStatus::Tracking &&
                     command.status != LineStatus::ShortGap)
            {
                status.fault = (command.status == LineStatus::NoSensor) ?
                    CHASSIS_FAULT_SENSOR : CHASSIS_FAULT_LINE;
                status.state = CHASSIS_FAULT;
                stop_pending = true;
            }
            else if (!motor_bus.setTwist(command.speed_mps, command.omega_rad_s,
                                         (uint8_t)control.acceleration))
            {
                if (++consecutive_tx_failures >= 3)
                {
                    status.fault = CHASSIS_FAULT_TX;
                    status.state = CHASSIS_FAULT;
                    stop_pending = true;
                }
            }
            else
            {
                consecutive_tx_failures = 0;
            }
        }
        else
        {
            if (was_running)
            {
                line_follower.reset();
                was_running = false;
                stop_pending = true;
            }
            status.command_speed_mps = 0.0f;
            status.command_omega_rad_s = 0.0f;
        }

        // 立即停车帧在发送忙时持续重试，直到 DMA 接受发送。
        if (stop_pending && motor_bus.stop()) stop_pending = false;
        if (status.state != CHASSIS_RUNNING)
        {
            status.command_speed_mps = 0.0f;
            status.command_omega_rad_s = 0.0f;
        }
        status.stop_pending = stop_pending;
        status.sent_left_rpm = motor_bus.sentLeftRpm();
        status.sent_right_rpm = motor_bus.sentRightRpm();
        status.gray_timeout_count = gray_timeout_count;
        status.imu_yaw_rad = imu.yaw_rad;
        status.imu_yaw_deg = imu.yaw_rad * 57.2957795f;
        status.imu_frame_count = imu.sequence;
        status.motor_tx_busy_count = BspUart_GetTxBusyCount(&uart2_inst);
        status.motor_ack_count = motor_bus.ack_count;
        status.motor_reject_count = motor_bus.reject_count;
        taskENTER_CRITICAL();
        for (uint8_t id = 1; id <= 4; ++id)
        {
            const MotorFeedback feedback = motor_bus.feedback(id);
            status.motor_feedback_rpm[id - 1] = feedback.rpm;
            status.motor_feedback_count[id - 1] = feedback.sequence;
        }
        taskEXIT_CRITICAL();

        publishStatus();

        // 错开速度指令的发送时段，每个控制周期轮询一台电机的实际转速。
        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(kControlPeriodMs / 2));
        if (!stop_pending)
        {
            motor_bus.requestVelocity(next_motor_id);
            next_motor_id = (next_motor_id == 4) ? 1 : (uint8_t)(next_motor_id + 1);
        }
        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(kControlPeriodMs / 2));
    }
}


} // 内部实现结束，上层只通过 chassis_app.h 使用底盘。

extern "C" void Chassis_GetStatus(ChassisStatus *result)
{
    if (result != nullptr) *result = readShared(g_chassis_status);
}

extern "C" void Chassis_Init(void)
{
    ch040.setYawDir(LineConfig::YawDirection);
    BspUart_InstRegist(&uart2_inst, &huart2, 64, BspUartType_DMA, BspUartType_DMA,
                       motorRxCallback);
    BspUart_InstRegist(&uart3_gray_inst, &huart3, 64, BspUartType_DMA, BspUartType_DMA,
                       grayRxCallback);
    BspUart_InstRegist(&uart6_imu_inst, &huart6, 64, BspUartType_DMA, BspUartType_DMA,
                       imuRxCallback);

    osThreadAttr_t gray_attr = {};
    gray_attr.name = "GrayTask";
    gray_attr.stack_size = 512 * 4;
    gray_attr.priority = osPriorityAboveNormal;
    gray_task_handle = osThreadNew(GrayTask, nullptr, &gray_attr);

    osThreadAttr_t chassis_attr = {};
    chassis_attr.name = "ChassisTask";
    chassis_attr.stack_size = 768 * 4;
    chassis_attr.priority = osPriorityHigh;
    chassis_task_handle = osThreadNew(ChassisTask, nullptr, &chassis_attr);

    if (gray_task_handle == nullptr || chassis_task_handle == nullptr)
        Error_Handler();
}
