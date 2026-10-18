#include "cpp_main.h"
#include "main.h"

extern "C" {
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
}

// 上层逻辑入口，每 20ms 调用一次；把按键、比赛状态机或上位机指令写在这里。
// 通过 Chassis_Start / Chassis_Stop / Chassis_SetSpeedRpm 控制底盘。
// 不要在这里直接调用 Emm 发送接口，避免与底盘任务争用串口。
static void UserControlUpdate()
{
    // 示例：在启动按键的触发分支中调用 Chassis_Start()。
    // 示例：需要调整速度时调用 Chassis_SetSpeedRpm(50.0f)。
    // 示例：在停止按键的触发分支中调用 Chassis_Stop()。
    // 默认不自动启动，可直接在 Keil Watch 中修改 g_chassis_control。
}

static void MainLogicTask(void *)
{
    TickType_t wake_tick = xTaskGetTickCount();
    for (;;)
    {
        UserControlUpdate();
        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(20));
    }
}

extern "C" void cpp_main(void)
{
    Chassis_Init();

    osThreadAttr_t attr = {};
    attr.name = "MainLogicTask";
    attr.stack_size = 512 * 4;
    attr.priority = osPriorityNormal;
    if (osThreadNew(MainLogicTask, nullptr, &attr) == nullptr) Error_Handler();
}
