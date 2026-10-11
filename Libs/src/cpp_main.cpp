#include "cpp_main.h"
#include "servo.hpp"
#include "main.h"
#include "stm32f407xx.h"
#include "ch040.h"

// 引入 CubeMX 生成的 C 语言全局变量 (必须用 extern "C")
extern "C" {
    #include "tim.h"  // 这里面有 extern TIM_HandleTypeDef htim1;
    #include "Emm_V5.h"
    #include "usart.h" // 这里面有 extern UART_HandleTypeDef huart2,同时也定义了串口实例uart1_inst
    #include "bsp_uart.h"
	#include "cmsis_os2.h"
	#include "FreeRTOS.h"
	#include "task.h"
}
	

Servo myServo; // 创建一个舵机对象
float a=0;
float b=3200;
float pos=12800;
bool speed_mode = false;
bool pos_mode = false;
bool stop = false;
bool dir = false;
bool servo_enabled = false;

volatile uint32_t imu_rx_count = 0;
volatile uint8_t imu_rx_len = 0;
volatile uint8_t imu_raw[32] = {0};

volatile float imu_yaw_rad = 0.0f;
volatile float imu_yaw_deg = 0.0f;

static osThreadId_t chassisTaskHandle;
static osThreadId_t mainLogicTaskHandle;

static void ChassisTask(void *argument);
static void MainLogicTask(void *argument);



void myRxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size){}
void ImuRxCallback(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size)
{
    if (huart != &huart6)
    {
        return;
    }

    imu_rx_count++;
    imu_rx_len = size;

    uint8_t copy_len = size;
    if (copy_len > 32)
    {
        copy_len = 32;
    }

    for (uint8_t i = 0; i < copy_len; i++)
    {
        imu_raw[i] = rxData[i];
    }

    ch040.analyze_data(rxData, size);

    imu_yaw_rad = ch040.getYaw();
    imu_yaw_deg = imu_yaw_rad * 57.2957795f;
}

void cpp_main(void) {

  BspUart_InstRegist(&uart1_inst, &huart1, 64, BspUartType_DMA, BspUartType_DMA, myRxCallback); // 注册串口实例，使用DMA模式接收
  BspUart_InstRegist(&uart6_imu_inst, &huart6, 64, BspUartType_DMA, BspUartType_DMA, ImuRxCallback); // 注册串口实例，使用DMA模式接收
  BspUart_InstRegist(&uart2_inst, &huart2, 64, BspUartType_DMA, BspUartType_DMA, myRxCallback); // 注册串口实例，使用DMA模式接收
  // Emm_V5_En_Control(4, true, false); // 上电使能电机，保持默认参数
    
//   /**********************************************************
// ***	上电延时500毫秒等待闭环初始化完毕
// **********************************************************/	
// HAL_Delay(500);
// Emm_V5_Vel_Control(4, 0, 100, 10, 0);
// Emm_V5_Pos_Control(4,0,100,0,3200,0,0);
// HAL_Delay(500);
// while(rxFrameFlag == false); rxFrameFlag = false;
// //  Emm_V5_Vel_Control(1, 0, 1500, 10, 0);
// 	HAL_Delay(500);

   	// while(rxCmd[1] != 0xFD || rxCmd[2] != 0x9F); rxFrameFlag = false;

    // myServo.Init(&htim1, TIM_CHANNEL_1); // 初始化舵机，使用 TIM1 的通道 1
    // myServo.Enable(); // 启动舵机输出

    //创建底盘控制任务
    osThreadAttr_t chassisTaskAttr = {};
    chassisTaskAttr.name = "ChassisTask";
    chassisTaskAttr.stack_size = 768 * 4;
    chassisTaskAttr.priority = osPriorityHigh;
    chassisTaskHandle = osThreadNew(ChassisTask, NULL, &chassisTaskAttr);
  
    // 创建主逻辑任务
    osThreadAttr_t mainLogicTaskAttr = {};
    mainLogicTaskAttr.name = "MainLogicTask";
    mainLogicTaskAttr.stack_size = 1024 * 4;
    mainLogicTaskAttr.priority = osPriorityNormal;
    mainLogicTaskHandle = osThreadNew(MainLogicTask, NULL, &mainLogicTaskAttr);

    //检查任务是否创建成功
    if (chassisTaskHandle == NULL || mainLogicTaskHandle == NULL)
    {
        Error_Handler();
    }

    // 超级死循环（代替 main.c 里面的 while(1)）
    while (1)
     {
      // myServo.SetAngle(a); // 设置舵机角度为 45 度
      if (speed_mode)
      {
        Emm_V5_Vel_Control(4, 0, 100, 10, 0);
        HAL_Delay(500);
        speed_mode = false;
        continue;
      }
      if (pos_mode)
      {
        /*分开控制*/
        // Emm_V5_Pos_Control(4,0,100,0,3200,0,0);
        // HAL_Delay(500);
        // Emm_V5_Pos_Control(2,0,100,0,3200,0,0);
        // HAL_Delay(500);
        // Emm_V5_Pos_Control(1,1,100,0,3200,0,0);
        // HAL_Delay(500);
        // Emm_V5_Pos_Control(3,1,100,0,3200,0,0);
        // HAL_Delay(500);
//        Emm_V5_Pos_Control(5,dir,100,0,b,0,0);
//        HAL_Delay(500);
        /*同步控制*/
        // Emm_V5_MMCL_Pos_Control(1,1,100,0,pos,0,0);
        // Emm_V5_MMCL_Pos_Control(3,1,100,0,pos,0,0);
        // Emm_V5_MMCL_Pos_Control(2,0,100,0,pos,0,0);
        // Emm_V5_MMCL_Pos_Control(4,0,100,0,pos,0,0);
        Emm_V5_MMCL_Pos_Control(1,1,5000,0,pos,0,0);
        Emm_V5_MMCL_Pos_Control(3,1,5000,0,pos,0,0);
        Emm_V5_MMCL_Pos_Control(2,0,5000,0,pos,0,0);
        Emm_V5_MMCL_Pos_Control(4,0,5000,0,pos,0,0);
        Emm_V5_Multi_Motor_Cmd(0);
        HAL_Delay(500);
        pos_mode = false;
        continue;
      }
      if (stop)
      {
        Emm_V5_Stop_Now(4, false);
        HAL_Delay(500);
        stop = false;
        continue;
      }
    }
}
static void ChassisTask(void *argument)
{
    uint32_t lastTick = osKernelGetTickCount();

    while (1)
    {
        uint32_t nowTick = osKernelGetTickCount();
        uint32_t dt = nowTick - lastTick;
        lastTick = nowTick;

        // 这里以后放底盘 5ms 控制
        // 例如：
        // 1. 读取陀螺仪 yaw
        // 2. 更新里程计
        // 3. 麦轮运动学解算
        // 4. 下发四个电机速度

        osDelay(5);
    }
}

static void MainLogicTask(void *argument)
{
    while (1)
    {
        if (speed_mode)
        {
            Emm_V5_Vel_Control(4, 0, 100, 10, 0);
            osDelay(500);
            speed_mode = false;
        }

        if (pos_mode)
        {
            Emm_V5_Pos_Control(5, dir, 100, 0, b, 0, 0);
            osDelay(500);
            pos_mode = false;
        }

        if (stop)
        {
            Emm_V5_Stop_Now(4, false);
            osDelay(500);
            stop = false;
        }
        if (servo_enabled) 
        {
            myServo.SetAngle(a); // 设置舵机角度为 a 度
            
        }

        osDelay(10);
    }
}