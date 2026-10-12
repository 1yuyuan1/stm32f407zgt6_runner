#ifndef BSP_UART_H
#define BSP_UART_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "string.h"

/**
 * @brief 串口工作模式定义
 * @note 对应了三种
 */
typedef enum
{
    BspUartType_Normal,
    BspUartType_IT,
    BspUartType_DMA,
}BspUart_TypeDef;



// 定义接收回调函数类型
typedef void (*BspUart_InstRxCallback)(UART_HandleTypeDef *huart, uint8_t *rxData, uint8_t size);

// 串口实例及其句柄定义
typedef struct
{
    UART_HandleTypeDef *huart;                      // 串口句柄
    BspUart_TypeDef rxtype;                           // Rx工作模式
    BspUart_TypeDef txtype;                           // Tx工作模式
    BspUart_InstRxCallback rx_callback;             // 接收回调函数

    uint8_t rx_buffer[64];                          // 接收缓冲区（最大64Byte）
    uint8_t rx_byte;                                // 单次接收的字节数，IT模式下始终为1    
    uint8_t rx_setlen;                             // 期望接收数据长度
    uint8_t rx_len;                                 // 实际接收数据长度
    volatile uint32_t rx_rearm_errors;
}BspUart_Instance;

#define BSPUART_TX_BUFFER_SIZE 512U

void BspUart_InstRegist(BspUart_Instance *inst, UART_HandleTypeDef *huart, uint8_t rx_setlen,
                        BspUart_TypeDef rxtype, BspUart_TypeDef txtype, BspUart_InstRxCallback rx_callback);

// 非阻塞发送会复制数据；HAL_BUSY 表示本帧未发送，调用者须重试。
HAL_StatusTypeDef BspUart_Transmit(BspUart_Instance inst, const uint8_t *data, uint16_t len);
uint32_t BspUart_GetTxBusyCount(const BspUart_Instance *inst);
uint32_t BspUart_GetTxErrorCount(const BspUart_Instance *inst);
// 仅由该串口的所属任务调用，用于恢复发送卡住或接收重新启动失败。
void BspUart_Service(BspUart_Instance *inst);

extern BspUart_Instance uart1_inst; 
extern BspUart_Instance uart6_imu_inst;
extern BspUart_Instance uart2_inst;
extern BspUart_Instance uart3_gray_inst;

#ifdef __cplusplus
}
#endif
#endif
