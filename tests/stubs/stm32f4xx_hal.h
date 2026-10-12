#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stdbool.h>
#define __IO volatile
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_TX 1U
#define HAL_UART_STATE_BUSY_RX 2U
#define DMA_IT_HT 0U
#define __HAL_DMA_DISABLE_IT(handle, mask) ((void)(handle), (void)(mask))
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t unused; } DMA_HandleTypeDef;
typedef struct
{
    uint32_t gState;
    uint32_t RxState;
    DMA_HandleTypeDef *hdmarx;
} UART_HandleTypeDef;
#ifdef __cplusplus
extern "C" {
#endif
uint32_t HAL_GetTick(void);
void Error_Handler(void);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
#ifdef __cplusplus
}
#endif
#endif
