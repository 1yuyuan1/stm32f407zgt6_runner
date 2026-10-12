#include "bsp_uart.h"

#define BSPUART_MAX_INSTANCES 6

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t buffer[BSPUART_TX_BUFFER_SIZE];
    volatile uint8_t busy;
    volatile uint32_t busy_count;
    volatile uint32_t error_count;
    uint32_t started_ms;
} BspUart_TxSlot;

static BspUart_Instance *bspuart_insts[BSPUART_MAX_INSTANCES] = {NULL};
static BspUart_TxSlot tx_slots[BSPUART_MAX_INSTANCES] = {0};
static int bspuart_inst_count = 0;

BspUart_Instance uart1_inst;
BspUart_Instance uart2_inst;       // 四台 X42S 电机共用的串口
BspUart_Instance uart3_gray_inst;  // 16 路灰度传感器串口
BspUart_Instance uart6_imu_inst;   // CH040 姿态传感器串口

static int BspUart_FindIndex(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < bspuart_inst_count; ++i)
    {
        if (bspuart_insts[i]->huart == huart) return i;
    }
    return -1;
}

static void BspUart_RearmRx(BspUart_Instance *inst)
{
    if (inst->rxtype == BspUartType_DMA)
    {
        if (HAL_UARTEx_ReceiveToIdle_DMA(inst->huart, inst->rx_buffer, inst->rx_setlen) == HAL_OK)
        {
            __HAL_DMA_DISABLE_IT(inst->huart->hdmarx, DMA_IT_HT);
        }
        else
        {
            ++inst->rx_rearm_errors;
        }
    }
    else if (inst->rxtype == BspUartType_IT)
    {
        if (HAL_UART_Receive_IT(inst->huart, &inst->rx_byte, 1) != HAL_OK)
        {
            ++inst->rx_rearm_errors;
        }
    }
}

void BspUart_InstRegist(BspUart_Instance *inst, UART_HandleTypeDef *huart, uint8_t rx_setlen,
                        BspUart_TypeDef rxtype, BspUart_TypeDef txtype, BspUart_InstRxCallback rx_callback)
{
    if (inst == NULL || huart == NULL || rx_setlen == 0 || rx_setlen > sizeof(inst->rx_buffer) ||
        bspuart_inst_count >= BSPUART_MAX_INSTANCES || BspUart_FindIndex(huart) >= 0)
    {
        Error_Handler();
        return;
    }

    inst->huart = huart;
    inst->rxtype = rxtype;
    inst->txtype = txtype;
    inst->rx_callback = rx_callback;
    inst->rx_setlen = rx_setlen;
    inst->rx_len = 0;
    inst->rx_rearm_errors = 0;
    memset(inst->rx_buffer, 0, sizeof(inst->rx_buffer));

    tx_slots[bspuart_inst_count].huart = huart;
    tx_slots[bspuart_inst_count].busy = 0;
    tx_slots[bspuart_inst_count].busy_count = 0;
    tx_slots[bspuart_inst_count].error_count = 0;
    bspuart_insts[bspuart_inst_count++] = inst;

    // 先登记串口实例再启动接收，避免 DMA 启动后立即触发回调时找不到实例。
    BspUart_RearmRx(inst);
}

HAL_StatusTypeDef BspUart_Transmit(BspUart_Instance inst, const uint8_t *data, uint16_t len)
{
    if (inst.huart == NULL || data == NULL || len == 0 || len > BSPUART_TX_BUFFER_SIZE)
        return HAL_ERROR;

    int index = BspUart_FindIndex(inst.huart);
    if (index < 0) return HAL_ERROR;
    BspUart_TxSlot *slot = &tx_slots[index];

    if (inst.txtype == BspUartType_Normal)
        return HAL_UART_Transmit(inst.huart, (uint8_t *)data, len, 100);

    if (slot->busy || inst.huart->gState != HAL_UART_STATE_READY)
    {
        ++slot->busy_count;
        return HAL_BUSY;
    }

    // DMA 或中断发送不会复制调用者的数据，必须保存到发送完成回调执行。
    memcpy(slot->buffer, data, len);
    slot->started_ms = HAL_GetTick();
    slot->busy = 1;

    HAL_StatusTypeDef result;
    if (inst.txtype == BspUartType_DMA)
        result = HAL_UART_Transmit_DMA(inst.huart, slot->buffer, len);
    else if (inst.txtype == BspUartType_IT)
        result = HAL_UART_Transmit_IT(inst.huart, slot->buffer, len);
    else
        result = HAL_ERROR;

    if (result != HAL_OK)
    {
        slot->busy = 0;
        ++slot->error_count;
    }
    return result;
}

uint32_t BspUart_GetTxBusyCount(const BspUart_Instance *inst)
{
    int index = (inst != NULL) ? BspUart_FindIndex(inst->huart) : -1;
    return (index >= 0) ? tx_slots[index].busy_count : 0;
}

uint32_t BspUart_GetTxErrorCount(const BspUart_Instance *inst)
{
    int index = (inst != NULL) ? BspUart_FindIndex(inst->huart) : -1;
    return (index >= 0) ? tx_slots[index].error_count : 0;
}

void BspUart_Service(BspUart_Instance *inst)
{
    int index = BspUart_FindIndex(inst->huart);
    if (index < 0) return;
    BspUart_TxSlot *slot = &tx_slots[index];
    if (slot->busy && HAL_GetTick() - slot->started_ms > 30U)
    {
        // 这些串口以不低于 115200 波特率发送短帧，超过 30ms 视为发送卡住。
        if (HAL_UART_AbortTransmit(inst->huart) == HAL_OK) slot->busy = 0;
        ++slot->error_count;
    }
    if (inst->huart->RxState == HAL_UART_STATE_READY) BspUart_RearmRx(inst);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    int index = BspUart_FindIndex(huart);
    if (index < 0) return;
    BspUart_Instance *inst = bspuart_insts[index];
    if (size > inst->rx_setlen) size = inst->rx_setlen;
    inst->rx_len = (uint8_t)size;
    if (inst->rx_callback != NULL && size > 0)
        inst->rx_callback(huart, inst->rx_buffer, (uint8_t)size);

    // 只处理实际收到的字节，避免清空整个 DMA 缓冲区而增加中断耗时。
    BspUart_RearmRx(inst);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    int index = BspUart_FindIndex(huart);
    if (index < 0) return;
    BspUart_Instance *inst = bspuart_insts[index];
    if (inst->rxtype != BspUartType_IT) return;
    inst->rx_len = 1;
    if (inst->rx_callback != NULL)
        inst->rx_callback(huart, &inst->rx_byte, 1);
    BspUart_RearmRx(inst);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    int index = BspUart_FindIndex(huart);
    if (index >= 0) tx_slots[index].busy = 0;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    int index = BspUart_FindIndex(huart);
    if (index < 0) return;
    ++tx_slots[index].error_count;
    if (huart->gState == HAL_UART_STATE_READY) tx_slots[index].busy = 0;
    if (huart->RxState == HAL_UART_STATE_READY) BspUart_RearmRx(bspuart_insts[index]);
}
