#include "Emm_V5.h"
#include "bsp_uart.h"

/**
 * @file Emm_V5.c
 * @brief Emm V5.0 步进电机驱动器串口协议命令实现。
 *
 * 命令帧以地址开头，以 0x6B 结束；多字节数值按高字节在前排列。
 * 普通接口立即发送；Emm_V5_MMCL_* 接口只追加到多机命令缓冲区。
 */

__IO uint16_t MMCL_count = 0, MMCL_cmd[MMCL_LEN] = {0};


/**
 * @file Emm_V5.c
 * @brief Emm V5.0 步进电机驱动器串口协议命令实现。
 *
 * 命令帧以地址开头，以 0x6B 结束；多字节数值按高字节在前排列。
 * 普通接口立即发送；Emm_V5_MMCL_* 接口只追加到多机命令缓冲区。
 */




/* ==================== 基础命令 ==================== */

/**
 * @brief 触发编码器校准。
 * @param addr 电机地址。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Trig_Encoder_Cal(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x06;
  cmd[2] =  0x45;
  cmd[3] =  0x6B;

	BspUart_Transmit(uart2_inst, cmd, 4);
}


/**
 * @brief 复位电机。
 * @param addr 电机地址。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Reset_Motor(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x08;
  cmd[2] =  0x97;
  cmd[3] =  0x6B;

	BspUart_Transmit(uart2_inst, cmd, 4);
}


/**
 * @brief 将当前位置清零。
 * @param addr 电机地址。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0A;
  cmd[2] =  0x6D;
  cmd[3] =  0x6B;

	BspUart_Transmit(uart2_inst, cmd, 4);
}


/**
 * @brief 解除堵转保护。
 * @param addr 电机地址。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0E;
  cmd[2] =  0x52;
  cmd[3] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 4);
}


/**
 * @brief 恢复出厂设置。
 * @param addr 电机地址。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Restore_Motor(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0F;
  cmd[2] =  0x5F;
  cmd[3] =  0x6B;

	BspUart_Transmit(uart2_inst, cmd, 4);
}


/**
 * @brief 发送多机命令缓冲区。
 * @param addr 电机地址。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Multi_Motor_Cmd(uint8_t addr)
{
  uint16_t i = 0, j = 0, len = 0; __IO static uint8_t cmd[MMCL_LEN] = {0};

	if(MMCL_count > 0)
	{

		len = MMCL_count + 5;

		cmd[0] = addr;
		cmd[1] = 0xAA;
		cmd[2] = (uint8_t)(len >> 8);
		cmd[3] = (uint8_t)(len);
		for(i=0,j=4; i < MMCL_count; i++,j++) { cmd[j] = MMCL_cmd[i]; }
		cmd[j] = 0x6B; ++j;

		BspUart_Transmit(uart2_inst, cmd, j);
	}
	else
	{
		MMCL_count = 0;
	}
}


/**
 * @brief 使能或关闭电机。
 * @param addr 电机地址。
 * @param state 使能状态，true 为使能。
 * @param snF 多机同步标志。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF3;
  cmd[2] =  0xAB;
  cmd[3] =  (uint8_t)state;
  cmd[4] =  snF;
  cmd[5] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 6);
}


/**
 * @brief 以速度模式控制电机。
 * @param addr 电机地址。
 * @param dir 方向，0 为 CW，其余值为 CCW。
 * @param vel 目标速度，单位 RPM。
 * @param acc 加速度，0 表示直接启动。
 * @param snF 多机同步标志。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF6;
  cmd[2] =  dir;
  cmd[3] =  (uint8_t)(vel >> 8);
  cmd[4] =  (uint8_t)(vel >> 0);
  cmd[5] =  acc;
  cmd[6] =  snF;
  cmd[7] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 8);
}


/**
 * @brief 以位置模式控制电机。
 * @param addr 电机地址。
 * @param dir 方向，0 为 CW，其余值为 CCW。
 * @param vel 目标速度，单位 RPM。
 * @param acc 加速度，0 表示直接启动。
 * @param clk 目标脉冲数。
 * @param raF false 为相对位置，true 为绝对位置。
 * @param snF 多机同步标志。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0xFD;
  cmd[2]  =  dir;
  cmd[3]  =  (uint8_t)(vel >> 8);
  cmd[4]  =  (uint8_t)(vel >> 0);
  cmd[5]  =  acc;
  cmd[6]  =  (uint8_t)(clk >> 24);
  cmd[7]  =  (uint8_t)(clk >> 16);
  cmd[8]  =  (uint8_t)(clk >> 8);
  cmd[9]  =  (uint8_t)(clk >> 0);
  cmd[10] =  raF;
  cmd[11] =  snF;
  cmd[12] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 13);
}


/**
 * @brief 立即停止电机。
 * @param addr 电机地址。
 * @param snF 多机同步标志。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFE;
  cmd[2] =  0x98;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 5);
}


/**
 * @brief 触发多机同步运动。
 * @param addr 电机地址。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFF;
  cmd[2] =  0x66;
  cmd[3] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 4);
}


/**
 * @brief 设置单圈回零参考点。
 * @param addr 电机地址。
 * @param svF true 表示保存参数。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x93;
  cmd[2] =  0x88;
  cmd[3] =  svF;
  cmd[4] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 5);
}


/**
 * @brief 触发回零。
 * @param addr 电机地址。
 * @param o_mode 回零模式。
 * @param snF 多机同步标志。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x9A;
  cmd[2] =  o_mode;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 5);
}


/**
 * @brief 强制中断回零。
 * @param addr 电机地址。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x9C;
  cmd[2] =  0x48;
  cmd[3] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 4);
}


/**
 * @brief 读取回零参数。
 * @param addr 电机地址。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Origin_Read_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x22;
  cmd[2] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 3);
}


/**
 * @brief 修改回零参数。
 * @param addr 电机地址。
 * @param svF true 表示保存参数。
 * @param o_mode 回零模式。
 * @param o_dir 回零方向。
 * @param o_vel 回零速度，单位 RPM。
 * @param o_tm 回零超时时间，单位 ms。
 * @param sl_vel 碰撞检测速度。
 * @param sl_ma 碰撞检测电流。
 * @param sl_ms 碰撞检测时间。
 * @param potF 上电自动回零标志。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  __IO static uint8_t cmd[32] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x4C;
  cmd[2] =  0xAE;
  cmd[3] =  svF;
  cmd[4] =  o_mode;
  cmd[5] =  o_dir;
  cmd[6]  =  (uint8_t)(o_vel >> 8);
  cmd[7]  =  (uint8_t)(o_vel >> 0);
  cmd[8]  =  (uint8_t)(o_tm >> 24);
  cmd[9]  =  (uint8_t)(o_tm >> 16);
  cmd[10] =  (uint8_t)(o_tm >> 8);
  cmd[11] =  (uint8_t)(o_tm >> 0);
  cmd[12] =  (uint8_t)(sl_vel >> 8);
  cmd[13] =  (uint8_t)(sl_vel >> 0);
  cmd[14] =  (uint8_t)(sl_ma >> 8);
  cmd[15] =  (uint8_t)(sl_ma >> 0);
  cmd[16] =  (uint8_t)(sl_ms >> 8);
  cmd[17] =  (uint8_t)(sl_ms >> 0);
  cmd[18] =  potF;
  cmd[19] =  0x6B;

  BspUart_Transmit(uart2_inst, cmd, 20);
}


/**
 * @brief 配置系统参数定时回传。
 * @param addr 电机地址。
 * @param s 系统参数类型。
 * @param time_ms 时间，单位 ms。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
{
  uint8_t i = 0; __IO static uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;

  cmd[i] = 0x11; ++i;

  cmd[i] = 0x18; ++i;

  switch(s)
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;
		case S_CBUS : cmd[i] = 0x26; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
		case S_ENCO : cmd[i] = 0x29; ++i; break;
		case S_CLKC : cmd[i] = 0x30; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
		case S_CLKI : cmd[i] = 0x32; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_SPOS : cmd[i] = 0x34; ++i; break;
		case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
		case S_VBAT : cmd[i] = 0x38; ++i; break;
		case S_TEMP : cmd[i] = 0x39; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;
		case S_OAF  : cmd[i] = 0x3C; ++i; break;
		case S_PIN  : cmd[i] = 0x3D; ++i; break;
    default: break;
  }

	cmd[i] = (uint8_t)(time_ms >> 8);  ++i;
	cmd[i] = (uint8_t)(time_ms >> 0);  ++i;

  cmd[i] = 0x6B; ++i;

  BspUart_Transmit(uart2_inst, cmd, i);
}


/**
 * @brief 读取指定系统参数。
 * @param addr 电机地址。
 * @param s 系统参数类型。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0; __IO static uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;

  switch(s)
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;
		case S_CBUS : cmd[i] = 0x26; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
		case S_ENCO : cmd[i] = 0x29; ++i; break;
		case S_CLKC : cmd[i] = 0x30; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
		case S_CLKI : cmd[i] = 0x32; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_SPOS : cmd[i] = 0x34; ++i; break;
		case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
		case S_VBAT : cmd[i] = 0x38; ++i; break;
		case S_TEMP : cmd[i] = 0x39; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;
		case S_OAF  : cmd[i] = 0x3C; ++i; break;
		case S_PIN  : cmd[i] = 0x3D; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, i);
}


/**
 * @brief 执行 Emm_V5_Modify_Motor_ID 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xAE;
  cmd[2] =  0x4B;
  cmd[3] =  svF;
  cmd[4] =  id;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 6);
}


/**
 * @brief 执行 Emm_V5_Modify_MicroStep 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x84;
  cmd[2] =  0x8A;
  cmd[3] =  svF;
  cmd[4] =  mstep;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 6);
}


/**
 * @brief 执行 Emm_V5_Modify_PDFlag 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_PDFlag(uint8_t addr, bool pdf)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x50;
  cmd[2] =  pdf;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 4);
}


/**
 * @brief 执行 Emm_V5_Read_Opt_Param_Sta 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Read_Opt_Param_Sta(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x1A;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 3);
}


/**
 * @brief 执行 Emm_V5_Modify_Motor_Type 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype)
{
  __IO static uint8_t cmd[16] = {0}; uint8_t MotType = 0;

	if(mottype) { MotType = 25; } else { MotType = 50; }

  cmd[0] =  addr;
  cmd[1] =  0xD7;
  cmd[2] =  0x35;
  cmd[3] =  svF;
  cmd[4] =  MotType;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 6);
}


/**
 * @brief 执行 Emm_V5_Modify_Firmware_Type 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xD5;
  cmd[2] =  0x69;
  cmd[3] =  svF;
  cmd[4] =  fwtype;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 6);
}


/**
 * @brief 执行 Emm_V5_Modify_Ctrl_Mode 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x46;
  cmd[2] =  0x69;
  cmd[3] =  svF;
  cmd[4] =  ctrl_mode;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 6);
}


/**
 * @brief 执行 Emm_V5_Modify_Motor_Dir 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xD4;
  cmd[2] =  0x60;
  cmd[3] =  svF;
  cmd[4] =  dir;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 6);
}


/**
 * @brief 执行 Emm_V5_Modify_Lock_Btn 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_Lock_Btn(uint8_t addr, bool svF, bool lock)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xD0;
  cmd[2] =  0xB3;
  cmd[3] =  svF;
  cmd[4] =  lock;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 6);
}


/**
 * @brief 执行 Emm_V5_Modify_S_Vel 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x4F;
  cmd[2] =  0x71;
  cmd[3] =  svF;
  cmd[4] =  s_vel;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 6);
}


/**
 * @brief 执行 Emm_V5_Modify_OM_mA 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x44;
  cmd[2] =  0x33;
  cmd[3] =  svF;
  cmd[4] =  (uint8_t)(om_ma >> 8);
	cmd[5] =  (uint8_t)(om_ma >> 0);
  cmd[6] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 7);
}


/**
 * @brief 执行 Emm_V5_Modify_FOC_mA 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x45;
  cmd[2] =  0x66;
  cmd[3] =  svF;
  cmd[4] =  (uint8_t)(foc_mA >> 8);
	cmd[5] =  (uint8_t)(foc_mA >> 0);
  cmd[6] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 7);
}


/**
 * @brief 执行 Emm_V5_Read_PID_Params 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Read_PID_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x21;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 3);
}


/**
 * @brief 执行 Emm_V5_Modify_PID_Params 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd)
{
  __IO static uint8_t cmd[20] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0x4A;
  cmd[2]  =  0xC3;
  cmd[3]  =  svF;
  cmd[4]  =  (uint8_t)(kp >> 24);
	cmd[5]  =  (uint8_t)(kp >> 16);
	cmd[6]  =  (uint8_t)(kp >> 8);
	cmd[7]  =  (uint8_t)(kp >> 0);
	cmd[8]  =  (uint8_t)(ki >> 24);
	cmd[9]  =  (uint8_t)(ki >> 16);
	cmd[10] =  (uint8_t)(ki >> 8);
	cmd[11] =  (uint8_t)(ki >> 0);
	cmd[12] =  (uint8_t)(kd >> 24);
	cmd[13] =  (uint8_t)(kd >> 16);
	cmd[14] =  (uint8_t)(kd >> 8);
	cmd[15] =  (uint8_t)(kd >> 0);
  cmd[16] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 17);
}


/**
 * @brief 执行 Emm_V5_Read_DMX512_Params 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Read_DMX512_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x49;
	cmd[2] =  0x78;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 4);
}


/**
 * @brief 执行 Emm_V5_Modify_DMX512_Params 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step)
{
  __IO static uint8_t cmd[32] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0xD9;
  cmd[2]  =  0x90;
  cmd[3]  =  svF;
  cmd[4]  =  (uint8_t)(tch >> 8);
  cmd[5]  =  (uint8_t)(tch >> 0);
	cmd[6]  =  nch;
	cmd[7]  =  mode;
	cmd[8]  =  (uint8_t)(vel >> 8);
  cmd[9]  =  (uint8_t)(vel >> 0);
	cmd[10] =  (uint8_t)(acc >> 8);
  cmd[11] =  (uint8_t)(acc >> 0);
	cmd[12] =  (uint8_t)(vel_step >> 8);
  cmd[13] =  (uint8_t)(vel_step >> 0);
  cmd[14]  = (uint8_t)(pos_step >> 24);
  cmd[15]  = (uint8_t)(pos_step >> 16);
  cmd[16] =  (uint8_t)(pos_step >> 8);
  cmd[17] =  (uint8_t)(pos_step >> 0);
  cmd[18] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 19);
}


/**
 * @brief 执行 Emm_V5_Read_Pos_Window 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Read_Pos_Window(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x41;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 3);
}


/**
 * @brief 执行 Emm_V5_Modify_Pos_Window 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xD1;
  cmd[2] =  0x07;
  cmd[3] =  svF;
  cmd[4] =  (uint8_t)(prw >> 8);
	cmd[5] =  (uint8_t)(prw >> 0);
  cmd[6] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 7);
}


/**
 * @brief 执行 Emm_V5_Read_Otocp 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Read_Otocp(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x13;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 3);
}


/**
 * @brief 执行 Emm_V5_Modify_Otocp 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0xD3;
  cmd[2]  =  0x56;
  cmd[3]  =  svF;
  cmd[4]  =  (uint8_t)(otp >> 8);
	cmd[5]  =  (uint8_t)(otp >> 0);
	cmd[6]  =  (uint8_t)(ocp >> 8);
	cmd[7]  =  (uint8_t)(ocp >> 0);
	cmd[8]  =  (uint8_t)(time_ms >> 8);
	cmd[9]  =  (uint8_t)(time_ms >> 0);
  cmd[10] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 11);
}


/**
 * @brief 执行 Emm_V5_Read_Heart_Protect 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Read_Heart_Protect(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x16;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 3);
}


/**
 * @brief 执行 Emm_V5_Modify_Heart_Protect 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0x68;
  cmd[2]  =  0x38;
  cmd[3]  =  svF;
  cmd[4]  =  (uint8_t)(hp >> 24);
	cmd[5]  =  (uint8_t)(hp >> 16);
	cmd[6]  =  (uint8_t)(hp >> 8);
	cmd[7]  =  (uint8_t)(hp >> 0);
  cmd[8]  =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 9);
}


/**
 * @brief 执行 Emm_V5_Read_Integral_Limit 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Read_Integral_Limit(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x23;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 3);
}


/**
 * @brief 执行 Emm_V5_Modify_Integral_Limit 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0x4B;
  cmd[2]  =  0x57;
  cmd[3]  =  svF;
  cmd[4]  =  (uint8_t)(il >> 24);
	cmd[5]  =  (uint8_t)(il >> 16);
	cmd[6]  =  (uint8_t)(il >> 8);
	cmd[7]  =  (uint8_t)(il >> 0);
  cmd[8]  =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 9);
}


/**
 * @brief 执行 Emm_V5_Read_System_State_Params 对应的驱动器协议命令。
 * @note 命令通过 UART2 发送，不等待驱动器应答。
 */
void Emm_V5_Read_System_State_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x43;
	cmd[2] =  0x7A;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 4);
}


/**
 * @brief 读取电机配置参数。
 * @param addr 电机地址。
 * @note 组帧后通过 UART2 发送，不等待应答。
 */
void Emm_V5_Read_Motor_Conf_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x42;
	cmd[2] =  0x6C;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)cmd, 4);
}


/**
 * @brief 触发编码器校准并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Trig_Encoder_Cal(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x06;
  cmd[2] =  0x45;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 复位电机并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Reset_Motor(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x08;
  cmd[2] =  0x97;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 将当前位置清零并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0A;
  cmd[2] =  0x6D;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 解除堵转保护并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0E;
  cmd[2] =  0x52;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 恢复出厂设置并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Restore_Motor(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0F;
  cmd[2] =  0x5F;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 使能或关闭电机并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @param state 使能状态，true 为使能。
 * @param snF 多机同步标志。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF3;
  cmd[2] =  0xAB;
  cmd[3] =  (uint8_t)state;
  cmd[4] =  snF;
  cmd[5] =  0x6B;

  for(j=0; j < 6; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 以速度模式控制电机并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @param dir 方向，0 为 CW，其余值为 CCW。
 * @param vel 目标速度，单位 RPM。
 * @param acc 加速度，0 表示直接启动。
 * @param snF 多机同步标志。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF6;
  cmd[2] =  dir;
  cmd[3] =  (uint8_t)(vel >> 8);
  cmd[4] =  (uint8_t)(vel >> 0);
  cmd[5] =  acc;
  cmd[6] =  snF;
  cmd[7] =  0x6B;

  for(j=0; j < 8; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 以位置模式控制电机并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @param dir 方向，0 为 CW，其余值为 CCW。
 * @param vel 目标速度，单位 RPM。
 * @param acc 加速度，0 表示直接启动。
 * @param clk 目标脉冲数。
 * @param raF false 为相对位置，true 为绝对位置。
 * @param snF 多机同步标志。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0xFD;
  cmd[2]  =  dir;
  cmd[3]  =  (uint8_t)(vel >> 8);
  cmd[4]  =  (uint8_t)(vel >> 0);
  cmd[5]  =  acc;
  cmd[6]  =  (uint8_t)(clk >> 24);
  cmd[7]  =  (uint8_t)(clk >> 16);
  cmd[8]  =  (uint8_t)(clk >> 8);
  cmd[9]  =  (uint8_t)(clk >> 0);
  cmd[10] =  raF;
  cmd[11] =  snF;
  cmd[12] =  0x6B;

  for(j=0; j < 13; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 立即停止电机并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @param snF 多机同步标志。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFE;
  cmd[2] =  0x98;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 触发多机同步运动并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Synchronous_motion(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFF;
  cmd[2] =  0x66;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 设置单圈回零参考点并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @param svF true 表示保存参数。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x93;
  cmd[2] =  0x88;
  cmd[3] =  svF;
  cmd[4] =  0x6B;

  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 触发回零并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @param o_mode 回零模式。
 * @param snF 多机同步标志。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x9A;
  cmd[2] =  o_mode;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 强制中断回零并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Origin_Interrupt(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x9C;
  cmd[2] =  0x48;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 修改回零参数并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @param svF true 表示保存参数。
 * @param o_mode 回零模式。
 * @param o_dir 回零方向。
 * @param o_vel 回零速度，单位 RPM。
 * @param o_tm 回零超时时间，单位 ms。
 * @param sl_vel 碰撞检测速度。
 * @param sl_ma 碰撞检测电流。
 * @param sl_ms 碰撞检测时间。
 * @param potF 上电自动回零标志。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t j = 0, cmd[32] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x4C;
  cmd[2] =  0xAE;
  cmd[3] =  svF;
  cmd[4] =  o_mode;
  cmd[5] =  o_dir;
  cmd[6]  =  (uint8_t)(o_vel >> 8);
  cmd[7]  =  (uint8_t)(o_vel >> 0);
  cmd[8]  =  (uint8_t)(o_tm >> 24);
  cmd[9]  =  (uint8_t)(o_tm >> 16);
  cmd[10] =  (uint8_t)(o_tm >> 8);
  cmd[11] =  (uint8_t)(o_tm >> 0);
  cmd[12] =  (uint8_t)(sl_vel >> 8);
  cmd[13] =  (uint8_t)(sl_vel >> 0);
  cmd[14] =  (uint8_t)(sl_ma >> 8);
  cmd[15] =  (uint8_t)(sl_ma >> 0);
  cmd[16] =  (uint8_t)(sl_ms >> 8);
  cmd[17] =  (uint8_t)(sl_ms >> 0);
  cmd[18] =  potF;
  cmd[19] =  0x6B;

  for(j=0; j < 20; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 配置系统参数定时回传并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @param s 系统参数类型。
 * @param time_ms 时间，单位 ms。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
{
  uint8_t i = 0, j = 0; uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;

  cmd[i] = 0x11; ++i;

  cmd[i] = 0x18; ++i;

  switch(s)
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;
		case S_CBUS : cmd[i] = 0x26; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
		case S_ENCO : cmd[i] = 0x29; ++i; break;
		case S_CLKC : cmd[i] = 0x30; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
		case S_CLKI : cmd[i] = 0x32; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_SPOS : cmd[i] = 0x34; ++i; break;
		case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
		case S_VBAT : cmd[i] = 0x38; ++i; break;
		case S_TEMP : cmd[i] = 0x39; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;
		case S_OAF  : cmd[i] = 0x3C; ++i; break;
		case S_PIN  : cmd[i] = 0x3D; ++i; break;
    default: break;
  }

	cmd[i] = (uint8_t)(time_ms >> 8);  ++i;
	cmd[i] = (uint8_t)(time_ms >> 0);  ++i;

  cmd[i] = 0x6B; ++i;

  for(j=0; j < i; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}


/**
 * @brief 读取指定系统参数并加入多机命令缓冲区。
 * @param addr 电机地址。
 * @param s 系统参数类型。
 * @note 本函数不发送 UART 数据。
 */
void Emm_V5_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0, j = 0; uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;

  switch(s)
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;
		case S_CBUS : cmd[i] = 0x26; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
		case S_ENCO : cmd[i] = 0x29; ++i; break;
		case S_CLKC : cmd[i] = 0x30; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
		case S_CLKI : cmd[i] = 0x32; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_SPOS : cmd[i] = 0x34; ++i; break;
		case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
		case S_VBAT : cmd[i] = 0x38; ++i; break;
		case S_TEMP : cmd[i] = 0x39; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;
		case S_OAF  : cmd[i] = 0x3C; ++i; break;
		case S_PIN  : cmd[i] = 0x3D; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;

  for(j=0; j < i; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}
