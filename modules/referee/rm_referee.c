/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "string.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "cmsis_os.h"

#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据

/**
 * @brief  读取裁判数据,在USARTProcess任务上下文中解析
 * @param  buff: 读取到的裁判系统原始数据
 * @param  buff_len: 本次实际接收到的数据长度
 * @retval 是否解析到有效帧
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static uint8_t JudgeReadData(uint8_t *buff, uint16_t buff_len)
{
	uint16_t data_length;
	uint32_t judge_length = 0; // 统计一帧数据长度
	uint8_t parsed = 0;
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return 0;

	if (buff_len < LEN_HEADER)
		return 0;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度(byte),用于CR16校验
			data_length = (uint16_t)(buff[DATA_LENGTH] | (buff[DATA_LENGTH + 1U] << 8));
			judge_length = (uint32_t)data_length + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			if (judge_length > buff_len)
				return 0;

			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			{
				parsed = 1;
				// 2个8位拼成16位int
				referee_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				// 第8个字节开始才是数据 data=7
				switch (referee_info.CmdID)
				{
				case ID_game_state: // 0x0001
					if (data_length >= LEN_game_state)
						memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
					break;
				case ID_game_result: // 0x0002
					if (data_length >= LEN_game_result)
						memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
					break;
				case ID_game_robot_survivors: // 0x0003
					if (data_length >= LEN_game_robot_HP)
						memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
					break;
				case ID_event_data: // 0x0101
					if (data_length >= LEN_event_data)
						memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
					break;
				case ID_supply_projectile_action: // 0x0102
					if (data_length >= LEN_supply_projectile_action)
						memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
					break;
				case ID_game_robot_state: // 0x0201
					if (data_length >= LEN_game_robot_state)
						memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
					break;
				case ID_power_heat_data: // 0x0202
					if (data_length >= LEN_power_heat_data)
						memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
					break;
				case ID_game_robot_pos: // 0x0203
					if (data_length >= LEN_game_robot_pos)
						memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
					break;
				case ID_buff_musk: // 0x0204
					if (data_length >= LEN_buff_musk)
						memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
					break;
				case ID_aerial_robot_energy: // 0x0205
					if (data_length >= LEN_aerial_robot_energy)
						memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
					break;
				case ID_robot_hurt: // 0x0206
					if (data_length >= LEN_robot_hurt)
						memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
					break;
				case ID_shoot_data: // 0x0207
					if (data_length >= LEN_shoot_data)
						memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
					break;
				case ID_student_interactive: // 0x0301   syhtodo接收代码未测试
					if (data_length >= LEN_receive_data)
						memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
					break;
				}
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		if (judge_length > 0 && judge_length < buff_len && *(buff + judge_length) == REFEREE_SOF)
		{ // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			parsed |= JudgeReadData(buff + judge_length, buff_len - judge_length);
		}
	}

	return parsed;
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	if (referee_usart_instance == NULL)
		return;

	if (JudgeReadData(referee_usart_instance->recv_buff, referee_usart_instance->recv_len))
	{
		DaemonReload(referee_daemon);
	}
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);
	if (referee_usart_instance == NULL)
	{
		LOGERROR("[rm_ref] USART register failed");
		return NULL;
	}

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	HAL_StatusTypeDef status = USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	if (status != HAL_OK && status != HAL_BUSY)
	{
		LOGWARNING("[rm_ref] send failed, status [%d]", status);
	}
	osDelay(115);
}
