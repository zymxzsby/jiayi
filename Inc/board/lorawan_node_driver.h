#ifndef _LORAWAN_NODE_DRIVER_H_
#define _LORAWAN_NODE_DRIVER_H_

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/*-------------------------����ʵ��������ӱ�Ҫ��ͷ�ļ�-------------------------*/
#include "stm32l4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "common.h"
/*=====================================END======================================*/


/** �û����ݲ�Ʒ��Ҫ�������º궨�������в���ATָ�Ĭ��ΪNULL�����˺궨�������ӵ�ָ����ᱻ���浽ģ���С�
 * @par ʾ��:
 * ������������ɨ��Ƶ��ʽΪ����ģ����Ƶ��1A2�������¸�д�궨�壺
 * @code
	#define AT_COMMAND_AND_SAVE		"AT+BAND=47,0",  \
									"AT+CHMASK=00FF" \
 * @endcode
*/


#ifndef DEBUG_LOG_LEVEL_1
#define DEBUG_LOG_LEVEL_1
#endif


/** ģ�������쳣�������ֵ��������ֵʱ��������λģ�� */
#define ABNORMAL_CONTINUOUS_COUNT_MAX			6

/* -------------------------------------------------------------------------------
 * 					!!!!���������û�����ƽ̨����Ҫ�����޸�!!!!
 *********************************************************************************
 *						!!!!�������£��������޸�!!!!
 --------------------------------------------------------------------------------*/


/*--------------------��־����ȼ���δβ����Խ�󣬵ȼ�Խ��----------------------*/
#ifdef DEBUG_LOG_LEVEL_1
#define DEBUG_LOG_LEVEL_0
#endif
/*=====================================END======================================*/

/*-----------------------------ģ�����Ӧ��-------------------------------------*/
#ifndef M_ADVANCED_APP
#define M_ADVANCED_APP  0
#endif
/*=====================================END======================================*/

/*--------------------------�������ݵ�֡����------------------------------------*/
/** ����λΪ1����ȷ��֡ */
#define CONFIRM_TYPE		0x10
/** ����λΪ0������ȷ��֡ */
#define UNCONFIRM_TYPE		0x00
/*=====================================END======================================*/


/*-------------------------����״̬�����š�����ֵ��ö��-------------------------*/
/**
 * ģ��ĸ��ֹ���״̬
 */
typedef enum {
    command, 	///< ����ģʽ
    transparent,///< ͸��ģʽ
    wakeup,		///< ����ģ��
    sleep,		///< ����ģ��
} node_status_t;

/**
 * ģ��ĸ��ֹ�������
 */
typedef enum {
    mode,	///< ģʽ����(�л������͸��)
    wake,	///< ��������(�л����ߺͻ���)
    stat,	///< ģ��ͨ��״̬����
    busy,	///< ģ���Ƿ�Ϊæ����
} node_gpio_t;

/**
 * GPIO��ƽ״̬
 */
typedef enum {
    low,	///< �͵�ƽ
    high,	///< �ߵ�ƽ
    unknow,	///< δ֪��ƽ
} gpio_level_t;

/**
 * �ͷ�������Դ�ĵȼ�
 */
typedef enum {
    all_list,	///< �ͷ�����������������Դ
    save_data,	///< ����������data��Ϊ�յ���Դ
    save_data_and_last,  ///< ����������data��Ϊ�յ���Դ�����һ����Դ
} free_level_t;

/**
 * �������ݵķ���״̬
 */
typedef enum {
    NODE_COMM_SUCC	       		= 0x01,		///< ͨ�ųɹ�
    NODE_NOT_JOINED				= 0x02,		///< ģ��δ����
    NODE_COMM_NO_ACK			= 0x04,		///< ȷ��֡δ�յ�ACK
    NODE_BUSY_BFE_RECV_UDATA	= 0x08,		///< ģ�鵱ǰ����æ״̬
    NODE_BUSY_ATR_COMM			= 0x10,		///< ģ�鴦���쳣״̬
    NODE_IDLE_ATR_RECV_UDATA	= 0x20,		///< ģ�鴮������Ӧ
    USER_DATA_SIZE_WRONG		= 0x40		///< ���ݰ����ȴ���
} execution_status_t;


/**
 * �ն�������λģ����źű���
 */
typedef union node_reset_single
{
    uint8_t value;

    struct
    {
        uint8_t frame_type_modify      	 : 1;	///< ��λ��������֡����
        uint8_t RFU          			 : 7;	///< ����λ��ʱ���������Ժ�ʹ��
    } Bits;
} node_reset_single_t;


/**
 * ����ͨ��Ϣ��������Ϣ
 */
typedef struct down_info {
    uint16_t size;			///< ҵ�����ݳ���
    uint8_t *business_data;	///< ҵ������
#ifdef USE_NODE_STATUS
    uint8_t result_ind;	///< ���ֶ����ģ��ʹ��˵����
    int8_t snr;			///< ����SNR���������У����ֵΪ0
    int8_t rssi;		///< ����RSSI���������У����ֵΪ0
    uint8_t datarate;	///< �������ж�Ӧ����������
#endif
} down_info_t;

/**
 * ������Ϣ�������ṹ
 */
typedef struct list {
    down_info_t down_info;   ///< ������Ϣ�ṹ
    struct list *next;
} down_list_t;

/*=====================================END======================================*/

/*------------------------------�ṩ���û��Ľӿں���----------------------------*/
void Node_Hard_Reset(void);
void nodeGpioConfig(node_gpio_t type, node_status_t value);
bool nodeCmdConfig(char *str);
bool nodeCmdInqiure(char *str,uint8_t *content);
bool nodeJoinNet(uint16_t time_second);
void nodeResetJoin(uint16_t time_second);
execution_status_t nodeDataCommunicate(uint8_t *buffer, uint8_t size, down_list_t **list_head);

#if M_ADVANCED_APP
execution_status_t node_block_send_lowpower(uint8_t frame_tpye, uint8_t *buffer, uint8_t size, down_list_t **list_head);
bool node_block_send_big_packet(uint8_t *buffer, uint16_t size, uint8_t resend_num, down_list_t **list_head);
#endif
/*=====================================END======================================*/

extern uint8_t confirm_continue_failure_count;

/*--------------------------------�ṩ���û��ı���------------------------------*/
extern bool node_join_successfully;
/*=====================================END======================================*/

#endif // _LORAWAN_MODULE_DRIVER_H_