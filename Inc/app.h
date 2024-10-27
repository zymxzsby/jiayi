#ifndef __APP_H
#define __APP_H
#include "stdint.h"




/* ����汾������ʱ�޸ĸ���Ϣ���� */
#define CODE_VERSION "V1.0.3"
/* ����汾������ʱ�޸ĸ���Ϣ���� */
#define PRINT_CODE_VERSION_INFO(format, ...)  debug_printf("******** ---Based on LoRaWAN sensor data transmission experiment "format"_%s %s --- ********\r\n", ##__VA_ARGS__, __DATE__, __TIME__)


/** ���ڽ������ݳ������ֵ 512�ֽ� */
#define USART_RECV_LEN_MAX 512

/** ������ʱʱ�䣺120s */
#define JOIN_TIME_120_SEC  120


typedef struct {
    char upcnt[10];
    char ackcnt[10];
    char toa[10];
    char nbt[10];

    char ch[10];
    char sf[10];
    char pwr[10];
    char per[10];

    char rssi[10];
    char snr[10];

} DEBUG;
typedef struct {
    uint8_t frameHeader; // ֡ͷ
    uint8_t devEui[2]; // DevEui����������ֽ�
    uint8_t dataType; // ��������
    uint16_t timestamps[2]; // ʱ���
    float avgTemp; // ƽ���¶�
    float maxTemp; // ����¶�
    float minTemp; // ����¶�
    float tempDiff; // �²�
    float variance; // ����
    float tempRate; // �¶ȱ仯��
    uint8_t frameFooter; // ֡β
} uploaddata;
void LoRaWAN_Func_Process(void);
void LoRaWAN_Borad_Info_Print(void);
void prepareAndSendTemperData(uploaddata *tempers); 
void my_ori_temper_write(void);
void my_process_temper_write(void);
void my_ori_temper_read_uint32_t(void);
void my_process_temper_read_uint32_t(void);
void ProcessDailyData(int temp_data_index) ;
void draw_temperline(void);
#endif



