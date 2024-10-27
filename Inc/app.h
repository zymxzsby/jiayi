#ifndef __APP_H
#define __APP_H
#include "stdint.h"




/* 软件版本，升级时修改该信息即可 */
#define CODE_VERSION "V1.0.3"
/* 软件版本，升级时修改该信息即可 */
#define PRINT_CODE_VERSION_INFO(format, ...)  debug_printf("******** ---Based on LoRaWAN sensor data transmission experiment "format"_%s %s --- ********\r\n", ##__VA_ARGS__, __DATE__, __TIME__)


/** 串口接收内容长度最大值 512字节 */
#define USART_RECV_LEN_MAX 512

/** 入网超时时间：120s */
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
    uint8_t frameHeader; // 帧头
    uint8_t devEui[2]; // DevEui的最后两个字节
    uint8_t dataType; // 数据类型
    uint16_t timestamps[2]; // 时间戳
    float avgTemp; // 平均温度
    float maxTemp; // 最高温度
    float minTemp; // 最低温度
    float tempDiff; // 温差
    float variance; // 方差
    float tempRate; // 温度变化率
    uint8_t frameFooter; // 帧尾
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



