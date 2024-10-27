#include <string.h>
#include <stdio.h>
#include <math.h>
#include "ST7789v.h"
#include "XPT2046.h"
#include "tim.h"
#include "app.h"
#include "usart.h"
#include "gpio.h"
#include "lorawan_node_driver.h"
#include "hdc1000.h"
#include "sensors_test.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_rtc.h"// RTC定时器，用于获取当前时间戳
#include "stm32l4xx_hal.h"// 用于Flash存储操作
#define FLASH_SAVE_ADDR   (0x08080000)//用于存储原始数据
#define FLASH_SAVE_ADDR1  (0x08090000)//用于存储处理后的数据
volatile uint32_t LastAddrWritten = FLASH_SAVE_ADDR1; // 全局变量记录最后写入地址
volatile uint32_t LastAddrWritten1 = FLASH_SAVE_ADDR; // 全局变量记录最后写入地址
//009569000001933D
extern DEVICE_MODE_T device_mode;
extern DEVICE_MODE_T *Device_Mode_str;
extern RTC_HandleTypeDef hrtc;
extern int temperatureReadFlag;
down_list_t *pphead = NULL;
char types[6]={0x01,0x02,0x03,0x04,0x05,0x06};
#define types_index  1
#define StartServerManageFlashAddress    ((u32)0x08036000)//读写起始地址（内部flash的主存储块地址从0x08036000开始）
#define MAX_TEMP_DATA 1440  // 每天最多1440个数据点（1分钟/次）
// 定义存储的起始地址和数据长度
#define FLASH_ADDRESS 0x080E0000  // 假设 Flash 存储起始地址
#define FLASH_PAGE_SIZEs 2048       // Flash 每页大小
//#define FLASH_DATA_SIZE (sizeof(TempData) * MAX_TEMP_DATA) // 总数据大小
typedef struct {
    float temper;
		float MAX_TEMPER;//最高温度
		float MIN_TEMPER;//最低温度	
		float VARIANCE;//方差
		float avg_temp;//平均温度
		float temper_distance;//温差
		float temper_change_rate;//温度变化率
    RTC_TimeTypeDef timestamp;
} TempData;
TempData temp_data_buffer[MAX_TEMP_DATA];
TempData temp_data_backup_buffer[30];

int temp_data_index = 0;  // 当前存储的数据索引
int day=0,day_upload=0,day_flag=0;//存储天数
unsigned int year,month,days;
unsigned char yearLow ,yearLowHex,monthHex,monthHexLow,dayHigh,dayLow,my_byte5,my_byte6;

//处理日常数据
void ProcessDailyData(int temp_data_index) {
		debug_printf("处理数据\n");
    float max_temp = 0, min_temp = 50, sum_temp = 0;
    float variance = 0, rate_of_change = 0;
		float sum_change=0;
		double sum_pow=0;
	 debug_printf("temp_data_index:%d\n",temp_data_index);
    for (int i = 0; i < temp_data_index; i++) {
        sum_temp += temp_data_buffer[i].temper;
			//debug_printf("temp_data_buffer[i].temper:%f\n",temp_data_buffer[i].temper);
        if (temp_data_buffer[i].temper > max_temp) max_temp = temp_data_buffer[i].temper;
        if (temp_data_buffer[i].temper < min_temp) min_temp = temp_data_buffer[i].temper;
    }
    // 保存处理结果至Flash
    // 结果包括：日平均温度、最高温度、最低温度、最大温差、方差、变化率等
		temp_data_backup_buffer[day].MAX_TEMPER = max_temp;//最高温度
		temp_data_backup_buffer[day].MIN_TEMPER = min_temp;//最低温度
		temp_data_backup_buffer[day].temper_distance = max_temp - min_temp;//最大温差
		temp_data_backup_buffer[day].avg_temp = sum_temp / temp_data_index;//平均温度
		
		for (int i = 0; i < temp_data_index-1; i++) {
			sum_change+=temp_data_buffer[i+1].temper-temp_data_buffer[i].temper;
			sum_pow+=pow(temp_data_buffer[i].temper-temp_data_backup_buffer[day].avg_temp,2);
    }
		sum_pow+=pow(temp_data_buffer[temp_data_index].temper-temp_data_backup_buffer[day].avg_temp,2);
		temp_data_backup_buffer[day].VARIANCE=sum_pow;//方差
		temp_data_backup_buffer[day].temper_change_rate = (sum_change / temp_data_index) / temp_data_backup_buffer[day].avg_temp * 100;//变化率
		debug_printf("最高温度: %f\n",		temp_data_backup_buffer[day].MAX_TEMPER);
		debug_printf("最低温度: %f\n",		temp_data_backup_buffer[day].MIN_TEMPER);
		debug_printf("最大温差: %f\n",		temp_data_backup_buffer[day].temper_distance);
		debug_printf("平均温度: %f\n",		temp_data_backup_buffer[day].avg_temp);
		debug_printf("方差: %f\n",		temp_data_backup_buffer[day].VARIANCE);
		debug_printf("变化率: %f\n",		temp_data_backup_buffer[day].temper_change_rate);

}

//-----------------Users application--------------------------
void LoRaWAN_Func_Process(void)
{
    static DEVICE_MODE_T dev_stat = NO_MODE;

    RTC_TimeTypeDef currentTime;
	  RTC_DateTypeDef currentDate;
	
    float temperss = 0,temper_temp=0;
		LCD_ShowString(0,20,"Mode:",PURPLE);
    switch((uint8_t)device_mode)
    {
    /* 指令模式 */
    case CMD_CONFIG_MODE:
    {
								  
				LCD_Fill(40,20,240,50,WHITE);
				LCD_ShowString(40,20,"[Command  Mode]",PURPLE);
        /* 如果不是command Configuration function, 则进入if语句,只执行一次 */
        if(dev_stat != CMD_CONFIG_MODE)
        {
            dev_stat = CMD_CONFIG_MODE;
            debug_printf("\r\n[Command Mode]\r\n");
					
            nodeGpioConfig(wake, wakeup);
            nodeGpioConfig(mode, command);
        }
        /* 等待usart2产生中断 */
        if(UART_TO_PC_RECEIVE_FLAG)
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            lpusart1_send_data(UART_TO_PC_RECEIVE_BUFFER,UART_TO_PC_RECEIVE_LENGTH);
        }
        /* 等待lpuart1产生中断 */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /* 透传模式 */
    case DATA_TRANSPORT_MODE:
    {
				LCD_Fill(40,20,240,50,WHITE);
				LCD_ShowString(40,20,"[Transperant  Mode]",PURPLE);
        /* 如果不是data transport function,则进入if语句,只执行一次 */
        if(dev_stat != DATA_TRANSPORT_MODE)
        {
            dev_stat = DATA_TRANSPORT_MODE;
            debug_printf("\r\n[Transperant Mode]\r\n");

					
            /* 模块入网判断 */
						if(nodeJoinNet(JOIN_TIME_120_SEC)){
						debug_printf("入网成功");
						}
						else{
						debug_printf("入网失败");
						}

            temperss = HDC1000_Read_Temper()/1000;

            nodeDataCommunicate((uint8_t*)&temperss,sizeof(temperss),&pphead);
        }

        /* 等待usart2产生中断 */
        if(UART_TO_PC_RECEIVE_FLAG && GET_BUSY_LEVEL)  //Ensure BUSY is high before sending data
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            nodeDataCommunicate((uint8_t*)UART_TO_PC_RECEIVE_BUFFER, UART_TO_PC_RECEIVE_LENGTH, &pphead);
        }

        /* 如果模块正忙, 则发送数据无效，并给出警告信息 */
        else if(UART_TO_PC_RECEIVE_FLAG && (GET_BUSY_LEVEL == 0))
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            debug_printf("--> Warning: Don't send data now! Module is busy!\r\n");
        }

        /* 等待lpuart1产生中断 */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /*工程模式*/
    case PRO_TRAINING_MODE:
    {
        /* 如果不是Class C云平台数据采集模式, 则进入if语句,只执行一次 */
        if(dev_stat != PRO_TRAINING_MODE)
        {
            dev_stat = PRO_TRAINING_MODE;
            debug_printf("\r\n[Project Mode]\r\n");
				LCD_Fill(40,20,240,50,WHITE);
				LCD_ShowString(40,20,"[Project  Mode]",PURPLE);
        }

				/* 你的实验代码位置 */
				/*温度终端数据处理
				  设备类型为Class A
				   1min/次的速率获取温度传感数值
				*/
				// 读取温度
				if(temperatureReadFlag)
				{
				temperatureReadFlag=0;
				temper_temp=HDC1000_Read_Temper();
				temperss= (float)temper_temp / 65536 * 165 - 40; // 温度转换公式
				LCD_ShowString(0,60,"tempers:",PURPLE);
				LCD_Fill(50,60,240,80,WHITE);
				LCD_ShowFloat(50,60,temperss,2,4,PURPLE);
				debug_printf("\r\n\r\n");
				debug_printf("tempers:%f",temperss);
				debug_printf("\r\n");
			  //获取板子时间，未更新实际时间
				HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);

				debug_printf("%02d/%02d/%02d\r\n",2000 + currentDate.Year, currentDate.Month, currentDate.Date);
				debug_printf("%02d:%02d:%02d\r\n",currentTime.Hours, currentTime.Minutes, currentTime.Seconds);

				year = 2000 + currentDate.Year;
				month = currentDate.Month;
				days = currentDate.Date;
				// 提取年份的低四位
				yearLow = (year % 100) / 16; // 十六进制表示
				yearLowHex = (year % 100) % 16; // 十六进制表示    
				// 月份转换为十六进制
				monthHex = month / 16; // 十六进制表示
				monthHexLow = month % 16; // 十六进制表示   
				// 日期转换为十六进制
				dayHigh = days / 16; // 十六进制表示
				dayLow = days % 16; // 十六进制表示
				// 组合字节5
				my_byte5 = (yearLow << 4) | monthHexLow;    
				// 组合字节6
				my_byte6 = (dayHigh << 4) | dayLow;
				debug_printf("Byte 5 (Year/Month): 0x%02X\n", my_byte5);
				debug_printf("Byte 6 (Day/Reserved): 0x%02X\n", my_byte6);				
        // 将数据存储在缓存中
        if (temp_data_index < 10&&temp_data_index>=0) {
        temp_data_buffer[temp_data_index].temper = temperss;
				debug_printf("temp_data_index:%d\n",temp_data_index);
				debug_printf("temp_data_buffer[temp_data_index]:%f\n",temp_data_buffer[temp_data_index].temper);
        temp_data_buffer[temp_data_index].timestamp = currentTime;      		
            }
				else{
						draw_temperline();
					//每天处理一次
					debug_printf("开始处理每日数据\n");								
					ProcessDailyData(temp_data_index);
					//上传数据
					if(day<30)
					{
						day_upload=day;
					  for(int i=0;i<day_upload+1;i++){
						debug_printf("上传第%d天\n",day_upload+1);
						 uploaddata tempers = {
						.frameHeader = 0xAA,
						.devEui = {0x93, 0x3D},
						.dataType = types[types_index], // 假设发送的是平均温度
						.timestamps = {my_byte5,my_byte6}, 
						.avgTemp = temp_data_backup_buffer[i].avg_temp,//平均温度
						.maxTemp = temp_data_backup_buffer[i].MAX_TEMPER,//最大温度
						.minTemp = temp_data_backup_buffer[i].MIN_TEMPER,//最小温度
						.tempDiff = temp_data_backup_buffer[i].temper_distance,//最大温差
						.variance = temp_data_backup_buffer[i].VARIANCE,//方差
						.tempRate = temp_data_backup_buffer[i].temper_change_rate,//变化率
						.frameFooter = 0x0F
						};
						prepareAndSendTemperData(&tempers);
						//上传失败就跳出
						if(day_flag){day_flag=0;day++;break;}
						else{day=0;}
						}
					}
					else{
						debug_printf("存储数据超出范围");
					}

						//处理数据写入flash	
						my_process_temper_write();
						//my_process_temper_read_uint32_t();
						temp_data_index=0;
						}
						// 数据备份到Flash
						my_ori_temper_write();
						temp_data_index++;	
						//my_ori_temper_read_uint32_t();
				}				
    }
    break;

    default:
        break;
    }
}
//// 定时器中断服务例程，用于每日定时处理数据
//void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
//    // 每日定时触发的回调函数
//    ProcessDailyData();
//}
void draw_temperline(void){
	debug_printf("开始画图\n");
	LCD_Fill(26,81+40,179,265,WHITE);
	for(int i=0;i<temp_data_index-1;i++){
	LCD_DrawLine(25,80+30,25,250+30,BLUE);	
	LCD_DrawLine(25,250+30,180,250+30,BLUE);
  //LCD_ShowNum(0,320-(27-25)*40-80,27,2,BLUE);
	LCD_ShowString(50,90,"termper line",PURPLE);
  LCD_ShowNum(0,275,0,2,BLUE);
	LCD_ShowString(21,108,"^",BLUE);
	LCD_ShowString(175,272,">",BLUE);		
	LCD_DrawLine(i*15+30,320-(temp_data_buffer[i].temper-25)*40-20, (i+1)*15+30,320-(temp_data_buffer[i+1].temper-25)*40-20,PURPLE);
	delay_10ms(10);
	}						

}
void prepareAndSendTemperData(uploaddata *tempers) {
	 debug_printf("上传数据");
    uint8_t buffer[sizeof(uploaddata)];
    
    // 按照格式填充buffer
    memcpy(buffer + 0, &tempers->frameHeader, 1);
    memcpy(buffer + 1, tempers->devEui, 2);
    memcpy(buffer + 3, &tempers->dataType, 1);
    memcpy(buffer + 4, &tempers->timestamps, 2);
    memcpy(buffer + 6, &tempers->avgTemp, sizeof(float));
    memcpy(buffer + 10, &tempers->maxTemp, sizeof(float));
    memcpy(buffer + 14, &tempers->minTemp, sizeof(float));
    memcpy(buffer + 18, &tempers->tempDiff, sizeof(float));
    memcpy(buffer + 22, &tempers->variance, sizeof(float));
    memcpy(buffer + 26, &tempers->tempRate, sizeof(float));
    memcpy(buffer + 30, &tempers->frameFooter, 1);

    // 调用函数发送数据
    nodeDataCommunicate(buffer, sizeof(buffer), &pphead);
	  if(pphead!=NULL){
		debug_printf("回传数据：%s\n",pphead->down_info.business_data);
		//LCD_ShowString(56,170,pphead->down_info.business_data,BLUE);//判断是否有回传数据 
		}
		else{
		debug_printf("没有回传数据，上传失败\n");
		day_flag=1;
		}
		
}
/**
 * @brief   开发板版本信息和按键使用说明信息打印
 * @details 上电所有灯会短暂亮100ms
 * @param   无
 * @return  无
 */
void LoRaWAN_Borad_Info_Print(void)
{
    debug_printf("\r\n\r\n");
    PRINT_CODE_VERSION_INFO("%s",CODE_VERSION);
    debug_printf("\r\n");
    debug_printf("--> Press Key1 to: \r\n");
    debug_printf("-->  - Enter command Mode\r\n");
    debug_printf("-->  - Enter Transparent Mode\r\n");
    debug_printf("--> Press Key2 to: \r\n");
    debug_printf("-->  - Enter Project Trainning Mode\r\n");
    LEDALL_ON;
    HAL_Delay(100);
    LEDALL_OFF;
}
void my_ori_temper_write(void){
	  if(temp_data_index==0){
		LastAddrWritten1=FLASH_SAVE_ADDR;
		}
    // 将数据转换为uint32_t数组以便写入Flash
    uint32_t flash_data[sizeof(TempData) / sizeof(uint32_t)];
	  int32_t length = sizeof(TempData) / sizeof(uint32_t);
	  debug_printf("write temp_data_index:%d\n",temp_data_index);
    // 确保 temp_data_index 在有效范围内
    if (temp_data_index >= 0 && temp_data_index <= MAX_TEMP_DATA) {
        memcpy(flash_data, &temp_data_buffer[temp_data_index - 1], sizeof(TempData));
        
        // 将数据写入Flash
        WriteFlash(LastAddrWritten1, flash_data, length);
			  LastAddrWritten += length * sizeof(uint32_t);
    } else {
        // 如果索引无效，则可以记录错误或者处理异常情况
        debug_printf("Error: Invalid temp_data_index.\n");
    }
}
void my_process_temper_write(void){
	  	  if(day==0){
		LastAddrWritten=FLASH_SAVE_ADDR1;
		}
    // 将数据转换为uint32_t数组以便写入Flash
    uint32_t flash_data[sizeof(TempData) / sizeof(uint32_t)];
    uint32_t length = sizeof(TempData) / sizeof(uint32_t);
    // 确保 temp_data_index 在有效范围内
    if (day <= 30) {
				debug_printf("写入flash第%d天\n",day);
			  memcpy(flash_data, &temp_data_backup_buffer[day], sizeof(TempData));
        
        // 将数据写入Flash
        WriteFlash(LastAddrWritten, flash_data, length);
			  // 更新已使用的最后一个地址
        LastAddrWritten += length * sizeof(uint32_t);			
    } else {
        // 如果索引无效，则可以记录错误或者处理异常情况
        debug_printf("Error: Invalid temp_data_index.\n");
    }
}
void my_ori_temper_read_uint32_t(void)
{
	// 将读取到的数据转换回原始结构体
	// 从Flash中读取数据
	uint32_t flash_data_read[sizeof(TempData) / sizeof(uint32_t)];
	Flash_Read(FLASH_SAVE_ADDR, flash_data_read, sizeof(TempData) / sizeof(uint32_t));
	memcpy(flash_data_read,&temp_data_buffer[temp_data_index - 1], sizeof(TempData));
}
void my_process_temper_read_uint32_t(void){
    // 将读取到的数据转换回原始结构体
		// 从Flash中读取数据
    uint32_t flash_data_read[sizeof(TempData) / sizeof(uint32_t)];
    Flash_Read(FLASH_SAVE_ADDR1, flash_data_read, sizeof(TempData) / sizeof(uint32_t));
		memcpy(flash_data_read,&temp_data_backup_buffer[day], sizeof(TempData));	
}
