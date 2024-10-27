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
#include "stm32l4xx_hal_rtc.h"// RTC��ʱ�������ڻ�ȡ��ǰʱ���
#include "stm32l4xx_hal.h"// ����Flash�洢����
#define FLASH_SAVE_ADDR   (0x08080000)//���ڴ洢ԭʼ����
#define FLASH_SAVE_ADDR1  (0x08090000)//���ڴ洢����������
volatile uint32_t LastAddrWritten = FLASH_SAVE_ADDR1; // ȫ�ֱ�����¼���д���ַ
volatile uint32_t LastAddrWritten1 = FLASH_SAVE_ADDR; // ȫ�ֱ�����¼���д���ַ
//009569000001933D
extern DEVICE_MODE_T device_mode;
extern DEVICE_MODE_T *Device_Mode_str;
extern RTC_HandleTypeDef hrtc;
extern int temperatureReadFlag;
down_list_t *pphead = NULL;
char types[6]={0x01,0x02,0x03,0x04,0x05,0x06};
#define types_index  1
#define StartServerManageFlashAddress    ((u32)0x08036000)//��д��ʼ��ַ���ڲ�flash�����洢���ַ��0x08036000��ʼ��
#define MAX_TEMP_DATA 1440  // ÿ�����1440�����ݵ㣨1����/�Σ�
// ����洢����ʼ��ַ�����ݳ���
#define FLASH_ADDRESS 0x080E0000  // ���� Flash �洢��ʼ��ַ
#define FLASH_PAGE_SIZEs 2048       // Flash ÿҳ��С
//#define FLASH_DATA_SIZE (sizeof(TempData) * MAX_TEMP_DATA) // �����ݴ�С
typedef struct {
    float temper;
		float MAX_TEMPER;//����¶�
		float MIN_TEMPER;//����¶�	
		float VARIANCE;//����
		float avg_temp;//ƽ���¶�
		float temper_distance;//�²�
		float temper_change_rate;//�¶ȱ仯��
    RTC_TimeTypeDef timestamp;
} TempData;
TempData temp_data_buffer[MAX_TEMP_DATA];
TempData temp_data_backup_buffer[30];

int temp_data_index = 0;  // ��ǰ�洢����������
int day=0,day_upload=0,day_flag=0;//�洢����
unsigned int year,month,days;
unsigned char yearLow ,yearLowHex,monthHex,monthHexLow,dayHigh,dayLow,my_byte5,my_byte6;

//�����ճ�����
void ProcessDailyData(int temp_data_index) {
		debug_printf("��������\n");
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
    // ���洦������Flash
    // �����������ƽ���¶ȡ�����¶ȡ�����¶ȡ�����²����仯�ʵ�
		temp_data_backup_buffer[day].MAX_TEMPER = max_temp;//����¶�
		temp_data_backup_buffer[day].MIN_TEMPER = min_temp;//����¶�
		temp_data_backup_buffer[day].temper_distance = max_temp - min_temp;//����²�
		temp_data_backup_buffer[day].avg_temp = sum_temp / temp_data_index;//ƽ���¶�
		
		for (int i = 0; i < temp_data_index-1; i++) {
			sum_change+=temp_data_buffer[i+1].temper-temp_data_buffer[i].temper;
			sum_pow+=pow(temp_data_buffer[i].temper-temp_data_backup_buffer[day].avg_temp,2);
    }
		sum_pow+=pow(temp_data_buffer[temp_data_index].temper-temp_data_backup_buffer[day].avg_temp,2);
		temp_data_backup_buffer[day].VARIANCE=sum_pow;//����
		temp_data_backup_buffer[day].temper_change_rate = (sum_change / temp_data_index) / temp_data_backup_buffer[day].avg_temp * 100;//�仯��
		debug_printf("����¶�: %f\n",		temp_data_backup_buffer[day].MAX_TEMPER);
		debug_printf("����¶�: %f\n",		temp_data_backup_buffer[day].MIN_TEMPER);
		debug_printf("����²�: %f\n",		temp_data_backup_buffer[day].temper_distance);
		debug_printf("ƽ���¶�: %f\n",		temp_data_backup_buffer[day].avg_temp);
		debug_printf("����: %f\n",		temp_data_backup_buffer[day].VARIANCE);
		debug_printf("�仯��: %f\n",		temp_data_backup_buffer[day].temper_change_rate);

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
    /* ָ��ģʽ */
    case CMD_CONFIG_MODE:
    {
								  
				LCD_Fill(40,20,240,50,WHITE);
				LCD_ShowString(40,20,"[Command  Mode]",PURPLE);
        /* �������command Configuration function, �����if���,ִֻ��һ�� */
        if(dev_stat != CMD_CONFIG_MODE)
        {
            dev_stat = CMD_CONFIG_MODE;
            debug_printf("\r\n[Command Mode]\r\n");
					
            nodeGpioConfig(wake, wakeup);
            nodeGpioConfig(mode, command);
        }
        /* �ȴ�usart2�����ж� */
        if(UART_TO_PC_RECEIVE_FLAG)
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            lpusart1_send_data(UART_TO_PC_RECEIVE_BUFFER,UART_TO_PC_RECEIVE_LENGTH);
        }
        /* �ȴ�lpuart1�����ж� */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /* ͸��ģʽ */
    case DATA_TRANSPORT_MODE:
    {
				LCD_Fill(40,20,240,50,WHITE);
				LCD_ShowString(40,20,"[Transperant  Mode]",PURPLE);
        /* �������data transport function,�����if���,ִֻ��һ�� */
        if(dev_stat != DATA_TRANSPORT_MODE)
        {
            dev_stat = DATA_TRANSPORT_MODE;
            debug_printf("\r\n[Transperant Mode]\r\n");

					
            /* ģ�������ж� */
						if(nodeJoinNet(JOIN_TIME_120_SEC)){
						debug_printf("�����ɹ�");
						}
						else{
						debug_printf("����ʧ��");
						}

            temperss = HDC1000_Read_Temper()/1000;

            nodeDataCommunicate((uint8_t*)&temperss,sizeof(temperss),&pphead);
        }

        /* �ȴ�usart2�����ж� */
        if(UART_TO_PC_RECEIVE_FLAG && GET_BUSY_LEVEL)  //Ensure BUSY is high before sending data
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            nodeDataCommunicate((uint8_t*)UART_TO_PC_RECEIVE_BUFFER, UART_TO_PC_RECEIVE_LENGTH, &pphead);
        }

        /* ���ģ����æ, ����������Ч��������������Ϣ */
        else if(UART_TO_PC_RECEIVE_FLAG && (GET_BUSY_LEVEL == 0))
        {
            UART_TO_PC_RECEIVE_FLAG = 0;
            debug_printf("--> Warning: Don't send data now! Module is busy!\r\n");
        }

        /* �ȴ�lpuart1�����ж� */
        if(UART_TO_LRM_RECEIVE_FLAG)
        {
            UART_TO_LRM_RECEIVE_FLAG = 0;
            usart2_send_data(UART_TO_LRM_RECEIVE_BUFFER,UART_TO_LRM_RECEIVE_LENGTH);
        }
    }
    break;

    /*����ģʽ*/
    case PRO_TRAINING_MODE:
    {
        /* �������Class C��ƽ̨���ݲɼ�ģʽ, �����if���,ִֻ��һ�� */
        if(dev_stat != PRO_TRAINING_MODE)
        {
            dev_stat = PRO_TRAINING_MODE;
            debug_printf("\r\n[Project Mode]\r\n");
				LCD_Fill(40,20,240,50,WHITE);
				LCD_ShowString(40,20,"[Project  Mode]",PURPLE);
        }

				/* ���ʵ�����λ�� */
				/*�¶��ն����ݴ���
				  �豸����ΪClass A
				   1min/�ε����ʻ�ȡ�¶ȴ�����ֵ
				*/
				// ��ȡ�¶�
				if(temperatureReadFlag)
				{
				temperatureReadFlag=0;
				temper_temp=HDC1000_Read_Temper();
				temperss= (float)temper_temp / 65536 * 165 - 40; // �¶�ת����ʽ
				LCD_ShowString(0,60,"tempers:",PURPLE);
				LCD_Fill(50,60,240,80,WHITE);
				LCD_ShowFloat(50,60,temperss,2,4,PURPLE);
				debug_printf("\r\n\r\n");
				debug_printf("tempers:%f",temperss);
				debug_printf("\r\n");
			  //��ȡ����ʱ�䣬δ����ʵ��ʱ��
				HAL_RTC_GetTime(&hrtc, &currentTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &currentDate, RTC_FORMAT_BIN);

				debug_printf("%02d/%02d/%02d\r\n",2000 + currentDate.Year, currentDate.Month, currentDate.Date);
				debug_printf("%02d:%02d:%02d\r\n",currentTime.Hours, currentTime.Minutes, currentTime.Seconds);

				year = 2000 + currentDate.Year;
				month = currentDate.Month;
				days = currentDate.Date;
				// ��ȡ��ݵĵ���λ
				yearLow = (year % 100) / 16; // ʮ�����Ʊ�ʾ
				yearLowHex = (year % 100) % 16; // ʮ�����Ʊ�ʾ    
				// �·�ת��Ϊʮ������
				monthHex = month / 16; // ʮ�����Ʊ�ʾ
				monthHexLow = month % 16; // ʮ�����Ʊ�ʾ   
				// ����ת��Ϊʮ������
				dayHigh = days / 16; // ʮ�����Ʊ�ʾ
				dayLow = days % 16; // ʮ�����Ʊ�ʾ
				// ����ֽ�5
				my_byte5 = (yearLow << 4) | monthHexLow;    
				// ����ֽ�6
				my_byte6 = (dayHigh << 4) | dayLow;
				debug_printf("Byte 5 (Year/Month): 0x%02X\n", my_byte5);
				debug_printf("Byte 6 (Day/Reserved): 0x%02X\n", my_byte6);				
        // �����ݴ洢�ڻ�����
        if (temp_data_index < 10&&temp_data_index>=0) {
        temp_data_buffer[temp_data_index].temper = temperss;
				debug_printf("temp_data_index:%d\n",temp_data_index);
				debug_printf("temp_data_buffer[temp_data_index]:%f\n",temp_data_buffer[temp_data_index].temper);
        temp_data_buffer[temp_data_index].timestamp = currentTime;      		
            }
				else{
						draw_temperline();
					//ÿ�촦��һ��
					debug_printf("��ʼ����ÿ������\n");								
					ProcessDailyData(temp_data_index);
					//�ϴ�����
					if(day<30)
					{
						day_upload=day;
					  for(int i=0;i<day_upload+1;i++){
						debug_printf("�ϴ���%d��\n",day_upload+1);
						 uploaddata tempers = {
						.frameHeader = 0xAA,
						.devEui = {0x93, 0x3D},
						.dataType = types[types_index], // ���跢�͵���ƽ���¶�
						.timestamps = {my_byte5,my_byte6}, 
						.avgTemp = temp_data_backup_buffer[i].avg_temp,//ƽ���¶�
						.maxTemp = temp_data_backup_buffer[i].MAX_TEMPER,//����¶�
						.minTemp = temp_data_backup_buffer[i].MIN_TEMPER,//��С�¶�
						.tempDiff = temp_data_backup_buffer[i].temper_distance,//����²�
						.variance = temp_data_backup_buffer[i].VARIANCE,//����
						.tempRate = temp_data_backup_buffer[i].temper_change_rate,//�仯��
						.frameFooter = 0x0F
						};
						prepareAndSendTemperData(&tempers);
						//�ϴ�ʧ�ܾ�����
						if(day_flag){day_flag=0;day++;break;}
						else{day=0;}
						}
					}
					else{
						debug_printf("�洢���ݳ�����Χ");
					}

						//��������д��flash	
						my_process_temper_write();
						//my_process_temper_read_uint32_t();
						temp_data_index=0;
						}
						// ���ݱ��ݵ�Flash
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
//// ��ʱ���жϷ������̣�����ÿ�ն�ʱ��������
//void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
//    // ÿ�ն�ʱ�����Ļص�����
//    ProcessDailyData();
//}
void draw_temperline(void){
	debug_printf("��ʼ��ͼ\n");
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
	 debug_printf("�ϴ�����");
    uint8_t buffer[sizeof(uploaddata)];
    
    // ���ո�ʽ���buffer
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

    // ���ú�����������
    nodeDataCommunicate(buffer, sizeof(buffer), &pphead);
	  if(pphead!=NULL){
		debug_printf("�ش����ݣ�%s\n",pphead->down_info.business_data);
		//LCD_ShowString(56,170,pphead->down_info.business_data,BLUE);//�ж��Ƿ��лش����� 
		}
		else{
		debug_printf("û�лش����ݣ��ϴ�ʧ��\n");
		day_flag=1;
		}
		
}
/**
 * @brief   ������汾��Ϣ�Ͱ���ʹ��˵����Ϣ��ӡ
 * @details �ϵ����еƻ������100ms
 * @param   ��
 * @return  ��
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
    // ������ת��Ϊuint32_t�����Ա�д��Flash
    uint32_t flash_data[sizeof(TempData) / sizeof(uint32_t)];
	  int32_t length = sizeof(TempData) / sizeof(uint32_t);
	  debug_printf("write temp_data_index:%d\n",temp_data_index);
    // ȷ�� temp_data_index ����Ч��Χ��
    if (temp_data_index >= 0 && temp_data_index <= MAX_TEMP_DATA) {
        memcpy(flash_data, &temp_data_buffer[temp_data_index - 1], sizeof(TempData));
        
        // ������д��Flash
        WriteFlash(LastAddrWritten1, flash_data, length);
			  LastAddrWritten += length * sizeof(uint32_t);
    } else {
        // ���������Ч������Լ�¼������ߴ����쳣���
        debug_printf("Error: Invalid temp_data_index.\n");
    }
}
void my_process_temper_write(void){
	  	  if(day==0){
		LastAddrWritten=FLASH_SAVE_ADDR1;
		}
    // ������ת��Ϊuint32_t�����Ա�д��Flash
    uint32_t flash_data[sizeof(TempData) / sizeof(uint32_t)];
    uint32_t length = sizeof(TempData) / sizeof(uint32_t);
    // ȷ�� temp_data_index ����Ч��Χ��
    if (day <= 30) {
				debug_printf("д��flash��%d��\n",day);
			  memcpy(flash_data, &temp_data_backup_buffer[day], sizeof(TempData));
        
        // ������д��Flash
        WriteFlash(LastAddrWritten, flash_data, length);
			  // ������ʹ�õ����һ����ַ
        LastAddrWritten += length * sizeof(uint32_t);			
    } else {
        // ���������Ч������Լ�¼������ߴ����쳣���
        debug_printf("Error: Invalid temp_data_index.\n");
    }
}
void my_ori_temper_read_uint32_t(void)
{
	// ����ȡ��������ת����ԭʼ�ṹ��
	// ��Flash�ж�ȡ����
	uint32_t flash_data_read[sizeof(TempData) / sizeof(uint32_t)];
	Flash_Read(FLASH_SAVE_ADDR, flash_data_read, sizeof(TempData) / sizeof(uint32_t));
	memcpy(flash_data_read,&temp_data_buffer[temp_data_index - 1], sizeof(TempData));
}
void my_process_temper_read_uint32_t(void){
    // ����ȡ��������ת����ԭʼ�ṹ��
		// ��Flash�ж�ȡ����
    uint32_t flash_data_read[sizeof(TempData) / sizeof(uint32_t)];
    Flash_Read(FLASH_SAVE_ADDR1, flash_data_read, sizeof(TempData) / sizeof(uint32_t));
		memcpy(flash_data_read,&temp_data_backup_buffer[day], sizeof(TempData));	
}
