#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
#include "lorawan_node_driver.h"
#include "usart.h"
#include "dma.h"
#include "rtc.h"
#include "gpio.h"
#include "tim.h"
#include "i2c.h"
#include "hdc1000.h"
#include "opt3001.h"
#include "MPL3115.h"
#include "mma8451.h"
#include "XPT2046.h"
#include "flash.h"
#define FLASH_SAVE_ADDR   (0x08080000)//���ڴ洢ԭʼ����
#define FLASH_SAVE_ADDR1  (0x08090000)//���ڴ洢����������
void Flash_Read(uint32_t address, uint32_t *data, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++) {
        data[i] = *(__IO uint32_t *)(address + (i * 8)); // ����Ϊ��λ��ȡFlash
			debug_printf("read_data = %d\r\n",data[i]);
    }
}

// ����洢����
void WriteFlash(uint32_t addr,uint32_t *Data,uint32_t L)
{
	uint32_t i=0;
	/* 1/4����FLASH*/
	HAL_FLASH_Unlock();
	/* 2/4����FLASH*/
	/*��ʼ��FLASH_EraseInitTypeDef*/
	/*������ʽҳ����FLASH_TYPEERASE_PAGES�������FLASH_TYPEERASE_MASSERASE*/
	/*����ҳ��*/
	/*������ַ*/
	FLASH_EraseInitTypeDef FlashSet;
	FlashSet.TypeErase = FLASH_TYPEERASE_PAGES;
	FlashSet.Page = addr;
	FlashSet.NbPages = 2;
	/*����PageError�����ò�������*/
	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&FlashSet, &PageError);
	/* 3/4��FLASH��д*/
	for(i=0;i<L;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr+8*i, Data[i]);
		//debug_printf("write_data = %d\r\n",Data[i]);
	}
	/* 4/4��סFLASH*/
	HAL_FLASH_Lock();
}

void my_flash_write_uint32_t(void)
{
	// ������ת��Ϊuint32_t�����Ա�д��Flash
	uint32_t flash_data[sizeof(int_data) / sizeof(uint32_t)];
	memcpy(flash_data, &int_data, sizeof(int_data));
	
	// ������д��Flash
	WriteFlash(FLASH_SAVE_ADDR, flash_data, sizeof(int_data) / sizeof(uint32_t));
}
void my_flash_read_uint32_t(void)
{
	// ����ȡ��������ת����ԭʼ�ṹ��
	// ��Flash�ж�ȡ����
	uint32_t flash_data_read[sizeof(int_data) / sizeof(uint32_t)];
	Flash_Read(FLASH_SAVE_ADDR, flash_data_read, sizeof(int_data) / sizeof(uint32_t));
	memcpy(&int_data, flash_data_read, sizeof(int_data));
}


