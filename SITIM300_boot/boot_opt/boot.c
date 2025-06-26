
#include "boot.h"
#include "uart_x.h"

#define FLASH_WAITETIME 50000//FLASH等待超时时间

#define	start_affirm  0x01    //升级确认
#define	up_mode  			0x02		//升级模式
#define	recv_succeed  0x03		//接收完成
#define	data_error  	0x04		//数据错误
#define	data_true  		0x05   	//数据正确
#define	up_data  			0x06    //升级数据

#define	load_A  			0x0A		//程序A
#define	load_B  			0x0B		//程序B
#define	load_C  			0x0C		//设备
#define	load_D  			0x0D		//算法
#define	load_E  			0x0E		//客户
#define	load_F  			0x0F		//备用
	
const uint32_t  load_flash_addr[3] = {
//0x08040000,//程序A
//0x08060000,//程序B
//0x08080000,//设备
//0x080A0000,//算法
//0x080C0000,//客户
//0x080E0000,//备用
	0x08020000,//程序A
  0x08020000,//程序A
	0x08060000,//设备参数
	
};
extern uint32_t cnt_jump;	
extern uint32_t COM1_EN;
extern uint32_t COM_check_flag;
uint8_t  phase = 0;
uint16_t crcy;
uint8_t  *RX_BUF;
uint16_t data_len;
uint32_t flash_addr;
uint32_t load_addr;
uint8_t  Cmd_type;
uint8_t  version_change = 0;//manual change version number
uint8_t  forbid_write_flag = 0;
Cal_Time_t watch_tim0;

uint8_t deal_recv(COMM_t * com)
{	
	RX_BUF = com->rxBuff;

	switch(phase)
	{
		case 0:						
						if(com->rxValid == 0) return 1;		
						com->rxValid =0;
			
						if(RX_BUF[0] != 0x55 || RX_BUF[1] != 0xaa) return 1;

						if(COM_check_flag)
						{
							COM_check_flag = 0;
//							if(com == &com_u1)	COM1_EN = 0;
						}

		
						data_len = RX_BUF[2] | (RX_BUF[3] << 8);
						crcy = Crc16Ccitt(&RX_BUF[4],data_len);
						if(((RX_BUF[data_len + 5] << 8) | RX_BUF[data_len + 4]) != crcy) return 1;
		
						Cmd_type = RX_BUF[4];												
						if(Cmd_type >= load_A && Cmd_type <= load_F)
						{								
								send_to_pc(start_affirm,com);
								load_addr = flash_addr = load_flash_addr[Cmd_type-load_A];
						}
						else if(Cmd_type == start_affirm)//主机发确认
						{								
								send_to_pc(up_mode,com);
						}
						else if(Cmd_type == up_data)//主机发10240 bin文件
						{		
								//2: 0D 0A ,  23:location of version in data package, version_change: manual change version number,
								if(RX_BUF[18] == '1') version_change = 1;
								else if(RX_BUF[18] == '0') version_change = 0;
								else version_change = 0;
							
//								if((load_addr==0x08060000) && version_change == 0)
//								{
//									if((str_compare((char *)(flash_addr+strlen("#STIM300,0#")+2), (char *)&RX_BUF[22],strlen("2500301"))) )
//									return 1;																	
//								}	
								TimeStart(watch_tim0);								
								H7flash_write(flash_addr,(uint32_t *)&RX_BUF[9],data_len-5);						
								flash_addr+=(data_len-5);		
								TimeEnd(watch_tim0);
								
								send_to_pc(data_true,com);
									
								version_change = 0;
								
								if(RX_BUF[5]==RX_BUF[7])//所有bin发完成
								phase = 1;
						}
		break;		
		case 1:							
						if(com->txValid)  break;							 
						send_to_pc(recv_succeed,com);
						phase = 2;
		break;
		case 2:
						if(com->txValid)  break;
						HAL_Delay(10);
						__disable_irq();
						HAL_RCC_DeInit();                           
						HAL_DeInit();                               
						__disable_irq();
						__HAL_RCC_USART1_FORCE_RESET();
						__HAL_RCC_USART1_RELEASE_RESET();
						iap_load_app(0x08020000);					
		break;
	}
}

uint8_t str_compare(char *s1, char *s2,uint32_t len)
{
	uint32_t i;
	for(i=0;i<len;i++)
	{
		if(s1[i]!=s2[i])
			break;
	}
	if(i != len)
		return 1;
	
	return 0;
	
}
void (*iapfun)(void);
void iap_load_app(uint32_t appxaddr)
{ 
		iapfun = 	(void (*)(void))  (*(uint32_t*)(appxaddr+4));	
		__set_MSP(*(uint32_t*)appxaddr);
		iapfun();	
}	


uint32_t GetSector(uint32_t Address)
{
  return (Address - FLASH_BASE) / FLASH_SECTOR_SIZE;
}

//读取指定地址的字(32位数据) 
//faddr:读地址 
//返回值:对应数据.

uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t *)faddr; 
}

uint8_t H7flash_write(uint32_t start_addr, uint32_t *pBuffer, uint32_t NumToWrite)
{
    HAL_StatusTypeDef FlashStatus = HAL_OK;
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t end_addr = start_addr + NumToWrite;
    uint32_t SectorError = 0;

    if (NumToWrite == 0 || start_addr < 0x08020000 || start_addr % 4 || end_addr > 0x08080000)
        return 1;

    HAL_FLASH_Unlock();

    uint32_t addrx = start_addr;
    while (addrx < end_addr) {
        if (STMFLASH_ReadWord(addrx) != 0xFFFFFFFF) {
            FlashEraseInit.Banks = FLASH_BANK_1;
            FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
            FlashEraseInit.Sector = GetSector(addrx);
            FlashEraseInit.NbSectors = 1;
            FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);
            FlashStatus = HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError);
            if (FlashStatus != HAL_OK)
                return 1;
        } else {
            addrx += 4;
        }
        FLASH_WaitForLastOperation(FLASH_WAITETIME, FLASH_BANK_1);
    }

    FlashStatus = FLASH_WaitForLastOperation(FLASH_WAITETIME, FLASH_BANK_1);
    if (FlashStatus == HAL_OK) 
		{
        while (start_addr + 32 <= end_addr) 
				{
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, start_addr, (uint32_t)pBuffer) != HAL_OK)
                break;
            start_addr += 32;
            pBuffer += 8;
        }

        // 处理最后一块不足 32 字节的情况
        uint32_t remain = end_addr - start_addr;
        if (remain > 0) 
				{
            uint8_t temp_buf[32];
            memset(temp_buf, 0xFF, 32);
            memcpy(temp_buf, (uint8_t *)pBuffer, remain);

            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, start_addr, (uint32_t )temp_buf) != HAL_OK)
                return 1;
        }
    }

    HAL_FLASH_Lock();
    return 0;
}


void send_to_ws(COMM_t * com)
{
	com->txBuff[0] = 0x0B;
	com->txBuff[1] = 0x0B;
	com->txLen = 2;
	com->txValid = 1;
}
void send_to_pc(uint8_t status,COMM_t * com)
{
	uint16_t crc16,sendlen;


	com->txBuff[0] = 0x55;    //数据的帧头
	com->txBuff[1] = 0xaa;

	com->txBuff[2] = 0x01;
	com->txBuff[3] = 0x00;

	com->txBuff[4] = status;

	sendlen = com->txBuff[2] | (com->txBuff[3] << 8);

	crc16 = Crc16Ccitt(&com->txBuff[4],sendlen);
	com->txBuff[5] = crc16 & 0xff;
	com->txBuff[6] = (crc16 >> 8) & 0xff;

	com->txLen = sendlen + 6;
	com->txValid = 1;
}
const  unsigned short ccitt_table[256] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
		0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
		0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
		0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
		0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
		0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
		0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
		0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
		0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
		0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
		0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
		0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
		0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
		0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
		0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
		0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
		0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
		0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
		0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
		0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
		0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
		0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
		0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
	};
unsigned short Crc16Ccitt(uint8_t *q, unsigned int len)
{
	unsigned short crc = 0;
	while (len-- > 0)
		crc = ccitt_table[(crc >> 8 ^ *q++) & 0xff] ^ (crc << 8);
	return crc;
}


