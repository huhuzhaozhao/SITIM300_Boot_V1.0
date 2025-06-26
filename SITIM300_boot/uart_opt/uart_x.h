
#ifndef __UART_X_H__
#define __UART_X_H__

#include "main.h"
#include "tim.h"
#include "string.h"

#define usCounter  (htim2.Instance->CNT)
//#define MAX_LEN_TX  2048
//#define MAX_LEN_RX  8192

#define MAX_LEN_TX  2048
#define MAX_LEN_RX  8192


typedef struct
{
uint32_t start_us,bak_us,period_us;
uint32_t max_us,set_us1,set_us2,cost_us;
} Cal_Time_t;

#define TimeStart(x) do {		\
				x.period_us=usCounter-x.bak_us;\
				x.bak_us=usCounter;\
				x.start_us = usCounter;\
	}while(0)
#define TimeEnd(x) do {		\
				x.cost_us = usCounter - x.start_us;\
				if(x.max_us<x.cost_us) x.max_us=x.cost_us;\
				if(6000<x.cost_us) x.set_us1=x.cost_us;\
				if(100<x.cost_us) x.set_us2=x.cost_us;\
	}while(0)

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;
#pragma pack(1)
typedef struct
{
	uint32_t txValid,txLen,rxValid,rxLen;
	uint8_t txBuff[MAX_LEN_TX];
	uint8_t rxBuff[MAX_LEN_RX];
		
	uint32_t tmpTest1;
		
	uint32_t NUM_bak;
	uint8_t checkPass;
	uint8_t phaseRx,phaseTx;
	uint32_t tickStart_us,tickStart_us_tx;
	uint8_t d_tx_8[8],d_rx_8[8];
} COMM_t;
#pragma pack()
extern COMM_t com_u4;
extern COMM_t com_u5;

void initUartDma(UART_HandleTypeDef * huart,COMM_t * comm);
void uart_dma__run(UART_HandleTypeDef * uart_x,COMM_t * comm);
uint8_t com_test(COMM_t * comm);

int putcharx(int c);
void putstr(const char *str);
void puthex(unsigned int val);
unsigned int str2hex(const char* s);
uint8_t com_transpond(COMM_t * comA,COMM_t * comB);


uint8_t cal_xor(uint8_t* data, uint8_t length);
uint8_t cal_sum(uint8_t* data, uint8_t length);

#endif /* __UART_X_H__ */

