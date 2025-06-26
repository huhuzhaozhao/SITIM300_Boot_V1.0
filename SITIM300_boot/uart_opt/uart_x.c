
#include "uart_x.h"

 __attribute__((section (".RAM_D1"))) COMM_t com_u4 ;//__attribute__((section(".ARM.__at_0x24000000")));
 //__attribute__((section (".RAM_D1"))) COMM_t com_u5 ;//__attribute__((section(".ARM.__at_0x24000000")));

void uart_dma__run(UART_HandleTypeDef * uart_x,COMM_t * comm) 
{	uint8_t dummy;

	switch(comm->phaseTx)
	{
	case 0:
		if (comm->txValid)
		{	
				if(comm->txLen ==0 ) 
				{comm->txValid = 0;break;}			
				((DMA_Base_Registers *)uart_x->hdmatx->StreamBaseAddress)->IFCR =0x3FUL << (uart_x->hdmatx->StreamIndex & 0x1FU);
				((DMA_Stream_TypeDef *)uart_x->hdmatx->Instance)->PAR = (uint32_t)&(uart_x->Instance->TDR);
				((DMA_Stream_TypeDef *)uart_x->hdmatx->Instance)->NDTR = comm->txLen;
				((DMA_Stream_TypeDef *)uart_x->hdmatx->Instance)->M0AR =(uint32_t)&comm->txBuff[0];// SrcAddress;
				

				__HAL_DMA_ENABLE(uart_x->hdmatx);
				comm->phaseTx=1;
				//SCB_CleanInvalidateDCache_by_Addr ((uint32_t *)comm->txBuff,MAX_LEN_TX);
			
		}
	break;
	case 1:
		
		if( 	(__HAL_DMA_GET_FLAG(uart_x->hdmatx,__HAL_DMA_GET_TC_FLAG_INDEX(uart_x->hdmatx)))
					&&	(uart_x->Instance->ISR & UART_FLAG_TC)
		  )//DMA_USART传输完成
		{
			comm->txValid=0;
			comm->tickStart_us_tx=usCounter;
			comm->phaseTx=2;
		}
	break;
	case 2:
		if((usCounter - comm->tickStart_us_tx) > 100 )
			comm->phaseTx=0;
	break;
	default: comm->phaseTx=0;
	}

	
	switch(comm->phaseRx)
	{
	case 0:
		if (comm->rxValid==0)
		{	//---------------------配置DMA接收usart数据---------------------------
			(uart_x->Instance->CR1)&= ~USART_CR1_RE;//关闭USART接收
				__HAL_DMA_DISABLE(uart_x->hdmarx);
					if(uart_x->Instance->ISR & UART_FLAG_RXNE)
						dummy=(uint8_t)(uart_x->Instance->RDR );
					((DMA_Base_Registers *)uart_x->hdmarx->StreamBaseAddress)->IFCR = 0x3FUL << (uart_x->hdmarx->StreamIndex & 0x1FU);
					((DMA_Stream_TypeDef *)uart_x->hdmarx->Instance)->NDTR =MAX_LEN_RX;//DataLength;
					((DMA_Stream_TypeDef *)uart_x->hdmarx->Instance)->M0AR =(uint32_t)&comm->rxBuff[0];//DesAddress;
					((DMA_Stream_TypeDef *)uart_x->hdmarx->Instance)->PAR = (uint32_t)&(uart_x->Instance->RDR);

				__HAL_DMA_ENABLE(uart_x->hdmarx);
			(uart_x->Instance->CR1)|= USART_CR1_RE;//打开USART接收
			comm->phaseRx=1;
		}
	break;
	case 1:
		if (  ((DMA_Stream_TypeDef *)uart_x->hdmarx->Instance)->NDTR !=MAX_LEN_RX )//start a new package receive
		{	comm->NUM_bak=((DMA_Stream_TypeDef *)uart_x->hdmarx->Instance)->NDTR;
			comm->tickStart_us=usCounter;//HAL_GetTick();//deltaUs=htim2.Instance->CNT-startUs;
			comm->phaseRx=2;
		}
		else if((((DMA_Stream_TypeDef *)uart_x->hdmarx->Instance)->CR & 0x00000001) == 0)
		{
			comm->rxValid = 0;
			comm->phaseRx = 0;
		}
	break;
	case 2:		
		if (((DMA_Stream_TypeDef *)uart_x->hdmarx->Instance)->NDTR != comm->NUM_bak)
		{	comm->tickStart_us=usCounter;
			comm->NUM_bak=((DMA_Stream_TypeDef *)uart_x->hdmarx->Instance)->NDTR;
		}
		else if ( usCounter - comm->tickStart_us > 100)//判断帧结束
		{	comm->tmpTest1=usCounter;
			comm->rxLen=MAX_LEN_RX-comm->NUM_bak;
			comm->rxValid=1;
			comm->phaseRx=0;
			//SCB_CleanInvalidateDCache_by_Addr ((uint32_t *)comm->rxBuff,MAX_LEN_RX);
		}
	break;
	default: comm->phaseRx=0;
	}
}

Cal_Time_t cal_time3;
uint8_t send_cnt;
uint8_t com_test(COMM_t * comm)
{
			if(comm->rxValid == 0) 
			return 1;			
			comm->rxValid = 0;
						
			memcpy(comm->txBuff,comm->rxBuff,comm->rxLen);
			comm->txLen = comm->rxLen;
			comm->txValid = 1;
}



uint8_t com_transpond(COMM_t * comA,COMM_t * comB)
{					
		if(comA->rxValid)
		{
			comA->rxValid = 0;
			comB->txValid = 1;
			comB->txLen = comA->rxLen;
			memcpy(comB->txBuff,comA->rxBuff,comA->rxLen);
		}
		if(comB->rxValid)
		{
			comB->rxValid = 0;
			comA->txValid = 1;
			comA->txLen = comB->rxLen;
			memcpy(comA->txBuff,comB->rxBuff,comB->rxLen);
		}
}

//void initUartDma(UART_HandleTypeDef * huart,COMM_t * comm)
//{

//	__HAL_DMA_DISABLE(huart->hdmatx);
//		((DMA_Base_Registers *)huart->hdmatx->StreamBaseAddress)->IFCR = 0x3FUL << (huart->hdmatx->StreamIndex & 0x1FU);
//		((DMA_Stream_TypeDef *)huart->hdmatx->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);/* Clear DBM bit */
//		((DMA_Stream_TypeDef *)huart->hdmatx->Instance)->NDTR = 2;//DataLength;/* Configure DMA Stream data length */
//		((DMA_Stream_TypeDef *)huart->hdmatx->Instance)->PAR = (uint32_t) &huart->Instance->TDR;//DstAddress;/*  Memory to Peripheral  Configure DMA Stream destination address */
//		((DMA_Stream_TypeDef *)huart->hdmatx->Instance)->M0AR =(uint32_t) &comm->d_tx_8[0];// SrcAddress;/* Configure DMA Stream source address */
//		SET_BIT(huart->Instance->CR3, USART_CR3_DMAT); //Enable the DMA transfer for transmit request by setting the DMAT bit in the UART CR3 register

//	
//	__HAL_DMA_DISABLE(huart->hdmarx);// Disable the peripheral Configure the source, destination address and the data length
//		((DMA_Base_Registers *)huart->hdmarx->StreamBaseAddress)->IFCR = 0x3FUL << (huart->hdmarx->StreamIndex & 0x1FU);//regs_dma->IFCR = 0x3FUL << (hdma->StreamIndex & 0x1FU);
//		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);/* Clear DBM bit */
//		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR =2;//DataLength;/* Configure DMA Stream data length */
//		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;//SrcAddress;/* Peripheral to Memory */ /* Configure DMA Stream source address */
//		((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M0AR =(uint32_t)&comm->d_rx_8[0];// DesAddress;/* Configure DMA Stream destination address */
//		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR); //Enable the DMA transfer for transmit request by setting the DMAT bit in the UART CR3 register
//	__HAL_DMA_ENABLE(huart->hdmatx);
	
//}
void initUartDma(UART_HandleTypeDef * huart, COMM_t * comm)
{
    // TX 配置
    __HAL_DMA_DISABLE(huart->hdmatx);
    ((DMA_Base_Registers *)huart->hdmatx->StreamBaseAddress)->IFCR = 0x3FUL << (huart->hdmatx->StreamIndex & 0x1FU);
    ((DMA_Stream_TypeDef *)huart->hdmatx->Instance)->CR &= ~DMA_SxCR_DBM;
    ((DMA_Stream_TypeDef *)huart->hdmatx->Instance)->PAR  = (uint32_t)&huart->Instance->TDR;
    ((DMA_Stream_TypeDef *)huart->hdmatx->Instance)->M0AR = (uint32_t)&comm->d_tx_8[0];
    ((DMA_Stream_TypeDef *)huart->hdmatx->Instance)->NDTR = 2;
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);

    // RX 配置
    __HAL_DMA_DISABLE(huart->hdmarx);
    ((DMA_Base_Registers *)huart->hdmarx->StreamBaseAddress)->IFCR = 0x3FUL << (huart->hdmarx->StreamIndex & 0x1FU);
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~DMA_SxCR_DBM;
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->PAR  = (uint32_t)&huart->Instance->RDR;
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->M0AR = (uint32_t)&comm->d_rx_8[0];
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->NDTR = 2;
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    // 启用串口
    SET_BIT(huart->Instance->CR1, USART_CR1_RE | USART_CR1_TE);
	__HAL_DMA_ENABLE_IT(huart->hdmatx, DMA_IT_TC);


    // 启动 DMA
    __HAL_DMA_ENABLE(huart->hdmatx);
    __HAL_DMA_ENABLE(huart->hdmarx);
}

int putcharx(int c)
{
	while ((UART7->ISR & (1<<7)) == 0);
	UART7->TDR = c;
	return c;
}
void putstr(const char *str)
{
	while (*str)
	{
		putcharx(*str);
		str++;
	}
}

void putdatas(const char *datas, int len)
{
	int i;
	for (i = 0; i < len; i++)
	{
		putcharx(datas[i]);
	}
}


const unsigned char hex_tab[]={'0','1','2','3','4','5','6','7',\
		                 '8','9','a','b','c','d','e','f'};

void puthex(unsigned int val)
{
	/* val: 0x12345678 */
	int i;
	int index;
	putstr("0x");
	for (i = 7; i >=0 ; i--)
	{
		index = (val >> (i*4)) & 0xf;
		putcharx(hex_tab[index]);
	}
}

unsigned int str2hex(const char* s)
{
	unsigned int sum=0;
    unsigned char c;
    unsigned int val;
	while ( *s == ' '  ||  *s == '\t') s++;

    if (*s == '0')s++;
    if (*s == 'x')s++;
    if (*s == 'X')s++;    

    c = *s;
	while (c)
	{
        if (c >= '0' && c <= '9')
            val = c - '0';
        else if (c >= 'a' && c <= 'z')
            val = c - 'a' + 10;
        else if (c >= 'A' && c <= 'Z')
            val = c - 'A' + 10;
		sum = sum * 16 + val;
		++s;
        c = *s;
	}
	return sum;
}
uint8_t cal_xor(uint8_t* data, uint8_t length) 
{
    uint8_t xorValue = 0;
    for(uint8_t i = 0; i < length; i++) 
		{
        xorValue ^= data[i];
    }
    return xorValue;
}
uint8_t cal_sum(uint8_t* data, uint8_t length) 
{
    uint8_t sumValue = 0;
    for(uint8_t i = 0; i < length; i++) 
		{
        sumValue += data[i];
    }
    return sumValue;
}