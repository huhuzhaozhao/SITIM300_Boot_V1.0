
#ifndef __BOOT_H__
#define __BOOT_H__

#include "main.h"
#include "uart_x.h"
extern uint8_t  version_change;
void __attribute__((noinline)) INTX_DISABLE(void);

uint8_t deal_recv(COMM_t * com);

uint16_t Crc16Ccitt(uint8_t *q, uint32_t len);

void iap_load_app(uint32_t appxaddr);
void send_to_pc(uint8_t status,COMM_t * com);

uint8_t H7flash_write(uint32_t start_addr,uint32_t *pBuffer,uint32_t NumToWrite);
uint32_t GetSector(uint32_t Address);

uint8_t str_compare(char *s1, char *s2,uint32_t len);


void send_to_ws(COMM_t * com);
#endif /* __BOOT_H__ */

