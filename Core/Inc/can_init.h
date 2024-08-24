#include "main.h"

#define STANDART_FORMAT  0
#define EXTENDED_FORMAT 1

#define DATA_FRAME      0
#define REMOTE_FRAME    1

#define ADDR0 0x0000E000
#define ADDR1 0x0000E001
#define ADDR2 0x0000E002
#define ADDR3 0x00001B9E
#define ADDR4 0x00000107
#define ADDR5 0x0000E017
#define ADDR6 0x0000E018
#define ADDR7 0x0000E016
#define ADDR8 0x0000E019
#define ADDR9 0x0000E020
#define ADDR10 0x36



typedef struct {
    unsigned int id;
    unsigned char data[8];
    unsigned char len;
    unsigned char format;
    unsigned char type;
}CAN_Msg;

void clock_enable(void);
void Set_TxMailBox(uint32_t TxMailBox, CAN_Msg msg);
void gpio_enable(void);
void can_init(void);
void can_filtre_ayarlama_takay03(uint32_t id, unsigned char format);
void can_mesaj_gonderla(void);
void readMessage(void);
void led_init(void);
void led_on(void);

extern CAN_Msg CAN_TxMsg;
extern CAN_Msg CAN_RxMsg;
