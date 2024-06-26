#include "main.h"

#define STANDART_FORMAT  0
#define EXTENDED_FORMAT 1

#define DATA_FRAME      0
#define REMOTE_FRAME    1

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
