#include "can_init.h"

void clock_enable(void){
    RCC->APB2ENR |= (1<<0); //enable alternate function clock
    RCC->APB2ENR |= (1<<2); //enable gpioa clock
    RCC->APB1ENR |= (1<<25); //enable can clock
}
void gpio_enable(void){
    
    //pa12 -> can_tx -> alternate function output push-pull
    //pa11 -> can_rx -> input push-pull
    
    GPIOA->CRH = ~(0xFFFFFFFF);

    GPIOA->CRH &= ~(1<<12);
    GPIOA->CRH &= ~(1<<13);

    GPIOA->CRH &=  ~(1<<14);
    GPIOA->CRH |=  (1<<15);

    GPIOA->CRH |= (1<<16);
    GPIOA->CRH |= (1<<17);

    GPIOA->CRH &= ~(1<<18);
    GPIOA->CRH |= (1<<19);

}
void can_init(void){

    NVIC->ISER[0] |= (1 << (USB_HP_CAN1_TX_IRQn  & 0x1F));//
    NVIC->ISER[0] |= (1 << (USB_LP_CAN1_RX0_IRQn & 0x1F));//
    
    CAN1->MCR = 0;
    CAN1->MCR &= ~(CAN_MCR_SLEEP);
    CAN1->MCR |= CAN_MCR_INRQ;
    
    
    CAN1->IER = (CAN_IER_FMPIE0 | CAN_IER_TMEIE);
    
    
    CAN1->MCR   &=    ~(1<<2);
    CAN1->MCR   &=    ~(1<<3);
    CAN1->MCR   |=     (1<<4);
    CAN1->MCR   &=    ~(1<<5);
    CAN1->MCR   &=    ~(1<<6);
    CAN1->MCR   &=    ~(1<<7);

    CAN1->BTR = 0;
	CAN1->BTR |= (1<<30);		//Loop Back Mode

    CAN1->BTR |= (3<<24);
    CAN1->BTR |= (4<<20);
    CAN1->BTR |= (1<<16);
    CAN1->BTR |= (3<<0);
    
    CAN1->BTR &= ~(1<<16);
    CAN1->BTR |= (1<<17);
    CAN1->BTR &= ~(1<<18);
    CAN1->BTR &= ~(1<<19);
    
    CAN1->BTR |=  (1<<20);
    CAN1->BTR |=  (1<<21);
    CAN1->BTR |=  (1<<22);

	
    CAN1->MCR &= ~(CAN_MCR_INRQ);				//exit initilization mode

	  CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_RTR;
	  CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_EXID;		//Standard Identifier
	  CAN1->sTxMailBox[0].TIR &= ~CAN_TI0R_STID;
	  CAN1->sTxMailBox[0].TIR |= (0xFF << CAN_TI0R_STID_Pos);				//Message ID = 1

    CAN1->sTxMailBox[0].TDTR |= (1<<3);
    CAN1->sTxMailBox[0].TDTR |= (1<<2);
    CAN1->sTxMailBox[0].TDTR |= (1<<1);
    CAN1->sTxMailBox[0].TDTR |= (1<<0);

}
    



