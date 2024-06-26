#include "can_init.h"


CAN_Msg CAN_TxMsg;
CAN_Msg CAN_RxMsg;

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
    
    CAN1->MCR   &=    ~(1<<16); //CAN working during debug
    CAN1->MCR   &=    ~(1<<2);  //Transfer priority driven by the identifier
    CAN1->MCR   &=    ~(1<<3);  //Receive FIFO locked mode 
    CAN1->MCR   |=     (1<<4);  //No automatic retransmission
    CAN1->MCR   &=    ~(1<<5);  //No automatic wake-up mode
    CAN1->MCR   &=    ~(1<<6);  //No automatic bus-off managment 
    CAN1->MCR   &=    ~(1<<7);  //Time triggered mode disabled

    CAN1->BTR = 0;              //Reset the bit timing register (reset value 0x0123 0000)

    CAN1->BTR   |=    (3<<24);  //sjw = 3
    CAN1->BTR   |=    (3<<0);   //brp = 3
    
    //ts1 = 2
    CAN1->BTR   &=   ~(1<<16);
    CAN1->BTR   |=    (1<<17);
    CAN1->BTR   &=   ~(1<<18);
    CAN1->BTR   &=   ~(1<<19);
    
  //ts2 = 3
    CAN1->BTR   |=    (1<<20);
    CAN1->BTR   |=    (1<<21);
    CAN1->BTR   |=    (1<<22);

	//tq = 144 MHz
    //tbs1 = 432 MHz
    //tbs2 = 576 MHz
    //trjw = 576 MHZ
    //0.000002 nominal bit time
    /*
        
    */

    CAN1->MCR &= ~(CAN_MCR_INRQ);				//exit initilization mode
    
    //Clear the MailBox TIR register (reset value 0xXXXX XXXX)
    CAN1->sTxMailBox[0].TIR &= ~0xFFFFFFFF;

    if (CAN_TxMsg.format == STANDAR_FORMAT) //Standart CAN identifier
        CAN1->sTxMailBox[0].TIR &= ~(1<<2);	//
    
    else if(CAN_TxMsg.format == EXTENDED_FORMAT) //Extended CAN identifier
        CAN1->sTxMailBox[0].TIR |= (1<<2);
    
    
    if(CAN_TxMsg.type == DATA_FRAME)  //Data frame
        CAN1->sTxMailBox[0].TIR &= ~(1<<1);
    
    else if(CAN_TxMsg.type == REMOTE_FRAME) //Remote Frame
        CAN1->sTxMailBox[0].TIR |= (1<<1);

    //Mailbox data length 8 (1111)
    CAN1->sTxMailBox[0].TDTR |= (1<<3);
    CAN1->sTxMailBox[0].TDTR |= (1<<2);
    CAN1->sTxMailBox[0].TDTR |= (1<<1);
    CAN1->sTxMailBox[0].TDTR |= (1<<0);
    
    CAN1->sTxMailBox[0].TIR |= (0xFF<<21); // Standart ID = 21 


}


void can_filtre_ayarlama_takay03(uint32_t id, unsigned char format){
    unsigned int CAN_MsgId = 0;
    static unsigned short CAN_FilterId = 0;

    if(CAN_FilterId )
    
    CAN1->FMR                               |=  (1<<0);                     //Filter init mode
    CAN1->FA1R                              &= ~(1<<CAN_FilterId);          //Deactive Filter Id
    CAN1->FS1R                              |=  (1<<CAN_FilterId);          //Single 32-bit scale configuration
    CAN1->FM1R                              |=  (1<<CAN_FilterId);          //Two 32-bit register List mode
    CAN1->sFilterRegister[CAN_FilterId].FR1  =  CAN_MsgId;                  //32-bit identifier
    //CAN1->sFilterRegister[CAN_FilterId].FR2  =  CAN_MsgId;                  //32-bit identifier
    CAN1->FFA1R                             &= ~(1<<CAN_FilterId);          //Filter assign to FIFO0
    CAN1->FA1R                              |=  (1<<CAN_FilterId);          //Active filter
    CAN1->FMR                               &= ~(1<<0);                     //Reset init mode

    CAN_FilterId += 1;
}

void readMessage(void){
  CAN_RxMsg.data[0] = (CAN1->sFIFOMailBox[0].RDLR);
  CAN_RxMsg.data[1] = (CAN1->sFIFOMailBox[0].RDLR >> 8);
  CAN_RxMsg.data[2] = (CAN1->sFIFOMailBox[0].RDLR >> 16);
  CAN_RxMsg.data[3] = (CAN1->sFIFOMailBox[0].RDLR >> 24);
  CAN_RxMsg.data[4] = (CAN1->sFIFOMailBox[0].RDHR);
  CAN_RxMsg.data[5] = (CAN1->sFIFOMailBox[0].RDHR >> 8);
  CAN_RxMsg.data[6] = (CAN1->sFIFOMailBox[0].RDHR >> 16);
  CAN_RxMsg.data[7] = (CAN1->sFIFOMailBox[0].RDHR >> 24);

  CAN1->RF0R |= CAN_RF0R_RFOM0;  //Release FIFO0 output mailbox
}

void can_mesaj_gonderla(void){
    //CAN1->sTxMailBox[0].TIR = 0; //reset TIR register, reset value = 0xXXXXXXXX
    //CAN1->sTxMailBox[0].TIR |= (1<<2); //extended identifier
    /*
      bits between 3 to 20 (18 bits) are responsible for LSBs of Extended identifier
      bits between 21 to 31 (11 bits) are resbonsible for MSBs of Extended identifier when bit 2 is 1 otherwise these bits are responsible for standart identifier (when bit 2 is 0)
      obviously these bits are for Mailboxes (for transmitting identifiers)
    */
    //CAN1->sTxMailBox[0].TIR |= (0x3FFFF<<3); 
    // Setup type information
    //CAN1->sTxMailBox[0].TIR &= ~(1<<1); // data frame 
    // Setup data bytes
    CAN1->sTxMailBox[0].TDLR = (((unsigned int)CAN_TxMsg.data[3] << 24) | 
                                ((unsigned int)CAN_TxMsg.data[2] << 16) |
                                ((unsigned int)CAN_TxMsg.data[1] <<  8) | 
    ((unsigned int)CAN_TxMsg.data[0]));
    CAN1->sTxMailBox[0].TDHR = (((unsigned int)CAN_TxMsg.data[7] << 24) | 
                                ((unsigned int)CAN_TxMsg.data[6] << 16) |
                                ((unsigned int)CAN_TxMsg.data[5] <<  8) |
                                ((unsigned int)CAN_TxMsg.data[4]));
    //SETUP LENGTH             
    CAN1->sTxMailBox[0].TDTR |= (1<<3);
    CAN1->sTxMailBox[0].TDTR |= (1<<2);
    CAN1->sTxMailBox[0].TDTR |= (1<<1);
    CAN1->sTxMailBox[0].TDTR |= (1<<0);
    CAN1->IER |= (1<<0);                   // Enable TME interrup
    CAN1->sTxMailBox[0].TIR |= (1<<0);     // Transmit message   
}

void USB_HP_CAN1_TX_IRQHandler(void){
    //CAN1->sTxMailBox[0].TIR = 0; //reset TIR register, reset value = 0xXXXXXXXX
    //CAN1->sTxMailBox[0].TIR |= (1<<2); //extended identifier
      
      //bits between 3 to 20 (18 bits) are responsible for LSBs of Extended identifier
      //bits between 21 to 31 (11 bits) are resbonsible for MSBs of Extended identifier when bit 2 is 1 otherwise these bits are responsible for standart identifier (when bit 2 is 0)
      //obviously these bits are for Mailboxes (for transmitting identifiers)
    // Setup type information
    //CAN1->sTxMailBox[0].TIR &= ~(1<<1); // data frame 
    // Setup data bytes
    CAN1->sTxMailBox[0].TDLR = (((unsigned int)CAN_TxMsg.data[3] << 24) | 
                                ((unsigned int)CAN_TxMsg.data[2] << 16) |
                                ((unsigned int)CAN_TxMsg.data[1] <<  8) | 
    ((unsigned int)CAN_TxMsg.data[0]));
    CAN1->sTxMailBox[0].TDHR = (((unsigned int)CAN_TxMsg.data[7] << 24) | 
                                ((unsigned int)CAN_TxMsg.data[6] << 16) |
                                ((unsigned int)CAN_TxMsg.data[5] <<  8) |
                                ((unsigned int)CAN_TxMsg.data[4]));
    //SETUP LENGTH             
    CAN1->sTxMailBox[0].TDTR |= (1<<3);
    CAN1->sTxMailBox[0].TDTR |= (1<<2);
    CAN1->sTxMailBox[0].TDTR |= (1<<1);
    CAN1->sTxMailBox[0].TDTR |= (1<<0);
    CAN1->IER |= (1<<0);                   // Enable TME interrup
    CAN1->sTxMailBox[0].TIR |= (1<<0);     // Transmit message

    //for(int i = 0; i<=10; i++);
}
