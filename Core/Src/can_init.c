#include "can_init.h"


CAN_Msg CAN_TxMsg;
CAN_Msg CAN_RxMsg;

void clock_enable(void){
    RCC->APB2ENR |= (1<<0); //enable alternate function clock
    RCC->APB2ENR |= (1<<2); //enable gpioa clock
    RCC->APB1ENR |= (1<<25); //enable can clock
}

void led_init(void){
    RCC->APB2ENR |= (1<<4);

    GPIOC->CRH |= (1<<20);
    GPIOC->CRH |= (1<<21);

    GPIOC->CRH &= ~(1<<22);
    GPIOC->CRH &= ~(1<<23);

    GPIOC->ODR |= (1<<13);
}
void led_on(void){
    GPIOC->ODR &= ~(1<<13);

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
    CAN1->MCR |=  CAN_MCR_INRQ;
    
    
    CAN1->IER |= (CAN_IER_FMPIE0 | CAN_IER_TMEIE);
    
    CAN1->MCR   &=    ~(1<<16); //CAN working during debug
    CAN1->MCR   &=    ~(1<<2);  //Transfer priority driven by the identifier
    CAN1->MCR   |=    (1<<3);  //Receive FIFO locked mode 
    CAN1->MCR   |=     (1<<4);  //No automatic retransmission
    CAN1->MCR   &=    ~(1<<5);  //No automatic wake-up mode
    CAN1->MCR   |=    (1<<6);  //No automatic bus-off managment 
    CAN1->MCR   &=    ~(1<<7);  //Time triggered mode disabled
 

    //CAN1->BTR |= (2<<0) | (6<<16) | (1<<20) | (1<<24);
    //CAN1->BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((         0x0F) << 16) | (          0x1FF)); 
    CAN1->BTR = 0x00050001;
    CAN1->BTR |= (1<<30); //Enable LoopBack Mode
    CAN1->MCR &= ~(CAN_MCR_INRQ);				//exit initilization mode
}


void can_filtre_ayarlama_takay03(uint32_t id, unsigned char format){
    /*
    unsigned int CAN_MsgIdx = 0;
    static unsigned short CAN_FilterId = 0;
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
    */
   static unsigned short CAN_filterIdx = 0;
         unsigned int   CAN_msgId     = 0;
  
  if (CAN_filterIdx > 13) {                       // check if Filter Memory is full
    return;
  }
                                                  // Setup identifier information
  if (format == STANDART_FORMAT)  {               // Standard ID
      CAN_msgId  |= (unsigned int)(id << 21);
  }  else  {                                      // Extended ID
      CAN_msgId  |= (unsigned int)(id <<  3);
  }

  CAN1->FMR  |=  CAN_FMR_FINIT;                    // set Initialisation mode for filter banks
  CAN1->FA1R &=  ~(unsigned int)(1 << CAN_filterIdx); // deactivate filter

                                                  // initialize filter   
  CAN1->FS1R |= (unsigned int)(1 << CAN_filterIdx);// set 32-bit scale configuration
  CAN1->FM1R |= (unsigned int)(1 << CAN_filterIdx);// set 2 32-bit identifier list mode

  CAN1->sFilterRegister[CAN_filterIdx].FR1 =  CAN_msgId; //  32-bit identifier
  CAN1->sFilterRegister[CAN_filterIdx].FR2 = CAN_msgId; //  32-bit identifier
    													   
  CAN1->FFA1R &= ~(unsigned int)(1 << CAN_filterIdx);  // assign filter to FIFO 0
  CAN1->FA1R  |=  (unsigned int)(1 << CAN_filterIdx);  // activate filter

  CAN1->FMR &= ~CAN_FMR_FINIT;                     // reset Initialisation mode for filter banks

  CAN_filterIdx += 1;                             // increase filter index

}

void readMessage(void){
                                                    // Read identifier information
  if ((CAN1->sFIFOMailBox[0].RIR & (1<<2)) == 0) { // Standard ID
    CAN_RxMsg.format = STANDART_FORMAT;
    CAN_RxMsg.id     = (CAN1->sFIFOMailBox[0].RIR >> 21);
  }  else  {                                          // Extended ID
    CAN_RxMsg.format = EXTENDED_FORMAT;
    CAN_RxMsg.id     = (CAN1->sFIFOMailBox[0].RIR >> 3);
  }
                                                  // Read type information
  if ((CAN1->sFIFOMailBox[0].RIR & (1<<1)) == 0) {
    CAN_RxMsg.type =   DATA_FRAME;                     // DATA   FRAME
  }  else  {
    CAN_RxMsg.type = REMOTE_FRAME;                     // REMOTE FRAME
  }
                                                  // Read length (number of received bytes)
  CAN_RxMsg.len = (unsigned char)0x0000000F & CAN1->sFIFOMailBox[0].RDTR;
                                                  // Read data bytes
  CAN_RxMsg.data[0] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR);
  CAN_RxMsg.data[1] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 8);
  CAN_RxMsg.data[2] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 16);
  CAN_RxMsg.data[3] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 24);

  CAN_RxMsg.data[4] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR);
  CAN_RxMsg.data[5] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 8);
  CAN_RxMsg.data[6] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 16);
  CAN_RxMsg.data[7] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 24);

  CAN1->RF0R |= CAN_RF0R_RFOM0;                    // Release FIFO 0 output mailbox

    if(CAN_RxMsg.id == 0x0FF){
        led_on();
    }

}

void Set_TxMailBox(uint32_t TxMailBox, CAN_Msg msg){
    //CAN1->sTxMailBox[0].TIR = 0; //reset TIR register, reset value = 0xXXXXXXXX
    //CAN1->sTxMailBox[0].TIR |= (1<<2); //extended identifier
    /*
      bits between 3 to 20 (18 bits) are responsible for LSBs of Extended identifier
      bits between 21 to 31 (11 bits) are resbonsible for MSBs of Extended identifier when bit 2 is 1 otherwise these bits are responsible for standart identifier (when bit 2 is 0)
      obviously these bits are for Mailboxes (for transmitting identifiers)
    */
        
    if (TxMailBox > 2)
      return;
    
    CAN1->sTxMailBox[TxMailBox].TDLR = (((unsigned int)msg.data[3]<<24) | 
                                        ((unsigned int)msg.data[2]<<16) |
                                        ((unsigned int)msg.data[1]<<8)  | 
                                        ((unsigned int)msg.data[0]));

    CAN1->sTxMailBox[TxMailBox].TDHR = (((unsigned int)msg.data[7]<<24) | 
                                        ((unsigned int)msg.data[6]<<16) |
                                        ((unsigned int)msg.data[5]<<8)  |
                                        ((unsigned int)msg.data[4]));

    CAN1->sTxMailBox[TxMailBox].TDTR |= (msg.len<<0);

  
    //Clear the MailBox TIR register (reset value 0xXXXX XXXX)
    CAN1->sTxMailBox[TxMailBox].TIR &= ~0xFFFFFFFF;

    if (msg.format == STANDART_FORMAT) //Standart CAN identifier
        CAN1->sTxMailBox[TxMailBox].TIR &= ~(1<<2);	//
    else if(msg.format == EXTENDED_FORMAT) //Extended CAN identifier
        CAN1->sTxMailBox[TxMailBox].TIR |= (1<<2);
    
    
    if(msg.type == DATA_FRAME)  //Data frame
        CAN1->sTxMailBox[TxMailBox].TIR &= ~(1<<1);
    else if(msg.type == REMOTE_FRAME) //Remote Frame
        CAN1->sTxMailBox[TxMailBox].TIR |= (1<<1);

    
    CAN1->sTxMailBox[TxMailBox].TIR |= (msg.id<<21); // Standart ID = 21 


}

void can_mesaj_gonderla(void){
    
    //CAN1->sTxMailBox[0].TIR |= (0x3FFFF<<3); 
    //CAN1->sTxMailBox[0].TIR &= ~(1<<1); // data frame 
    //CAN1->sTxMailBox[0].TDLR = (((unsigned int)CAN_TxMsg.data[3] << 24) | 
    //                        ((unsigned int)CAN_TxMsg.data[2] << 16) |
    //                        ((unsigned int)CAN_TxMsg.data[1] <<  8) | 
    //((unsigned int)CAN_TxMsg.data[0]));
    //CAN1->sTxMailBox[0].TDHR = (((unsigned int)CAN_TxMsg.data[7] << 24) | 
    //                        ((unsigned int)CAN_TxMsg.data[6] << 16) |
    //                        ((unsigned int)CAN_TxMsg.data[5] <<  8) |
    //                        ((unsigned int)CAN_TxMsg.data[4]));
    //CAN1->sTxMailBox[0].TDTR |= (1<<3);
    //CAN1->sTxMailBox[0].TDTR |= (1<<2);
    //CAN1->sTxMailBox[0].TDTR |= (1<<1);
    //CAN1->sTxMailBox[0].TDTR |= (1<<0);
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

    can_mesaj_gonderla(); 
    for(int i = 0; i<=1000000; i++);
}

void USB_LP_CAN1_RX0_IRQHandler(void){
    led_on();
    readMessage();
}

