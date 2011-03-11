/**
 * MiniDspNode key stroke example
 * 2011-03-11 www.norduino.com http://opensource.org/licenses/mit-license.php
 * Copyright 2011 Robomotic ltd.
 */

// MSP430 family include
#include <msp430x20x1.h>
//==============================================================================
#define  RF24L01_CE_0        P1OUT &=~BIT5         
#define  RF24L01_CE_1        P1OUT |= BIT5        
//=============================RF24L01 CSN ==================================
#define  RF24L01_CSN_0       P1OUT &=~BIT4        
#define  RF24L01_CSN_1       P1OUT |= BIT4     
//=============================RF24L01 SCK======================================
#define  RF24L01_SCK_0       P1OUT &=~BIT3      
#define  RF24L01_SCK_1       P1OUT |= BIT3   
//==============================RF24L01 MISO PIN=========================================
#define  RF24L01_MISO_0      P1OUT &=~BIT1
#define  RF24L01_MISO_1      P1OUT |= BIT1
//============================RF24L01 MOSI PIN==================================
#define  RF24L01_MOSI_0      P1OUT &=~BIT2
#define  RF24L01_MOSI_1      P1OUT |= BIT2
//===========================IRQ PIN ============================================
#define  RF24L01_IRQ_0       P1OUT &=~BIT0      
#define  RF24L01_IRQ_1       P1OUT |= BIT0
//==============================================================================
#define  LED1_0              P2OUT &=~BIT7          //LED1 ON
#define  LED1_1              P2OUT |= BIT7          //LED1 OFF
#define  LED2_0              P1OUT &=~BIT7         //LED2 ON
#define  LED2_1              P1OUT |= BIT7          //LED2 OFF
//==========================NRF24L01============================================
#define TX_ADR_WIDTH    5   	// 5 uints TX address width
#define RX_ADR_WIDTH    5   	// 5 uints RX address width
#define TX_PLOAD_WIDTH  32  	// 32 TX payload
#define RX_PLOAD_WIDTH  32  	// 32 uints TX payload
//****************************************************************// 
// SPI(nRF24L01) commands 
#define READ_REG        0x00  // Define read command to register 
#define WRITE_REG       0x20  // Define write command to register 
#define RD_RX_PLOAD     0x61  // Define RX payload register address 
#define WR_TX_PLOAD     0xA0  // Define TX payload register address 
#define FLUSH_TX        0xE1  // Define flush TX register command 
#define FLUSH_RX        0xE2  // Define flush RX register command 
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command 
#define NOP1            0xFF  // Define No Operation, might be used to read status register 

//***************************************************// 
// SPI(nRF24L01) registers(addresses) 
#define CONFIG          0x00  // 'Config' register address 
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address 
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address 
#define SETUP_AW        0x03  // 'Setup address width' register address 
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address 
#define RF_CH           0x05  // 'RF channel' register address 
#define RF_SETUP        0x06  // 'RF setup' register address 
#define STATUS          0x07  // 'Status' register address 
#define OBSERVE_TX      0x08  // 'Observe TX' register address 
#define CD              0x09  // 'Carrier Detect' register address 
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address 
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address 
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address 
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address 
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address 
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address 
#define TX_ADDR         0x10  // 'TX address' register address 
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address 
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address 
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address 
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address 
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address 
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address 
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address 
//=============================RF24l01 init config=====================================
char  TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//TX address
char  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//RX address
char  sta;
char  tf, RxBuf[32],TxBuf[32];
//===========================MiniDsp Port IO setup ==========================================
void RF24L01_IO_set(void)
{
      P1DIR = 0xbc;  P1SEL=0x00;   P1IE=0x00;
      P2DIR = 0x80;  P2SEL=0x00;   P2IE=0x00;
}
//========================delay for about 5ms=============================================
void ms_delay(void)
{ 
   unsigned int i=40000;
    while (i != 0)
    {
        i--;
    }
}
//=======================delay for about 2*s ops ================================
void Delay(int s)
{
	unsigned int i,j;
	for(i=0; i<s; i++);
	for(j=0; j<s; j++);
}
//******************************************************************************************
// Delay for about n us
//******************************************************************************************
void inerDelay_us(int n)
{
	for(;n>0;n--);
}
/************************************************** 
Function: SPI_RW(); 
 
Description: 
  Writes one byte to nRF24L01, and return the byte read 
  from nRF24L01 during write, according to SPI protocol  */ 
/**************************************************/ 
char SPI_RW(char data)
{
	char i,temp=0;
   	for(i=0;i<8;i++) // output 8-bit
   	{
	if((data & 0x80)==0x80)
	{
		RF24L01_MOSI_1;         // output 'uchar', MSB to MOSI
	}
	else
	{
	 	RF24L01_MOSI_0; 
	}	
//==============================================================================
		data = (data << 1);            // shift next bit into MSB..
		temp<<=1;
		RF24L01_SCK_1;                // Set SCK high..
		if((P1IN&0x02))temp++;         // capture current MISO bit
		RF24L01_SCK_0;              // ..then set SCK low again
   	}
    return(temp);           		  // return read uchar
}
/************************************************** 
Function: SPI_Read(); 
 
Description: 
  Read one byte from nRF24L01 register, 'reg'  */ 
/**************************************************/ 
char SPI_Read(char reg)
{
	char reg_val;
	RF24L01_CSN_0;           // CSN low, initialize SPI communication...
	SPI_RW(reg);            // Select register to read from..
	reg_val = SPI_RW(0);    // ..then read registervalue
	RF24L01_CSN_1;         // CSN high, terminate SPI communication
	return(reg_val);       //  return register value
}
/************************************************** 
Function: SPI_RW_Reg(); 
 
Description: 
  Writes value 'value' to register 'reg' */ 
/**************************************************/ 
char SPI_RW_Reg(char reg, char value)
{
	char status1;
	RF24L01_CSN_0;                   // CSN low, init SPI transaction
	status1 = SPI_RW(reg);      // select register
	SPI_RW(value);             // ..and write value to it..
	RF24L01_CSN_1;                   // CSN high again
	return(status1);            // return nRF24L01 status uchar
}
/************************************************** 
Function: SPI_Read_Buf(); 
 
Description: 
  Reads 'bytes' #of bytes from register 'reg' 
  Typically used to read RX payload, Rx/Tx address */ 
/**************************************************/ 
char SPI_Read_Buf(char reg, char *pBuf, char chars)
{
	char status2,uchar_ctr;
	RF24L01_CSN_0;                    		// Set CSN low, init SPI tranaction
	status2 = SPI_RW(reg);       		// Select register to write to and read status uchar
	for(uchar_ctr=0;uchar_ctr<chars;uchar_ctr++)
        {
	pBuf[uchar_ctr] = SPI_RW(0);    // 
        }
	RF24L01_CSN_1;                     // Set CSN high       
	return(status2);                    // return nRF24L01 status uchar
}
/************************************************** 
Function: SPI_Write_Buf(); 
 
Description: 
  Writes 'bytes' from pBuff to register 'chars' 
  Typically used to write TX payload, Rx/Tx address */ 
/**************************************************/ 
char SPI_Write_Buf(char reg, char *pBuf, char chars)
{
	char status1,uchar_ctr;
	RF24L01_CSN_0;             // Set CSN low, init SPI tranaction     
	status1 = SPI_RW(reg);    // Select register to write to and read status byte
	for(uchar_ctr=0; uchar_ctr<chars; uchar_ctr++) // then write all byte in buffer(*pBuf)
        {
	SPI_RW(*pBuf++);
        }
	RF24L01_CSN_1;           // Set CSN high
	return(status1);    		  // 
}
//****************************************************************************************************/
// void SetRX_Mode(void)
//功能：数据接收配置 
//****************************************************************************************************/
void SetRX_Mode(void)
{
	RF24L01_CE_0 ;
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);  // Set PWR_UP bit, enable CRC(2 bytes) & Prim:RX. RX_DR enabled..
	RF24L01_CE_1; // Set CE pin high to enable RX device
	inerDelay_us(1000);//delay for about 1 second
}
//******************************************************************************************************/
// unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
// RX a packet
//******************************************************************************************************/
char nRF24L01_RxPacket(char* rx_buf)
{
    char revale=0;
	sta=SPI_Read(STATUS);	     // read register STATUS's value	
	if(sta&0x40)                 // success!
	{
	    RF24L01_CE_0 ; 			//SPI enable
	    SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
	    revale =1;			//set flag to 1
	}
	SPI_RW_Reg(WRITE_REG+STATUS,sta);  
	return revale;
}
//***********************************************************************************************************
//void nRF24L01_TxPacket(char * tx_buf)
//TX a packet
//**********************************************************************************************************/
void nRF24L01_TxPacket(char * tx_buf)
{
	RF24L01_CE_0 ;			//StandBy
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);   // RX_Addr0 same as TX_Adr for Auto.Ack
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 		 // Writes data to TX payload
        SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);   		 // Set PWR_UP bit, enable CRC(2 bytes) & Prim:TX. MAX_RT & TX_DS enabled..
	RF24L01_CE_1;		 // finish
	inerDelay_us(600);
}
//****************************************************************************************
//NRF24L01 init
//***************************************************************************************/
void init_NRF24L01(void)
{
        inerDelay_us(100);
 	RF24L01_CE_0 ;    // chip enable
 	RF24L01_CSN_1;   // Spi disable 
 	RF24L01_SCK_0;   // Spi clock line init high
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);   // Writes TX_Address to nRF24L01 
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack 
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);     // enable Auto.Ack:Pipe0
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);   // Enable Pipe0
	SPI_RW_Reg(WRITE_REG + RF_CH, 0);        //   Select channel 0
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); 
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR 
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);    // Set PWR_UP bit, enable CRC(2 bytes) & Prim:RX. RX_DR enabled..
}
//=============================================================================
main()
{     

      WDTCTL = WDTPW + WDTHOLD; //disabel timer 
      RF24L01_IO_set();
      init_NRF24L01() ;
      LED1_1;LED2_1; // green and red led on
      while(1)
      {
        if((P2IN&0x40)==0)   //IF IO 2.6 is pulled high  KEY1: p2.6
	  {
           LED2_1;LED1_0; tf = 1 ; //we need to send a code
	   TxBuf[1] = 0x29 ;  //just any value will do
	  }
       if((P1IN&0x40)==0)	 //IF IO 1.6 is pulled high  KEY2: p1.6
	  {	
           LED2_0;LED1_1;tf = 1 ;  //we need to send a code
	   TxBuf[2] = 0x30 ;   //just any value will do
	  }
        //===if a key stroke was detected we need to send ====/
          if (tf==1)
          {	
	    nRF24L01_TxPacket(TxBuf);	// send the buffer packet
            SPI_RW_Reg(WRITE_REG+STATUS,0XFF); 
	    TxBuf[1] = 0xff;	//then clear it
	    TxBuf[2] = 0xff;	//then clear it
	    tf=0; //send flag is off
	    Delay(1000); 
	   }
      SetRX_Mode();               
      if(nRF24L01_RxPacket(RxBuf))   //then wait for a packet from the peer 
	{	
	if(RxBuf[1]==0x29) //if 29 is received green on red off
	{	
           LED2_1;LED1_0;
	}
	if(RxBuf[2]==0x30) //if 29 is received green off red on
	{
           LED2_0;LED1_1;
	}
        }
 	    Delay(10000); 
 
        }
}