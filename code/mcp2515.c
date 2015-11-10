/*
** File:mcp2515.c
** Date:2014/11/25
** Author:lHw
** Discription: CAN-bus init&function
*/
#include "io430.h"
#include "mcp2515.h"
#include "protocol.h"
#include "string.h"
#include "global_data.h"

#define DELAY(n) for(uint16 _i=0;_i<n;_i++)  // (n=1)=4ns

/*global var*/
extern uint8 FlagINT;
extern uint8 FlagRX0BF;
extern uint8 FlagRX1BF;
extern uint8 FlagTX0RST;
extern uint8 FlagTX1RST;
extern uint8 FlagTX2RST;
//definenation
/*local var*/
const uint16 PSIZE = 1024;
__no_init _RDataTmp Rbuf[PSIZE];
uint16 Rbuf_p; //收到包数
uint16 Lbuf_p; // 重组包数
uint16 Test1;

_ConfTransmt TranFrame;
void TranFramConfig(void)
{
  TranFrame.StandardIDH = CanIDH;                  // standard frame ID high bits
  TranFrame._StandardIDL_BOb._Bit.StandardID3bits = CanIDL; // standard frame id low bits
  TranFrame._StandardIDL_BOb._Bit.ExtendID2bits = 0x00;
  TranFrame._StandardIDL_BOb._Bit.ExIdeEnable = 0x00;
  TranFrame.ExtendIDL = 0x00;
  TranFrame.ExtendIDH = 0x00;
  TranFrame._DataLength_BOb._Bit.DLC = 0x08;       // data length
  TranFrame._DataLength_BOb._Bit.RTR = 0x00;    // data frame or remot transmit request
}


uint8 mcp2515_init()
{
  // IO P3.0= spics~
  P3DIR |=  0x01;
  P3DS  |=  0x01;
  P3SEL &= ~0x01;
  // P3.1=spisi,P3.2=spiso,P3.3=spisck
  P3SEL |= 0x0E;
  //usci b0 spi mode init
  //reset
  UCB0CTL1 |= UCSWRST;  
  // Master mode + Captured first edge,inactive state low
  UCB0CTL0 |= UCMST  + UCCKPH ;
  // MSB first, 8bit, 3pin
  UCB0CTL0 |= UCMSB;
  // synchronous
  UCB0CTL0 |= UCSYNC;
  // SMCLK
  UCB0CTL1 |= UCSSEL0+UCSSEL1;
  // 24MHz 1Mbps
  UCB0BR0 = 0x18;
  UCB0BR1 = 0x00; 
  // set
  UCB0CTL1 &= ~UCSWRST;
  // enable RXIE
  UCB0IE = UCRXIE;
  return 0;
  
}

void irp_pin_init( void )
{
  // init Interrupt INT/RX1BF/RX0BF/TX1BF/TX0BF
  P2DIR &=~0x02;		// Set P2.1/p2.2/p2.3/p2.4/p2.5/p2.6 to in direction
  P2REN |= 0x02;		// Enableinternal resistance
  P2IES |= 0x02;		// Hi/Lo edge
  P2IFG &=~0x02;		// IFG cleared
  P2IE  |= 0x02;		// interrupt enabled
}


/* transmit a byte by spi */
uint8 opption_spi( uint8 Order, uint8 Addr, uint8 Data )
{
  CSLOW;
  // wait txbuffer empty
  while( !(UCB0IFG__SPI_bit.UCTXIFG) ){}
  // delay 25*clock period(102ns)
  DELAY(35);
  UCB0TXBUF = Order;
  DELAY(35);
  UCB0TXBUF = Addr;
  DELAY(35);
  UCB0TXBUF = Data;
  DELAY(35);
  CSHIGH;
  // delay 25*CP (106ns)  
  DELAY(35);
  return UCB0RXBUF;
}


/* transmit a byte by spi one signal mode */
uint8 opption_spi_signal( uint8 Order )
{
  CSLOW;
  // wait txbuffer empty
  while( !(UCB0IFG__SPI_bit.UCTXIFG) ){}
  // delay 25*clock period(102ns)
  DELAY(35);
  UCB0TXBUF = Order;
  DELAY(35);
  CSHIGH;
  // delay 25*CP (106ns)  
  DELAY(35);
  return UCB0RXBUF;
}

uint8 opption_spi_alwaysread( uint8 Order )
{
  CSLOW;
  while( !(UCB0IFG__SPI_bit.UCTXIFG) ){}
  DELAY(35);
  UCB0TXBUF = Order;
  DELAY(35);
  return UCB0RXBUF;
}


/* transmit a byte by spi */
uint8 bit_mod_spi( uint8 Order, uint8 Addr, uint8 Mod, uint8 Data )
{
  CSLOW;
  //while( !UCB0IFG__SPI_bit.UCTXIFG ){}
  DELAY(35);
  UCB0TXBUF = Order;
  DELAY(35);
  UCB0TXBUF = Addr;
  DELAY(35);
  UCB0TXBUF = Mod;
  DELAY(35);
  UCB0TXBUF = Data;
  DELAY(35);
  CSHIGH;  
  DELAY(35);
  
  return 0;
}

/* config txbuffer */
uint8 transmit_data(void)
{
  opption_spi( SPI_WRITE, 0x31, TranFrame.StandardIDH );
  opption_spi( SPI_WRITE, 0x32, TranFrame._StandardIDL_BOb.StandardIDL );
  opption_spi( SPI_READ, 0x31, 0 );
  opption_spi( SPI_READ, 0x32, 0 );
  return 0;
}

uint8 conf_rxbuff(void)
{
  uint8 tmp = MOD_CON; 
  opption_spi( SPI_WRITE, CAN_CTRL, tmp );

  opption_spi( SPI_WRITE, 0x00, 0x80 );      // filter h
  opption_spi( SPI_WRITE, 0x01, 0x2a );      // filter l standardframe
  opption_spi( SPI_WRITE, 0x02, 0x00 );      // filter H of extend
  opption_spi( SPI_WRITE, 0x03, 0x01 );      // filter L of extend
  opption_spi( SPI_WRITE, 0x20, 0xff );     
  opption_spi( SPI_WRITE, 0x21, 0xff );
  opption_spi( SPI_WRITE, 0x22, 0xff );
  opption_spi( SPI_WRITE, 0x23, 0xff );
  
  opption_spi( SPI_WRITE, 0x2b, 0x01 );      // 中断标志使能
  opption_spi( SPI_READ, 0x00, 0x00 );
  opption_spi( SPI_READ, 0x01, 0x00 );
  
  //10kbps
  opption_spi( SPI_WRITE, 0x2a, 0x2c );//CNF1 0x2c 10k ;0x04 100k
  opption_spi( SPI_WRITE, 0x29, 0xba );//CNF2 0xba 10k ;0xb8 100k
  opption_spi( SPI_WRITE, 0x28, 0x07 );//CNF3 0x07 10k ;0x07 100k
    tmp = RXB0_ALLRE + RXB0_RXF0+RXB0_BUKT;
  opption_spi( SPI_WRITE, CAN_RXCT0, tmp );
  opption_spi( SPI_WRITE, CAN_BFPCTRL, 0x05 ); // rxb0 中断脚使能
  opption_spi( SPI_READ, CAN_RXCT0, 0x00 );
  opption_spi( SPI_READ, CAN_CTRL, 0x00 );
  return 0;
}

uint8 working_mode(void)
{
  uint8 tmp = MOD_DEF ;
  opption_spi( SPI_WRITE, CAN_CTRL, tmp );
  opption_spi( SPI_READ,  CAN_CTRL, 0x00);
  return 0;
}
uint8 working_mode_osm(void)
{
  uint8 tmp = MOD_DEF + MOD_OSM;
  opption_spi( SPI_WRITE, CAN_CTRL, tmp );
  opption_spi( SPI_READ,  CAN_CTRL, 0x00);
  return 0;
}

static uint8 MloaFlag;  //仲裁失败计数器
static uint8 TxerFlag; // 总线错误计数器
_Can_Package CanSendTmp[10];
uint8 PCmessInp=0;
// bapi
uint8 requ_bapi( _Package* Data )
{
  uint8* l = (uint8*)Data;
  uint8 length=0;
  uint8 flag=0;
  uint8 send_times=0;
  if( Data->Length > 20 )return 1;
  if(Data->Type == 0xa2){
    length = Data->Length-12;
    memcpy( (uint8*)&CanSendTmp[PCmessInp], &l[12], length );
    PCmessInp++;
  }
  while(PCmessInp){
    send_times++;
    flag = requ_send( &CanSendTmp[PCmessInp-1], length );
    if( flag == 0 ){
      PCmessInp--;
    }
    if(send_times>3){
      send_times=0;
      return 1;
    }
  }
  return 0;
}
uint16 test1;
/* CAN send frame */
const uint8 CANINTF = 0x2c;
uint8 requ_send( _Can_Package* Data, uint8 Length )
{
  uint8 j;        //报文总数
  uint8* data = (uint8*)Data;
  uint8 statues_tmp;
  // wait txbuffer empty
  //opption_spi( SPI_WRITE, 0x30, 0 ); // TXB0CTRL CLEAR
  while( !(UCB0IFG__SPI_bit.UCTXIFG) ){}
  // delay 25*clock period(102ns)
  //if( Data->TargetAdd != 0x0100 )return 0;
  transmit_data();
  CSLOW;
  DELAY(35);
  UCB0TXBUF = 0x41;            // load txb0 rester,from trb0SD0(0x36)  
  for(j=0;j<Length;j++){       
    DELAY(35);
    UCB0TXBUF = data[j];
  } 
  DELAY(35);
  CSHIGH;  
  DELAY(35);
  opption_spi( SPI_WRITE, 0x35, Length );
  opption_spi_signal( 0x81 );   // RTS T0
  opption_spi( SPI_WRITE, 0x30, 0x0b ); // TXB0CTRL CLEAR 
  statues_tmp = opption_spi( SPI_READ, 0x30, 0x00 );
  if(statues_tmp&0x70){                 // 0x30 TB0CTRL
    if( (opption_spi( SPI_READ, CANINTF, 0x00 ) & 0x04) ){
      bit_mod_spi( 0x05, CANINTF, 0x04, 0x00 );          // clear INTF 
      }  
     //delay 25*CP (102ns)
    if( opption_spi( SPI_READ, 0x30, 0x00) & ABTF ){  //发送失败
      if( (opption_spi( SPI_READ, 0x30, 0x00 ) & MLOA ) ){
        MloaFlag++;  //仲裁失败
        return 1;
      }
      if( (opption_spi( SPI_READ, 0x30, 0x00 ) & TXERR) ){
        TxerFlag++;  //总线错误
        return 1;
      }
      return 0;
    }   
  }
  return 0;
}

const uint8 READb0SID = 0x90;
//const uint8 READb0DATA = 0x92;
/* read data to buffer */
uint8 u0_readata_tobuffer(void)
{
  uint8 i;
  uint8* r = (uint8*)&Rbuf[Rbuf_p];
  uint8 length = 13;
  if( (opption_spi( SPI_READ, CANINTF, 0x00 ) & 0x01) ){
    bit_mod_spi( 0x05, 0x2c, 0xff, 0x00 );          // clear INTF
  }
  CSLOW;
  while( !(UCB0IFG__SPI_bit.UCTXIFG) ){}
  DELAY(35);
  UCB0TXBUF = READb0SID;
  DELAY(35);
  for( i=0;i<length;i++ ){              // STANDDARDID+EXID+LENGTH+DATA[8] = 13
   DELAY(35);
    UCB0TXBUF = READb0SID;
    DELAY(35);
    *r++ = UCB0RXBUF;
    if(i==4){
      length=Rbuf[Rbuf_p]._DataLength_BOb._Bit.DLC+5;
    }
  }
  DELAY(35);
  CSHIGH;
  bit_mod_spi( 0x05, 0x2c, 0xff, 0x00 );          // clear INTF
  Rbuf_p++;
  Rbuf_p = Rbuf_p%PSIZE;
  return 0;
}

// receive data test 
uint8 RbufTest[512];
uint8 u0_readata_tobuffer_test( void )
{
  uint8* r = &RbufTest[Rbuf_p];
  CSLOW;
  while( !(UCB0IFG__SPI_bit.UCTXIFG) ){}
  DELAY(35);
  UCB0TXBUF = READb0SID;
  DELAY(35);
  UCB0TXBUF = READb0SID;
  *r++ = UCB0RXBUF; 
  DELAY(35);
  CSHIGH;
  Rbuf_p++;
  Rbuf_p = Rbuf_p%PSIZE;
  return 0;
  
}

// wu ma lv test send
uint8 resend_test( void )
{
    if(Rbuf_p!=Lbuf_p){            // 检查CAN是否收到有效数据 
    Lbuf_p++;
    Lbuf_p = Lbuf_p%PSIZE;
    while( !(UCB0IFG__SPI_bit.UCTXIFG) ){}
    // delay 25*clock period(102ns)
    //if( Data->TargetAdd != 0x0100 )return 0;
    CSLOW;
    DELAY(35);
    UCB0TXBUF = 0x41;      
    DELAY(35);
    UCB0TXBUF = RbufTest[Lbuf_p];
    DELAY(35);
    CSHIGH;  
    DELAY(35);
    opption_spi( SPI_WRITE, 0x35, 1 );
    opption_spi_signal( 0x81 );   // RTS T0
    }
    return 0;    
}

/* 检查各种中断标志并作处理等等... */
uint8 check_flag(void)
{
  while(FlagRX0BF){
    FlagRX0BF = 0;
    u0_readata_tobuffer();
    //u0_readata_tobuffer_test();
    //resend_test();
  }
  return 0;
}

_Package SendtoPC;
// 重新组包 jiapi
uint8 reload_package( _RDataTmp* Data)
{
  uint8 *p = (uint8*)&SendtoPC;
  uint8 *r = (uint8*)Data;
  uint16 addr;
  uint8 length = Data->_DataLength_BOb._Bit.DLC;
  SendtoPC.Head = 0x7279;
  SendtoPC.Length = Data->_DataLength_BOb._Bit.DLC+12;
  SendtoPC.Type = 0xa2;
  addr = __swap_bytes( 1 );
  SendtoPC.SourceAdd = addr;
  SendtoPC.TargetAdd = 0x0000;
  SendtoPC.Hop = 0x00;
  SendtoPC.Stamp = 0x0000;
  SendtoPC.AddChecking = 0xff; // jiaoyan
  memcpy( &p[12], &r[5], length );

  return 0;

}



/* CANINIT */
uint8 can_init( void )
{
  mcp2515_init();
  CSHIGH;
  opption_spi( SPI_READ, 0x31, 0 );
  opption_spi( SPI_READ, 0x32, 0 );
  TranFramConfig();
  opption_spi_signal( 0xc0 );    // RESET
  DELAY(1000);                  // wait for init finish
  DELAY(1000);
  DELAY(1000);
  DELAY(1000);                  // wait for init finish
  DELAY(1000);
  DELAY(1000);   
  //opption_spi_signal( 0xc0 );    // RESET
  conf_rxbuff();
  working_mode();
  transmit_data();
  DELAY(1000);
  DELAY(1000);
  DELAY(1000);
    DELAY(1000);                  // wait for init finish
  DELAY(1000);
  DELAY(1000); 
  irp_pin_init();
  DELAY(1000);
  DELAY(1000);
  DELAY(1000);
    DELAY(1000);                  // wait for init finish
  DELAY(1000);
  DELAY(1000); 
  
  return 0;
}

#if 0
/* CAN working */

uint8 can_working( void )
{
  check_flag();                               // 收数据函数
  if(Rbuf_p!=Lbuf_p){            // 检查CAN是否收到有效数据 
     // 把SendtoPC 数据通过 UART0 写出
    reload_package( &Rbuf[Lbuf_p] );
    UartW(0, (uint8*)&SendtoPC, SendtoPC.Length );
    Lbuf_p++;
    Lbuf_p = Lbuf_p%PSIZE;
    Rbuf_p = Rbuf_p%PSIZE;
  }    
  if(opption_spi( SPI_READ, 0x2d, 0x00 )==0x40){     // over load
    bit_mod_spi( 0x05, 0x2d, 0xff, 0x00 );          // clear INTE
    bit_mod_spi( 0x05, 0x2c, 0xff, 0x00 );          // clear INTF
    opption_spi( SPI_READ, 0x2C, 0x00 );            //eflg
  } 
  return 0;
}

#endif

// CAN working for colecter
_RDataTmp* can_working_colect( void )
{
  uint16 temp;
  //check_flag();
  if(Rbuf_p!=Lbuf_p){            // 检查CAN是否收到有效数据 
    temp = Lbuf_p;
    Lbuf_p++;
    Lbuf_p = Lbuf_p%PSIZE;
    return &Rbuf[temp];
  }    
  if(opption_spi( SPI_READ, 0x2d, 0x00 )==0x40){     // over load
    bit_mod_spi( 0x05, 0x2d, 0xff, 0x00 );          // clear IF
    bit_mod_spi( 0x05, 0x2c, 0xff, 0x00 );          // clear IF
    opption_spi( SPI_READ, 0x2C, 0x00 );            //eflg
  } 
  return 0;
}

#pragma vector = USCI_B0_VECTOR
__interrupt void spi0_isr( void )
{

  switch(__even_in_range(UCB0IV,4)){
  case 0: break;
  case 2: 
   
    //UCB0IFG = 0;
    break;
  case 4: 
    
    break;
  }
}

#pragma vector = PORT2_VECTOR
__interrupt void port2_isr(void)
{
 
  if(P2IFG & P2IFG1){
    FlagRX0BF = 1;
    check_flag();
    P2IFG &= ~P2IFG1;
  }  
  if(P2IFG & P2IFG2){
    FlagRX1BF = 1;
    P2IFG &= ~P2IFG2;
  }
  if(P2IFG & P2IFG3){
    //FlagRX0BF = 1;
    //u0_readata_tobuffer();
    //opption_spi( SPI_READ, 0x2d, 0x00 );//EFLG
    P2IFG &= ~P2IFG3;
  }
  if(P2IFG & P2IFG4 ){
    FlagTX0RST = 1;
    P2IFG &= ~P2IFG4;
  }
  if(P2IFG & P2IFG5 ){
    FlagTX1RST = 1;
    P2IFG &= ~P2IFG5;
  }
  if(P2IFG & P2IFG6 ){
    FlagTX2RST = 1;
    P2IFG &= ~P2IFG6;
  }
}

