/*
** file: mcp2515.h
** author: lhw
** date: 2014/11/27
** des:  mcp2515 head file
*/


#ifndef __mcp2515_h
#define __mcp2515_h

//#include "global_data.h"
#include "protocol.h"
//#include "io430x54xa.h"


/*SPI Order*/
enum {
  SPI_RESET =0xc0,
  SPI_WRITE =0x02,
  SPI_READ  =0x03,
  // read status
  SPI_RSTAT = 0xa0,
  // rx status
  SPI_RXSTA = 0xb0
};

/*CAN register*/
enum {
  // contrl
  CAN_CTRL = 0x0f,
  // statues
  CAN_STAT = 0x0e,
  // error flag
  CAN_EFLG = 0x2d,
  // RXB0CTRL
  CAN_RXCT0= 0x60,
  // RXB1CTRL 
  CAN_RXCT1= 0x70,
  // BFPCTRL
  CAN_BFPCTRL = 0x0c
};

/* CANCTRL every bits set value */
enum {
  // working mode
  MOD_DEF = 0x00,
  MOD_SLP = 0x20,
  // loopback
  MOD_LOP = 0x40,
  // interception only
  MOD_CEP = 0x60,
  // config mode
  MOD_CON = 0x80,
  //  ABAT 
  MOD_ABAT= 0x10,
  // send once only
  MOD_OSM = 0x08,
  // CLKEN 
  MOD_CLK = 0x04,
  // CLK Prescale
  MOD_CP1 = 0x00,
  MOD_CP2 = 0x01,
  MOD_CP4 = 0x02, 
  MOD_CP8 = 0x03  
};

/* RXB0CTRL*/
enum {
  // RXM:RX working mode
  // off shield/fliter function re all frame
  RXB0_ALLRE = 0x60,
  // re filter pass extension valid frame
  RXB0_FPEVF = 0x40,
  // re filter pass standard valid frame
  RXB0_FPSVF = 0x20,
  // re filter pass extension and standard valid frame
  RXB0_FPESVF= 0x00,
  RXB0_RXRTR = 0x08,
  RXB0_BUKT  = 0x04,
  RXB0_RXF1  = 0x02,
  RXB0_RXF0  = 0x00
};

/* RXB1CTRL */
enum {
  //                         re all frame
  RXB1_ALLRE = 0x60,
  // re          extension
  RXB1_FPEVF = 0x40,
  // re          standard
  RXB1_FPSVF = 0x20,
  // re   flitered EX. and ST.
  RXB1_FPESVF= 0x00,
  RXB1_RXRTR = 0x80,
  RXB1_RXF5  = 0x05,
  RXB1_RXF4  = 0x04,
  RXB1_RXF3  = 0x03, 
  RXB1_RXF2  = 0x02,
  RXB1_RXF1  = 0x01, 
  RXB1_RXF0  = 0x00
};

// TXBNCTRL
enum{
  ABTF = 0x40,
  MLOA = 0x20,
  TXERR= 0x10,
  TXREQ= 0x08,
  TXP1 = 0X03,
  TXP2 = 0x02,
  TXP3 = 0x01,
  TXP4 = 0x00 
};


/* Transmitons buffer configuration 
   Standard frame
   Extend frame 
   */
typedef struct
{
  uint8 StandardIDH;
  union{
    struct{ 
      uint8 ExtendID2bits   : 2;  // extend frame two High bits in Standdard low byte
      uint8                 : 1;
      uint8 ExIdeEnable     : 1;
      uint8                 : 1;
      uint8 StandardID3bits : 3;
    }_Bit;
    uint8 StandardIDL;
  }_StandardIDL_BOb;
  uint8 ExtendIDH;         // extend frame High byte
  uint8 ExtendIDL;         //              Low  byte
  union{
    struct{
      uint8 DLC          : 4;  // data length code max=8bytes
      uint8              : 2;
      uint8 RTR          : 1;  // remot transmit request
      uint8              : 1;
    }_Bit;
    uint8 DataLength;
  }_DataLength_BOb;

}_ConfTransmt;

/* Recive Buffer configration*/
typedef struct
{
  uint8 StandardIDH;
  union{
    struct{
      uint8 ExtendID2bit    : 2; // extend frame two highest bits in SIDL
      uint8                 : 1;
      uint8 IDE             : 1;  // extend ID signed
      uint8 SRR             : 1;  // standrad frame remot request
      uint8 StandardID3bits : 3;  
    }_Bit;
    uint8 StandardIDL;
  }_StandardIDL_BOb;
  uint8 ExtendIDH;
  uint8 ExtendIDL;
  union{
    struct{
      uint8 DLC            : 4; // show receive data length
      uint8 RB0            : 1; //
      uint8 RB1            : 1; 
      uint8 RTR            : 1; // extend frame remot request
      uint8                : 1; 
    }_Bit;
    uint8 DataLength;
  }_DataLength_BOb;
}_ConfRebuffer;


// function declaration
uint8 mcp2515_init( void );
uint8 opption_spi( uint8, uint8, uint8 );
uint8 opption_spi_signal( uint8 );
uint8 bit_mod_spi( uint8, uint8, uint8, uint8 );
uint8 working_mode(void);
uint8 requ_send(_Can_Package*, uint8 );
//uint8 requ_bapi( _Package* );
uint8 transmit_data(void);
uint8 conf_rxbuff(void);
void TranFramConfig(void);
uint8 reload_package(void);      
uint8 check_flag(void);
uint8 u0_readata_tobuffer(void);
_Package* return_device_data( void );
uint8 can_init(void);
uint8 can_working(void);
_RDataTmp* can_working_colect( void );
uint8 working_mode_osm(void);

#endif
