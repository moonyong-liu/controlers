/*
  File: global_data.h
  Author: lhw
  Data: 2014/12/11
  Desc: This file placed the project needs global  definition
        Typedef and enum and struct */

#ifndef __GLOBAL_DATA_H
#define __GLOBAL_DATA_H
#include "protocol.h"

//for mcp2515.c          

enum{
  CH4      = 0x01,
  CO       = 0x02,
  CO2      = 0x03
};

const uint8 ConfigInfo = 0xa2;
const uint8 CanIDH = 0x01;
const uint8 CanIDL = 0x01; // 3bits max = 7

#define RELAYON  P1OUT &= ~0x10
#define RELAYOFF P1OUT |=  0x10
#define FANPOWOFF RELAYON
#define FANPOWON  RELAYOFF
#define RELAYSET    1
#define RELAYRESET  0

#define CSLOW     P3OUT &= ~0x01
#define CSHIGH    P3OUT |=  0x01



#define FORCELIGHT P9OUT |= 0x10
#define FORCENIGHT P9OUT &= ~0x10

#define HANDLIGHT P9OUT |= 0x20
#define AUTOLIGHT P9OUT &=~0x20

#define SENLIGHT if(LoCoExe->ExeType==Fan)P9OUT&=~0x40;\
                 else(P9OUT|=0x40)
#define SENNIGHT if(LoCoExe->ExeType==Fan)P9OUT|=0x40;\
                 else(P9OUT&=~0x40)

#define CANLIGHT P9OUT |= 0x80
#define CANNIGHT P9OUT &=~0x80
#define CANLEDBLINK P9OUT ^= 0x80

//logic_control.c
#define HALFPERCENT 50
#define ONEPERCENT 100
#define ONEHALFPER 150
#define TWOPERCENT 200
#define THEPERCENT 300

#define DELAY(n) for(uint16 _i=0;_i<n;_i++)  // (n=1)=4ns

#define READRELAY (P1OUT&0x10)>0?0:1
#define READFEEDBACK (P5IN&0x03)

#define HAND (P1IN&0x08)>0?1:0
#define FORCE  (P1IN&0x04)>0?1:0

//1875 * 0.008 = 15s
#define ASSOTIMEOUT 1875 
//3750 * 0.008 = 60s
#define RELAYONDELY 7500   

#endif
