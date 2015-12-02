/*
  File: logic_control.h
  Author: lhw
  Data: 2014/12/11
  Desc:  */
#include "io430.h"
#include "string.h"
#include "protocol.h"
#include "mcp2515.h"
#include "logic_control.h"
#include "global_data.h"

// internal variable
static _ExecuteInfo ExeCofig;
static _ExecuteAsso ExeAssoInfo;
static _ExecuteInfo *LoCoExe = (_ExecuteInfo *)0x1800;      //pointer on local config info
static _ExecuteAsso *LoCoAsso = (_ExecuteAsso *)0x1880;
static _OtptSign *Statues = (_OtptSign*)0x1900;
static _OtptSign  StatuesTmp;
//static _Warn     *Warning = (_Warn*)0x1901;
static _WarnCtrl      WarningTmp;
static _Can_Package SendResult;
static uint8 FanStop=0;           // fan status flag
_Can_Package AckMess;
uint8 ResetCan; //reset mcp2515

// gobal define or const
const uint8 ExeSet        = 0x00;
const uint8 AssoSet       = 0x05;
const uint8 Power         = 0x14;
const uint8 Fan           = 0x15;
const uint8 War           = 0x16;
const uint8 LocoHand      = 0x03;
const uint8 LocoAuto      = 0x02;
const uint8 ManConfigAll  = 0x02;
const uint8 ManConfigSelf = 0x03;
const uint8 ManAssoSelf   = 0x06;
const uint8 PcHand        = 0x01;

uint16 ConeCheck[64];            // asso timeout count buffer
uint16 LastValue;                // soft filter ,receive sensor value
uint16 NoMessage = 1875;         // No message so longer in 15s
uint16 DelayFlag;                // delay intterupt
uint16 FlagTa0Xs=CanIDH*5+125;       // nS send 1 time count delay 1s send
uint16 RelayOnSign;             // count relay delay time

uint8 CutDownSign;               // record who cut the power deveice 
uint8 LastDevice=0xff;           // consign lasttime which device cutdown the power

uint8 FlagHand;                  // check_auto();ta0();
uint8 FlagHandQ=250;             // 2s ;ta0
uint8 FlagHandOK;                // check_auto();TA0();
uint8 ForceCount;                // ta1
uint8 ForceSign;                 // ta1
uint8 FlagTnsend=0;              // time to send message
uint8 Led500ms=13;               // CAN LED BLINK PERIOD 
uint8 TimeFlag;                  // delay function 
uint8 StartDelay=1;                // relay delay on begin

// controler
// external variable

//------------------------------------------------------------------------------
// internal function 

// search ID in lib
static uint8 search_id( uint8 Id )
{
  uint8 i=0;
  uint8 loop = LoCoAsso->Quantity;
  if( loop == 0xff )return 0xff;
  for(i=0;i<loop;i++){
    if( Id == LoCoAsso->AssoInfo[i].AssoID ){
      return i;
    }
  }
  return 0xff;
}

// CutDownSign table
// assotype_1  0x01 + 0xfe == 0xff
// assotype_2  0x02 + 0xfd == 0xff
// assotype_3  0x04 + 0xfb == 0xff
// assotype_4  0x08 + 0xf7 == 0xff
// assotype_5  0x10 + 0xef == 0xff
// assotype_6  0x20 + 0xdf == 0xff

// assotype1 工作面 断电、报警
static uint8 assotype_1( _Warn *WarValue, uint8 CurrentDevice, uint16 value ) 
{
  if( LoCoExe->ExeType == Power ){
    if( WarValue->_Bit.SensorWar == 1 ){          // asso sensor warning 
      //ConeCheck[CurrentDevice] = 0;             // think about The sensor no data,clear or not
      //return 2;
    }
    else{                                      // asso sensor data ok
      ConeCheck[CurrentDevice] = 0;            // clear 30s count
    }
    if( value >= ONEHALFPER || WarValue->_Bit.SensorWar==1 ){
      RELAYOFF; // 断电
      CutDownSign |= 0x01;       // sign I cut the power
      return 1;
    }
    else if( value <= ONEPERCENT){
      if(!(CutDownSign&0xfe)){           // other assotype didn`t cut the power
        RELAYON;              // RELAY ON   
      }
      CutDownSign &= ~0x01;      // sign I repower
    }
  }
  else if( LoCoExe->ExeType == War ){
    if( value >= ONEPERCENT ){
      RELAYON;              // RELAY ON delay count begin // 报警
    }
    else{
      RELAYOFF;
    }
  }
  return 0;
}

// 回流风 断电、报警
static uint8 assotype_2( _Warn *WarValue, uint8 CurrentDevice, uint16 value )
{
  if( LoCoExe->ExeType == Power ){
    if( WarValue->_Bit.SensorWar == 1 ){          // asso sensor warning 
      //ConeCheck[CurrentDevice] = 0;             // think about The sensor no data,clear or not
      //return 2;
    }
    else{                                      // asso sensor data ok
      ConeCheck[CurrentDevice] = 0;            // clear 30s count
    }
    if( value >= ONEPERCENT || WarValue->_Bit.SensorWar==1 ){
      CutDownSign |= 0x02;       // sign I cut the power
      RELAYOFF; 
      return 1;
    }
    else{
      if(!(CutDownSign&0xfd)){           // other assotype didn`t cut the power
         RELAYON;              // RELAY ON 
      }
      CutDownSign &= ~0x02;      // sign I repower
    }
  }
  else if( LoCoExe->ExeType == War ){
    if( value >= ONEPERCENT ){
      RELAYON;
    }
    else{
      RELAYOFF;
    }
  }
  return 0;
}
// 被串工作面 断电、报警
static uint8 assotype_3( _Warn *WarValue, uint8 CurrentDevice, uint16 value )
{
  if( LoCoExe->ExeType == Power ){
    if( WarValue->_Bit.SensorWar == 1 ){          // asso sensor warning 
      //ConeCheck[CurrentDevice] = 0;             // think about The sensor no data,clear or not
    }
    else{                                      // asso sensor data ok
      ConeCheck[CurrentDevice] = 0;            // clear 30s count
    }
    if( value >= HALFPERCENT || WarValue->_Bit.SensorWar==1 ){
      CutDownSign |= 0x04;       // sign I cut the power
      RELAYOFF;
      return 1;
    }
    else{
      if(!(CutDownSign&0xfb)){           // other assotype didn`t cut the power
         RELAYON;              // RELAY ON 
      }
      CutDownSign &= ~0x04;      // sign I repower
    }
  }
  else if( LoCoExe->ExeType == War ){
    if( value >= HALFPERCENT ){
      RELAYON;
    }
    else{
      RELAYOFF;
    }
  }
  return 0;
}

// CH4>3.0% stop fan <1.5 start fan
static uint8 assotype_4( _Warn *WarValue, uint8 CurrentDevice, uint16 Value )
{
  if( LoCoExe->ExeType == Fan ){
    if( WarValue->_Bit.SensorWar == 1 ){          // asso sensor warning 
      //ConeCheck[CurrentDevice] = 0;             // think about The sensor no data,clear or not
    }
    else{                                      // asso sensor data ok
      ConeCheck[CurrentDevice] = 0;            // clear 30s count
    }
    if( Value >= THEPERCENT && FanStop==1 ){
      FANPOWOFF;                    // fan power off
      CutDownSign |= 0x08;       // sign I cut the power
      LastDevice = LoCoAsso->AssoInfo[CurrentDevice].AssoID;
      return 1;
    }
    else if( Value <= ONEHALFPER ){
      if( LastDevice != 0xff ){       // not first in 
        if( LastDevice == LoCoAsso->AssoInfo[CurrentDevice].AssoID ){
          if(!(CutDownSign&0xf7)){           // other assotype didn`t cut the power
             FANPOWON;              // fan power on         
          }
          CutDownSign &= ~0x08;      // sign I repower
        }
      }
      else{                                // first in
        if(!(CutDownSign&0xf7)){           // other assotype didn`t cut the power
           FANPOWON;              // fan power on       
        }
        CutDownSign &= ~0x08;      // sign I repower
      }
    }
  }
  return 0;
}

// all powerdevice need
static uint8 assotype_5( _Warn *WarValue, uint8 CurrentDevice )
{
  if( LoCoExe->ExeType == Power  ||  LoCoExe->ExeType == Fan ){
    if( WarValue->_Bit.SensorWar == 1 ){          // asso sensor warning 
      //ConeCheck[CurrentDevice] = 0;             // think about The sensor no data,clear or not
      //return 2;
    }
    else{                                      // asso sensor data ok
      ConeCheck[CurrentDevice] = 0;            // clear 30s count
    }
    if( (WarValue->_Bit.CutofValue == 1) || ( WarValue->_Bit.SensorWar == 1 ) ){
      if(LoCoExe->ExeType == Fan){
        FanStop = 1;             // FAN status : stop
      }
      else{
        RELAYOFF;
      }
      CutDownSign |= 0x10;       // sign I cut the power
      return 1;
    }
    else{
      FanStop = 0;            // FAN status : RUN
      if(!(CutDownSign&0xef)){           // other assotype didn`t cut the power
         RELAYON;              // RELAY ON 
      }
    }
    CutDownSign &= ~0x10;      // sign I repower
  }
  else if( LoCoExe->ExeType == War ){
    if((WarValue->_Bit.AlmValue == 1)){
      RELAYON;
    }
    else{
      RELAYOFF;
    }
  }
  return 0;
}

// 总 电 源 开 关 
// 所有关联节点无数据30s后 断电
static uint8 assotype_6( _Warn *Value, uint8 CurrentDevice )
{
  if( LoCoExe->ExeType == Power ){
    if( Value->_Bit.SensorWar == 1 ){          // asso sensor warning 
      //ConeCheck[CurrentDevice] = 0;             // think about The sensor no data,clear or not
      return 2;
    }
    else{                                      // asso sensor data ok
      ConeCheck[CurrentDevice] = 0;            // clear 30s count
    }
  }
  return 0;
}


enum{
  Asso_H = 0x05,
  ExeC_H = 0x00,
  ExeM_H = 0x09,
  ColM_H = 0x07,
  StaM_H = 0x47
};
// send message 
static uint8 send_exe_message( uint8 whodid )
{ 
  uint8 length=6;
  //uint8 flag=0;
  //uint8 send_times=0;
  uint8 fedback=0;
  uint8 fedbackerr=0; //kui dian yi chang
  uint8 readrelay=0;
  memset((void *)&SendResult,0,length);
  SendResult._Type.Type = ExeM_H;
  SendResult._ID.ID = CanIDH;
  if(LoCoExe->ExeType==Fan){
    readrelay = !READRELAY;
  }
  else{
    readrelay = READRELAY;
  }
  SendResult._DataForWho._OpDevFe.OtptSign._Bit.OtptVal |= readrelay;
  WarningTmp._Bit.DeviceERR = 0; 
  if( READFEEDBACK == 0x01 ){
    if(LoCoExe->ExeType==Fan){
      fedback = 0;
    }
    else{
      fedback = 1;
    }
  }
  else if( READFEEDBACK == 0x02){
    if(LoCoExe->ExeType==Fan){
      fedback = 1;
    }
    else{
      fedback = 0;
    }
  }
  else if( READFEEDBACK == 0x03){
    WarningTmp._Bit.DeviceERR = 1;
    //backoff on: p5.0  off: p5.1
    fedback = 0;
  }
  else{
    fedback = 0;
    fedbackerr = 1;
  }
  SendResult._DataForWho._OpDevFe.OtptSign._Bit.Fedback |= fedback;
  SendResult._DataForWho._OpDevFe.OtptSign._Bit.FedbkErr |= fedbackerr;
  SendResult._DataForWho._OpDevFe.OtptSign._Bit.CtrlMode |= Statues->_Bit.CtrlMode;
  SendResult._DataForWho._OpDevFe.OtptSign._Bit.Force |= Statues->_Bit.Force;
  SendResult._DataForWho._OpDevFe.Warning = WarningTmp;
  SendResult._DataForWho._OpDevFe.OtptSign._Bit.AssoEn |= StatuesTmp._Bit.AssoEn;
  if( StatuesTmp._Bit.AssoEn ){
    SendResult._DataForWho._OpDevFe.AssoID = LoCoAsso->AssoInfo[whodid].AssoID;
    SendResult._DataForWho._OpDevFe.AssoType = LoCoAsso->AssoInfo[whodid].AssoType;
  }
  else{
    SendResult._DataForWho._OpDevFe.AssoID = 0;
    SendResult._DataForWho._OpDevFe.AssoType = 0;
  }
  

  requ_send( &SendResult, length );
#if 0
  while(flag){
    send_times++;
    flag = requ_send( &SendResult, length );
    if(send_times>3){
      send_times=0;
      return 1;
    }
  }
#endif
  //DELAY(30000);
  return 0;
}

// send config information
static uint8 send_configinfo(uint16 hold)
{
  //uint8 flag = 0;
  //uint8 send_times = 0;
  
  FlagTa0Xs=CanIDH*5+1000;       // nS send 1 time clear timer wait 8s continue send
  delay(hold);
  uint8 datalen;
  SendResult._Type.Type = ExeC_H;
  SendResult._ID.ID = CanIDH;
  SendResult._DataForWho._ExecuteInfo.ExeType = LoCoExe->ExeType;
  datalen = 3;
  requ_send( &SendResult, datalen);
#if 0
  while(flag){
    send_times++;
    flag = requ_send( &AckMess, datalen );
    if(send_times>3){
      send_times=0;
      return 1;
    }
  }
#endif
  return 0;
}


// send associate
static uint8 send_assoinfo(void)
{
  volatile uint8 i,flag;
  uint8 assolen=LoCoAsso->Quantity;
  uint8 datalen;
  //uint8 send_times = 0;
  
  for(i=0;i<assolen;){
    SendResult._Type.Type = Asso_H;
    SendResult._ID.ID = CanIDH;
    SendResult._DataForWho._AssoInfo.AssoID = LoCoAsso->AssoInfo[i].AssoID;
    SendResult._DataForWho._AssoInfo.AssoType = LoCoAsso->AssoInfo[i].AssoType;
    datalen = 4;
    flag = requ_send( &SendResult, datalen);
#if 0
    while(flag){
      send_times++;
      flag = requ_send( &AckMess, datalen );
      if(send_times>3){
        send_times=0;
        break;
      }
    } 
#endif
    if( flag == 0 ){
      i++; 
    }
    else{
      i=i;
    }
    delay(150);  
  }
  return 0;
}


// ack pc send message
static uint8 ackmessage( _RDataTmp* Data )
{
  //uint8 flag=0;
  //uint8 send_times=0;
  uint8 *s = (uint8*)Data;
  uint8 *t = (uint8*)&AckMess;
  uint8 length = Data->_DataLength_BOb._Bit.DLC;
  memcpy( t, &s[5], length );
  requ_send( &AckMess, length );
  delay(150);
#if 0
  while(flag){
    send_times++;
    flag = requ_send( &AckMess, length );
    if(send_times>3){
      send_times=0;
      return 1;
    }
  }
#endif
  return 0;
}

// TIME A0 Init
static void Ta0Init(void)
{
//TA0设置
	//TA0CCTL0 = CCIE;		   // CCR0 interrupt enabled
	TA0CCR0 = 3000;
	TA0CTL = TASSEL_2 + ID__8+MC_1 + TACLR;	// SMCLK/8, up mode, clear TAR
	TA0EX0=TAIDEX_7;                        // SMCLK/8/8
//	P4DIR |= 0x08;							// Set P4.3 to output direction	__no_operation();
	//24M/8/8/3000=  125Hz
}

static void Ta1Init(void)
{
//TA1设置
	//TA1CCTL0 = CCIE;		   // CCR0 interrupt enabled
	TA1CCR0 = 150;
	TA1CTL = TASSEL_2 + ID__8 + MC_1 + TACLR;	// SMCLK/8, up mode, clear TAR
	TA1EX0=TAIDEX_1;                        // SMCLK/8/8
	//24M/8/2/150=  10000Hz 100us
}
static void pin_out_init( void )
{
  // RELAYON p1.4
  P1DIR |=  0x10;
  P1DS  |=  0x10;
  P1SEL &= ~0x10;
  RELAYOFF;
  
  //can,sen,auto,force p9.7.6.5.4
  P9DIR |= 0xF0;
  P9DS  |= 0xf0;
  P9SEL &= ~0xf0;
  P9OUT &= ~0xf0;
}

// pin in init 
static void pin_in_init(void)
{
  //backoff on: p5.0  off: p5.1
  P5DIR &= 0xfc;	
  P5REN |= 0x03;		
  P5SEL &= 0xfc;
  
  // HAND/AUTOp1.2 FORCE p1.3
  P1DIR &= 0xf3;
  P1REN |= 0x0C;
  P1SEL &= 0xf3;
}

// writeflash associate
static uint8 write_flash_asso(void)
{
  __disable_interrupt();			// 5xx Workaround: Disable global
                                      // interrupt while erasing. Re-Enable
                                      // GIE if needed
  FCTL3 = FWKEY;                          // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                    // Set Erase bit
  //*(unsigned int *)LoCoInPo = 0;         // Dummy write to erase Flash seg
  FCTL1 = FWKEY+WRT;                      // Set WRT bit for write operation

  *LoCoAsso = ExeAssoInfo;                 // write flash
  
  FCTL1 = FWKEY;                          // Clear WRT bit
  FCTL3 = FWKEY+LOCK;                     // Set LOCK bit

  __enable_interrupt();
  return 0;
}

// write flash execonfig 
static uint8 write_flash_exe(void)
{
  __disable_interrupt();			// 5xx Workaround: Disable global
                                      // interrupt while erasing. Re-Enable
                                      // GIE if needed
  FCTL3 = FWKEY;                          // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                    // Set Erase bit
  //*(unsigned int *)LoCoInPo = 0;         // Dummy write to erase Flash seg
  FCTL1 = FWKEY+WRT;                      // Set WRT bit for write operation

  *LoCoExe = ExeCofig;                 // write flash
  
  FCTL1 = FWKEY;                          // Clear WRT bit
  FCTL3 = FWKEY+LOCK;                     // Set LOCK bit

  __enable_interrupt();
  return 0;
}

// write flash statues 
static uint8 write_flash_statues(void)
{
  __disable_interrupt();			// 5xx Workaround: Disable global
                                      // interrupt while erasing. Re-Enable
                                      // GIE if needed
  FCTL3 = FWKEY;                          // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                    // Set Erase bit
  FCTL1 = FWKEY+WRT;                      // Set WRT bit for write operation

  *Statues = StatuesTmp;                 // write flash
  
  FCTL1 = FWKEY;                          // Clear WRT bit
  FCTL3 = FWKEY+LOCK;                     // Set LOCK bit

  __enable_interrupt();
  return 0;
}
#if 0
// write flash warrning 
static uint8 write_flash_Warning(void)
{
  __disable_interrupt();			// 5xx Workaround: Disable global
                                      // interrupt while erasing. Re-Enable
                                      // GIE if needed
  FCTL3 = FWKEY;                          // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                    // Set Erase bit
  FCTL1 = FWKEY+WRT;                      // Set WRT bit for write operation

  *Warning = WarningTmp;                 // write flash
  
  FCTL1 = FWKEY;                          // Clear WRT bit
  FCTL3 = FWKEY+LOCK;                     // Set LOCK bit

  __enable_interrupt();
  return 0;
}
#endif
//clear flash
static uint8 clear_flash_exe(void)
{
  //擦除flash        
  __disable_interrupt();			// 5xx Workaround: Disable global
  FCTL3 = FWKEY;                          // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                    // Set Erase bit
  *(unsigned int *)LoCoExe = 0;         // Dummy write to erase Flash seg 0x1800  
  FCTL3 = FWKEY+LOCK;                     // Set LOCK bit
  __enable_interrupt();
  return 0;
}

static uint8 clear_flash_asso(void)
{
  //擦除flash        
  __disable_interrupt();			// 5xx Workaround: Disable global
  FCTL3 = FWKEY;                          // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                    // Set Erase bit
  *(unsigned int *)LoCoAsso = 0;         // Dummy write to erase Flash seg 0x1880  
  FCTL3 = FWKEY+LOCK;                     // Set LOCK bit
  __enable_interrupt();
  return 0;
}

static uint8 clear_flash_statues(void)
{
  //擦除flash        
  __disable_interrupt();			// 5xx Workaround: Disable global
  FCTL3 = FWKEY;                          // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                    // Set Erase bit
  *(unsigned int *)Statues = 0;         // Dummy write to erase Flash seg 0x1900  
  FCTL3 = FWKEY+LOCK;                     // Set LOCK bit
  __enable_interrupt();
  return 0;
}
#if 0
// LCEAR WAR FLASH
static uint8 clear_flash_Warning(void)
{
  //擦除flash        
  __disable_interrupt();			// 5xx Workaround: Disable global
  FCTL3 = FWKEY;                          // Clear Lock bit
  FCTL1 = FWKEY+ERASE;                    // Set Erase bit
  *(unsigned int *)Warning = 0;         // Dummy write to erase Flash seg 0x1901  
  FCTL3 = FWKEY+LOCK;                     // Set LOCK bit
  __enable_interrupt();
  return 0;
}
#endif

#if 0 
static uint8 lvbo( uint16 Value )
{
  if( Value>=50000 )return 1;
  if( LastValue != 0 ){
    if( Value>LastValue ){
      if( Value-LastValue > 2000 ){
        LastValue = Value;
        return 1;
      }
    }
    else{
      if( LastValue-Value > 2000 ){
        LastValue = Value;
        return 1;
      }
    }
  }
  LastValue = Value;
  return 0;
}
#endif

// statues init
static uint8 statues_init(void)
{
  if( Statues->Byte == 0xff ){
    StatuesTmp._Bit.AssoEn = 0;
    StatuesTmp._Bit.CtrlMode = 2;
    StatuesTmp._Bit.Fedback = 0;
    StatuesTmp._Bit.Force = 0;
    StatuesTmp._Bit.OtptVal = 0;
    StatuesTmp._Bit.x = 0;
    write_flash_statues();
    DELAY(1000);
  }  
  return 0;
}

// check auto or hand button
uint8 check_auto(void)
{
  if( HAND ){
    FlagHand = 1;    // for TA0();
  }
  else{
    FlagHand = 0;
  }
  if( FlagHandOK ){  // from TA0();
    FlagHandOK = 0;
    if( Statues->_Bit.CtrlMode == LocoHand ){
      StatuesTmp = *Statues;
      StatuesTmp._Bit.CtrlMode = LocoAuto;
      clear_flash_statues();
      write_flash_statues();
      AUTOLIGHT;
      FORCENIGHT;
    }
    else if( Statues->_Bit.CtrlMode == LocoAuto || Statues->_Bit.CtrlMode == PcHand ){
      StatuesTmp = *Statues;
      StatuesTmp._Bit.CtrlMode = LocoHand;
      clear_flash_statues();
      write_flash_statues();
      HANDLIGHT;
    }
  }
  if(Statues->_Bit.CtrlMode == LocoHand){
    if(ForceCount>250){
      ForceCount=0;
      if(ForceSign){
        if( LoCoExe->ExeType == Fan ){
          RELAYON;
        }
        else{
          RELAYOFF;
        }
        FORCELIGHT;
      }
      else{
        if( LoCoExe->ExeType == Fan ){
          RELAYOFF;
        }
        else{
          RELAYON; 
        }
        FORCENIGHT;
      }      
    }
  }
  if( READFEEDBACK == 0x01 ){
    SENLIGHT;
  }
  else{
    SENNIGHT;
  }
  if( Statues->_Bit.CtrlMode == LocoHand ){
    HANDLIGHT;
  }
  else{
    AUTOLIGHT;
  }
  return 0;
}


// check overtime not receive package
uint8 check_overtime( void )
{
  uint8 asso_number;
  uint8 tmp1;
  if( Statues->_Bit.CtrlMode != LocoAuto ){      // if hand control
    return 1;
  }
  asso_number = LoCoAsso->Quantity;
    if( LoCoAsso->Quantity != 0xff ){
    asso_number = LoCoAsso->Quantity;
  }
  else{
    asso_number=0;
  }
  for(tmp1=0; tmp1<asso_number;tmp1++){
    if( ConeCheck[tmp1] >= ASSOTIMEOUT ){       // asso sensor didn`t receive data 30s
      if(LoCoExe->ExeType == Fan){
        RELAYON;
      }
      else{
        RELAYOFF;                                // power off
      }
      CutDownSign |= 0x20;                     // sign I cut the power
      LastDevice = LoCoAsso->AssoInfo[tmp1].AssoID;
      return 1;
    }
    else{
      if( LastDevice != 0xff ){        // not first in
        if( LastDevice == LoCoAsso->AssoInfo[tmp1].AssoID ){
          if(!(CutDownSign&0xdf)){                 // other assotype didn`t cut the power
             RELAYON;              // RELAY ON  
          }
          CutDownSign &= ~0x20;      // sign I repower
        }
      }
      else{                                // first in
        if(!(CutDownSign&0xdf)){           // other assotype didn`t cut the power
           RELAYON;              // RELAY ON         
        }
        CutDownSign &= ~0x20;      // sign I repower
      }
    }
  }
  return 0;
}
// start 1min delay
uint8 check_realyon(void)
{
  if( Statues->_Bit.CtrlMode == LocoAuto ){
    if( RelayOnSign >= RELAYONDELY ){      // wait realyondely
      StartDelay=0;
    }
  }
  else if( Statues->_Bit.CtrlMode == PcHand ){
    RelayOnSign = 0;
  }
  return 0;
}

// power signal feedback
uint8 check_feedback_working( void )
{
  if( READFEEDBACK ){
    
  }
  return 0;
}

// check not config ok
uint8 check_config_faile( void )
{
  if( LoCoExe->DevSeted == 1 ){
    if( LoCoExe->ExeType>=0x14 && LoCoExe->ExeType<=0x16 ){
      WarningTmp._Bit.ExeNotCofig = 0;
    }
    else{
      WarningTmp._Bit.ExeNotCofig = 1;
    }
  }
  else{
    WarningTmp._Bit.ExeNotCofig = 1;
  }
  if( LoCoAsso->Quantity != 0xff ){
    if( LoCoAsso->Quantity>0 && LoCoAsso->Quantity<=0xfe ){
      WarningTmp._Bit.AssoNotCofig = 0;
    }
    else{
      WarningTmp._Bit.AssoNotCofig = 1;
    }
  }
  else{
    WarningTmp._Bit.AssoNotCofig = 1;
  }
  
  return 0;
}
//------------------------------------------------------------------------------
// extern function

// logic controler init
uint8 logic_init( void )
{
  Ta0Init();
  Ta1Init();
  pin_out_init();
  pin_in_init();
  statues_init();
  return 0;
}


// executor working function (main)


uint8 exe_working(void)
{
  while(FlagTnsend){     // 到时间发数据 FLAG
    FlagTnsend = 0;
    send_exe_message(0);
  }
  if( ResetCan == 1){
    ResetCan = 0;
    WDTCTL = WDTPW+WDTHOLD;//Stop wdt
    WDTCTL = WDTPW+WDTSSEL__VLO+WDTCNTCL_L+WDTIS__8192+WDTHOLD;//Stop wdt
    can_init();
    WDTCTL = WDTPW+WDTSSEL__VLO+WDTCNTCL_L+WDTIS__8192;		// Run WDT  
  }
  check_auto();
  check_overtime();
  check_realyon();
  check_config_faile();
  return 0;
}


//const uint8 PcAuto      = 0x00;
uint8 handle_data_logic( _RDataTmp* NewData )
{
  uint8 i=0; 
  uint8 assotype=0; // 关联ID在数组中的位置
  uint16 senvalue=0;
  _Warn senwarn;
  uint8 number;
  uint8 sontype;
  uint8 p_to_id;  // point to which ID
  uint8 whodid=0;   // which ID change the controler
  uint8 tmp1=0;  // clear asso timeout count buffer
  uint8 asso_number; // cler asso timeout number
  NoMessage = 0;
    if( NewData->Data._ID.ID == CanIDH ){   // 给自己
      if( NewData->Data._Type.Type == ExeSet ){     // 执行器设置
        ExeCofig.ExeType = NewData->Data._DataForWho._ExecuteInfo.ExeType; //保存执行器类型
        ExeCofig.DevSeted = 1;
        clear_flash_exe();
        write_flash_exe(); 
      }
      else if( NewData->Data._Type.Type == AssoSet ){   //关联设置
        ackmessage( NewData );
        sontype = NewData->Data._Type.TypeA;
        if( sontype == 0x00 ){
          ExeAssoInfo = *LoCoAsso;
          number = ExeAssoInfo.Quantity;
          if(number == 0xff)number=0;
          if(number >= 0xfe)return 1;            //FLASH full
          for(i=0;i<number;i++){
            if(ExeAssoInfo.AssoInfo[i].AssoID == NewData->Data._DataForWho._AssoInfo.AssoID){
              return 1;
            } //重复数据不写
          }
          ExeAssoInfo.AssoInfo[number].AssoID = NewData->Data._DataForWho._AssoInfo.AssoID;
          ExeAssoInfo.AssoInfo[number].AssoType = NewData->Data._DataForWho._AssoInfo.AssoType;
          number++;
          ExeAssoInfo.Quantity = number;
          clear_flash_asso();
          write_flash_asso();
        }
        else if( sontype == 0x01 ){
          asso_number = LoCoAsso->Quantity;
          for(tmp1=0; tmp1<asso_number;tmp1++){
            ConeCheck[tmp1] =0;
          }
          clear_flash_asso();
          LastDevice=0xff;
        }      
      }
      else if( NewData->Data._Type.Type == 0x08 ){  // 输出设备
        if( Statues->_Bit.CtrlMode == LocoHand )return 1;  // 若是本地手动则无效
        sontype = NewData->Data._Type.TypeA;
        if( sontype == 0){                             // 输出操作
          if( NewData->Data._DataForWho._OpDev.Force == 0x01 ){   // 强制 远程手动
            StatuesTmp = *Statues;  
            StatuesTmp._Bit.Force = 1;
            StatuesTmp._Bit.CtrlMode = PcHand;
            clear_flash_statues();
            write_flash_statues();
            if( NewData->Data._DataForWho._OpDev.OValue == 0x01 ){  //输出
              if( LoCoExe->ExeType == Fan ){
                RELAYOFF;
              }
              else{
                RELAYON;
              }
            }
            else if( NewData->Data._DataForWho._OpDev.OValue == 0x00 ){
              if( LoCoExe->ExeType == Fan ){
                RELAYON;
              }
              else{
                RELAYOFF;
              }
            }
          }
          else if(  NewData->Data._DataForWho._OpDev.Force == 0x00 ){ //取消强制 本地自动
            StatuesTmp = *Statues;  
            StatuesTmp._Bit.Force = 0;
            StatuesTmp._Bit.CtrlMode = LocoAuto;
            clear_flash_statues();
            write_flash_statues();
          }
        }
        //ackmessage( NewData );
      }
      else if( NewData->Data._Type.Type == ManAssoSelf ){    //查询关联信息
        if( LoCoAsso->Quantity != 0xff ){
          sontype = NewData->Data._Type.TypeA;
          if( sontype == 0x00 ){
            send_assoinfo();
          }
        }
      }
      else if( NewData->Data._Type.Type == ManConfigSelf ){  //查询本机配置信息
        //if( LoCoExe->DevSeted ){
          sontype = NewData->Data._Type.TypeA;
          if( sontype == 0x00 ){
            send_configinfo(0);
          }  
        //}
      }
    }// end of 给自己
    else if( NewData->Data._Type.Type == ManConfigAll ){     //查询所有设备配置信息
      //if( LoCoExe->DevSeted ){
        sontype = NewData->Data._Type.TypeA;
        //if( sontype == 0x00 ){
          send_configinfo(CanIDH*480);
        //}
      //}
    }
    else{                  // 其他CAN节点信息
      if( StartDelay == 1 )return 1;         // start 1min delay
      if( Statues->_Bit.CtrlMode != LocoAuto )return 1;
      if( NewData->Data._Type.Type == ColM_H ){
        //assotype = search_id( NewData->Data._ID.ID );
        p_to_id = search_id( NewData->Data._ID.ID );
        if( p_to_id == 0xff )return 1;
        assotype = LoCoAsso->AssoInfo[p_to_id].AssoType;
        senwarn = NewData->Data._DataForWho._CoDa.Warning;
        senvalue = NewData->Data._DataForWho._CoDa.Value;
        senvalue = __swap_bytes( senvalue );
        //if( lvbo( senvalue ) )return 1;            // lvbo
        switch( assotype ){
        case 1:whodid = assotype_1( &senwarn, p_to_id, senvalue );break;
        case 2:whodid = assotype_2( &senwarn, p_to_id, senvalue );break;
        case 3:whodid = assotype_3( &senwarn, p_to_id, senvalue );break;
        case 4:whodid = assotype_4( &senwarn, p_to_id, senvalue );break;
        case 5:whodid = assotype_5( &senwarn, p_to_id );break;
        case 6:whodid = assotype_6( &senwarn, p_to_id );break;
        default: break;
        }
        if( whodid == 1 ){
          FlagTa0Xs = 800;  //clear timer
          StatuesTmp._Bit.AssoEn = 1; 
          WarningTmp._Bit.AssoDevAlm = 1;
          send_exe_message( p_to_id );
          WarningTmp._Bit.AssoDevAlm = 0;
          StatuesTmp._Bit.AssoEn = 0;
        }
      }
    }
  return 0;
}

void delay( uint16 time )
{
  DelayFlag=time;
  TimeFlag=1;
  while(TimeFlag){
    WDTCTL = WDTPW+WDTSSEL__VLO+WDTCNTCL_L+WDTIS__8192;// Feed WDT   
  }
  
}
// 定时器A0，设定频率125Hz(0.008S)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{  
  uint8 asso_number;
  uint8 tmp1;
  // n秒发一次数据计数器 
  if( FlagTa0Xs != 0 ){
    FlagTa0Xs--;
  }
  else{
    FlagTa0Xs = 750;  // 6s send once
    FlagTnsend = 1;
  }
  if( FlagHand ){    // if press
    if( FlagHandQ != 0){
      FlagHandQ--;
    }
    else{
      FlagHandQ = 250;
      FlagHandOK = 1;
    }
  }
  else{            // if not press
    FlagHandQ = 250;
  }
  if(Led500ms==0){                  //CAN/SENSOR LED BLINK PERIOD
    Led500ms=62;
    if(NoMessage>=30){
      NoMessage = NoMessage;
      CANLEDBLINK;
    }
    else{
      CANLIGHT;
      NoMessage++;
    }
  }
  else{
    Led500ms--;
  }
  if(FORCE){
    if(ForceCount>=250){
      //ForceCount = 250;
      ForceSign ^= 1;
    }
    ForceCount++;
  }
  else{
    ForceCount = 0;
  }
  // check asso device time out
  if( StartDelay == 0){
    if( Statues->_Bit.CtrlMode == LocoAuto ){
      if( LoCoAsso->Quantity != 0xff ){
        asso_number = LoCoAsso->Quantity;
      }
      else{
        asso_number=0;
      }
      for(tmp1=0; tmp1<asso_number;tmp1++){
        if( ConeCheck[tmp1] >= ASSOTIMEOUT ){  
        }
        else{
          ConeCheck[tmp1]++;
        }
      }
    }
  }
  // check relay on sign ok
  if( StartDelay ){                  // begin count to set relay
    if( RelayOnSign <= RELAYONDELY ){       // less than RELAYONDELY
      RelayOnSign++;                 // go up
    }
  }
  else{
    RelayOnSign=0;                   // clear relay count times
  }

}
// delay time = 100us
// 100us
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
  
  if(DelayFlag==0){
    TimeFlag = 0;
  }
  else{
    DelayFlag--;
  }

}
