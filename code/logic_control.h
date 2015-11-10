/*
  File: logic_control.h
  Author: lhw
  Data: 2014/12/11
  Desc: Header by logic_control.c */
  
#ifndef __LOGIC_CONTROL_H
#define __LOGIC_CONTROL_H
#include "protocol.h"
    
uint8 handle_data_logic( _RDataTmp* );
uint8 search_id( uint8 Id );
uint8 assotype_1( uint16 );
uint8 assotype_2( uint16 );
uint8 assotype_3( uint16 );
uint8 assotype_4( uint16, uint8 );
uint8 assotype_5( _Warn* );
uint8 exe_working(void);
uint8 logic_init(void);
void delay( uint16 );

#endif
