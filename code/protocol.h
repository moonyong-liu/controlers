/*
  File: protocol.h 
  Author: lhw
  Data: 2014/12/11
  Desc: protocol for all device */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H
//#include "global_data.h"
#define _yr_					       0x7279		//ID ͨѶ��
#define TRUE                 1   /* page 207 K+R 2nd Edition */
#define FALSE                0
#define PackHeadSize				 12					//ͨ�Ű�ͷ��С
#define TxBufLength          1024
#define RxBufPacketNum       15
          
/* declaration */
typedef signed char             int8;
typedef signed int              int16;
typedef signed long             int32;
typedef signed long long        int64;
typedef unsigned char           uint8;
typedef unsigned int            uint16;
typedef unsigned long           uint32;
typedef unsigned long long      uint64;


/* definiton */
enum{ ERROR = 0, SUCCESS = !ERROR };
enum{ RESET = 0, SET = !RESET };
enum{ DISABLE = 0, ENABLE = !DISABLE };

#pragma pack(1)

/* ͨ����ѡ�� */
enum {
  Null = 0,
  Ch1  = 0x1,
  Ch2  = 0x2,
  Ch3  = 0x3,
  Ch4  = 0x4,
  Ch5  = 0x5,
  Ch6  = 0x6,
  Ch7  = 0x7,
  Ch8  = 0x8,
  Ch9  = 0x9,
  Ch10 = 0xA,
  Ch11 = 0xB,
  Ch12 = 0xC
  
};

/* ͨ������ */
enum {
  CURRENT  = 0x0,
  FREQTYPE = 0x1,
  REED     = 0x2  
};


/* beep or not */
enum{
  BEEP1   = 0x1,
  BEEP2   = 0x2,
  BEEP3   = 0x3,
  BEEP4   = 0x4
};
/* switch or not*/ 
enum{
  SWITCH1   = 0x1,
  SWITCH2   = 0x2,
  SWITCH3   = 0x3,
  SWITCH4   = 0x4
};

/* beep or notbeep */
typedef enum{
  SWITCH = 0, BEEP = !SWITCH
}BeorSw;

/* warning or no warning */
typedef enum{
  NOTWRNING = 0, WRNING = !NOTWRNING
}_WrorNowr;

/*  ͨ������A �ṹ */
typedef struct {
  uint16 SWEnaleDsable : 3;
  uint16 BeepOrNotBeep : 3;
  uint16 SetRestByAuto : 1;
  uint16 OnOffByHand   : 1;
  uint16 CurrFreqSelt  : 2;
  uint16 ChannelSel    : 4;
  uint16 NotDef        : 2;  
} __BitOptionA;


/*  Ӧ������ ״̬ λ�� */
typedef struct {
  uint16 ReChannelNum   : 4;
  uint16 ReChannelType  : 2;
  uint16 ReOnOffStatus  : 3;
  uint16 NotDef         : 7;  
  
} __BitOptRe;


/* �������ݱ���~״̬λ�� */
typedef struct {
  uint16 AnBdcastDetec : 2;  // Ӧ����λ
  uint16 OnOffSta      : 4;  // ���������״̬
  uint16 Error10       : 1;  // ����10
  uint16 Error9        : 1;  // ����9
  uint16 Error8        : 1;  // ����8
  uint16 Error7        : 1;  // ����7
  uint16 Error6        : 1;  // ����6
  uint16 Error5        : 1;  // ����5
  uint16 Error4        : 1;  // ����4
  uint16 Error3        : 1;  // ����3
  uint16 Error2        : 1;  // ����2
  uint16 Error1        : 1;  // ����1  
} __ConvDaSta;

// CAN ID 
typedef union{
  struct{
    uint8 CANID : 6; 
    uint8 DevID : 2;
  }_Bit;
  uint8 _Byte;
}_DevID_BOb;


/* ����������*/
typedef union{
  struct{
    uint8 SensorType : 6;
    uint8 SingalType : 2;
  }_Bit;
  uint8 _Byte;
}_SenTy_Bob;

// ������Ϣ
typedef struct{
  uint8 AssoID;
  uint8 AssoType;
}_Asso;// ����
/* ������������Ϣ���籣��ṹ�� 15Bytes */
typedef struct{
  struct{
    uint8 Type    : 6;
    uint8 TypeA   : 2;
  }_Type;
  struct{
    uint8 ID;
  }_ID;
  union{
    struct{
      _SenTy_Bob SenTy;     // sensor type
      uint16 HighAlog; //highest Analog value
      uint16 HighDigtal;  //highest Digital value
    }_StepA;
    struct{
      uint16 LowAlog;  //lowest Analog value
      uint16 LowDigtal;   //lowest Digital value
      uint16 HighWarr; //highest warrning
    }_StepB;
    struct{
      uint16 LowWarr;  //lowest warrning
      uint16 dmenxian; //�ϵ�����
      uint16 qmenxian; //��������
    } _StepC;
  }_Step;
}_SensorInfo;

typedef struct{
  uint8 Enable;
  _SensorInfo Info[3];
}_Info;

// ִ��������
typedef struct{
  uint8 DevSeted;
  uint8 ExeType;
}_ExecuteInfo;  //execute info.
// ������Ϣ
typedef struct{
  uint8 Quantity;           // ��������
  _Asso AssoInfo[50];
}_ExecuteAsso;  // execute associate info.

/* ���������������� */
typedef struct{
  //_SenTy_Bob SenTy;     // sensor type
  uint16 CalcResult;
  uint8  Warning;
}_SensorResult;

typedef union{
    struct{
      uint8 UpValueWar    : 1;
      uint8 DownValueWar  : 1;
      uint8 PowerWar      : 1;
      uint8 SensorWar     : 1;
      uint8 NoconfigWar   : 1;
      uint8 y             : 1;
      uint8 z             : 1;
      uint8 CanWar        : 1;

    }_Bit;
    uint8 Byte;
}_Warn;


typedef union{
    struct{
      uint8 DeviceERR     : 1;
      uint8 AssoDevAlm    : 1;
      uint8 AssoNotCofig  : 1;
      uint8 ExeNotCofig   : 1;
      uint8 x             : 1;
      uint8 y             : 1;
      uint8 z             : 1;
      uint8 CanWar        : 1;

    }_Bit;
    uint8 Byte;
}_WarnCtrl;

//control status
typedef union{
  struct{
  uint8 x             : 2;
  uint8 AssoEn        : 1;
  uint8 Force         : 1;
  uint8 CtrlMode      : 2;
  uint8 Fedback       : 1;
  uint8 OtptVal       : 1; 
  }_Bit;
  uint8 Byte;
}_OtptSign;


/* CAN FREAM
** bytes
** 
*/ 

typedef struct{
  struct{
    uint8 Type    : 6;
    uint8 TypeA   : 2;
  }_Type;
  struct{
    uint8 ID;
  }_ID;
  // personnal location protocol
  union{
    uint8 OtherMessage[6];           // �����豸��Ϣ
    /* environmental monitoring*/
    struct{
      uint8 ExeType;
    }_ExecuteInfo;  //execute info.
    struct{
      _SenTy_Bob SenTy;     // sensor type
      uint16 HighAlog; //highest Analog value
      uint16 HighDigtal;  //highest  Digital value
    }_ConfigSenA;
    struct{
      uint16 LowAlog;  //lowest Analog value
      uint16 LowDigtal;   //lowest Digital value
      uint16 HighWarr; //highest warrning
    }_ConfigSenB;
    struct{
      uint16 LowWarr;  //lowest warrning
      uint16 dmenxian;
      uint16 qmenxian;
    }_ConfigSenC;
    struct{
      uint16 StationID;
    }_AcStation;  //access station
    struct{
      uint16 StationID;
      uint16 DevID;
    }_AcDev;     // access device
    struct{
      uint8 AssoID;
      uint8 AssoType;
    }_AssoInfo;  // logitic
    struct{ 
      _SenTy_Bob SenTy;     // sensor type
      uint16 Value;        
      _Warn Warning;
    }_CoDa; // Collect Data  
    struct{
      uint8 OValue;
      uint8 Force;
    }_OpDev; // Output device 
    struct{
      _OtptSign OtptSign;  
      _WarnCtrl  Warning;
      uint8     AssoID;
      uint8     AssoType;
    }_OpDevFe;          // output device Feedback
  }_DataForWho;
}_Can_Package;


/* ���� */
typedef struct{
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
  _Can_Package Data;
}_RDataTmp;            // ����ID+���ݻ���


/* ͨ�ñ���
** 114 bytes
** 
*/

typedef struct{
  // ��ͷ���� 12 bytes
  uint16 Head;                 // ͷ
  
  uint8  Length;// ����  because little-end
  uint8  Type;  // ����  
                                       
  uint16 TargetAdd;            // Ŀ�ĵ�ַ
  uint16 SourceAdd;            // Դ��ַ  
  uint16 Stamp;                // ʱ���

  uint8  Hop;                  // ����   idem.
  uint8  AddChecking;          // ��У�� 
  // personnal location protocol
  union{
    uint8 OtherMessage[102];           // �����豸��Ϣ
    // �������ݽṹ�� 12bytes
    struct{
      uint16 ChannelData[11];    // 0-11 ͨ������
      union{
         __ConvDaSta BitConvDaSta;  // �������ݱ���״̬ λ��
         uint16 ConvDaSta;          // �������ݱ���״̬ �ֶ�ȡ
      }__BitOrUint16;
    } __ConvData;
    // Ӧ��������Ϣ�ṹ��
    struct{
      union {
        __BitOptRe BitOptionReply; // Ӧ������״̬ λ��
        uint16 OptionReply;        // Ӧ������״̬ �ֶ�
      }__BitOrUint16;
    } __AnswerConfig;
    // ������Ϣ�ṹ��
    struct{
      union{
        __BitOptionA BitOptionA;     // ͨ������A�ṹλ��
        uint16 OptionA;              // ͨ������A �ֶ�ȡ
      }__BitoOpti;
      uint16 LowerOfCollect;       // �ɼ���Χ����
      uint16 UpperOfCollect;       // �ɼ���Χ����
      uint16 LowerOfPhyValue;      // ����ֵ����
      uint16 UpperOfPhyValue;      // ����ֵ����
      uint16 LowerOfAlarmPoint;    // ����������
      uint16 UpperOfAlarmPoint;    // ����������
      uint16 LowerOfCtrl;          // ���޿��Ƶ�
      uint16 UpperOfCtrl;          // ���޿��Ƶ�
      uint16 HystValuOfCtrlPoint;   // ���Ƶ��ͻ�ֵ Hysteresis value        
      
      uint8  PackageCont;          // ������Ϣ���� ����С�˴洢package�� ctrl �轻��λ��
      uint8  CtrlMode;              // ���Ʒ�ʽ
      
    }__ConfigInfo;
    // �㲥����ʶ����
    uint16 DetecSign;               // ����ʶ
    struct{
      uint16 SrcAdr;               // ����·������Ľڵ�
      uint16 DestAdr;              // ·��Ŀ��ڵ�
      uint16 RelayAdr;             // �ϴ�ת������Ϣ�Ľڵ� ��ʼֵ=DestAdr
      uint8  Hop;                  // �ӷ���ڵ㵽���������ڵ������ ��ʼֵ=0 ÿת��1��+1 
    }__ts_RREQ;
    struct{
      uint16 SrcAdr;               // �ظ�·������Ľڵ�
      uint16 DestAdr;              // ����·������Ľڵ�
      uint16 RelayAdr;             // �ϴ�ת������Ϣ�Ľڵ� ��ʼֵ=DestAdr
      uint8  Hop;                  // �ӷ������������������ڵ����������ʼֵ=0 ÿת��1��+1
    }__ts_RREP;
    struct{
      uint8  Number;                // ���ڲ��ɴ�ڵ���
      uint8  Route;                 // ���ɴ�ڵ��·��
      uint16 Adr[50];              // ���ɴ�ڵ�
    }__ts_RERR;
    /* environmental monitoring*/
    struct{
      uint8 SonType;
      uint8 DevID;
      uint8 ExeType;
    }_ExecuteInfo;  //execute info.
    struct{
      uint8 SonType;         // package son tpye
      uint8 DevID;    //device ID
      _SenTy_Bob SenTy;     // sensor type
      uint8 IO;        // Input or Output
      uint16 HighAlog; //highest Analog value
      uint16 HighDigtal;  //highest  Digital value
      uint16 LowAlog;  //lowest Analog value
      uint16 LowDigtal;   //lowest Digital value
      uint16 HighWarr; //highest warrning
      uint16 LowWarr;  //lowest warrning
      uint16 dmenxian;
      uint16 qmenxian;
    }_ConfigInfo;  // config info.
    struct{
      uint8 SonType;         // package son tpye
      uint16 StationID;
    }_AcStation;  //access station
    struct{
      uint8 SonType;         // package son tpye
      uint16 StationID;
      uint8 DevID;
    }_AcDev;     // access device
    struct{
      uint8 SonType;         // package son tpye
      uint8 ExeDevID;        // execute device ID
      uint8 Quantity;           // ��������
      _Asso AssoInfo[50];
    }_Logitic;  // logitic
    struct{
      uint8 SonType;         // package son tpye
      uint8 DevID;    //device ID 
      _SenTy_Bob SenTy;     // sensor type
      uint16 Value;        
      uint8 Warning;
    }_CoDa; // Collect Data  
    struct{
      uint8 SonType;         // package son tpye
      uint8 DevID;    //device ID
      uint8 OValue;        // output value
      uint8 Force; 
    }_OpDev; // Output device down
    struct{
      uint8 SonType;         // package son tpye
      uint8 DevID;    //device ID
      uint8 OValue;       
      uint8 Feedback;    
      uint8 CoMode;       // control mode
      uint8 Force;
      uint8 Warning;
      uint8 AssoValid;  // ������Ϣ��Ч
      uint8 AssoDevice; 
      uint8 AssoType;  
    }_OpDevFe;          // output device Feedback
  }_DataForWho;
}_Package;


#endif 
