/*
  File: protocol.h 
  Author: lhw
  Data: 2014/12/11
  Desc: protocol for all device */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H
//#include "global_data.h"
#define _yr_					       0x7279		//ID 通讯用
#define TRUE                 1   /* page 207 K+R 2nd Edition */
#define FALSE                0
#define PackHeadSize				 12					//通信包头大小
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

/* 通道号选择 */
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

/* 通道类型 */
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

/*  通道设置A 结构 */
typedef struct {
  uint16 SWEnaleDsable : 3;
  uint16 BeepOrNotBeep : 3;
  uint16 SetRestByAuto : 1;
  uint16 OnOffByHand   : 1;
  uint16 CurrFreqSelt  : 2;
  uint16 ChannelSel    : 4;
  uint16 NotDef        : 2;  
} __BitOptionA;


/*  应答配置 状态 位段 */
typedef struct {
  uint16 ReChannelNum   : 4;
  uint16 ReChannelType  : 2;
  uint16 ReOnOffStatus  : 3;
  uint16 NotDef         : 7;  
  
} __BitOptRe;


/* 常规数据报文~状态位段 */
typedef struct {
  uint16 AnBdcastDetec : 2;  // 应答检测位
  uint16 OnOffSta      : 4;  // 开关量输出状态
  uint16 Error10       : 1;  // 故障10
  uint16 Error9        : 1;  // 故障9
  uint16 Error8        : 1;  // 故障8
  uint16 Error7        : 1;  // 故障7
  uint16 Error6        : 1;  // 故障6
  uint16 Error5        : 1;  // 故障5
  uint16 Error4        : 1;  // 故障4
  uint16 Error3        : 1;  // 故障3
  uint16 Error2        : 1;  // 故障2
  uint16 Error1        : 1;  // 故障1  
} __ConvDaSta;

// CAN ID 
typedef union{
  struct{
    uint8 CANID : 6; 
    uint8 DevID : 2;
  }_Bit;
  uint8 _Byte;
}_DevID_BOb;


/* 传感器类型*/
typedef union{
  struct{
    uint8 SensorType : 6;
    uint8 SingalType : 2;
  }_Bit;
  uint8 _Byte;
}_SenTy_Bob;

// 关联信息
typedef struct{
  uint8 AssoID;
  uint8 AssoType;
}_Asso;// 关联
/* 传感器配置信息掉电保存结构体 15Bytes */
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
      uint16 dmenxian; //断电门限
      uint16 qmenxian; //启动门限
    } _StepC;
  }_Step;
}_SensorInfo;

typedef struct{
  uint8 Enable;
  _SensorInfo Info[3];
}_Info;

// 执行器配置
typedef struct{
  uint8 DevSeted;
  uint8 ExeType;
}_ExecuteInfo;  //execute info.
// 关联信息
typedef struct{
  uint8 Quantity;           // 关联数量
  _Asso AssoInfo[50];
}_ExecuteAsso;  // execute associate info.

/* 传感器计算结果保存 */
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
    uint8 OtherMessage[6];           // 其他设备信息
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


/* 数据 */
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
}_RDataTmp;            // 接收ID+数据缓存


/* 通用报文
** 114 bytes
** 
*/

typedef struct{
  // 包头部分 12 bytes
  uint16 Head;                 // 头
  
  uint8  Length;// 长度  because little-end
  uint8  Type;  // 类型  
                                       
  uint16 TargetAdd;            // 目的地址
  uint16 SourceAdd;            // 源地址  
  uint16 Stamp;                // 时间戳

  uint8  Hop;                  // 跳数   idem.
  uint8  AddChecking;          // 和校验 
  // personnal location protocol
  union{
    uint8 OtherMessage[102];           // 其他设备信息
    // 常规数据结构体 12bytes
    struct{
      uint16 ChannelData[11];    // 0-11 通道数据
      union{
         __ConvDaSta BitConvDaSta;  // 常规数据报文状态 位段
         uint16 ConvDaSta;          // 常规数据报文状态 字读取
      }__BitOrUint16;
    } __ConvData;
    // 应答配置信息结构体
    struct{
      union {
        __BitOptRe BitOptionReply; // 应答配置状态 位段
        uint16 OptionReply;        // 应答配置状态 字读
      }__BitOrUint16;
    } __AnswerConfig;
    // 配置信息结构体
    struct{
      union{
        __BitOptionA BitOptionA;     // 通道设置A结构位段
        uint16 OptionA;              // 通道设置A 字读取
      }__BitoOpti;
      uint16 LowerOfCollect;       // 采集范围下限
      uint16 UpperOfCollect;       // 采集范围上限
      uint16 LowerOfPhyValue;      // 物理值下限
      uint16 UpperOfPhyValue;      // 物理值上限
      uint16 LowerOfAlarmPoint;    // 报警点下限
      uint16 UpperOfAlarmPoint;    // 报警点上限
      uint16 LowerOfCtrl;          // 下限控制点
      uint16 UpperOfCtrl;          // 上限控制点
      uint16 HystValuOfCtrlPoint;   // 控制点滞回值 Hysteresis value        
      
      uint8  PackageCont;          // 配置信息条数 由于小端存储package和 ctrl 需交换位置
      uint8  CtrlMode;              // 控制方式
      
    }__ConfigInfo;
    // 广播检测标识定义
    uint16 DetecSign;               // 检测标识
    struct{
      uint16 SrcAdr;               // 发起路由请求的节点
      uint16 DestAdr;              // 路由目标节点
      uint16 RelayAdr;             // 上次转发此消息的节点 初始值=DestAdr
      uint8  Hop;                  // 从发起节点到处理该请求节点的跳数 初始值=0 每转发1次+1 
    }__ts_RREQ;
    struct{
      uint16 SrcAdr;               // 回复路由请求的节点
      uint16 DestAdr;              // 发出路由请求的节点
      uint16 RelayAdr;             // 上次转发此消息的节点 初始值=DestAdr
      uint8  Hop;                  // 从发起及诶单到处理该请求节点的跳数，初始值=0 每转发1次+1
    }__ts_RREP;
    struct{
      uint8  Number;                // 包内不可达节点数
      uint8  Route;                 // 不可达节点的路由
      uint16 Adr[50];              // 不可达节点
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
      uint8 Quantity;           // 关联数量
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
      uint8 AssoValid;  // 关联信息有效
      uint8 AssoDevice; 
      uint8 AssoType;  
    }_OpDevFe;          // output device Feedback
  }_DataForWho;
}_Package;


#endif 
