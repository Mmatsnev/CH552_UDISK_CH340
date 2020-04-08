#include "CH554.H"
#include "DEBUG.H"
#include <stdio.h>
#include <string.h>
#pragma  NOAREGS
#define   USE_MOS     0
#define   RTS_HIGH   (P1 |= 0x10) //RTS HIGH
#define   DTR_HIGH   (P1 |= 0x20) //DTR HIGH
#define   RTS_LOW    (P1 &= ~0x10) //RTS Low
#define   DTR_LOW    (P1 &= ~0x20) //DTR Low
#define   FLAG_HIGH		(P3 |= 0x04)
#define   FLAG_LOW		(P3 &= ~0x04)

#define   OPEN_CH340   1
#define   OPEN_UDISK   1
#define   USE_UART_UDISK   0

#define   DISK_SEC_LEN  0x00001000
#define   DISK_SEC_NUM  0x00000800
#define   DISK_SEC_LAST  DISK_SEC_NUM- 1


UINT8X  Ep0Buffer[DEFAULT_ENDP0_SIZE] _at_ 0x0000;
#if    OPEN_CH340
UINT8X  Ep2Buffer[2*MAX_PACKET_SIZE] _at_ 0x0008;
UINT8X  Ep1Buffer[MAX_PACKET_SIZE] _at_ 0x00a0;
#endif
#if    OPEN_UDISK
UINT8X  Ep3Buffer[2*MAX_PACKET_SIZE] _at_ 0x00f0;
#endif

#if  (OPEN_CH340 || USE_UART_UDISK)
UINT8X LineCoding[7]={0x00,0xe1,0x00,0x00,0x00,0x00,0x08};   //初始化波特率为57600，1停止位，无校验，8数据位。
#define  UART_REV_LEN   128                                        //串口1接收缓冲区大小
UINT8I Receive_Uart_Buf[UART_REV_LEN];                              //串口1接收缓冲区
UINT32 NowUart1Buad;
volatile UINT8I Uart_Input_Point = 0;                               //循环缓冲区写入指针，总线复位需要初始化为0
volatile UINT8I Uart_Output_Point = 0;                              //循环缓冲区取出指针，总线复位需要初始化为0
volatile UINT8I UartByteCount = 0;                                  //当前缓冲区剩余待取字节数
void clearUart1(){
    //if(UartByteCount > 0){
        UartByteCount = 0;
        Uart_Output_Point = 0;
        Uart_Input_Point = 0;
				U1RI = 0;
		//}
}
#endif

UINT16  SetupLen;
UINT8   SetupReq,UsbConfig = 0;
UINT8   dataNum = 0;
PUINT8  pDescr;
#define UsbSetupBuf     ((PUSB_SETUP_REQ)Ep0Buffer)

#if (!OPEN_CH340 && OPEN_UDISK)
UINT8C  DevDesc[] = {
    0x12, 0x01, 
    0x10, 0x01,
    0x00, 0x00,
    0x00, 0x08,
    0x44, 0x33,
    0x33, 0x35,
    0x00, 0x01,
    0x01, 0x02,
    0x00, 0x01
    };
#else
UINT8C  DevDesc[] = {
    0x12, 0x01, 
    0x10, 0x01,
    0x00, 0x00,
    0x00, 0x08,
    0x86, 0x1a,
		0x01, 0x95,
    //0x23, 0x75,
    0x63, 0x01,
    0x01, 0x02,
    0x00, 0x01
    };
#endif

#if (!OPEN_CH340 && OPEN_UDISK)
UINT8C CfgDesc[] = { 
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0xa0, 0x32,
    0x09, 0x04, 0x00, 0x00, 0x02, 0x08, 0x06, 0x50, 0x00,                     
    0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00
    };
#elif (OPEN_CH340 && !OPEN_UDISK)
UINT8C CfgDesc[]={
    0x09, 0x02, 0x27, 0x00, 0x01, 0x01, 0x00, 0x80, 0xf0,              //配置描述符，接口描述符,端点描述符
    0x09, 0x04, 0x00, 0x00, 0x03, 0xff, 0x01, 0x02, 0x00,           
    0x07, 0x05, 0x82, 0x02, 0x20, 0x00, 0x00,                        //批量上传端点
    0x07, 0x05, 0x02, 0x02, 0x20, 0x00, 0x00,                        //批量下传端点      
    0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x01};                       //中断上传端
#elif (OPEN_CH340 && OPEN_UDISK)
UINT8C  CfgDesc[] = { 
    0x09, 0x02, 0x3E, 0x00, 0x02, 0x01, 0x00, 0x80, 0xf0,
	  //ch340
    0x09, 0x04, 0x00, 0x00, 0x03, 0xff, 0x01, 0x02, 0x00,
    0x07, 0x05, 0x82, 0x02, 0x20, 0x00, 0x00,
    0x07, 0x05, 0x02, 0x02, 0x20, 0x00, 0x00,
    0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0a,
    //udisk
    0x09, 0x04, 0x01, 0x00, 0x02, 0x08, 0x06, 0x50, 0x00,                     
    0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x0a
    };
#else

#endif

#if  OPEN_UDISK
volatile UINT8 UDISKPORT_IN = 0 ;
volatile UINT8 UDISKPORT_OUT = 0 ;
UINT8 UDISKDOWN = 0x00;
UINT8C	DBINQUITY[]={
    0x00,             //Peripheral Device Type
    0x80, 			//
    0x02 ,			//ISO/ECMA
    0x02 ,			//
    0x1f ,			//Additional Length

    00 ,			//Reserved
    00 ,			//Reserved
    00 ,				//Reserved

    'w' ,			//Vendor Information
    'c' ,			//
    'h' ,			//
    '.' ,			//
    'c' ,			//
    'n' ,			//
    ' ' ,			//
    ' ' ,			//


    0xc7,			//Product Identification
    0xdf, 			//
    0xba,			//
    0xe3,			//
    0xb5,			//
    0xe7,			//
    0xd7,			//
    0xd3,			//
    0x55,			//
    0xc5,			//
    0xcc,			//
    0xb7,			//
    0xbd,			//
    0xb0,			//
    0xb8,			//
    0x00,          //

    '1' ,			//Product Revision Level
    '.' ,			//
    '1' ,			//
    '0'  			//
};

UINT8C DBFORMATCAP[]={0x00,0x00,0x00,0x08,(DISK_SEC_NUM>>24)&0xFF, (DISK_SEC_NUM>>16)&0xFF, (DISK_SEC_NUM>>8)&0xFF, DISK_SEC_NUM&0xFF, 0x00, 0x00, 0x02, 0x00}; //可格式化容量 0x0800 * 0x1000
UINT8C DBCAPACITY[]={(DISK_SEC_LAST>>24)&0xFF, (DISK_SEC_LAST>>16)&0xFF, (DISK_SEC_LAST>>8)&0xFF, DISK_SEC_LAST&0xFF ,(DISK_SEC_LEN>>24)&0xFF, (DISK_SEC_LEN>>16)&0xFF, (DISK_SEC_LEN>>8)&0xFF, DISK_SEC_LEN&0xFF};
UINT8C modesense3F[]={0x0b, 0x00, 0x00, 0x08, (DISK_SEC_NUM>>24)&0xFF, (DISK_SEC_NUM>>16)&0xFF, (DISK_SEC_NUM>>8)&0xFF, DISK_SEC_NUM&0xFF, 0x00, 0x00, 0x02, 0x00 }; 
UINT8C mode5sense3F[]={0x00, 0x06, 0x00, 0x00, 0x08, 0x00, 0x00, 0x08,(DISK_SEC_NUM>>24)&0xFF, (DISK_SEC_NUM>>16)&0xFF, (DISK_SEC_NUM>>8)&0xFF, DISK_SEC_NUM&0xFF, 00, 00, 02, 00 };  //物理扇区数
/*	
UINT8C DBFORMATCAP[]={0x00,0x00,0x00,0x08,0x00,0x00,0x40,0x00,0x00,0x00,0x02,0x00}; //可格式化容量
UINT8C DBCAPACITY[]={0x00,0x00,0x40,0x00,0x00,0x00,0x02,0x00};
UINT8C modesense3F[]={0x0b, 0x00, 0x00, 0x08, 0x00,0x00,0x40,0x00,0x00, 0x00, 0x02, 0x00 };   //写保护(0x80换成0x00可以去除写保护)
UINT8C mode5sense3F[]={0x00, 0x06, 0x00, 0x00, 0x08, 0x00, 0x00, 0x08,0x00,0x00,0x40,0x00, 00, 00, 02, 00 };  //物理扇区数*/

#if !USE_UART_UDISK
/*
UINT8C DBR[512]={
          
	       0xeb,0xfe,0x90,0x4d,0x53,0x44,0x4f,0x53, 0x35,0x2e,0x30,0x00,0x02,0x08,0x20,0x00, 
         //0x02,0x00,0x00,0x00,0x00,0xf0,0x00,0x00, 0x3f,0x00,0xff,0x00,0x00,0x00,0x00,0x00,
          0x02,0x00,0x00,0x00,0x20,0xf0,0x00,0x00, 0x3f,0x00,0xff,0x00,0x00,0x00,0x00,0x00,
				 //0x00,0xe0,0xec,0x00,0x39,0x3b,0x00,0x00, 0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,
	       0x00,0x00,0x00,0x00,0x08,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,
				 0x01,0x00,0x06,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x80,0x00,0x29,0x00,0x00,0x8a,0x49,0x4e, 0x4f,0x20,0x4e,0x41,0x4d,0x45,0x20,0x20,
				 0x20,0x20,0x46,0x41,0x54,0x33,0x32,0x20, 0x20,0x20,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x55,0xaa,
};
UINT8C FAT[512]={
	       0xF0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0x0F,0x00,0x00,0x00,0x00,
         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	
};*/
#endif
UINT8 MYBUF[512] = {0x00};

#define FORMAT_UNIT 	0x04
#define INQUIRY       0x12
#define FORMATCAP 		0x23						  
#define MODE_SELECT 	0x15
#define MODE_SENSE5 	0x5A
#define MODE_SENSE 		0x1A
#define PER_RES_IN 		0x5E
#define PER_RES_OUT 	0x5F
#define PRE_OR_MED 		0x1E
#define READ          0x28
#define READ_CAPACITY 0x25
#define RELEASE       0x17
#define REQUEST_SENSE 0x03
#define RESERVE       0x16
#define STA_STO_UNIT 	0x1B
#define SYN_CACHE 		0x35
#define TEST_UNIT 		0x00
#define VERIFY        0x2F
#define WRITE         0x2A
#define WRITE_BUFFER 	0x3B

UINT8C  MAX_LUN[] = {0};
typedef union _CBWCB{
    unsigned char buf1[16];
}CBWCB;
typedef  union _MASS_PARA {
    unsigned char buf[64];
    struct  _SENSE{
        unsigned char ErrorCode;
        unsigned char Reserved1;
        unsigned char SenseKey;
        unsigned char Information[4];
        unsigned char AddSenseLength;
        unsigned char Reserved2[4];
        unsigned char AddSenseCode;
        unsigned char AddSenseCodeQua;
        unsigned char Reserved3[4];
    }Sense;
    struct  _CBW{
        unsigned char dCBWsig[4];
        unsigned char dCBWTag[4];
        unsigned long dCBWDatL;
        unsigned char bmCBWFlags;
        unsigned char bCBWLUN;
        unsigned char bCBWCBLength;
        CBWCB        cbwcb;
    }cbw;
    struct _CSW{
        unsigned char buf2[13];
    }csw;
}MASS_PARA;

union {
    unsigned long mDataLength;             //数据长度
    unsigned char mdataLen[4];             //
}UFI_Length;
unsigned char mdCBWTag[4];						     //dCBWTag
MASS_PARA  MassPara;

volatile UINT8I UpPoint3Status  = 0;       //端点3的上报状态,为1时表示，buf已经被填充，可以上传了。
volatile UINT16 MYBUFLEN = 0;

//UINT8 WriteBuf[64];
volatile UINT32I SENDDATALEN = 0;          //udisk发送端应该发送的数据长度。
#if  USE_UART_UDISK

#endif
volatile UINT8I UdiskOCCUart = 0;          //udisk读取flash的标志，0为不读，1为读
volatile UINT8I Point3IN  = 0;             //端点3的IN包标志，用于udisk
volatile UINT8I Point3OUT = 0;             //端点3的OUT包标志，用于udisk

bit    UDISKBULKUP=0;									     //数据上传
bit    UDISKCSW=0;										     //CSW上传标志
bit    UDISKBULKDOWN = 0;						       //数据下传	

UINT32 Locate_Addr;
unsigned char	BcswStatus;                  //CSW状态
unsigned char mSenseKey;
unsigned char mASC;
unsigned char *pBuf;
unsigned long SecNum;                              //当前操作的扇区号

void UDISKUpCsw()
{
    unsigned char i;											 //如果数据为0
    pBuf=&MassPara.buf[0];
    UDISKCSW=0;														 //上传CSW
    UDISKBULKUP=0;                         //取消数据上传
    MassPara.buf[0]=0x55;                  //dCSWSignature
    MassPara.buf[1]=0x53;
    MassPara.buf[2]=0x42;
    MassPara.buf[3]=0x53;
    MassPara.buf[4]=mdCBWTag[0];
    MassPara.buf[5]=mdCBWTag[1];
    MassPara.buf[6]=mdCBWTag[2];
    MassPara.buf[7]=mdCBWTag[3];
    MassPara.buf[8]=UFI_Length.mdataLen[3];
    MassPara.buf[9]=UFI_Length.mdataLen[2];
    MassPara.buf[10]=UFI_Length.mdataLen[1];
    MassPara.buf[11]=UFI_Length.mdataLen[0];
    MassPara.buf[12]=BcswStatus;
    for(i = 0;i<13;i++)
    {
        Ep3Buffer[MAX_PACKET_SIZE+i] = *pBuf;
        pBuf++;
    }
    UEP3_T_LEN = 13;
    UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;          // 允许上传
}
/*
void SetBaud115200(){
		UINT32 x;
		UINT8 x2;
    //if(NowUart1Buad != 512000){
        NowUart1Buad = 512000;
        IE_UART1 = 0;
				//x = 10 * FREQ_SYS / UART1_BUAD / 16;                                       //如果更改主频，注意x的值不要溢出                            
				x = 10 * FREQ_SYS / 512000 / 16;	
				x2 = x % 10;
				x /= 10;
				if ( x2 >= 5 ) x ++;                                                       //四舍五入
				SBAUD1 =  0 - x;
        IE_UART1 = 1;
		//}

}*/
void setBaud(UINT32 baud){
		UINT32 x;
		UINT8 x2;
		printf("baud=0x%lX\r\n",baud);
    //if(NowUart1Buad != baud){
        NowUart1Buad = baud;
        IE_UART1 = 0;
				x = 10 * FREQ_SYS / baud / 16;	
				x2 = x % 10;
				x /= 10;
				if ( x2 >= 5 ) x ++;                                                       //四舍五入
				SBAUD1 =  0 - x;
        IE_UART1 = 1;
		//}
}
UINT16 tempSendLen = 0;
UINT8 needPause = 0;
UINT8 Pausefirst = 0;
UINT8 udiskIsOk = 0;
void UDISKbulkUpData(){											           //调用端点1上传数据
		unsigned char len,i;
#if USE_UART_UDISK
    if(UpPoint3Status == 1){
        if(UFI_Length.mDataLength>MAX_PACKET_SIZE){
            len=MAX_PACKET_SIZE;
            UFI_Length.mDataLength-=MAX_PACKET_SIZE;
        }
        else{
            len= (unsigned char) UFI_Length.mDataLength;
            UFI_Length.mDataLength=0;
            UDISKBULKUP=0;
        }
        for(i = 0;i<len;i++)
        {
            Ep3Buffer[MAX_PACKET_SIZE+i] = *pBuf;
            pBuf++;
        }
        UEP3_T_LEN = len;

				
        UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;        // 允许上传
				if(SENDDATALEN > 0){
					SENDDATALEN -= len;
				}
				if((SENDDATALEN % 0x200) == 0 && (SENDDATALEN > 0)){
					UdiskOCCUart = 1;
				}
        if(UDISKBULKUP == 0){
					UpPoint3Status = 0;
				}
    }
#else
		if(UFI_Length.mDataLength>MAX_PACKET_SIZE){
			len=MAX_PACKET_SIZE;
			UFI_Length.mDataLength-=MAX_PACKET_SIZE;
		}
		else {
			len= (unsigned char) UFI_Length.mDataLength;
			UFI_Length.mDataLength=0;
			UDISKBULKUP=0;
		}		
		{
			for(i = 0;i<len;i++)
			{
				Ep3Buffer[MAX_PACKET_SIZE+i] = *pBuf;
				pBuf++;
			}
		}
		UEP3_T_LEN = len;
		UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;        // 允许上传
		if(SENDDATALEN >= 0x200){
			tempSendLen +=(UINT16)len;
			if((tempSendLen == 0x200) && (UDISKBULKUP != 0)){
				printf("======need pause=======\r\n");
				needPause = 1;
				Pausefirst = 1;
				//mDelaymS(200);
				tempSendLen = 0;
				SENDDATALEN -= 0x200;
			}else if((tempSendLen == 0x200) && (UDISKBULKUP == 0)){
				tempSendLen = 0;
				needPause = 0;
				SENDDATALEN -= 0x200;
				printf("======not need pause=======\r\n");
			}else{
			}
		}
#endif
}
UINT8 notDownOK = 0;

void UDISKBulkDownData(){
    UINT32 time = 800;
		unsigned char templen;
    unsigned char i;
	UINT8 sendData[64];
		templen = USB_RX_LEN;	
		
		for(i = 0; i != templen; i++){
			sendData[i] =Ep3Buffer[i];
		}
		//setBaud(512000);
		udiskIsOk = 0;
    /*for(i = 0; i < templen; i++){
			WriteBuf[i] = Ep3Buffer[i];
		}*/
		FLAG_LOW;
		/*
		CH554UART1SendByte(0x55);
    CH554UART1SendByte(0xBB);
    CH554UART1SendByte(0x01);
		CH554UART1SendByte((UINT8)((templen >> 24) & 0xFF));
    CH554UART1SendByte((UINT8)((templen >> 16) & 0xFF));
    CH554UART1SendByte((UINT8)((templen >> 8 ) & 0xFF));
		CH554UART1SendByte((UINT8)((templen >> 0 ) & 0xFF));
    CH554UART1SendByte((UINT8)((Locate_Addr >> 24) & 0xFF));
    CH554UART1SendByte((UINT8)((Locate_Addr >> 16) & 0xFF));
		CH554UART1SendByte((UINT8)((Locate_Addr >> 8 ) & 0xFF));
		CH554UART1SendByte((UINT8)((Locate_Addr >> 0 ) & 0xFF));*/
		//while(time--);
		time = 4800;
		while(time--);
		for(i = 0; i < templen; i++){
			CH554UART1SendByte(sendData[i]);
		}
#if  USE_UART_UDISK	
#endif
    Locate_Addr += templen;
		//printf("UFI_Length.mDataLength = 0x%lX \r\n",UFI_Length.mDataLength);
		UFI_Length.mDataLength-=templen;                              //全局数据长度减掉当前获得的长度

		if(UFI_Length.mDataLength==0){														//如果数据为0,则传送CSW
			UDISKBULKDOWN=0;
			UDISKUpCsw();                                          //上传CSW
    }
}

void sendCMD(){
    //UdiskOCCUart = 1;
		//SetBaud115200();
		setBaud(512000);
    //printf("udiskRead\r\n");
    //printf("UFI_Length.mDataLength=0x%lx\r\n",UFI_Length.mDataLength);
    //printf("SecNum=0x%lx\r\n",SecNum);
    clearUart1();
	  udiskIsOk = 0;
    SENDDATALEN = UFI_Length.mDataLength;
    CH554UART1SendByte((UINT8)0x55);
    CH554UART1SendByte((UINT8)0xBB);
    CH554UART1SendByte((UINT8)0x00);//readflash
		CH554UART1SendByte((UINT8)((UFI_Length.mDataLength >> 24) & 0xFF));
		CH554UART1SendByte((UINT8)((UFI_Length.mDataLength >> 16) & 0xFF));
		CH554UART1SendByte((UINT8)((UFI_Length.mDataLength >> 8 ) & 0xFF));
		CH554UART1SendByte((UINT8)((UFI_Length.mDataLength      ) & 0xFF));
		CH554UART1SendByte((UINT8)(((SecNum) >> 24) & 0xFF));
		CH554UART1SendByte((UINT8)(((SecNum) >> 16) & 0xFF));
		CH554UART1SendByte((UINT8)(((SecNum) >> 8 ) & 0xFF));
		CH554UART1SendByte((UINT8)(((SecNum)      ) & 0xFF));	
}

void  UFI_write(void ){//Erase
	
		UINT32 num=0x00;
    UINT32 tempLen;
		UINT32 temptime = 250000;
		udiskIsOk = 0;
		//SetBaud115200();
		setBaud(512000);
		clearUart1();
		FLAG_LOW;
    SecNum = ((UINT32)MassPara.cbw.cbwcb.buf1[2]<<24) | ((UINT32)MassPara.cbw.cbwcb.buf1[3]<<16) | ((UINT32)MassPara.cbw.cbwcb.buf1[4]<<8) | (UINT32)MassPara.cbw.cbwcb.buf1[5];
    Locate_Addr = SecNum * DISK_SEC_LEN;
    num = (UINT32)MassPara.cbw.cbwcb.buf1[8];
    tempLen = num * DISK_SEC_LEN;
		CH554UART1SendByte(0x55);
		CH554UART1SendByte(0xBB);
		CH554UART1SendByte(0xFF);
		CH554UART1SendByte((UINT8)((tempLen >> 24)&0xFF));
		CH554UART1SendByte((UINT8)((tempLen >> 16)&0xFF));
		CH554UART1SendByte((UINT8)((tempLen >> 8 )&0xFF));
		CH554UART1SendByte((UINT8)((tempLen      )&0xFF));
		CH554UART1SendByte((UINT8)((SecNum >> 24)&0xFF));
		CH554UART1SendByte((UINT8)((SecNum >> 16)&0xFF));
		CH554UART1SendByte((UINT8)((SecNum >> 8 )&0xFF));
		CH554UART1SendByte((UINT8)((SecNum      )&0xFF));
		//if(tempLen > 0x1000){
			while(temptime--);
		//}else{
		//	temptime = 5000;
		//	while(temptime--);
		//}
		/*
		printf("Erase flash\r\n");
		printf("Locate_Addr = 0x%lx\r\n",Locate_Addr);
		printf("tempLen = 0x%lx\r\n",tempLen);
		printf("SecNum= 0x%lX\r\n",SecNum);
		printf("num= 0x%lX\r\n",num);
		printf("MassPara.cbw.cbwcb.buf1[8]=0x%bX\r\n",MassPara.cbw.cbwcb.buf1[8]);
		printf("MassPara.cbw.cbwcb.buf1[7]=0x%bX\r\n",MassPara.cbw.cbwcb.buf1[7]);*/
		BcswStatus=0;
		mSenseKey=0;
		mASC=0;
}
#if  USE_UART_UDISK
#endif
UINT8 beginRead = 0;
void uDiskRead(){
		beginRead = 1;
		FLAG_LOW;
		needPause = 1;
	udiskIsOk = 0;
    UFI_Length.mDataLength=(((UINT32)MassPara.cbw.cbwcb.buf1[7]<<8) | (UINT32)MassPara.cbw.cbwcb.buf1[8])*DISK_SEC_LEN;  //发送长度
    SecNum = ((UINT32)MassPara.cbw.cbwcb.buf1[2]<<24) | ((UINT32)MassPara.cbw.cbwcb.buf1[3]<<16) | ((UINT32)MassPara.cbw.cbwcb.buf1[4]<<8) | (UINT32)MassPara.cbw.cbwcb.buf1[5];//起始扇区号
    SENDDATALEN = UFI_Length.mDataLength;
    sendCMD();
    //UpPoint3Status = 0;
#if  USE_UART_UDISK
#else
	/*
    if(SecNum==0 || SecNum==6)
    {
        pBuf = DBR;
    }else if(SecNum==0x20 || SecNum==0x3b59){
        pBuf = FAT;
    }else{
        pBuf = MYBUF;
    }*/
    BcswStatus=0;
    mSenseKey=0;
    mASC=0;	
#endif
}

void  UFI_perOrMed(void ){				//允许移出磁盘
		BcswStatus=0;
		mSenseKey=0;
		mASC=0;
}

void  UFI_staStoUnit(void ){     //请求装载卸载设备
		UDISKBULKDOWN=0;
		UDISKBULKUP=0;
		BcswStatus=0;
		mSenseKey=0;
		mASC=0;
}

void  UFI_modeSense5(void ){

		if(MassPara.cbw.cbwcb.buf1[2]==0x3F){
			if ( UFI_Length.mDataLength > sizeof(mode5sense3F) ) UFI_Length.mDataLength = sizeof(mode5sense3F);
			pBuf=mode5sense3F;
			BcswStatus=0;
			mSenseKey=0;
			mASC=0;
		}
		else {
			UDISKBULKUP=0;
			mSenseKey=5;
			mASC=0x20;
			BcswStatus=1;
			UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
			BcswStatus= 1;
	  }
}

void UFI_Hunding(void ){		
    switch(MassPara.cbw.cbwcb.buf1[0]){
        case INQUIRY:                
            pBuf = DBINQUITY;					                                      //查询U盘信息
            if(UFI_Length.mDataLength>sizeof(DBINQUITY)) UFI_Length.mDataLength=sizeof(DBINQUITY);
            BcswStatus=0;
            mSenseKey=0;
            mASC=0;
            UpPoint3Status = 1;
            break;
        case FORMATCAP:                                                              //可格式化容量（模拟8G盘）
            pBuf = DBFORMATCAP;
            if(UFI_Length.mDataLength>sizeof(DBFORMATCAP)) UFI_Length.mDataLength=sizeof(DBFORMATCAP);
            BcswStatus=0;
            mSenseKey=0;
            mASC=0;	
            UpPoint3Status = 1;				
            break;
        case WRITE:	
            UFI_write();
            break;
        case PRE_OR_MED:
						UFI_perOrMed();
						break;
				case STA_STO_UNIT:
						UFI_staStoUnit();
						break;
        case TEST_UNIT:
            UDISKBULKDOWN=0;
            UDISKBULKUP=0;
            BcswStatus=0;			
            mSenseKey=0;
            mASC=0;
            UpPoint3Status = 1;
            break;
        case READ:
            uDiskRead();
            break;
        case REQUEST_SENSE:          
            MassPara.Sense.ErrorCode=0x70;
            MassPara.Sense.Reserved1=0;
            MassPara.Sense.SenseKey=mSenseKey;
            MassPara.Sense.Information[0]=0;
            MassPara.Sense.Information[1]=0;
            MassPara.Sense.Information[2]=0;
            MassPara.Sense.Information[3]=0;
            MassPara.Sense.AddSenseLength=0x0a;
            MassPara.Sense.Reserved2[0]=0;
            MassPara.Sense.Reserved2[1]=0;
            MassPara.Sense.Reserved2[2]=0;
            MassPara.Sense.Reserved2[3]=0;
            MassPara.Sense.AddSenseCode=mASC;
            MassPara.Sense.AddSenseCodeQua=0;
            MassPara.Sense.Reserved3[0]=0;
            MassPara.Sense.Reserved3[1]=0;
            MassPara.Sense.Reserved3[2]=0;
            MassPara.Sense.Reserved3[3]=0;
            pBuf=MassPara.buf;
            if ( UFI_Length.mDataLength > 18 ) UFI_Length.mDataLength = 18;
            BcswStatus=0;
            mSenseKey=0;
            mASC=0;				
            UpPoint3Status = 1;
            break;
        case READ_CAPACITY:
            if ( UFI_Length.mDataLength > sizeof(DBCAPACITY) ) UFI_Length.mDataLength = sizeof(DBCAPACITY);
            pBuf=(unsigned char*)DBCAPACITY;	
            BcswStatus=0;
            mSenseKey=0;
            mASC=0;  
            UpPoint3Status = 1;				
            break;
        case MODE_SENSE:
            if ( UFI_Length.mDataLength > sizeof(modesense3F) ) UFI_Length.mDataLength = sizeof(modesense3F);
            pBuf=(unsigned char*)modesense3F;	
            BcswStatus=0;
            mSenseKey=0;
            mASC=0;
            UpPoint3Status = 1;				
            break;
				case MODE_SENSE5:
						UFI_modeSense5();
						break;
				case 0xA3:
						//UDISKBULKDOWN=0;
            //UDISKBULKUP=0;
            BcswStatus=0;			
            mSenseKey=0;
            mASC=0;
            UpPoint3Status = 1;
						break;
				case 0xC0:
					  //UDISKBULKDOWN=0;
            //UDISKBULKUP=0;
            BcswStatus=0;			
            mSenseKey=0;
            mASC=0;
            UpPoint3Status = 1;
					/*
						mSenseKey=5;
						mASC=0x20;
						BcswStatus=1;
						UDISKBULKUP = 0;
						UpPoint3Status = 1;	
						UEP3_CTRL = UEP3_CTRL | MASK_UEP_T_RES ;
						UEP3_CTRL = UEP3_CTRL | MASK_UEP_R_RES ;
						BcswStatus = 2;*/
            break;
						/*
						mSenseKey=5;
						mASC=0x20;
						BcswStatus=1;
						UEP1_CTRL = UEP1_CTRL | MASK_UEP_T_RES ;
						UEP1_CTRL = UEP1_CTRL | MASK_UEP_R_RES ;
						BcswStatus = 2;
						break;*/
        default:
						mSenseKey=5;
						mASC=0x20;
						BcswStatus=1;
						UDISKBULKUP = 0;
						UEP3_CTRL = UEP3_CTRL | MASK_UEP_T_RES ;
						UEP3_CTRL = UEP3_CTRL | MASK_UEP_R_RES ;
						BcswStatus = 2;
						break;
					/*
            mSenseKey=5;
            mASC=0x24;
            BcswStatus=1;
            UpPoint3Status = 1;
            if(UDISKBULKUP){
                UEP3_CTRL = UEP3_CTRL | UEP_T_RES_STALL;
            }else{
                UEP3_CTRL = UEP3_CTRL | UEP_R_RES_STALL;
            }
            break;*/
    }
}

void UDISKBulkOnly(){
    if(MassPara.buf[0]==0x55){
        if(MassPara.buf[1]==0x53){
            if(MassPara.buf[2]==0x42){
                if(MassPara.buf[3]==0x43){
                    UFI_Length.mdataLen[3] = *(unsigned char *)(&MassPara.cbw.dCBWDatL);             /* 将PC机的低字节在前的16位字数据转换为C51的高字节在前的数据 */
                    UFI_Length.mdataLen[2] = *( (unsigned char *)(&MassPara.cbw.dCBWDatL) + 1 );
                    UFI_Length.mdataLen[1] = *( (unsigned char *)(&MassPara.cbw.dCBWDatL) + 2 );
                    UFI_Length.mdataLen[0] = *( (unsigned char *)(&MassPara.cbw.dCBWDatL) + 3 );
                    mdCBWTag[0]=MassPara.buf[4];
                    mdCBWTag[1]=MassPara.buf[5];
                    mdCBWTag[2]=MassPara.buf[6];
                    mdCBWTag[3]=MassPara.buf[7];										     //取出数据长度
                    if(UFI_Length.mDataLength){
                        UDISKBULKDOWN=(MassPara.cbw.bmCBWFlags&0X80)?0:1;	             //判断是上传还是下传数据
                        UDISKBULKUP=(MassPara.cbw.bmCBWFlags&0X80)?1:0;
                    }
                    UDISKCSW=1;
                    UFI_Hunding();                                                       //调用UFI协议处理
                }else
                    UEP3_CTRL = UEP3_CTRL | UEP_T_RES_STALL ;
            }else
                UEP3_CTRL = UEP3_CTRL | UEP_T_RES_STALL ;
        }else
            UEP3_CTRL = UEP3_CTRL | UEP_T_RES_STALL ;
    }else
        UEP3_CTRL = UEP3_CTRL | UEP_T_RES_STALL ;
}


#endif		
		

#if  (OPEN_CH340 || USE_UART_UDISK)
UINT32 UserBaud = 115200;
void Config_Uart1(UINT8 *cfg_uart){
    UINT32 uart1_buad = 0;
	UINT32 x;
	UINT8 x2;
        *((UINT8 *)&uart1_buad) = cfg_uart[3];
        *((UINT8 *)&uart1_buad+1) = cfg_uart[2];
        *((UINT8 *)&uart1_buad+2) = cfg_uart[1];
        *((UINT8 *)&uart1_buad+3) = cfg_uart[0];
        IE_UART1 = 0;
        //SBAUD1 = 0 - FREQ_SYS/16/uart1_buad;
	x = 10 * FREQ_SYS / uart1_buad / 16;                                       //如果更改主频，注意x的值不要溢出                            
	x2 = x % 10;
	x /= 10;
	if ( x2 >= 5 ) x ++;                                                       //四舍五入
	SBAUD1 =  0 - x;
        IE_UART1 = 1;
			UserBaud = uart1_buad;
			NowUart1Buad = uart1_buad;
}

void SetBaud57600(){
    if(NowUart1Buad != 57600){
        NowUart1Buad = 57600;
        IE_UART1 = 0;
        SBAUD1 = 0 - FREQ_SYS/16/NowUart1Buad;
        IE_UART1 = 1;
		}
}

void Uart1_ISR(void) interrupt INT_NO_UART1{
    if(U1RI)   //收到数据
    {
				Receive_Uart_Buf[Uart_Input_Point++] = SBUF1;
				UartByteCount++;                    //当前缓冲区剩余待取字节数
				if(Uart_Input_Point>=UART_REV_LEN)
           Uart_Input_Point = 0;           //写入指针
        U1RI =0;		
    }
}


#endif
#if   OPEN_CH340
UINT8   RTS_DTR = 0;
UINT8   baudFlag0,baudFlag1 =0;
UINT8   baud0,baud1 = 0;
UINT8C DataBuf[26]={
  0x30,0x00,0xc3,0x00,0xff,0xec,0x9f,0xec,0xff,0xec,0xdf,0xec,
  0xdf,0xec,0xdf,0xec,0x9f,0xec,0x9f,0xec,0x9f,0xec,0x9f,0xec,
  0xff,0xec};



volatile UINT8I USBByteCount = 0;                                   //代表USB端点接收到的数据
volatile UINT8I USBBufOutPoint = 0;                                 //取数据指针
volatile UINT8I UpPoint2_Busy  = 0;                                 //cdc上传端点是否忙标志



void SetDTR_RTS(UINT8 temp){
#if USE_UART_UDISK
    if(!UdiskOCCUart){
#endif
        switch(temp){
            case 0x9f:
#if  USE_MOS
								RTS_LOW;
								DTR_LOW;
#else
								P3 |=  0x08; //P3.3 HIGH
								RTS_HIGH;
								DTR_HIGH;
#endif
                break;
            case 0xdf:
#if 	USE_MOS
								RTS_HIGH;
								DTR_LOW;
#else
								P3 &=  ~0x08; //P3.3 LOW
								RTS_LOW;
								DTR_HIGH;
#endif
                break;
            case 0xff:
#if  USE_MOS
								RTS_LOW;
								DTR_LOW;
#else
								P3 |=  0x08; //RTS HIGH
								RTS_HIGH;
								DTR_HIGH;
#endif
                break;
            case 0xbf:
#if  USE_MOS
								RTS_LOW;
								DTR_HIGH;
#else
								P3 |=  0x08; //RTS HIGH
								RTS_HIGH;
								DTR_LOW;
#endif
                break;
            default:
                break;
        }
#if USE_UART_UDISK
    }
#endif
}

void UsbSetBaud(UINT8 Baud0,UINT8 Baud1){
    switch(Baud0){
        case 0x80:
            switch(Baud1){
                case 0x96:                    //110 = 0x6E
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0x00;
                    LineCoding[0] = 0x6E;
                    break;
                case 0xd9:                    //300 = 0x12C
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0x01;
                    LineCoding[0] = 0x2C;
                    break;
                default:
                    break;
            }
            break;
        case 0x81:
            switch(Baud1){
                case 0x64:                    //600 = 0x258
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0x02;
                    LineCoding[0] = 0x58;
                    break;
                case 0xb2:                    //1200 = 0x4B0
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0x04;
                    LineCoding[0] = 0xB0;
                    break;
                case 0xd9:                    //2400 = 0x960
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0x09;
                    LineCoding[0] = 0x60;
                    break;
                default:
                    break;
            }
            break;
        case 0x82:
            switch(Baud1){
                case 0x64:                    //4800 = 0x12C0
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0x12;
                    LineCoding[0] = 0xC0;
                    break;
                case 0xb2:                    //9600 = 0x2580
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0x25;
                    LineCoding[0] = 0x80;
                    break;
                case 0xcc:                    //14400 = 0x3840
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0x38;
                    LineCoding[0] = 0x40;
                    break;
                case 0xd9:                    //19200 = 0x4B00
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0x4B;
                    LineCoding[0] = 0x00;
                    break;
                default:
                    break;
            }
            break;
        case 0x83:
            switch(Baud1){
                case 0x64:                    //38400 = 0x9600
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0x96;
                    LineCoding[0] = 0x00;
                    break;
                case 0x95:                    //5600 = 0xDAC0
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0xDA;
                    LineCoding[0] = 0xC0;
                    break;
                case 0x98:                    //57600 = 0xE100
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x00;
                    LineCoding[1] = 0xE1;
                    LineCoding[0] = 0x00;
                    break;
                case 0xcc:                    //115200 = 0x1c200
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x01;
                    LineCoding[1] = 0xC2;
                    LineCoding[0] = 0x00;
                    break;
                case 0xd1:                    //128000 = 0x1f400
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x01;
                    LineCoding[1] = 0xF4;
                    LineCoding[0] = 0x00;
                    break;
                case 0xe6:                    //230400 = 0x38400
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x03;
                    LineCoding[1] = 0x84;
                    LineCoding[0] = 0x00;
                    break;
                case 0xe9:                    //256000 = 0x3e800
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x03;
                    LineCoding[1] = 0xE8;
                    LineCoding[0] = 0x00;
                    break;
                case 0xf3:                    //460800 = 0x70800
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x07;
                    LineCoding[1] = 0x08;
                    LineCoding[0] = 0x00;
                    break;
                case 0xf4:                    //512000 = 0x7D000
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x07;
                    LineCoding[1] = 0xD0;
                    LineCoding[0] = 0x00;
                    break;
                case 0xf6:                    //600000 = 0x927C0
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x09;
                    LineCoding[1] = 0x27;
                    LineCoding[0] = 0xC0;
                    break;
                case 0xf8:                    //750000 = 0xB71B0
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x0B;
                    LineCoding[1] = 0x71;
                    LineCoding[0] = 0xB0;
                    break;
                default:
                    break;
            }
            break;
        case 0x87:
            switch(Baud1){
                case 0xf3:                    //921600 = 0xE1000
                    LineCoding[3] = 0x00;
                    LineCoding[2] = 0x0E;
                    LineCoding[1] = 0x10;
                    LineCoding[0] = 0x00;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
		Config_Uart1(LineCoding);
}
#endif
void InitUSB_Device(){
    USB_CTRL = 0x00;                                                //清空USB控制寄存器
    USB_CTRL &= ~bUC_HOST_MODE;                                     //该位为选择设备模式
    USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;         //USB设备和内部上拉使能,在中断期间中断标志未清除前自动返回NAK
    USB_DEV_AD = 0x00;                                              //设备地址初始化
    USB_CTRL &= ~bUC_LOW_SPEED;
    UDEV_CTRL &= ~bUD_LOW_SPEED; 	                                  //选择全速12M模式，默认方式
    UDEV_CTRL = bUD_PD_DIS;                                         // 禁止DP/DM下拉电阻
    UDEV_CTRL |= bUD_PORT_EN;                                       //使能物理端口

    UEP0_DMA = Ep0Buffer;
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    UEP0_T_LEN = 0;

    UEP2_3_MOD = 0xCC;
    UEP4_1_MOD = 0X40;

#if  OPEN_CH340
    UEP1_DMA = Ep1Buffer;
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
		UEP1_T_LEN = 0;

    UEP2_DMA = Ep2Buffer;
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
    UEP2_T_LEN = 0;
#endif
#if  OPEN_UDISK                                                     //udisk使用端点3
    UEP3_DMA = Ep3Buffer;
    UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
    UEP3_T_LEN = 0;
#endif

    USB_INT_EN |= bUIE_SUSPEND;                                     //使能设备挂起中断
    USB_INT_EN |= bUIE_TRANSFER;                                    //使能USB传输完成中断
    USB_INT_EN |= bUIE_BUS_RST;                                     //使能设备模式USB总线复位中断
    USB_INT_FG |= 0x1F;                                             //清中断标志
    IE_USB = 1;                                                     //允许单片机中断
    EA = 1;
}


void USB_DeviceInterrupt( void ) interrupt INT_NO_USB  using 1{
    UINT8 len;
#if OPEN_UDISK
    UINT8 uTempLen;
#endif
    if(UIF_TRANSFER){                                               // USB传输完成
        switch( USB_INT_ST & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) ){  //分析操作令牌和端点号
            case UIS_TOKEN_SETUP | 0:                               //SETUP包
                len = USB_RX_LEN;
                if(len == sizeof(USB_SETUP_REQ)){
                    SetupLen = ((UINT16)UsbSetupBuf->wLengthH<<8) | (UsbSetupBuf->wLengthL);      
#if OPEN_CH340
                    RTS_DTR = UsbSetupBuf->wValueL;
#endif
                    len = 0;
                    SetupReq = UsbSetupBuf->bRequest;
                    if((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD ){
                        switch(SetupReq){
#if OPEN_CH340
													
                            case 0xC0:
                                pDescr = &DataBuf[dataNum];
                                len = 2;
                                if(dataNum<24){
                                    dataNum += 2;
                                }else{
                                    dataNum = 24;
																}
                                break;
                            case 0x40:
                                len = 9;
                                break;
                            case 0xa4:                                //操作DTR RTS
																if(udiskIsOk == 1){
																	FLAG_HIGH;
																	clearUart1();
																	SetDTR_RTS(RTS_DTR);
																}
																/*
																if((beginRead == 0x00) && (needPause == 0x00)){
																	FLAG_HIGH;
																	clearUart1();
																	//SetDTR_RTS(RTS_DTR);
																}*/
                                break;
                            case 0x9a:                                //设置波特率
                                baudFlag0 = UsbSetupBuf->wValueL;
                                baudFlag1 = UsbSetupBuf->wValueH;
                                if((baudFlag0 == 0x12) && (baudFlag1 == 0x13)){
                                    baud0 = UsbSetupBuf->wIndexL;
                                    baud1 = UsbSetupBuf->wIndexH;
																		//setBaud(512000);
																	//SetBaud115200();
																		if(udiskIsOk == 1){
																			//SetBaud115200();
																			//printf("UsbSetBaud\r\n");
																			//clearUart1();
																			UsbSetBaud(baud0,baud1);
																		}
                                    //UsbSetBaud(baud0,baud1);
                                }
                                break;
#endif
#if OPEN_UDISK
                            case 0xFE:
                                pDescr = (PUINT8)( &MAX_LUN[0] );
                                len = 1;
                                break;
#endif
                            default:
                                len = 0xFF;
                                break;
													
                        }
#if  OPEN_UDISK
												//if(len != 0xFF)
#endif
												{
													if(SetupLen > len){
                            SetupLen = len;
													}
													len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;
													memcpy(Ep0Buffer,pDescr,len);
													SetupLen -= len;
													pDescr += len;
												}

                    }else{
                        switch(SetupReq){
                            case USB_GET_DESCRIPTOR:
                                switch(UsbSetupBuf->wValueH){
                                    case 1:
                                        pDescr = DevDesc;
                                        len = sizeof(DevDesc);
                                        break;
                                    case 2:
                                        pDescr = CfgDesc;
                                        len = sizeof(CfgDesc);
                                        break;
                                    default:
                                        len = 0xff;
                                        break;
                                }
                                if( SetupLen > len ){
                                    SetupLen = len;
																}
                                len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;
                                memcpy(Ep0Buffer,pDescr,len);
                                SetupLen -= len;
                                pDescr += len;
                                break;
                            case USB_SET_ADDRESS:
                                SetupLen = UsbSetupBuf->wValueL;
                                break;
                            case USB_GET_CONFIGURATION:
                                Ep0Buffer[0] = UsbConfig;
                                if(SetupLen >= 1){
                                    len = 1;
                                }
                                break;
                            case USB_SET_CONFIGURATION:
                                UsbConfig = UsbSetupBuf->wValueL;
                                break;
                            case 0x0A:
                                break;
                            case USB_CLEAR_FEATURE:
                                if(( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP){
                                    switch(UsbSetupBuf->wIndexL){
                                        case 0x81:
                                            UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                            break;
                                        case 0x82:
                                            UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                            break;
                                        case 0x02:
                                            UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                            break;
                                        /*case 0x01:
                                            UEP1_CTRL = UEP1_CTRL & ~ (bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                            break;*/
#if OPEN_UDISK
                                        case 0x83:
                                            UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                            if(UDISKCSW){
                                                UDISKCSW = 0;
                                                UDISKUpCsw();
                                            }
                                            break;
                                        case 0x03:
                                            UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                            if(UDISKCSW){
                                                UDISKCSW = 0;
                                                UDISKUpCsw();
                                            }
                                            break;
#endif
                                        default:
                                            len = 0xFF;
                                            break;
																		}
                                }else{
                                    len = 0xFF;
                                }
                                break;
                            case USB_SET_FEATURE:
                                if(( UsbSetupBuf->bRequestType & 0x1F ) == 0x00){
                                    if((((UINT16)UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01){
                                        if(CfgDesc[ 7 ] & 0x20){
                                            //设置唤醒使能标志
                                        }else{
                                            len = 0xFF;
                                        }
                                    }else{
                                        len = 0xFF;
                                    }
                                }else if((UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_ENDP){
                                    if((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00){
                                        switch(((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL){
                                            case 0x81:
                                                UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;// 设置端点1 IN STALL 
                                                break;
                                            case 0x82:
                                                UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;//设置端点2 IN STALL 
                                                break;
                                            case 0x02:
                                                UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;// 设置端点2 OUT Stall 
                                                break;
#if OPEN_UDISK
                                            case 0x83:
                                                UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点3 IN STALL */
                                                break;
                                            case 0x03:
                                                UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点3 OUT Stall */
                                                break;
#endif
                                            default:
                                                len = 0xFF;
                                                break;
                                        }
                                    }else{
                                        len = 0xFF;
                                    }
                                }else{
                                    len = 0xFF;
                                }
                                break;
                            case USB_GET_STATUS:
#if OPEN_UDISK
                                if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE ){
                                    Ep0Buffer[0] = 0x02;
                                    Ep0Buffer[1] = 0x00;
                                }else{
                                    Ep0Buffer[0] = 0x00;
                                    Ep0Buffer[1] = 0x00;
																}
#else
                                Ep0Buffer[0] = 0x00;
                                Ep0Buffer[1] = 0x00;
#endif
                                if(SetupLen >= 2){
                                    len = 2;
                                }else{
                                    len = SetupLen;
                                }
                                break;
                            default:
                                len = 0xFF;
                                break;
                        }
                    }
                }else{
                    len = 0xFF;
                }
                if(len == 0xFF){
                    SetupReq = 0xFF;
                    //UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
                }else if(len <= DEFAULT_ENDP0_SIZE){
                    UEP0_T_LEN = len;
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
                }else{
                    UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                    UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
                }
                break;
            case UIS_TOKEN_IN | 0:
                switch(SetupReq){
                    case USB_GET_DESCRIPTOR:
                        len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;     //本次传输长度
                        memcpy( Ep0Buffer, pDescr, len );
                        SetupLen -= len;
                        pDescr += len;
                        UEP0_T_LEN = len;
                        UEP0_CTRL ^= bUEP_T_TOG;                                            //同步标志位翻转
                        break;
                    case USB_SET_ADDRESS:
                        USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                        break;
                    default:
                        //UEP0_T_LEN = 0;
                        //UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                        break;
                }
                if(len == 0xff){
                    UEP0_T_LEN = 0;
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                }
                break;
            case UIS_TOKEN_OUT | 0:
                len = USB_RX_LEN;
                UEP0_T_LEN = 0;                                                             //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_ACK;                                  //默认数据包是DATA0,返回应答ACK
                break;
#if OPEN_CH340
            case UIS_TOKEN_IN | 2:
                UEP2_T_LEN = 0;
                UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
                UpPoint2_Busy = 0;// not  busy
                break;
            case UIS_TOKEN_OUT | 2:
                if(U_TOG_OK){
                    len = USB_RX_LEN;
                    USBByteCount = USB_RX_LEN;
                    USBBufOutPoint = 0;
                    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;
                }
                break;
#endif
#if OPEN_UDISK
            case UIS_TOKEN_IN | 3:
								if(UDISKBULKUP){
									UDISKPORT_IN++;
									UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;//暂停上传
								}else if(UDISKCSW){
									UDISKCSW = 0;
									UDISKUpCsw();
								}else{
								}
                //UEP3_T_LEN = 0;
								/*
                UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;
                Point3IN = 1;*/
                break;
            case UIS_TOKEN_OUT | 3:
                if(U_TOG_OK){
									if(UDISKBULKDOWN){
										//UDISKDOWN++;
										
										UDISKBulkDownData();
										
									}else{
										uTempLen = USB_RX_LEN;
										if(!uTempLen)break;
										for(len = 0; len < uTempLen; len++){
											MassPara.buf[len]=Ep3Buffer[len];
										}
										UDISKBulkOnly();
										if(!UDISKBULKDOWN){
											if(!UDISKBULKUP){
												UDISKUpCsw();
											}else{
												UDISKPORT_OUT++;
												UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_T_RES_NAK;
											}
										}
										
									}
                }
                break;
#endif
            default:
                break;
        }
        UIF_TRANSFER = 0;                                                           //写0清空中断
    }
    if(UIF_BUS_RST){
				printf("Reset\r\n");
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
#if OPEN_UDISK
        UEP3_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
#endif
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0;
#if OPEN_CH340
        Uart_Input_Point = 0;   //循环缓冲区输入指针
        Uart_Output_Point = 0;  //循环缓冲区读出指针
        UartByteCount = 0;      //当前缓冲区剩余待取字节数
        USBByteCount = 0;       //USB端点收到的长度
        UpPoint2_Busy = 0;
#endif
        UsbConfig = 0;          //清除配置值
    }
    if(UIF_SUSPEND){
        UIF_SUSPEND = 0;
        if(USB_MIS_ST & bUMS_SUSPEND){
            while ( XBUS_AUX & bUART0_TX )
            {;}
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;                      //USB或者RXD0/1有信号时可被唤醒
            PCON |= PD;
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = 0x00;
        }
    }else{
				USB_INT_FG = 0xFF;                                                             //清中断标志
        
    }
}

void esp32Rest(){
#if   USE_MOS
		RTS_LOW;
		DTR_HIGH;
		mDelaymS(10); 
		DTR_LOW;
		RTS_LOW;
#else
		P3 |= 0x08;  //P3.3 HIGH
		RTS_HIGH;
		DTR_LOW;
    mDelaymS(10); 
		DTR_HIGH;
		RTS_HIGH;
		P3 |= 0x08; //p3.3 HIGH
#endif
}


void main(){
		UINT32 LastLen = 0;
		UINT8 recv_done = 0;
    UINT8 tempLen = 0;
    UINT8 Uart_Timeout = 0;
		UINT32 udisk_timeout = 0;
		UINT8  udisk_timeoutfreq = 0;
		UINT32 TIMEOUTREC = 0;
	  UINT8 TIMEOUTUDISK = 0;
    CfgFsys( );                                                     //配置系统时钟
    mDelaymS(2000);                                                   //等待内部晶振稳定
#if OPEN_CH340
    //P1_MOD_OC=0x00;
    //P1_DIR_PU=0xFF;
		//p1_6  rx
		//p1_7  tx
		P1_MOD_OC &= ~0x40;//rx 推挽输出
		P1_DIR_PU &= ~0x40; //rx 高阻输入模式
	  P1_MOD_OC |= 0x80; //tx 开漏输出
		P1_DIR_PU &= ~0x80; //tx 禁止上拉
#if  USE_MOS
		P1_MOD_OC &= ~0x10; //RTS 推挽输出
		P1_DIR_PU |= 0x10; //RTS 输出
		P1_MOD_OC &= ~0x20; //DTR 推挽输出
		P1_DIR_PU |= 0x20; //DTR 输出
		RTS_LOW;
		DTR_LOW;
#else
		P3_MOD_OC |= 0x08;  //P3.3 开漏输出
		P3_DIR_PU &= ~0x08; //P3.3 禁止上拉
		P3_MOD_OC |= 0x04;  //P3.2 开漏输出 FLAG
		P3_DIR_PU &= ~0x04; //P3.2 禁止上拉 FLAG
		P1_MOD_OC |= 0x10;  //RTS  开漏输出
		P1_DIR_PU &= ~0x10; //RTS  禁止上拉
		P1_MOD_OC |= 0x20;  //DTR  开漏输出
		P1_DIR_PU &= ~0x20; //DTR  禁止上拉
		RTS_HIGH;
		DTR_HIGH;
#endif

#endif
    //esp32Rest();
    mDelaymS(200);                                                   //等待内部晶振稳定
    mInitSTDIO( );                                                  //初始化串口0，用于查看调试打印信息

#if (OPEN_CH340 || USE_UART_UDISK)
    UART1Setup( );                                                  //初始化串口1，用于CH340
		//SetBaud115200();
		//setBaud(115200);
		setBaud(512000);
#endif
#if USE_UART_UDISK
    MYBUFLEN = 0;
#endif
		printf("hello\r\n");
		
		//mDelaymS(2000); 
    InitUSB_Device();                                               //初始化USB配置
		clearUart1();
		MYBUFLEN = 0;
		FLAG_LOW;
    while(1)
    {
			first:
			if(UsbConfig){
#if OPEN_CH340
				if(USBByteCount){//USB 接收端点有数据
					//udiskIsOk = 1;
					if(udiskIsOk == 0){
						udiskIsOk = 1;
						printf("udiskIsOk = 1\r\n");
						setBaud(115200);
					}
					CH554UART1SendByte(Ep2Buffer[USBBufOutPoint++]);
					USBByteCount--;
					if(USBByteCount==0)
						UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
				}
#endif
				if(UartByteCount){
					Uart_Timeout++;}
				if((beginRead == 1) && (udiskIsOk == 0)){
					//
					tempLen = UartByteCount;
					if(tempLen > 31 || Uart_Timeout > 80){
						if(tempLen > 0){
							if(Uart_Output_Point + tempLen > UART_REV_LEN){
								tempLen = UART_REV_LEN-Uart_Output_Point;
							}
							UartByteCount -= tempLen;
							memcpy(MYBUF + MYBUFLEN,&Receive_Uart_Buf[Uart_Output_Point],tempLen);
							MYBUFLEN += tempLen;
							Uart_Output_Point+=tempLen;
							if(Uart_Output_Point >= UART_REV_LEN){
								Uart_Output_Point = 0;
							}
							Uart_Timeout = 0;
						}
						if(MYBUFLEN == 0x200){
							//printf("MYBUFLEN == 0x200\r\n");
							LastLen = SENDDATALEN - MYBUFLEN;
							pBuf = MYBUF;
							MYBUFLEN = 0;
							tempLen = 0;
							needPause = 0;
							beginRead = 0;
							clearUart1();
							//udisk_timeout = 0;
							//udisk_timeoutfreq = 0;
							//FLAG_HIGH;
						}else{
						}
						
					}else{
						
						if(tempLen == 0){
							TIMEOUTREC++;
							if(TIMEOUTREC > 0xFFF0){
								TIMEOUTREC = 0;
								TIMEOUTUDISK++;
							}
							if(TIMEOUTUDISK > 3){
								TIMEOUTUDISK = 0;
								TIMEOUTREC = 0;
								FLAG_LOW;
								//setBaud(512000);
								MYBUFLEN = 0;
								tempLen = 0;
								Uart_Timeout = 0;
								//clearUart1();
								
								CH554UART1SendByte(0x55);
								CH554UART1SendByte(0xBB);
								CH554UART1SendByte(0x03);
								CH554UART1SendByte(0x00);
								CH554UART1SendByte(0x00);
								CH554UART1SendByte(0x00);
								CH554UART1SendByte(0x00);
								CH554UART1SendByte(0x00);
								CH554UART1SendByte(0x00);
								CH554UART1SendByte(0x00);
								CH554UART1SendByte(0x00);
								printf("+++++++++++++++++++++++++++++++++++++\r\n");
								goto first;
							}
						}
					}
				}else{
					if(!UpPoint2_Busy)   //端点不繁忙（空闲后的第一包数据，只用作触发上传）
					{
						tempLen = UartByteCount;
            if(tempLen > 0)
            {
							if(tempLen>31 || Uart_Timeout>100)
              {        
								Uart_Timeout = 0;
                if(Uart_Output_Point+tempLen>UART_REV_LEN)
									tempLen = UART_REV_LEN-Uart_Output_Point;
                UartByteCount -= tempLen;            
                //写上传端点
                memcpy(Ep2Buffer+MAX_PACKET_SIZE,&Receive_Uart_Buf[Uart_Output_Point],tempLen);
                Uart_Output_Point+=tempLen;
                if(Uart_Output_Point>=UART_REV_LEN)
									Uart_Output_Point = 0;                        
                UEP2_T_LEN = tempLen;                                                    //预使用发送长度一定要清空
                UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //应答ACK
                UpPoint2_Busy = 1;
							}
						}
					}
				}

				if(UDISKPORT_IN > 0){
					if(UDISKBULKUP){
						if(needPause == 0){
							//printf("UDISKPORT_IN\r\n");
							UDISKPORT_IN--;
							UDISKbulkUpData();
							//mDelaymS(5);
						}
					}
				}
				 if(UDISKPORT_OUT > 0){
					if(UDISKBULKUP){
						if(needPause == 0){
							//printf("UDISKPORT_OUT\r\n");
							UDISKPORT_OUT--;
							UDISKbulkUpData();
							//mDelaymS(5);
						}
					}
				}
				if((LastLen >0) && (beginRead == 0)){
					//printf("SENDDATALEN=0x%lX\r\n",SENDDATALEN);
					//udisk_timeout = 0;
					beginRead = 1;
					FLAG_LOW;
					//SetBaud115200();
					setBaud(512000);
					//clearUart1();
					mDelaymS(1);
					CH554UART1SendByte(0x55);
					CH554UART1SendByte(0xBB);
					CH554UART1SendByte(0x04);
					CH554UART1SendByte(0x00);
					CH554UART1SendByte(0x00);
					CH554UART1SendByte(0x00);
					CH554UART1SendByte(0x00);
					CH554UART1SendByte(0x00);
					CH554UART1SendByte(0x00);
					CH554UART1SendByte(0xFF);
					CH554UART1SendByte(0xFF);
					//mDelaymS(100);
				}
				if((udiskIsOk == 0)&&(beginRead == 0)){
					udisk_timeout++;
					if(udisk_timeout > 0xFFF0){
						udisk_timeoutfreq++;
						udisk_timeout = 0;
					}
					if(udisk_timeoutfreq > 5){
						udisk_timeoutfreq = 0;
						udisk_timeout = 0;
						udiskIsOk = 1;
						FLAG_HIGH;
						setBaud(UserBaud);
						printf("timeout\r\n");
					}
					//mDelaymS(1);
				}
			}
		}
}