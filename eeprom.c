#include "eeprom.h"
#include <intrins.h>
#include <STC8A8K64D4.H>

#define IAP_ADDRESS 0x0000

typedef unsigned char BYTE;
typedef unsigned int WORD;

/* IAP SFR 由 STC8A8K64D4.H 定义，无需手动声明 */

/*Define ISP/IAP/EEPROM operation const for IAP_CONTR*/
//#define ENABLE_IAP 0x80           //if SYSCLK<30MHz
//#define ENABLE_IAP 0x81           //if SYSCLK<24MHz
#define ENABLE_IAP  0x82            //if SYSCLK<20MHz
//#define ENABLE_IAP 0x83           //if SYSCLK<12MHz
//#define ENABLE_IAP 0x84           //if SYSCLK<6MHz
//#define ENABLE_IAP 0x85           //if SYSCLK<3MHz
//#define ENABLE_IAP 0x86           //if SYSCLK<2MHz
//#define ENABLE_IAP 0x87           //if SYSCLK<1MHz

/*Define ISP/IAP/EEPROM command*/
#define CMD_IDLE    0               //Stand-By
#define CMD_READ    1               //Byte-Read
#define CMD_PROGRAM 2               //Byte-Program
#define CMD_ERASE   3               //Sector-Erase

/* STC8A8K64D4 专用：IAP_TPS 设置 EEPROM 擦写等待时间，值为 CPU 主频 MHz */
#define IAP_TPS_VALUE  11           // 11.0592MHz -> 取 11



/*----------------------------
Software delay function
----------------------------*/
void Delay(BYTE n)
{
    WORD x;

    while (n--)
    {
        x = 0;
        while (++x);
    }
}

/*----------------------------
Disable ISP/IAP/EEPROM function
Make MCU in a safe state
----------------------------*/
void IapIdle()
{
    IAP_CONTR = 0;                  //Close IAP function
    IAP_CMD = 0;                    //Clear command to standby
    IAP_TRIG = 0;                   //Clear trigger register
    IAP_ADDRH = 0x80;               //Data ptr point to non-EEPROM area
    IAP_ADDRL = 0;                  //Clear IAP address to prevent misuse
}

/*----------------------------
Read one byte from ISP/IAP/EEPROM area
Input: addr (ISP/IAP/EEPROM address)
Output:Flash data
----------------------------*/
BYTE IapReadByte(WORD addr)
{
    BYTE dat;

    EA = 0;                         //关闭中断，防止IAP触发序列被打断
    IAP_TPS = IAP_TPS_VALUE;         //设置 EEPROM 擦写等待时间（STC8A8K64D4 专用）
    IAP_CONTR = ENABLE_IAP;
    IAP_CMD = CMD_READ;
    IAP_ADDRL = addr;
    IAP_ADDRH = addr >> 8;
    IAP_TRIG = 0x5a;
    IAP_TRIG = 0xa5;
    _nop_();
    dat = IAP_DATA;
    EA = 1;                          //恢复中断
    IapIdle();

    return dat;
}

/*----------------------------
Program one byte to ISP/IAP/EEPROM area
Input: addr (ISP/IAP/EEPROM address)
       dat (ISP/IAP/EEPROM data)
Output:-
----------------------------*/
void IapProgramByte(WORD addr, BYTE dat)
{
    EA = 0;                         //关闭中断，防止IAP触发序列被打断
    IAP_TPS = IAP_TPS_VALUE;         //设置 EEPROM 擦写等待时间（STC8A8K64D4 专用）
    IAP_CONTR = ENABLE_IAP;
    IAP_CMD = CMD_PROGRAM;
    IAP_ADDRL = addr;
    IAP_ADDRH = addr >> 8;
    IAP_DATA = dat;
    IAP_TRIG = 0x5a;
    IAP_TRIG = 0xa5;
    _nop_();
    EA = 1;                          //恢复中断
    IapIdle();
}

/*----------------------------
Erase one sector area
Input: addr (ISP/IAP/EEPROM address)
Output:-
----------------------------*/
void IapEraseSector(WORD addr)
{
    EA = 0;                         //关闭中断，防止IAP触发序列被打断
    IAP_TPS = IAP_TPS_VALUE;         //设置 EEPROM 擦写等待时间（STC8A8K64D4 专用）
    IAP_CONTR = ENABLE_IAP;
    IAP_CMD = CMD_ERASE;
    IAP_ADDRL = addr;
    IAP_ADDRH = addr >> 8;
    IAP_TRIG = 0x5a;
    IAP_TRIG = 0xa5;
    _nop_();
    EA = 1;                          //恢复中断
    IapIdle();
}




int read_params(unsigned char *buf,int len)
{
	int i = 0;
	for (i = 0; i < len; i++) {          //Program 512 bytes data into data flash    
    buf[i] = IapReadByte(IAP_ADDRESS+i);
  }
	
  return 1;	
}

int save_params(unsigned char *buf,int len)
{
	int i;

	/* 1. 擦除扇区 */
	IapEraseSector(IAP_ADDRESS);
	Delay(20);

	/* 2. 验证擦除结果（只检查实际使用的长度） */
	for (i = 0; i < len; i++) {
		if (IapReadByte(IAP_ADDRESS + i) != 0xff)
			return 0;
	}

	/* 3. 逐字节写入，期间喂狗防复位 */
	for (i = 0; i < len; i++) {
		IapProgramByte(IAP_ADDRESS + i, (BYTE)buf[i]);
	}

	/* 4. 验证写入结果 */
	for (i = 0; i < len; i++) {
		if (IapReadByte(IAP_ADDRESS + i) != (BYTE)buf[i])
			return -1;
	}

	return 1;
}
