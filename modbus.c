#include "modbus.h"
#include "debugUart.h"
//#include "stc.h"
#include "crc.h"
#include "intrins.h"
#include "eeprom.h"
#include "board.h"
#include "modbus_reg_config.h"
#include <STC8A8K64D4.H>

#define FUNCTION_READ_COILS_1                 1     			//读线圈，功能码 1
#define FUNCTION_READ_DISCREATE_INPUT_2       2     			//读离散量输入，功能码 2
#define FUNCTION_READ_HOLDING_REGISTERS_3     3     			//读保持寄存器，功能码 3
#define FUNCTION_READ_INPUT_REGISTERS_4       4     			//读输入寄存器，功能码4
#define FUNCTION_WRITE_SINGLE_COIL_5          5     			//写单个线圈，功能码 5
#define FUNCTION_WRITE_SINGLE_REGISTER_6      6     			//写单个寄存器，功能码 6
#define FUNCTION_WRITE_MULTIPLE_COILS_F       0xf   			//写多个线圈，功能码 0xf
#define FUNCTION_WRITE_MULTIPLE_REGISTERS_10  0x10  			//写多个寄存器,功能码 0x10
#define FUNCTION_READ_WRITE_MULTIPLE_REGISTERS_17  0x17  	//读写多个寄存器


unsigned char dev_address = 1;   			    								//设备地址,下载之前修改
unsigned char dev_broadcast_address = 0;  								//设备广播地址

unsigned short  var1 ;
unsigned short  var2 ;
unsigned char 	var ;
unsigned char 	Out1 ;
unsigned char 	Out2 ;
unsigned char 	In1 =1 ;
unsigned char 	In2 =1;

unsigned char xdata send_modbus_buffer[256];
unsigned char xdata localbuf[256];
char xdata msg[64];
int xdata send_modbus_buf_count = 0;
extern volatile unsigned char xdata g_holding_regs_dirty;

void send_buffer(unsigned char *buf,int len);

void send_bad_msg(unsigned char address,unsigned char function,unsigned char errorCode)
{
	unsigned char return_msg[5];
	unsigned short crc = 0;
	
	return_msg[0] = address;
	return_msg[1] = 0x80 + function;
	return_msg[2] = errorCode;
	crc = crc16(return_msg,3);
	return_msg[3] = crc;
	return_msg[4] = crc >> 8;
	send_buffer(return_msg,5);  	//发送数据给主机
}

int broadcast_process(unsigned char *buf,int len)
{
	buf = buf;
	len = len;
//	debug_out("broadcast_process\r\n\0");
	return 1;
}
/*
int write_multiple_register(unsigned short start_address,unsigned short value,unsigned char *localbuf,unsigned char localbufLength)
{
	unsigned char index = 0;
	sprintf(msg,"write_multiple_register()\r\n\0");
	debug_out(msg);
	sprintf(msg,"start_address=%u,cnt=%u\r\n\0",start_address,value);
	debug_out(msg);
	for (index = 0; index < localbufLength; index++) {
		sprintf(msg,"buf[%d]=0x%x, %d\r\n\0",(int)index,(int)localbuf[index],(int)localbuf[index]);
		debug_out(msg);
	}
	return 1;
}
*/

/*
int write_multiple_coils(unsigned short start_address,unsigned short value,unsigned char *localbuf,unsigned char localbufLength)
{
	unsigned char index = 0;
	sprintf(msg,"write_multiple_coils()\r\n\0");
	debug_out(msg);
	sprintf(msg,"start_address=%u,cnt=%u\r\n\0",start_address,value);
	debug_out(msg);
	for (index = 0; index < localbufLength; index++) {
		sprintf(msg,"buf[%d]=0x%x, %d\r\n\0",(int)index,(int)localbuf[index],(int)localbuf[index]);
		debug_out(msg);
	}
	return 1;
}
*/
int write_multiple_register(unsigned short start_address,unsigned short value,unsigned char *localbuf,unsigned char localbufLength)
{
	unsigned char index;
	unsigned short reg_value;
	unsigned short address;

	if (value == 0 || localbufLength != (unsigned char)(value * 2)) {
		return 0;
	}

	if (start_address >= HOLDING_REG_COUNT) {
		return 0;
	}

	if (value > (HOLDING_REG_COUNT - start_address)) {
		return 0;
	}

	for (index = 0; index < value; index++) {
		address = start_address + index;
		reg_value = ((unsigned short)localbuf[index * 2] << 8) + localbuf[index * 2 + 1];
		g_holding_regs[address] = reg_value;
	}

	g_holding_regs[HLD_PARAM_MAGIC_OFFSET] = HLD_PARAM_MAGIC_VALUE;
	g_holding_regs_dirty = 1;
	return 1;
}

int write_single_register(unsigned short address,unsigned short value)	//写单个保持寄存器
{
	if (address < HOLDING_REG_COUNT)
	{
		g_holding_regs[address] = value;
		g_holding_regs[HLD_PARAM_MAGIC_OFFSET] = HLD_PARAM_MAGIC_VALUE;
		g_holding_regs_dirty = 1;
		return 1;
	}

	return 0;
}

int write_single_coil(unsigned short address,unsigned short value)	//写单个线圈
{
	if (address < COIL_COUNT)
	{
		g_coils[address] = (value > 0) ? 1 : 0;
		return 1;
	}
		
//	sprintf(msg,"write_single_coil()\r\n\0");
//	debug_out(msg);
//	sprintf(msg,"start_address=%u,value=%u\r\n\0",address,value);
//	debug_out(msg);
	
	if (address == 0x000) 
		{
				if (value>0)
			{
				Out1 = 1;
			}
			else
			{
				Out1 = 0 ;
			}
		}
	if (address == 0x001) 
		{
			if (value>0)
			{
				Out2 = 1;
			}
			else
			{
				Out2 = 0 ;
			}
		}
		return 1;
}

int function_READ_COILS_1(unsigned char *buf,int len) 				//读线圈，功能码 1， 0x0000
{
	int ret = 1;
	unsigned char out_count = 0;       //输出字节数
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出数量
	unsigned short crc  = 0;
	int send_total_count = 0;
	unsigned char var2 = 0;
	unsigned char i ;
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
//	sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
//	debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D0) 
	{
		send_bad_msg(buf[0],buf[1],3);   		//发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok,地址+输出数量是否ok,否则发送 异常码 2
	
	//读取线圈是否ok,否则发送 异常码 4
	out_count = count / 8;
	if (count % 8 > 0) out_count++;
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   	//地址
	send_modbus_buffer[1] = buf[1];   	//功能
	send_modbus_buffer[2] = out_count;  //输出字节数
	while (out_count--) 
	{
		
		var2 = 0;
		for (i = 0; i < 8; i++) 
		{
			if ((start_address + i) < COIL_COUNT)
			{
				if (g_coils[start_address + i] > 0)
				{
					var2 |= 1 << i;
				}
			}
		}
		
//		for (i = 0; i < 8; i++) 
//		{
//			if (start_address+i == 0001) 
//			{
//				if (Out2 > 0) 
//				{
//					var2 |= 1 << i; 
//				}
//			}
//		}

	send_modbus_buffer[3+send_modbus_buf_count] = var2;				//读线圈值，0x0000
	send_modbus_buf_count++;
	start_address += 8;

  }		
	send_modbus_buffer[2] = send_modbus_buf_count;	 			//获取 字节数
	send_total_count = send_modbus_buf_count + 5;    			//总的要发送的字节数
	crc = crc16(send_modbus_buffer,send_total_count-2);  	//计算crc
	send_modbus_buffer[send_total_count-2] = crc; 
	send_modbus_buffer[send_total_count-1] = crc >> 8;
	send_buffer(send_modbus_buffer,send_total_count);  		//发送给主机
	
//	debug_out("function_READ_COILS_1\r\n\0");
	return ret;
}


int function_READ_DISCREATE_INPUT_2(unsigned char *buf,int len)			//读离散量输入，功能码 2，1x1000
{
  int ret = 1;
	unsigned char out_count = 0;       //输出字节数
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出数量
	unsigned short crc  = 0;
	int send_total_count = 0;
	unsigned char var = 0;                //0是案例，也可以是其他值
	unsigned char i ;

	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
//	sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
//	debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D0) 
	{
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok,地址+输出数量是否ok,否则发送 异常码 2
	
	//读取线圈是否ok,否则发送 异常码 4
	out_count = count / 8;
	if (count % 8 > 0) out_count++;
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   //地址
	send_modbus_buffer[1] = buf[1];   //功能
	send_modbus_buffer[2] = out_count;  //输出字节数
	while (out_count--) 
	{
		var = 0;                				//0是案例，也可以是其他值
//	  if (start_address == 0000) 			//读离散输入的值
//		{
//			var = 0x55;
//		}
		for (i = 0; i < 8; i++) 
		{
			if ((start_address + i) < DI_COUNT)
			{
				if (g_discrete_inputs[start_address + i] > 0)
				{
					var |= 1 << i;
				}
			}
		}
		
//		for (i = 0; i < 8; i++) 
//		{
//			if (start_address+i == 0x101) 
//			{
//				if (In2 > 0) 
//				{
//					var |= 1 << i; 
//				}
//			}
//		}
		send_modbus_buffer[3+send_modbus_buf_count] = var; 		//读输入值，1x0000
		send_modbus_buf_count++;
		start_address+=8;
  }		
	send_modbus_buffer[2] = send_modbus_buf_count;	 				//获取 字节数
	send_total_count = send_modbus_buf_count + 5;    				//总的要发送的字节数
	crc = crc16(send_modbus_buffer,send_total_count-2);  		//计算crc
	send_modbus_buffer[send_total_count-2] = crc; 
	send_modbus_buffer[send_total_count-1] = crc >> 8;
	send_buffer(send_modbus_buffer,send_total_count);  			//发送给主机
	
//	debug_out("function_READ_DISCREATE_INPUT_2\r\n\0");
	return ret;
}

int function_READ_HOLDING_REGISTERS_3(unsigned char *buf,int len)		//读保持寄存器，功能码 3，4x4000
{
  int ret = 1;
	unsigned char out_count = 0;       //输出字节数
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出数量
	unsigned short crc  = 0;
	int send_total_count = 0;
	unsigned short  sendValue = 0;
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
//	sprintf(msg,"addr=0x%x,count=%x\r\n\0",start_address,count);
//	debug_out(msg);
//	sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
//	debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D) 
	{
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}

	if (start_address >= HOLDING_REG_COUNT || (start_address + count) > HOLDING_REG_COUNT) {
		send_bad_msg(buf[0],buf[1],2);   //发送 异常码 2
		ret = 1;
		return ret;
	}

	//读取寄存器是否ok,否则发送 异常码 4
	
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   //地址
	send_modbus_buffer[1] = buf[1];   //功能
	send_modbus_buffer[2] = out_count;  //输出字节数
	while (count--) 
	{
		sendValue = 0;

		if (start_address < HOLDING_REG_COUNT)
		{
			sendValue = g_holding_regs[start_address];
		}
	
		if (start_address == 0x402) 			//读保持寄存器的值
		{
			sendValue = 4444;
		}
		if (start_address == 0x401) 
		{
			sendValue = 5555;
		}
		if (start_address == 0x400) 
		{
			sendValue = var1;
		}
	  send_modbus_buffer[3+send_modbus_buf_count] = sendValue >> 8; 	//寄存器Hi
		send_modbus_buf_count++;
		send_modbus_buffer[3+send_modbus_buf_count] = sendValue; 				//寄存器Lo
		send_modbus_buf_count++;
		start_address++;
		/*
	  send_modbus_buffer[3+send_modbus_buf_count] = P2; //寄存器Hi
		send_modbus_buf_count++;
		send_modbus_buffer[3+send_modbus_buf_count] = P2; //寄存器Lo
		send_modbus_buf_count++;
		*/
	}		
	send_modbus_buffer[2] = send_modbus_buf_count;				//重新计算获得 内容的字节数
	send_total_count = send_modbus_buf_count + 5;   			//发送的总长度 字节数
	crc = crc16(send_modbus_buffer,send_total_count-2);  	//计算crc校验和
	send_modbus_buffer[send_total_count-2] = crc; 
	send_modbus_buffer[send_total_count-1] = crc >> 8;
	send_buffer(send_modbus_buffer,send_total_count);  		//发送给主机
	
//	debug_out("function_READ_HOLDING_REGISTERS_3\r\n\0");
	return ret;
}

int function_READ_INPUT_REGISTERS_4(unsigned char *buf,int len)				//读输入寄存器，功能码4,3x3000
{
	int ret = 1;
	unsigned char out_count = 0;       //输出字节数
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出数量
	unsigned short crc  = 0;
	int send_total_count = 0;
	unsigned short sendValue = 0;
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
//	sprintf(msg,"addr=0x%x,count=%x\r\n\0",start_address,count);
//	debug_out(msg);
//	sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
//	debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 1 || count > 0x07D) 
	{
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok,地址+输出数量是否ok,否则发送 异常码 2
	
	//读取输入寄存器是否ok,否则发送 异常码 4
	
	send_modbus_buf_count = 0;
	send_modbus_buffer[0] = buf[0];   	//地址
	send_modbus_buffer[1] = buf[1];   	//功能
	send_modbus_buffer[2] = out_count;  //输出字节数
	while (count--) 
	{
		sendValue = 0;

		if (start_address < INPUT_REG_COUNT)
		{
			sendValue = g_input_regs[start_address];
		}
	
		if (start_address == 0x300) 			//读输入寄存器的值
		{
			sendValue = 3333;
		}
		if (start_address == 0x301) 
		{
			sendValue = 2222;
		}
		if (start_address == 0x302) 
		{
			sendValue = 1111;
		}
		
//		if (start_address == 0x303) 
//		{
//			sendValue = getadc();
//		}
	  send_modbus_buffer[3+send_modbus_buf_count] = sendValue >> 8; //寄存器Hi
		send_modbus_buf_count++;
		send_modbus_buffer[3+send_modbus_buf_count] = sendValue; 			//寄存器Lo
		send_modbus_buf_count++;
		start_address++;
		
  }		
	send_modbus_buffer[2] = send_modbus_buf_count;	//重新计算获得 内容的字节数
	send_total_count = send_modbus_buf_count + 5;   //发送的总长度 字节数
	crc = crc16(send_modbus_buffer,send_total_count-2);  //计算crc校验和
	send_modbus_buffer[send_total_count-2] = crc; 
	send_modbus_buffer[send_total_count-1] = crc >> 8;
	send_buffer(send_modbus_buffer,send_total_count);  //发送给主机
	
//	debug_out("function_READ_INPUT_REGISTERS_4\r\n\0");
	return ret;
}

int function_WRITE_SINGLE_COIL_5(unsigned char *buf,int len)				//写单个线圈，功能码 5
{
	int ret = 1;
	unsigned short start_address = 0;  //起始地址
	unsigned short value = 0;          //输出量
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	value  = buf[4] << 8;
	value += buf[5];
//	sprintf(msg,"addr=0x%x,value=%x\r\n\0",start_address,value);
//	debug_out(msg);
//	sprintf(msg,"addr=%u,value=%u\r\n\0",start_address,value);
//	debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (value != 0x0 && value != 0xFF00) 
	{
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok, 否则发送 异常码 2
	
	//写线圈是否ok,否则发送 异常码 4
	if (write_single_coil(start_address,value)) 
	{
		send_buffer(buf,len);            		//发送 正确数据给主机   
	} else {
		send_bad_msg(buf[0],buf[1],4);   		//发送 异常码 4
	}
	ret = 1;	
	
//	debug_out("function_WRITE_SINGLE_COIL_5\r\n\0");
	return ret;
}

int function_WRITE_SINGLE_REGISTER_6(unsigned char *buf,int len)			//写单个保持寄存器，功能码 6
{
  int ret = 1;
	unsigned short start_address = 0;  //起始地址
	unsigned short value = 0;          //输出量
	
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	value  = buf[4] << 8;
	value += buf[5];
//	sprintf(msg,"addr=0x%x,value=%x\r\n\0",start_address,value);
//	debug_out(msg);
//	sprintf(msg,"addr=%u,value=%u\r\n\0",start_address,value);
//	debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (value < 0x0 || value > 0xFFFF) 
	{
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok, 否则发送 异常码 2
	
	//写线圈是否ok,否则发送 异常码 4
	if (write_single_register(start_address,value)) 
	{
		send_buffer(buf,len);            //发送 正确数据给主机   
	} else {
		send_bad_msg(buf[0],buf[1],4);   //发送 异常码 4
	}
	ret = 1;	
	
//	debug_out("function_WRITE_SINGLE_REGISTER_6\r\n\0");
	return ret;
}

/*
int function_WRITE_MULTIPLE_COILS_F(unsigned char *buf,int len)				//写多个线圈，功能码 0xf
{
  int ret = 1;
	unsigned short crc = 0;
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出量
	unsigned char localbufLength = 0;
	
	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];
	sprintf(msg,"addr=0x%x,count=%x\r\n\0",start_address,count);
	debug_out(msg);
	sprintf(msg,"addr=%u,count=%u\r\n\0",start_address,count);
	debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (count < 0x1 || count > 0x7B0) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok, 否则发送 异常码 2
	
	localbufLength = buf[6];
	for (index = 0; index < localbufLength; index++) {
    localbuf[index] = buf[7+index];		
	}
	//写线圈是否ok,否则发送 异常码 4
	if (write_multiple_coils(start_address,count,localbuf,localbufLength)) 
	{
		crc = crc16(buf,6);
		buf[6] = crc;
		buf[7] = crc >> 8;
		send_buffer(buf,8);            //发送 正确数据给主机   
	} else {
		send_bad_msg(buf[0],buf[1],4);   //发送 异常码 4
	}
	ret = 1;	
	
	debug_out("function_WRITE_MULTIPLE_COILS_F\r\n\0");
	return ret;
}
*/

int function_WRITE_MULTIPLE_REGISTERS_10(unsigned char *buf,int len)			//写多个寄存器,功能码 0x10
{
	int ret = 1;
	unsigned short crc = 0;
	unsigned short start_address = 0;  //起始地址
	unsigned short count = 0;          //输出量
	unsigned char localbufLength = 0;

	//输入的起始地址和输出的数量
	start_address  = buf[2] << 8;
	start_address += buf[3];
	count  = buf[4] << 8;
	count += buf[5];

	//数量是否有效，如果无效 则发送异常码 3
	if (count < 0x1 || count > 0x7B) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3
		ret = 1;
		return ret;
	}

	//地址范围检查，防止越界
	if (start_address >= HOLDING_REG_COUNT || (start_address + count) > HOLDING_REG_COUNT) {
		send_bad_msg(buf[0],buf[1],2);   //发送 异常码 2
		ret = 1;
		return ret;
	}

	localbufLength = buf[6];

	if (localbufLength != (unsigned char)(count * 2)) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3
		ret = 1;
		return ret;
	}

	if (!write_multiple_register(start_address,count,&buf[7],localbufLength)) {
		send_bad_msg(buf[0],buf[1],4);   //发送 异常码 4
		ret = 1;
		return ret;
	}

	//发送正确响应（回显地址、功能码、起始地址、数量、CRC）
	crc = crc16(buf,6);
	buf[6] = crc;
	buf[7] = crc >> 8;
	send_buffer(buf,8);            //发送 正确数据给主机

	ret = 1;
	return ret;
}

/*
int function_READ_WRITE_MULTIPLE_REGISTERS_17(unsigned char *buf,int len)			//读写多个寄存器
{
  int ret = 1;
	unsigned short crc = 0;
	unsigned short read_start_address = 0;  //起始地址
	unsigned short write_start_address = 0;  //起始地址
	unsigned short readCount = 0;          //输出量	
	unsigned char writebytes = 0;
	unsigned short writeCount = 0;          //输出量	
	unsigned char localbufLength = 0;
	unsigned char index = 0;
	int send_total_count = 0;
	
	//输入的起始地址和输出的数量
	read_start_address  = buf[2] << 8;
	read_start_address += buf[3];
	readCount  = buf[4] << 8;
	readCount += buf[5];
	
	write_start_address  = buf[6] << 8;
	write_start_address += buf[7];
	writeCount  = buf[8] << 8;
	writeCount += buf[9];
	
	writebytes = buf[10];
	
	sprintf(msg,"raddr=0x%x,rcnt=0x%x\r\n\0",read_start_address,readCount);
	debug_out(msg);
	sprintf(msg,"raddr=%u,rcnt=%u\r\n\0",read_start_address,readCount);
	debug_out(msg);
	
	sprintf(msg,"waddr=0x%x,wcnt=0x%x\r\n\0",write_start_address,writeCount);
	debug_out(msg);
	sprintf(msg,"waddr=%u,wcnt=%u\r\n\0",write_start_address,writeCount);
	debug_out(msg);
	
	//数量是否有效，如果无效 则发送异常码 3
	if (readCount < 0x1 || readCount > 0x7D) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	if (writeCount < 0x1 || writeCount > 0x79) {
		send_bad_msg(buf[0],buf[1],3);   //发送 异常码 3 
		ret = 1;
    return ret;		
	}
	
	//地址是否ok, 否则发送 异常码 2
	
	localbufLength = writebytes;
	for (index = 0; index < localbufLength; index++) {
    localbuf[index] = buf[11+index];		
	}
	//写寄存器是否ok,否则发送 异常码 4
	if (write_multiple_register(write_start_address,writeCount,localbuf,localbufLength)) {
		send_modbus_buf_count = 0;
	  send_modbus_buffer[0] = buf[0];   //地址
	  send_modbus_buffer[1] = buf[1];   //功能
	  send_modbus_buffer[2] = readCount;  //输出字节数
	  while (readCount--) {
	    send_modbus_buffer[3+send_modbus_buf_count] = P0; //寄存器Hi
		  send_modbus_buf_count++;
		  send_modbus_buffer[3+send_modbus_buf_count] = P0; //寄存器Lo
		  send_modbus_buf_count++;
    }		
	  send_modbus_buffer[2] = send_modbus_buf_count;	//重新计算获得 内容的字节数
	  send_total_count = send_modbus_buf_count + 5;   //发送的总长度 字节数
	  crc = crc16(send_modbus_buffer,send_total_count-2);  //计算crc校验和
	  send_modbus_buffer[send_total_count-2] = crc; 
	  send_modbus_buffer[send_total_count-1] = crc >> 8;
	  send_buffer(send_modbus_buffer,send_total_count);  //发送给主机		
	} else {
		send_bad_msg(buf[0],buf[1],4);   //发送 异常码 4
	}
	ret = 1;	
	
	debug_out("function_READ_WRITE_MULTIPLE_REGISTERS_17\r\n\0");
	return ret;
}
*/


//不支持的功能码
int not_support_function_code(unsigned char address,unsigned char function)
{
	int ret = 0;	
	send_bad_msg(address,function,1);
//	debug_out("not_support_function_code\r\n\0");
	
	return ret;
}

//分析收到的数据，并处理
int parse_recv_buffer(unsigned char *buf,int len)
{	
	int ret = 0;
	unsigned short temp_crc = 0;
	unsigned short temp_crc2 = 0;
	
//	sprintf(msg,"len=%d\r\n\0",len);   //输出长度值
//	debug_out(msg);
	
	if (len < 4)     return 0;         //入参判断
	if (buf == NULL) return 0;
	
	if (buf[0] != dev_address && buf[0] != dev_broadcast_address) 			//不是有效的地址，丢弃
	{  
//		debug_out("not valid address\r\n\0"); 
		ret = 1;
		return ret;
	}
	temp_crc  = buf[len-1] << 8;
	temp_crc += buf[len-2];
	temp_crc2 = crc16(buf,len-2);   //计算crc
	//sprintf(msg,"crc1=0x%X\r\n\0",temp_crc);
	//debug_out(msg);
  //sprintf(msg,"crc2=0x%X\r\n\0",temp_crc2);
	//debug_out(msg);	
	if (temp_crc != temp_crc2) 										//错误的crc值
	{  
//		debug_out("bad crc value\r\n\0"); 
		ret = 1;
		return ret;
	}
	
	if (buf[0] == 0) 														 //广播地址，进入广播地址处理函数
	{  
		return broadcast_process(buf,len);
	}
	
	//处理 多个 功能码
	switch (buf[1]) 
	{
		case FUNCTION_READ_COILS_1                 : ret = function_READ_COILS_1(buf,len); break;   			     	   	//读线圈，功能码 1
		case FUNCTION_READ_DISCREATE_INPUT_2       : ret = function_READ_DISCREATE_INPUT_2(buf,len); break;       	//读离散量输入，功能码 2
		case FUNCTION_READ_HOLDING_REGISTERS_3     : ret = function_READ_HOLDING_REGISTERS_3(buf,len); break;     	//读保持寄存器，功能码 3
		case FUNCTION_READ_INPUT_REGISTERS_4       : ret = function_READ_INPUT_REGISTERS_4(buf,len); break;       	//读输入寄存器，功能码4
		case FUNCTION_WRITE_SINGLE_COIL_5          : ret = function_WRITE_SINGLE_COIL_5(buf,len); break;          	//写单个线圈，功能码 5
		case FUNCTION_WRITE_SINGLE_REGISTER_6      : ret = function_WRITE_SINGLE_REGISTER_6(buf,len); break;      	//写单个寄存器，功能码 6

//		case FUNCTION_WRITE_MULTIPLE_COILS_F       : ret = function_WRITE_MULTIPLE_COILS_F(buf,len); break;       //写多个线圈，功能码 0xf
		case FUNCTION_WRITE_MULTIPLE_REGISTERS_10  : ret = function_WRITE_MULTIPLE_REGISTERS_10(buf,len); break;  //写多个寄存器,功能码 0x10
//		case FUNCTION_READ_WRITE_MULTIPLE_REGISTERS_17  : ret = function_READ_WRITE_MULTIPLE_REGISTERS_17(buf,len); break; //读写多个寄存器		
		default: ret = not_support_function_code(buf[0],buf[1]);        //不支持的功能码
	}	
	
	return ret;
}
