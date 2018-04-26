#include "X1000.h"

#define share_data	*(volatile unsigned int*)(0xf4001ff0)
#define share_data2	*(volatile unsigned int*)(0xf4001ff4)

void trap_entry(void)
{
	/*Now Noting to do*/
}

int  main()
{
	int data  	= 0x00000000;
	int dout 	= 0x00000000;
	int type	= 0x00000000;
	int buffer  = 0x00000000;
	int status  = 0x00000000;
	unsigned bit_cnt	= 0x00000000;
	unsigned scan_size	= 0x00000000;
	unsigned tms_flag	= 0x00000000;
	
	while(1)
	{
		dout = share_data;
		if(dout & 0x40000000)													//write only
		{
			share_data = 0;
			(*(volatile unsigned int *)0x10010340) = dout;						//设置TMS TDI,输出CLK0
			(*(volatile unsigned int *)0x10010340) = dout | 1;					//设置TMS TDI,输出CLK1
		}
		if(dout & 0x20000000)													//write for <= 8
		{
//			0000  0000  0000  0000  0000  0000  0000  0000
//			状态        类型	  FTMS  次------数	数------据
			buffer = 0x00000000;
			scan_size = (dout & 0x0000ff00) >> 8;
			type = (dout & 0x00f00000);
			status = (*(volatile unsigned int *)0x10010340);
			tms_flag = dout & 0x00010000;
			if (type == 0x00200000) {
				share_data = 0;
				for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
					status = (status & 0xFFFFFFF8) | (((bit_cnt == scan_size-1) && tms_flag) ? (1 << 1) : 0);//TMS
					int bcval = 1 << bit_cnt;
					if (dout & bcval)
						status = status | (1 << 2);								//TDI
					(*(volatile unsigned int *)0x10010340) = status;			//设置TMS TDI,输出CLK0
					(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
				}
			} else {
				for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
					status = (status & 0xFFFFFFF8) | (((bit_cnt == scan_size-1) && tms_flag) ? (1 << 1) : 0);//TMS
					int bcval = 1 << bit_cnt;

					if ((type != 0x00100000) && (dout & bcval))					//SCAN_IN=0x00100000
						status = status | (1 << 2);								//TDI
					(*(volatile unsigned int *)0x10010340) = status;			//设置TMS TDI,输出CLK0
					if((*(volatile unsigned int *)0x10010300) & 0x00000008)
						buffer |= bcval;
					(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
				}
				share_data = buffer;
			}
		}
		if(dout & 0x10000000)													//write for 32
		{
//			0000  0000  0000  0000  0000  0000  0000  0000
//			状态        类型	  FTMS  次------数	数------据
			data = share_data2;
			buffer = 0x00000000;
			scan_size = (dout & 0x0000ff00) >> 8;
			type = (dout & 0x00f00000);
			status = (*(volatile unsigned int *)0x10010340);
			tms_flag = dout & 0x00010000;
			if (type == 0x00200000) {
				share_data = 0;
				for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
					status = (status & 0xFFFFFFF8) | (((bit_cnt == scan_size-1) && tms_flag) ? (1 << 1) : 0);//TMS
					int bcval = 1 << bit_cnt;
					if (data & bcval)
						status = status | (1 << 2);								//TDI
					(*(volatile unsigned int *)0x10010340) = status;			//设置TMS TDI,输出CLK0
					(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
				}
			} else {
				for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
					status = (status & 0xFFFFFFF8) | (((bit_cnt == scan_size-1) && tms_flag) ? (1 << 1) : 0);//TMS
					int bcval = 1 << bit_cnt;

					if ((type != 0x00100000) && (data & bcval))					//SCAN_IN=0x00100000
						status = status | (1 << 2);								//TDI
					(*(volatile unsigned int *)0x10010340) = status;			//设置TMS TDI,输出CLK0
					if((*(volatile unsigned int *)0x10010300) & 0x00000008)
						buffer |= bcval;
					(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
				}
				share_data2 = buffer;
				share_data = 0;
			}
		}
	}
	return 0;
}

