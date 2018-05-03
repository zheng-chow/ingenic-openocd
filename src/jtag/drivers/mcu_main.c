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
	int skip	= 0x00000000;
	int buffer 	= 0x00000000;
	int status  	= 0x00000000;
	int tms_count	= 0x00000000;
	unsigned tms_scan	= 0x00000000;
	unsigned bit_cnt	= 0x00000000;
	unsigned scan_size	= 0x00000000;
	unsigned tms_flag	= 0x00000000;
	
	while(1) {
		dout = share_data;
		if(dout & 0x80000000) {												//write only
//  			0000  0000  0000  0000  0000  0000  0000  0000
//  			状态              skip   tms_scan    tms_count
			int i = 0;
			share_data = 0;
			skip = (dout & 0x000f0000) >> 16;
			tms_scan = (dout & 0x0000ff00) >> 8;
			tms_count = (dout & 0x000000ff);
			status = (*(volatile unsigned int *)0x10010340);

			for (i = skip; i < tms_count; i++) {
				status = (status & 0xFFFFFFF8) | (((tms_scan >> i) & 1) << 1);					//TMS
				(*(volatile unsigned int *)0x10010340) = status;						//设置TMS TDI,输出CLK0
				(*(volatile unsigned int *)0x10010340) = status | 1;						//设置TMS TDI,输出CLK1
			}
			(*(volatile unsigned int *)0x10010340) = status;							//设置TMS TDI,输出CLK0
		}
		if(dout & 0x40000000) {												//write for <= 8
//			0000  0000  0000  0000  0000  0000  0000  0000
//			状态        类型   FTMS  次------数  数------据
			buffer = 0x00000000;
			scan_size = (dout & 0x0000ff00) >> 8;
			type = (dout & 0x00f00000);
			status = (*(volatile unsigned int *)0x10010340);
			if (type == 0x00200000) {
				share_data = 0;
				for (bit_cnt = 0; bit_cnt < (scan_size-1); bit_cnt++) {
					status = (status & 0xFFFFFFF8) | (((dout >> bit_cnt) & 1) << 2);			//TMS
					(*(volatile unsigned int *)0x10010340) = status;					//设置TMS TDI,输出CLK0
					(*(volatile unsigned int *)0x10010340) = status | 1;					//设置TMS TDI,输出CLK1
				}
				status = (status & 0xFFFFFFF8) | (((dout >> bit_cnt) & 1) << 2) | (1 << 1);			//TMS
				(*(volatile unsigned int *)0x10010340) = status;						//设置TMS TDI,输出CLK0
				(*(volatile unsigned int *)0x10010340) = status | 1;						//设置TMS TDI,输出CLK1
			} else {
				if (type != 0x00100000) {
					for (bit_cnt = 0; bit_cnt < (scan_size-1); bit_cnt++) {
						status = (status & 0xFFFFFFF8) | (((dout >> bit_cnt) & 1) << 2);		//TMS
						(*(volatile unsigned int *)0x10010340) = status;				//设置TMS TDI,输出CLK0
						if((*(volatile unsigned int *)0x10010300) & 0x00000008)
							buffer |= 1 << bit_cnt;
						(*(volatile unsigned int *)0x10010340) = status | 1;				//设置TMS TDI,输出CLK1
					}
					status = (status & 0xFFFFFFF8) | (((dout >> bit_cnt) & 1) << 2) | (1 << 1);		//TMS
					(*(volatile unsigned int *)0x10010340) = status;					//设置TMS TDI,输出CLK0
					if((*(volatile unsigned int *)0x10010300) & 0x00000008)
						buffer |= 1 << bit_cnt;
					share_data = buffer;
					(*(volatile unsigned int *)0x10010340) = status | 1;					//设置TMS TDI,输出CLK1
				} else {
					for (bit_cnt = 0; bit_cnt < (scan_size-1); bit_cnt++) {
						(*(volatile unsigned int *)0x10010340) = status;				//设置TMS TDI,输出CLK0
						if((*(volatile unsigned int *)0x10010300) & 0x00000008)
							buffer |= 1 << bit_cnt;
						(*(volatile unsigned int *)0x10010340) = status | 1;				//设置TMS TDI,输出CLK1
					}
					status = (status & 0xFFFFFFF8) | (1 << 1);	//TMS
					(*(volatile unsigned int *)0x10010340) = status;					//设置TMS TDI,输出CLK0
					if((*(volatile unsigned int *)0x10010300) & 0x00000008)
						buffer |= 1 << bit_cnt;
					share_data = buffer;
					(*(volatile unsigned int *)0x10010340) = status | 1;
				}
			}
		}
		if(dout & 0x20000000) {	//write for 32
//			0000  0000  0000  0000  0000  0000  0000  0000
//			状态        类型   FTMS  次------数  数------据
			data = share_data2;
			buffer = 0x00000000;
			type = (dout & 0x00f00000);
			status = (*(volatile unsigned int *)0x10010340);
			tms_flag = dout & 0x00010000;
			if (type == 0x00200000) {
				share_data = 0;
				for (bit_cnt = 0; bit_cnt < (31); bit_cnt++) {
					status = (status & 0xFFFFFFF8) | (((data >> bit_cnt) & 1) << 2);				//TMS
					(*(volatile unsigned int *)0x10010340) = status;						//设置TMS TDI,输出CLK0
					(*(volatile unsigned int *)0x10010340) = status | 1;						//设置TMS TDI,输出CLK1
				}
				status = (status & 0xFFFFFFF8) | (((data >> bit_cnt) & 1) << 2) | (tms_flag ? (1 << 1) : 0);		//TMS
				(*(volatile unsigned int *)0x10010340) = status;							//设置TMS TDI,输出CLK0
				(*(volatile unsigned int *)0x10010340) = status | 1;							//设置TMS TDI,输出CLK1
			} else {
				if (type != 0x00100000) {
					for (bit_cnt = 0; bit_cnt < (31); bit_cnt++) {
						status = (status & 0xFFFFFFF8) | (((data >> bit_cnt) & 1) << 2);			//TMS
						(*(volatile unsigned int *)0x10010340) = status;					//设置TMS TDI,输出CLK0
						if((*(volatile unsigned int *)0x10010300) & 0x00000008)
							buffer |= 1 << bit_cnt;
						(*(volatile unsigned int *)0x10010340) = status | 1;					//设置TMS TDI,输出CLK1
					}
					status = (status & 0xFFFFFFF8) | (((data >> bit_cnt) & 1) << 2) | (tms_flag ? (1 << 1) : 0);	//TMS
					(*(volatile unsigned int *)0x10010340) = status;						//设置TMS TDI,输出CLK0
					if((*(volatile unsigned int *)0x10010300) & 0x00000008)
						buffer |= 1 << bit_cnt;
					share_data2 = buffer;
					share_data = 0;
					(*(volatile unsigned int *)0x10010340) = status | 1;						//设置TMS TDI,输出CLK1
				} else {
					for (bit_cnt = 0; bit_cnt < (31); bit_cnt++) {
						(*(volatile unsigned int *)0x10010340) = status;					//设置TMS TDI,输出CLK0
						if((*(volatile unsigned int *)0x10010300) & 0x00000008)
							buffer |= 1 << bit_cnt;
						(*(volatile unsigned int *)0x10010340) = status | 1;					//设置TMS TDI,输出CLK1
					}
					status = (status & 0xFFFFFFF8) | (tms_flag ? (1 << 1) : 0);					//TMS
					(*(volatile unsigned int *)0x10010340) = status;						//设置TMS TDI,输出CLK0
					if((*(volatile unsigned int *)0x10010300) & 0x00000008)
						buffer |= 1 << bit_cnt;
					share_data2 = buffer;
					share_data = 0;
					(*(volatile unsigned int *)0x10010340) = status | 1;
				}
			}
		}
	}
	return 0;
}

