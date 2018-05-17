#define share_data	*(volatile unsigned int*)(0xf4001ff0)
#define share_data2	*(volatile unsigned int*)(0xf4001ff4)

void trap_entry(void)
{
	/*Now Noting to do*/
}

int  main()
{
	int i = 0;
	int data  	= 0x00000000;
	int dout 	= 0x00000000;
	int type	= 0x00000000;
	int skip	= 0x00000000;
	int buffer 	= 0x00000000;
	int status  	= 0x00000030;
	int tms_count	= 0x00000000;
	unsigned tms_scan	= 0x00000000;
	unsigned bit_cnt	= 0x00000000;
	unsigned scan_size	= 0x00000000;
	unsigned tms_flag	= 0x00000000;
	unsigned tap_flag	= 0x00000000;

	while(1) {
		dout = share_data;
		if(dout & 0x08000000) {									//write for <= 8
//			0000  0000  0000  0000  0000  0000  0000  0000
//			      状态  类型   FTMS  次------数  数------据
			buffer = 0x00000000;
			scan_size = (dout & 0x0000ff00) >> 8;
			type = (dout & 0x00f00000);
			if (__builtin_expect(scan_size == 5, 1)) {
				for (i = 0; i < 4; i++) {
					status = 0x30 | ((3 >> i) & 1) << 1;
					(*(volatile unsigned int *)0x10010340) = status;
					(*(volatile unsigned int *)0x10010340) = status | 1;
				}
			}
			if (__builtin_expect(scan_size == 7, 0)) {
				for (i = 0; i < 7; i++) {
					status = 0x30 | ((27 >> i) & 1) << 1;
					(*(volatile unsigned int *)0x10010340) = status;
					(*(volatile unsigned int *)0x10010340) = status | 1;
				}
			}
			if (__builtin_expect(type == 0x00200000, 1)) {
				share_data = 0;
				for (bit_cnt = 0; bit_cnt < (scan_size-1); bit_cnt++) {
					status = 0x30 | ((dout >> bit_cnt) & 1) << 2;			//TDI
					(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
					(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
				}
				status = 0x30 | (((dout >> bit_cnt) & 1) << 2) | (1 << 1);
				(*(volatile unsigned int *)0x10010340) = status;
				(*(volatile unsigned int *)0x10010340) = status | 1;
			} else {
				if (type != 0x00100000) {
					for (bit_cnt = 0; bit_cnt < (scan_size-1); bit_cnt++) {
						status = 0x30 | ((dout >> bit_cnt) & 1) << 2;
						(*(volatile unsigned int *)0x10010340) = status;
						if((*(volatile unsigned int *)0x10010300) & 0x00000008)
							buffer = buffer | (1 << bit_cnt);
						(*(volatile unsigned int *)0x10010340) = status | 1;
					}
					status = 0x30 | (((dout >> bit_cnt) & 1) << 2) | (1 << 1);	//TDI
					(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
					if((*(volatile unsigned int *)0x10010300) & 0x00000008)
						buffer = buffer | (1 << bit_cnt);
					share_data = buffer;
					(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
				} else {
					for (bit_cnt = 0; bit_cnt < (scan_size-1); bit_cnt++) {
						(*(volatile unsigned int *)0x10010340) = 0x30;
						if((*(volatile unsigned int *)0x10010300) & 0x00000008)
							buffer = buffer | (1 << bit_cnt);
						(*(volatile unsigned int *)0x10010340) = 0x31;
					}
					(*(volatile unsigned int *)0x10010340) = 0x32;
					if((*(volatile unsigned int *)0x10010300) & 0x00000008)
						buffer = buffer | (1 << bit_cnt);
					share_data = buffer;
					(*(volatile unsigned int *)0x10010340) = 0x33;
				}
			}
			for (i = 1; i < 3; i++) {
				status = 0x30 | ((3 >> i) & 1) << 1;
				(*(volatile unsigned int *)0x10010340) = status;
				(*(volatile unsigned int *)0x10010340) = status | 1;
			}
			(*(volatile unsigned int *)0x10010340) = status;
		}
		if(dout & 0x04000000) {	//write for 84
//			0000  0000  0000  0000  0000  0000  0000  0000
//			FTAP  状态  类型   FTMS  次------数  数------据
			data = share_data2;
///////////////////////////////////////////////////////////////////////////
			type = (dout & 0x00f00000);
			if (__builtin_expect(type == 0x00200000, 1))
				share_data = 0;
 			for (i = 0; i < 4; i++) {
				status = 0x30 | ((3 >> i) & 1) << 1;
				(*(volatile unsigned int *)0x10010340) = status;
				(*(volatile unsigned int *)0x10010340) = status | 1;
			}
			for (bit_cnt = 0; bit_cnt < (5-1); bit_cnt++) {
				status = 0x30 | ((0x9 >> bit_cnt) & 1) << 2;			//TDI
				(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
				(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
			}
			status = 0x30 | (((0x9 >> 4) & 1) << 2) | (1 << 1);
			(*(volatile unsigned int *)0x10010340) = status;
			(*(volatile unsigned int *)0x10010340) = status | 1;
			for (i = 1; i < 3; i++) {
				status = 0x30 | ((3 >> i) & 1) << 1;
				(*(volatile unsigned int *)0x10010340) = status;
				(*(volatile unsigned int *)0x10010340) = status | 1;
			}
//			(*(volatile unsigned int *)0x10010340) = status;
			for (i = 0; i < 3; i++) {
				status = 0x30 | ((1 >> i) & 1) << 1;				//TMS
				(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
				(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
			}
			for (bit_cnt = 0; bit_cnt < 31; bit_cnt++) {
				status = 0x30 | (((data >> bit_cnt) & 1) << 2);
				(*(volatile unsigned int *)0x10010340) = status;
				(*(volatile unsigned int *)0x10010340) = status | 1;
			}
			status = 0x30 | (((data >> 31) & 1) << 2) | (1 << 1);
			(*(volatile unsigned int *)0x10010340) = status;
			(*(volatile unsigned int *)0x10010340) = status | 1;
			for (i = 1; i < 3; i++) {
				status = 0x30 | ((3 >> i) & 1) << 1;				//TMS
				(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
				(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
			}
//			(*(volatile unsigned int *)0x10010340) = status;			//设置TMS TDI,输出CLK0
			for (i = 0; i < 4; i++) {
				status = 0x30 | ((3 >> i) & 1) << 1;
				(*(volatile unsigned int *)0x10010340) = status;
				(*(volatile unsigned int *)0x10010340) = status | 1;
			}
			for (bit_cnt = 0; bit_cnt < (5-1); bit_cnt++) {
				status = 0x30 | ((0xA >> bit_cnt) & 1) << 2;			//TDI
				(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
				(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
			}
			status = 0x30 | (((0xA >> 4) & 1) << 2) | (1 << 1);
			(*(volatile unsigned int *)0x10010340) = status;
			(*(volatile unsigned int *)0x10010340) = status | 1;
			for (i = 1; i < 3; i++) {
				status = 0x30 | ((3 >> i) & 1) << 1;
				(*(volatile unsigned int *)0x10010340) = status;
				(*(volatile unsigned int *)0x10010340) = status | 1;
			}
//			(*(volatile unsigned int *)0x10010340) = status;
			for (i = 0; i < 3; i++) {
				status = 0x30 | ((1 >> i) & 1) << 1;				//TMS
				(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
				(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
			}
			for (bit_cnt = 0; bit_cnt < 31; bit_cnt++) {
				status = 0x30 | (((0x8000c000 >> bit_cnt) & 1) << 2);
				(*(volatile unsigned int *)0x10010340) = status;
				(*(volatile unsigned int *)0x10010340) = status | 1;
			}
			status = 0x30 | (((0x8000c000 >> 31) & 1) << 2) | (1 << 1);
			(*(volatile unsigned int *)0x10010340) = status;
			(*(volatile unsigned int *)0x10010340) = status | 1;
			for (i = 1; i < 3; i++) {
				status = 0x30 | ((3 >> i) & 1) << 1;				//TMS
				(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
				(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
			}
			(*(volatile unsigned int *)0x10010340) = status;			//设置TMS TDI,输出CLK0
/********************************************************************************************************************************/
		}
		if(dout & 0x02000000) {	//write for 32
//			0000  0000  0000  0000  0000  0000  0000  0000
//			FTAP  状态  类型   FTMS  次------数  数------据
			data = share_data2;
			buffer = 0x00000000;
			type = (dout & 0x00f00000);
			tms_flag = dout & 0x00030000;
			tap_flag = dout & 0x30000000;
			if (__builtin_expect(tap_flag == 0x10000000, 1)) {
				for (i = 0; i < 3; i++) {
					status = 0x30 | ((1 >> i) & 1) << 1;				//TMS
					(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
					(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
				}
			}
			if (__builtin_expect(tap_flag == 0x20000000, 0)) {
				for (i = 0; i < 7; i++) {
					status = 0x30 | ((23 >> i) & 1) << 1;				//TMS
					(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
					(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
				}
			}
			if (type == 0x00200000) {
				share_data = 0;
				for (bit_cnt = 0; bit_cnt < 31; bit_cnt++) {
					status = 0x30 | (((data >> bit_cnt) & 1) << 2);
					(*(volatile unsigned int *)0x10010340) = status;
					(*(volatile unsigned int *)0x10010340) = status | 1;
				}
				status = 0x30 | (((data >> 31) & 1) << 2) | (tms_flag ? (1 << 1) : 0);
				(*(volatile unsigned int *)0x10010340) = status;
				(*(volatile unsigned int *)0x10010340) = status | 1;
			} else {
				if (type != 0x00100000) {
					for (bit_cnt = 0; bit_cnt < 31; bit_cnt++) {
						status = 0x30 | (((data >> bit_cnt) & 1) << 2);
						(*(volatile unsigned int *)0x10010340) = status;
						if((*(volatile unsigned int *)0x10010300) & 0x00000008)
							buffer = buffer | (1 << bit_cnt);
						(*(volatile unsigned int *)0x10010340) = status | 1;
					}
					status = 0x30 | (((data >> 31) & 1) << 2) | (tms_flag ? (1 << 1) : 0);
					(*(volatile unsigned int *)0x10010340) = status;
					if((*(volatile unsigned int *)0x10010300) & 0x00000008)
						buffer = buffer | (1 << 31);
					share_data2 = buffer;
					share_data = 0;
					(*(volatile unsigned int *)0x10010340) = status | 1;
				} else {
					for (bit_cnt = 0; bit_cnt < 31; bit_cnt++) {
						(*(volatile unsigned int *)0x10010340) = 0x30;
						if((*(volatile unsigned int *)0x10010300) & 0x00000008)
							buffer = buffer | (1 << bit_cnt);
						(*(volatile unsigned int *)0x10010340) = 0x31;
					}
					status = 0x30 | (tms_flag ? (1 << 1) : 0);
					(*(volatile unsigned int *)0x10010340) = status;
					if((*(volatile unsigned int *)0x10010300) & 0x00000008)
						buffer = buffer | (1 << 31);
					share_data2 = buffer;
					share_data = 0;
					(*(volatile unsigned int *)0x10010340) = status | 1;
				}
			}
			if (__builtin_expect(tms_flag == 0x00010000, 1)) {
				for (i = 1; i < 3; i++) {
					status = 0x30 | ((3 >> i) & 1) << 1;				//TMS
					(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
					(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
				}
				(*(volatile unsigned int *)0x10010340) = status;			//设置TMS TDI,输出CLK0
			}
			if (__builtin_expect(tms_flag == 0x00020000, 0)) {
				for (i = 1; i < 2; i++) {
					status = 0x30 | ((1 >> i) & 1) << 1;				//TMS
					(*(volatile unsigned int *)0x10010340) = status;		//设置TMS TDI,输出CLK0
					(*(volatile unsigned int *)0x10010340) = status | 1;		//设置TMS TDI,输出CLK1
				}
				(*(volatile unsigned int *)0x10010340) = status;			//设置TMS TDI,输出CLK0
			}
		}
		if(__builtin_expect(dout & 0x01000000, 0)) {									//write TAP
//  			0000  0000  0000  0000  0000  0000  0000  0000
//  			      状态        skip   tms_scan    tms_count
			i = 0;
			share_data = 0;
			skip = (dout & 0x000f0000) >> 16;
			tms_scan = (dout & 0x0000ff00) >> 8;
			tms_count = (dout & 0x000000ff);
			for (i = skip; i < tms_count; i++) {
				status = 0x30 | ((tms_scan >> i) & 1) << 1;				//TMS
				(*(volatile unsigned int *)0x10010340) = status;			//设置TMS TDI,输出CLK0
				(*(volatile unsigned int *)0x10010340) = status | 1;			//设置TMS TDI,输出CLK1
			}
			(*(volatile unsigned int *)0x10010340) = status;				//设置TMS TDI,输出CLK0
		}
	}
	return 0;
}

