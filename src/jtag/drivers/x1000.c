/***************************************************************************
 *   Copyright (C) 2017 by Zhou Yanjie, zhou_yan_jie@163.com               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "bitbang.h"

#include <sys/mman.h>

#define GPIO_BASE	(0x10010000) /* GPIO controller */

#define PBPIN		(*(pio_base+0x100/4))
#define PBINT		(*(pio_base+0x110/4))
#define PBINTS		(*(pio_base+0x114/4))
#define PBINTC		(*(pio_base+0x118/4))
#define PBMSK		(*(pio_base+0x120/4))
#define PBMSKS		(*(pio_base+0x124/4))
#define PBMSKC		(*(pio_base+0x128/4))
#define PBPAT1		(*(pio_base+0x130/4))
#define PBPAT1S		(*(pio_base+0x134/4))
#define PBPAT1C		(*(pio_base+0x138/4))
#define PBPAT0		(*(pio_base+0x140/4))
#define PBPAT0S		(*(pio_base+0x144/4))
#define PBPAT0C		(*(pio_base+0x148/4))

#define PDPIN		(*(pio_base+0x300/4))
#define PDINT		(*(pio_base+0x310/4))
#define PDINTS		(*(pio_base+0x314/4))
#define PDINTC		(*(pio_base+0x318/4))
#define PDMSK		(*(pio_base+0x320/4))
#define PDMSKS		(*(pio_base+0x324/4))
#define PDMSKC		(*(pio_base+0x328/4))
#define PDPAT1		(*(pio_base+0x330/4))
#define PDPAT1S		(*(pio_base+0x334/4))
#define PDPAT1C		(*(pio_base+0x338/4))
#define PDPAT0		(*(pio_base+0x340/4))
#define PDPAT0S		(*(pio_base+0x344/4))
#define PDPAT0C		(*(pio_base+0x348/4))

#define PZINTS		(*(pio_base+0x714/4))
#define PZINTC		(*(pio_base+0x718/4))
#define PZMSKS		(*(pio_base+0x724/4))
#define PZMSKC		(*(pio_base+0x728/4))
#define PZPAT1S		(*(pio_base+0x734/4))
#define PZPAT1C		(*(pio_base+0x738/4))
#define PZPAT0S		(*(pio_base+0x744/4))
#define PZPAT0C		(*(pio_base+0x748/4))
#define PZGID2LD	(*(pio_base+0x7f0/4))

static int dev_mem_fd;
static volatile uint32_t *pio_base;

static int x1000_read(void);
static void x1000_write(int tck, int tms, int tdi);
static void x1000_reset(int trst, int srst);

static int x1000_init(void);
static int x1000_quit(void);

/* gpio numbers for each gpio. Negative values are invalid */
static int tck_gpio = -1;
static int tms_gpio = -1;
static int tdi_gpio = -1;
static int tdo_gpio = -1;
static int trst_gpio = -1;
static int srst_gpio = -1;
static int led_gpio = -1;

/* Transition delay coefficients */
static int speed_coeff = 113714;
static int speed_offset = 28;
static int port_status;
static unsigned int jtag_delay;

static int x1000_read(void)
{
	return !!(PDPIN & 1<<tdo_gpio);
}

static void x1000_write(int tck, int tms, int tdi)
{
	//uint32_t set = tck<<tck_gpio | tms<<tms_gpio | tdi<<tdi_gpio;		//mod
	//uint32_t clear = !tck<<tck_gpio | !tms<<tms_gpio | !tdi<<tdi_gpio;	//mod
	port_status = (port_status & ~(1<<tck_gpio | 1<<tms_gpio | 1<<tdi_gpio)) | tck<<tck_gpio | tms<<tms_gpio | tdi<<tdi_gpio;

	//PDPAT0S = set;							//mod
	//PDPAT0C = clear;							//mod
	PDPAT0 = port_status;

	for (unsigned int i = 0; i < jtag_delay; i++)				//mod
		asm volatile ("");						//mod
}										//mod

/* (1) assert or (0) deassert reset lines */
static void x1000_reset(int trst, int srst)					//mod
{										//mod
//	uint32_t set = 0;							//mod
//	uint32_t clear = 0;							//mod
										//mod
	if (trst_gpio > 0) {							//mod
		//set |= !trst<<trst_gpio;					//mod
		//clear |= trst<<trst_gpio;					//mod
		port_status = (port_status & ~(1<<trst_gpio)) | !trst<<trst_gpio;
	}									//mod

	if (srst_gpio > 0) {							//mod
		//set |= !srst<<srst_gpio;					//mod
		//clear |= srst<<srst_gpio;					//mod
		port_status = (port_status & ~(1<<srst_gpio)) | !srst<<srst_gpio;
	}									//mod

	//PDPAT0S = set;							//mod
	//PDPAT0C = clear;							//mod
	PDPAT0 = port_status;
}										//mod

static void x1000_led(int on)
{
	if (on)
		PBPAT0S = 1<<led_gpio;
	else
		PBPAT0C = 1<<led_gpio;
}

static int x1000_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = speed_coeff/khz - speed_offset;
	if (*jtag_speed < 0)
		*jtag_speed = 0;
	return ERROR_OK;
}

static int x1000_speed_div(int speed, int *khz)
{
	*khz = speed_coeff/(speed + speed_offset);
	return ERROR_OK;
}

static int x1000_speed(int speed)
{
	jtag_delay = speed;
	printf("jtag_delay = %08X\n",jtag_delay);
	return ERROR_OK;
}

static int is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio <= 32;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tms_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], tdi_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD_CTX,
			"GPIO config: tck = %d, tms = %d, tdi = %d, tdi = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD_CTX, "X1000 num: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD_CTX, "X1000 num: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD_CTX, "X1000 num: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD_CTX, "X1000 num: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD_CTX, "X1000 num: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD_CTX, "X1000 num: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_jtag_gpionum_led)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], led_gpio);

	command_print(CMD_CTX, "X1000 num: led = %d", led_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(x1000_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}
	return ERROR_OK;
}

static const struct command_registration x1000_command_handlers[] = {
	{
		.name = "x1000_jtag_nums",
		.handler = &x1000_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "(tck tms tdi tdo)* ",
	},
	{
		.name = "x1000_tck_num",
		.handler = &x1000_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
	},
	{
		.name = "x1000_tms_num",
		.handler = &x1000_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
	},
	{
		.name = "x1000_tdo_num",
		.handler = &x1000_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
	},
	{
		.name = "x1000_tdi_num",
		.handler = &x1000_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
	},
	{
		.name = "x1000_srst_num",
		.handler = &x1000_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
	},
	{
		.name = "x1000_trst_num",
		.handler = &x1000_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
	},
	{
		.name = "x1000_led_num",
		.handler = &x1000_handle_jtag_gpionum_led,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for led.",
	},
	{
		.name = "x1000_speed_coeffs",
		.handler = &x1000_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
	},
	COMMAND_REGISTRATION_DONE
};

static struct bitbang_interface x1000_bitbang = {
	.read = x1000_read,
	.write = x1000_write,
	.reset = x1000_reset,
	.blink = x1000_led,
};

struct jtag_interface x1000_interface = {
	.name = "x1000",
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
	.transports = jtag_only,
	.speed = x1000_speed,
	.khz = x1000_khz,
	.speed_div = x1000_speed_div,
	.commands = x1000_command_handlers,
	.init = x1000_init,
	.quit = x1000_quit,
};

static int x1000_init(void)							//mod
{
	bitbang_interface = &x1000_bitbang;					//mod

	if (!is_gpio_valid(tdo_gpio) || !is_gpio_valid(tdi_gpio) ||		//mod
		!is_gpio_valid(tck_gpio) || !is_gpio_valid(tms_gpio) ||		//mod
		(trst_gpio != -1 && !is_gpio_valid(trst_gpio)) ||		//mod
		(srst_gpio != -1 && !is_gpio_valid(srst_gpio)) ||
		(led_gpio != -1 && !is_gpio_valid(led_gpio)))			//mod
		return ERROR_JTAG_INIT_FAILED;					//mod

	dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);				//mod
	if (dev_mem_fd < 0) {							//mod
		perror("open");							//mod
		return ERROR_JTAG_INIT_FAILED;					//mod
	}									//mod

	pio_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,	//mod
				MAP_SHARED, dev_mem_fd, 0x10010000);		//mod

	if (pio_base == MAP_FAILED) {						//mod
		perror("mmap");							//mod
		close(dev_mem_fd);						//mod
		return ERROR_JTAG_INIT_FAILED;					//mod
	}									//mod

	PZINTC = 1<<tdo_gpio | 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;		//mod
	PZMSKS = 1<<tdo_gpio | 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;		//mod
	PZPAT1S = 1<<tdo_gpio;							//mod
	PZPAT1C = 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;			//mod
	PZPAT0S = 1<<tms_gpio;							//mod
	PZPAT0C = 1<<tdi_gpio | 1<<tck_gpio;					//mod
	PZGID2LD = 0x3;								//mod

	if (trst_gpio != -1) {							//mod
		PZINTC = 1 << trst_gpio;					//mod
		PZMSKS = 1 << trst_gpio;					//mod
		PZPAT1C = 1 << trst_gpio;					//mod
		PZPAT0S = 1 << trst_gpio;					//mod
		PZGID2LD = 0x3;							//mod
	}									//mod
	if (srst_gpio != -1) {							//mod
		PZINTC = 1 << srst_gpio;					//mod
		PZMSKS = 1 << srst_gpio;					//mod
		PZPAT1C = 1 << srst_gpio;					//mod
		PZPAT0S = 1 << srst_gpio;					//mod
		PZGID2LD = 0x3;							//mod
	}									//mod
	if (led_gpio != -1) {
		PZINTC = 1 << led_gpio;
		PZMSKS = 1 << led_gpio;
		PZPAT1C = 1 << led_gpio;
		PZPAT0C = 1 << led_gpio;
		PZGID2LD = 0x1;
	}

	port_status = PDPAT0;

	LOG_INFO("GPIO JTAG bitbang driver");

	printf("port_status = %08X\n",port_status);

	return ERROR_OK;
}

static int x1000_quit(void)							//mod
{										//mod
	PZINTC = 1<<tdo_gpio | 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;		//mod
	PZMSKS = 1<<tdo_gpio | 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;		//mod
	PZPAT1S = 1<<tdo_gpio;							//mod
	PZPAT1C = 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;			//mod
	PZPAT0C = 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;			//mod
	PZGID2LD = 0x3;								//mod

	if (trst_gpio != -1) {							//mod
		PZINTC = 1 << trst_gpio;					//mod
		PZMSKS = 1 << trst_gpio;					//mod
		PZPAT1C = 1 << trst_gpio;					//mod
		PZPAT0C = 1 << trst_gpio;					//mod
		PZGID2LD = 0x3;							//mod
	}									//mod
	if (srst_gpio != -1) {							//mod
		PZINTC = 1 << srst_gpio;					//mod
		PZMSKS = 1 << srst_gpio;					//mod
		PZPAT1C = 1 << srst_gpio;					//mod
		PZPAT0C = 1 << srst_gpio;					//mod
		PZGID2LD = 0x3;							//mod
	}									//mod
	if (led_gpio != -1) {
		PZINTC = 1 << led_gpio;
		PZMSKS = 1 << led_gpio;
		PZPAT1C = 1 << led_gpio;
		PZPAT0C = 1 << led_gpio;
		PZGID2LD = 0x1;
	}

	return ERROR_OK;
}

