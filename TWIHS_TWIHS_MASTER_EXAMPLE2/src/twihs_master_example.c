/**
 * \file
 *
 * \brief TWIHS MASTER Example for SAM.
 *
 * Copyright (c) 2015-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage TWI MASTER Example
 *
 * \section intro Introduction
 *
 * The application demonstrates how to use the SAM TWIHS driver to access an
 * external serial EEPROM chip.
 *
 * \section Requirements
 *
 * This package can be used with the following setup:
 *  - SAMV71 Xplained Ultra kit
 *  - SAME70 Xplained kit
 *
 * \section files Main files:
 *  - twihs.c SAM Two-Wire Interface driver implementation.
 *  - twihs.h SAM Two-Wire Interface driver definitions.
 *  - TWIHS_master_example.c Example application.
 *
 * \section exampledescription Description of the Example
 * Upon startup, the program configures PIOs for console UART, LEDs and TWIHS
 * connected to EEPROM on board. Then it configures the TWIHS driver and data
 * package. The clock of I2C bus is set as 400kHz.
 * After initializing the master mode, the example sends test pattern to the
 * EEPROM. When sending is complete, TWIHS driver reads the memory and saves the
 * content in the reception buffer. Then the program compares the content
 * received with the test pattern sent before and prints the comparison result.
 * The corresponding LED is turned on.
 *
 * On SAM Xplained Pro, the EEPROM is simulated though the TWIHS_slave_example,
 * which means 2 SAM Xplained Pro boards are required to run this example.
 * One board executes this example as the twi master and another runs
 * TWIHS_salve_example as the simulated EEPROM.
 * The TWI PINs and GND PIN should be connected between these 2 boards.
 * More information of the simulated EEPROM implementation can be referred in
 * the description of TWIHS_slave_example.
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR EWARM.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 * Support and FAQ: http://www.atmel.com/design-support/
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "led.h"
#include "BNO055.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/** Wait Time */
#define WAIT_TIME   10
/** TWI Bus Clock 400kHz */
#define TWIHS_CLK     400000

enum setup_bno055
{
	ndof,
	amg
	}register_value;

enum registr_write{
	opr
	}registr_w;

enum registr_read{
	temp,
	pitch_m,
	pitch_l,
	roll_m,
	roll_l,
	heading_m,
	heading_l
	}registr_r;

static const uint8_t data_tx[] = {NDOF_MODE, AMG_MODE};

/** Reception buffer */
static int8_t data_rx[7] = { 0 };
	
static uint8_t register_bno055[] = {OPR_MODE, TEMP, EUL_Pitch_MSB, \
	EUL_Pitch_LSB, EUL_Roll_MSB, EUL_Roll_LSB, EUL_Heading_MSB,\
	EUL_Heading_LSB, GYR_DATA_X_LSB, ACC_DATA_X_LSB};

/** Global timestamp in milliseconds since start of application */
volatile uint32_t g_ul_ms_ticks = 0;

/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 *  increments the timestamp counter.
 */
void SysTick_Handler(void)
{
	g_ul_ms_ticks++;
}

/**
 *  \brief Configure the Console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 *  \brief Wait for the given number of milliseconds (using the dwTimeStamp
 *         generated by the SAM microcontrollers' system tick).
 *  \param ul_dly_ticks  Delay to wait for, in milliseconds.
 */
static void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}


void setup_BNO055(uint8_t mode, uint8_t data, twihs_packet_t point);
void read_BNO055(uint8_t mode, uint8_t data, twihs_packet_t point);

int main(void)
{
	uint32_t i, flag_rx_comp;
	twihs_options_t opt;
	twihs_packet_t packet_tx1, packet_rx1;
	int16_t Temp, Pitch, Roll, Heading;
	
	/* Initialize the SAM system */
	sysclk_init();

	/* Initialize the board */
	board_init();

	/* Turn off LEDs */
	LED_Off(LED0);

    /* Initialize the console UART */
	configure_console();

	/* Configure systick for 1 ms */
	puts("Configure system tick to get 1ms tick period.\r");
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-E- Systick configuration error\r");
		while (1) {
			/* Capture error */
		}
	}

	/* Enable the peripheral clock for TWI */
	pmc_enable_periph_clk(ID_TWIHS0);

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
	opt.speed      = TWIHS_CLK;

	if (twihs_master_init(TWIHS0, &opt) != TWIHS_SUCCESS) {
		puts("-E-\tTWI master initialization failed.\r");
		while (1) {
			/* Capture error */
		}
	}
	
	registr_w = opr;
	register_value = ndof;
	
	setup_BNO055((uint8_t)registr_w, (uint8_t)register_value, packet_tx1);

	/* Wait at least 10 ms */
	
	mdelay(WAIT_TIME);
	
	registr_r = temp;
	i = 0;
	flag_rx_comp = 0;
	
	while(1)
	{
		if(registr_r <= heading_l)
		{
			read_BNO055(registr_r, i, packet_rx1);
			registr_r++;
			i++;
		}
		else 
		{
			flag_rx_comp = 1;
			registr_r = temp;
			i = 0;		
		}
		
		if(flag_rx_comp)
		{
			Temp = data_rx[temp];
			Pitch = ((data_rx[pitch_m]<<8)|(data_rx[pitch_l]))/16;
			Roll = ((data_rx[roll_m]<<8)|(data_rx[roll_l]))/16;
			Heading = ((data_rx[heading_m]<<8)|(data_rx[heading_l]))/16;
			flag_rx_comp = 0;		
		}
	}


	/* Match */
	puts("Data comparison:\tMatched!\r");
	LED_On(LED0);
	while (1) {
	}
}


void setup_BNO055(uint8_t mode, uint8_t data, twihs_packet_t point)
{
	
//	if(mode );
	point.chip        = ADDRESS_BNO055;
	point.addr[0]     = register_bno055[mode]; //>> 8;
	//	packet_tx.addr[1]     = EEPROM_MEM_ADDR;
	point.addr_length = 0x01;
	point.buffer      = &data_tx[data];
	point.length      = 0x01;
	
	/* Send test pattern to EEPROM */
	if (twihs_master_write(TWIHS0, &point) != TWIHS_SUCCESS) {
		puts("-E-\tTWI master write packet failed.\r");
		while (1) {
			/* Capture error */
		}
	}
}


void read_BNO055(uint8_t mode, uint8_t data, twihs_packet_t point_rx)
{
	
	//	if(mode );
	point_rx.chip        = ADDRESS_BNO055;
	point_rx.addr[0]     = register_bno055[mode];
	//	packet_rx.addr[1]     = packet_tx.addr[1];
	point_rx.addr_length = 0x01;
	point_rx.buffer      = &data_rx[data];
	point_rx.length      = 0x01;
	
	/* Send test pattern to EEPROM */
	if (twihs_master_read(TWIHS0, &point_rx) != TWIHS_SUCCESS) {
		puts("-E-\tTWI master write packet failed.\r");
		while (1) {
			/* Capture error */
		}
	}
}


/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
