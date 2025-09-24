/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"
#include "hardware/spi.h"
#include "adxl382.h"
#include <errno.h>
#include "hardware/uart.h"

/* Example code to talk to a ADXL382 acceleromater sensor via SPI.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefore SPI) cannot be used at 5v.

   You will need to use a level shifter on the SPI lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board and a generic adxl382 board, other
   boards may vary.

   GPIO 16 (pin 21) MISO/spi0_rx-> SDO/SDO on adxl382 board
   GPIO 17 (pin 22) Chip select -> CSB/!CS on adxl382 board
   GPIO 18 (pin 24) SCK/spi0_sclk -> SCL/SCK on adxl382 board
   GPIO 19 (pin 25) M OSI/spi0_tx -> SDA/SDI on adxl382 board
   3.3v (pin 36) -> VS 3V3 pin on adxl382 board
   GND (pin 38)  -> GND on adxl382 board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.

*/

#define ADXL38X_FIFO_SIZE 318		   // Number of entries in the FIFO buffer. Not # of bytes.
#define ADXL38X_DATA_SIZE_WITH_CH 3	   // 16 bits of measurement +2 bits of channel ID
#define ADXL38X_DATA_SIZE_WITHOUT_CH 2 // 16 bits of measurement
#define NUM_AXES 3					   // X,Y,Z
#define SPI_CLK_MHZ 1000 * 8000		   // This example will use SPI0 at 4MHz
#define MAX_SEQUENTIAL_FIFO_READS 12   // Max allowed is ((SPI_CLK_MHZ/(16000))-8)/24 assuming ADXL38X_DATA_SIZE_WITH_CH

enum Fault_Codes
{
	NO_ERROR,
	SPI_COMM,
	FIFO_UNMATCH,
	BOOT_ERROR,
	SPI_PIN_DEF_ERROR,
	ACCEL_INIT_ERROR
};

#define DEBUG_PRINT(msg, ...)       \
	do                              \
	{                               \
		printf(msg, ##__VA_ARGS__); \
		fflush(stdout);             \
	} while (0)

#define STREAM_PRINT(...)    \
	do                       \
	{                        \
		printf(__VA_ARGS__); \
		fflush(stdout);      \
	} while (0)

uint8_t register_value;
uint8_t status_reg;
uint8_t fifo_status[2];
uint8_t fifo_data[ADXL38X_FIFO_SIZE * ADXL38X_DATA_SIZE_WITH_CH];
uint16_t set_fifo_queue_depth = 0x3C; //  60 entries in FIFO
// uint16_t set_fifo_queue_depth = 0x5A; //  90 entries in FIFO
uint16_t fifo_queue_depth = 2;
bool chID_enable = true; // FIFO channel id
uint8_t fifo_read_bytes;
uint32_t total_samples_read = 0;
struct adxl38x_fractional_val data_frac[15];
static char getaxis(uint8_t chID);
int pos = 0;

// Dplicate entire ADXL buffer in RAM w/ measurement #!
char serial_buf[ADXL38X_FIFO_SIZE * (4 + NUM_AXES * ADXL38X_DATA_SIZE_WITH_CH)];

// LED vars, enums, and structs
const uint16_t LED_PIN = PICO_DEFAULT_LED_PIN;
volatile uint8_t led_state = 0;

void set_led_state(uint8_t state)
{
	if (state)
	{
		led_state = 1;
		gpio_put(LED_PIN, 1);
	}
	else
	{
		gpio_put(LED_PIN, 0);
		led_state = 0;
	}
}

void toggle_led()
{
	set_led_state(!(led_state & 0x01));
}

int32_t fault_handler(int32_t error_code)
{
	set_led_state(0);
	switch (error_code)
	{
	case SPI_COMM:
		DEBUG_PRINT("Error: SPI Comm Error occurred!\n");
		break;

	case FIFO_UNMATCH:
		DEBUG_PRINT("Error: Number of entries in FIFO not matching the number set in FIFO config\n");
		break;

	case BOOT_ERROR:
		DEBUG_PRINT("Error durring boot.  Check chip config and connections.\n");
		break;
	case SPI_PIN_DEF_ERROR:
		DEBUG_PRINT("Error durring boot. One or more SPI pins not defined.\n");
		break;
	case -EAGAIN: // This came from the existing code base, not sure why it's done this way.
		DEBUG_PRINT("Error: ADX acceleromter reset was not successful\n");
		break;
	case ACCEL_INIT_ERROR:
		DEBUG_PRINT("Error: ADXL acceleromter could not be initialized\n");
		break;
	}
	if (error_code)
	{
		DEBUG_PRINT("Halting due to fault! Error code: %d, 0x%X\n", error_code, error_code);
		exit(EXIT_FAILURE);
	}
	else
		return (0);
}

// from https://github.com/raspberrypi/pico-examples/blob/master/clocks/hello_48MHz/hello_48MHz.c
void measure_freqs(void)
{
	uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
	uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
	uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
	uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
	uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
	uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
	uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
#ifdef CLOCKS_FC0_SRC_VALUE_CLK_RTC
	uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);
#endif

	printf("pll_sys  = %dkHz\n", f_pll_sys);
	printf("pll_usb  = %dkHz\n", f_pll_usb);
	printf("rosc     = %dkHz\n", f_rosc);
	printf("clk_sys  = %dkHz\n", f_clk_sys);
	printf("clk_peri = %dkHz\n", f_clk_peri);
	printf("clk_usb  = %dkHz\n", f_clk_usb);
	printf("clk_adc  = %dkHz\n", f_clk_adc);
#ifdef CLOCKS_FC0_SRC_VALUE_CLK_RTC
	printf("clk_rtc  = %dkHz\n", f_clk_rtc);
#endif

	// Can't measure clk_ref / xosc as it is the ref
}

int32_t setup_pi_pico()
{
	uint8_t fault_code = 0;

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	// Initialize UART0 with a baud rate of 9600
	/*uart_init(uart0, 9600);
	#gpio_set_function(0, GPIO_FUNC_UART); // TX
	gpio_set_function(1, GPIO_FUNC_UART); // RX

	// Set a new baud rate if needed
	uart_set_baudrate(uart0, 115200);*/
	stdio_init_all();
	sleep_ms(100);
	// Set the system clock to 200MHz
	if (set_sys_clock_khz(200000, true))
	{
		printf("Clock set to 200 MHz\n");
	}
	else
	{
		printf("Failed to set clock\n");
	}
	// Re init uart now that clk_peri has changed
	stdio_init_all();
	sleep_ms(100);

#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/adxl382_spi example requires a board with SPI pins
	fault_code = SPI_PIN_DEF_ERROR;
#endif
	if (fault_code)
	{
		fault_handler(SPI_PIN_DEF_ERROR);
	}
	else
	{

		DEBUG_PRINT("Hello, adxl382! Reading raw data from registers via SPI...\n");
		spi_init(spi_default, SPI_CLK_MHZ);
		// Set SPI format
		spi_set_format(spi0, // SPI instance
					   8,	 // Number of bits per transfer
					   0,	 // Polarity (CPOL)
					   0,	 // Phase (CPHA)
					   SPI_MSB_FIRST);
		gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
		gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
		gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
		// Make the SPI pins available to picotool
		bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

		// Chip select is active-low, so we'll initialise it to a driven-high state
		gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
		gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
		gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
		// Make the CS pin available to picotool
		bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

		if (spi_is_writable(spi_default))
		{
			DEBUG_PRINT("SPI is writable\n");
		}
		sleep_ms(100);
	}
	return (fault_code);
}

int32_t config_accelerometer()
{
	uint8_t fault_code = 0;

	// Also puts the part in standby mode, All configuration register writes must be completed with the ADXL382 in standby mode
	fault_code = adxl38x_init();
	// Reset Accelerometer
	fault_code = adxl38x_soft_reset();

	// Set Accelerometer measurement range
	if (!fault_code)
		fault_code = adxl38x_set_range(ADXL38X_OP_MODE, ADXL38X_MASK_RANGE, ADXL382_RANGE_15G);
	if (!fault_code)
		DEBUG_PRINT("ADXL382 range is set to 15g\n");

	// FIFO sequence

	// Set DIG_EN register to 0x78 (Enable XYZ axes and FIFO enable)
	if (!fault_code)
	{
		register_value = 0x78;
		fault_code = write_register(ADXL38X_DIG_EN, &register_value, 1);
	}
	if (!fault_code)
		DEBUG_PRINT("Enable XYZ axes and FIFO\n");

	// Set FIFO_CFG0 to 0x60 (Channel ID enable and FIFO stream mode)
	if (!fault_code)
	{
		fault_code = adxl38x_accel_set_FIFO(set_fifo_queue_depth,
											false, ADXL38X_FIFO_STREAM, true, false);
	}
	if (!fault_code)
		DEBUG_PRINT("Set FIFO_CFG0 to 0x%X (Channel ID enable and FIFO stream mode, set FIFO_SAMPLES to %d)\n", set_fifo_queue_depth, set_fifo_queue_depth);

	// Set INT0_MAP0 to 0x08 (FIFO_WATERMARK_INT0)
	if (!fault_code)
	{
		register_value = 0x08;
		fault_code = write_register(ADXL38X_INT0_MAP0, &register_value, 1);
	}
	if (!fault_code)
		DEBUG_PRINT("Set INT0_MAP0 to 0x08 (FIFO_WATERMARK_INT0)\n");

	// Put the part in HP mode and read data when FIFO watermark pin is set
	if (!fault_code)
	{
		fault_code = adxl38x_set_op_mode(ADXL38X_OP_MODE, ADXL38X_MASK_OP_MODE, ADXL38X_MODE_HP);
	}
	if (!fault_code)
	{
		DEBUG_PRINT("Device is in HP mode\n");
		// WAIT 500ms after going into HP mode.
		sleep_ms(500);
		// Set the number of bytes to read per sample
		if (chID_enable)
			fifo_read_bytes = 3;
		else
			fifo_read_bytes = 2;
	}
	return (fault_code);
}

void fifo_data_to_readable_string(uint8_t *adxl_data, uint8_t *ser_buf, uint32_t num_entries, uint32_t num_entries_init_val)
{
	uint32_t i, j;
	uint32_t buff_idx = 0;
	uint32_t temp_idx = 0;

	for (i = 0; i < num_entries / NUM_AXES; i++)
	{
		buff_idx += sprintf(ser_buf + buff_idx, "#%08X", num_entries_init_val + i, buff_idx);
		for (j = 0; j < NUM_AXES; j++)
		{
			temp_idx = (i * NUM_AXES + j) * fifo_read_bytes;
			buff_idx += sprintf(ser_buf + buff_idx, ", %c%02X%02X", getaxis(adxl_data[temp_idx]), adxl_data[temp_idx + 1], adxl_data[temp_idx + 2], buff_idx);
		}
		buff_idx += sprintf(ser_buf + buff_idx, "\n", buff_idx);
	}
	ser_buf[buff_idx] = '\0';
}

uint32_t fifo_data_to_data_stream(uint8_t *adxl_data, uint8_t *ser_buf, uint32_t num_entries, uint32_t num_entries_init_val)
{
	uint32_t i, j;
	uint32_t buff_idx = 0;
	uint32_t temp_idx = 0;

	// One "sample" is data from all the axes, so read all axes, then increment counter.
	for (i = 0; i < num_entries / NUM_AXES; i++)
	{
		uint32_t count = num_entries_init_val + i;
		memcpy(ser_buf + buff_idx, &count, sizeof(count));
		buff_idx += sizeof(count);
		for (j = 0; j < NUM_AXES; j++)
		{
			temp_idx = (i * NUM_AXES + j) * fifo_read_bytes;
			memcpy(ser_buf + buff_idx, adxl_data + temp_idx, fifo_read_bytes);
			buff_idx += fifo_read_bytes;
		}
	}
	return (buff_idx);
}

int main()
{
	int32_t flt_code = 0;
	uint32_t count = 0;

	if (setup_pi_pico())
	{
		fault_handler(BOOT_ERROR);
	};
	set_led_state(1);

	// Wait for USB.  Flash LED.
	while (!stdio_usb_connected())
	{
		DEBUG_PRINT("USB not connected yet\n");
		sleep_ms(100);
		toggle_led();
	}
	set_led_state(1); // Ensure the LED is on after an arbitrary number of toggles while waiting for USB.
	DEBUG_PRINT("USB port is successfully initialised\n");

	flt_code = config_accelerometer();
	sleep_ms(10);
	if (flt_code)
		DEBUG_PRINT("ADXL is successfully initialised\n");
	else
		fault_handler(flt_code);

	// reset count
	count = 0;
	while (true)
	{
		set_led_state(1);

		// DEBUG_PRINT("Starting watermark check\n");

		// Read status to determine if FIFO_WATERMARK bit set
		flt_code = read_register(ADXL38X_STATUS0, 1, &status_reg);
		if (flt_code)
			fault_handler(SPI_COMM);

		if (status_reg & (1 << 3))
		{
			// Read FIFO status and data if FIFO_WATERMARK is set
			do
			{
				flt_code = read_register(ADXL38X_FIFO_STATUS0, 2, fifo_status);
				if (flt_code)
					fault_handler(SPI_COMM);
				fifo_queue_depth = (fifo_status[0] | ((uint16_t)fifo_status[1] << 8));
				fifo_queue_depth = fifo_queue_depth & 0x01ff;
				// DEBUG_PRINT("Fifo entries =  %d\n", fifo_queue_depth);

				// clear fifo_data buffer
				// memset(fifo_data, 0, sizeof(fifo_data));

				// set how many datapoints to read based on clockrate
				uint32_t num_entries_to_read = 0;
				if (fifo_queue_depth > MAX_SEQUENTIAL_FIFO_READS)
				{
					num_entries_to_read = MAX_SEQUENTIAL_FIFO_READS;
				}
				else
					num_entries_to_read = fifo_queue_depth;

				// DEBUG_PRINT("Reading %u entries", num_entries_to_read);
				//  wait for the measurement in progress to finish
				do
				{
					flt_code = read_register(ADXL38X_STATUS3, 1, &status_reg);
					if (flt_code)
						fault_handler(SPI_COMM);
				} while (!(status_reg & (0x01)));

				// read the data & process to USB-->UART
				flt_code = read_register(ADXL38X_FIFO_DATA, num_entries_to_read * fifo_read_bytes, fifo_data);
				if (flt_code)
					fault_handler(SPI_COMM);
				else
				{
					// fifo_data_to_readable_string(fifo_data, serial_buf, set_fifo_queue_depth, total_samples_read);
					// DEBUG_PRINT("%s", serial_buf);
					uint32_t bytes_to_write = fifo_data_to_data_stream(fifo_data, serial_buf, num_entries_to_read, total_samples_read);
					fwrite(serial_buf, sizeof(serial_buf[0]), bytes_to_write - 1, stdout); // not sure why bytes_to_write-1 is needed, but otherwise I get an extra byte written
					fflush(stdout);
					// Update counters
					total_samples_read += num_entries_to_read;
					fifo_queue_depth -= num_entries_to_read;
				}
			} while (fifo_queue_depth > 0);
		}
		else
			sleep_us(5);
	}
	DEBUG_PRINT("End\n");
	set_led_state(0);
}

/***************************************************************************/
/**
 * @brief Assigns axis based on channel index
 *
 * @param chID         - Channel index
 *
 * @return ret         - Corresponding channel ID for channel index provided
 *******************************************************************************/
static char getaxis(uint8_t chID)
{
	if (chID)
		return chID > 1 ? 'z' : 'y';
	return 'x';
}
