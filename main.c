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
#include "pico/critical_section.h"

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
#define FIFO_DATA_BUFFER_SIZE ADXL38X_FIFO_SIZE *ADXL38X_DATA_SIZE_WITH_CH
#define UART_BUF_SIZE 1024 * 2 // Lots of RAM, FIFO is only 320 max of 3 byte entries.  This gives lots of room for any overhead to make it human readable.
#define NUM_UART_BUFFERS 2

enum Fault_Codes
{
	NO_ERROR,
	SPI_COMM,
	FIFO_UNMATCH,
	BOOT_ERROR,
	SPI_PIN_DEF_ERROR,
	ACCEL_INIT_ERROR,
	FIFO_OVERFLOW
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

// A single buffer
typedef struct
{
	uint8_t *data;		 // pointer to memory
	size_t size;		 // total capacity
	size_t num_elements; // number of elements in buffer
} Buffer_t;

// Double buffer wrapper
typedef struct
{
	Buffer_t buf[NUM_UART_BUFFERS]; // 2 or more buffers
	uint8_t active;					// index of the buffer (0 or 1)
} DoubleBuffer_t;

uint8_t register_value;
uint8_t status_reg;
uint8_t fifo_status[2];
uint8_t fifo_data[FIFO_DATA_BUFFER_SIZE]; // Can fit max possible size
uint16_t set_fifo_queue_depth = MAX_SEQUENTIAL_FIFO_READS;

DoubleBuffer_t serialBuffers;
uint8_t uart_buff0[UART_BUF_SIZE];
uint8_t uart_buff1[UART_BUF_SIZE];

// LED vars, enums, and structs
const uint16_t LED_PIN = PICO_DEFAULT_LED_PIN;
volatile uint8_t led_state = 0;

// For pausing ints during critical sections.
critical_section_t my_critical_section;

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

// initialize a buffer
void buffer_init(Buffer_t *b, uint8_t *storage, size_t size)
{
	b->data = storage;
	b->size = size;
	b->num_elements = 0;
}

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

	critical_section_init(&my_critical_section);
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
	set_sys_clock_khz(200000, true);
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
		sleep_ms(100);
	}
}

uint32_t clear_data_ready_flag(void)
// reads the high and low byte data registers for all enabled axes to clear the Data ready flag.
// The data should already be in the FIFO, so can throw this away
{
	uint8_t junk_buf[NUM_AXES * ADXL38X_DATA_SIZE_WITH_CH];
	int32_t flt_code;

	// Address will show as 0x2B on logic analyzer due to R/W bit.
	flt_code = read_register(ADXL38X_XDATA_H, NUM_AXES * 2, junk_buf);
	return (flt_code);
}

uint32_t empty_fifo_buffer(void)
// reads the entire FIFO buffer and then reads the status byte to clear the FIFO flags.
// TODO: ensure the buffer is pointing to first axis.  I have seen some cases where the next data read is from axis 2.
{
	uint8_t junk_buf[NUM_AXES * ADXL38X_DATA_SIZE_WITH_CH];
	int32_t flt_code;

	// Empty the buffer
	flt_code = read_register(ADXL38X_FIFO_DATA, ADXL38X_FIFO_SIZE * ADXL38X_DATA_SIZE_WITH_CH, fifo_data);
	// Read status to clear full bit. Address will show as 0x23 on logic analyzer due to R/W bit.
	flt_code = read_register(ADXL38X_STATUS0, 1, &status_reg);
	return (flt_code);
}

void fifo_data_to_readable_string(uint8_t *adxl_data, Buffer_t *ser_buf, uint32_t num_entries, uint32_t num_entries_init_val)
{
	uint32_t i, j;
	uint32_t buff_idx = 0;
	uint32_t temp_idx = 0;

	ser_buf->num_elements = 0;
	for (i = 0; i < num_entries / NUM_AXES; i++)
	{
		buff_idx = ser_buf->num_elements;
		ser_buf->num_elements += sprintf((ser_buf->data) + buff_idx, "#%08X", num_entries_init_val + i, buff_idx); // TODO this is no longer correct.  Need to fix the idea of a sample vs an entry to resolve this.
		for (j = 0; j < NUM_AXES; j++)
		{
			temp_idx = (i * NUM_AXES + j) * ADXL38X_DATA_SIZE_WITH_CH;
			buff_idx = ser_buf->num_elements;
			ser_buf->num_elements += sprintf((ser_buf->data) + buff_idx, ", %c%02X%02X", getaxis(adxl_data[temp_idx]), adxl_data[temp_idx + 1], adxl_data[temp_idx + 2], buff_idx);
		}
		ser_buf->num_elements += sprintf((ser_buf->data) + ser_buf->num_elements, "\n", ser_buf->num_elements);
	}
	ser_buf->data[ser_buf->num_elements] = '\0';
	ser_buf->num_elements++;
}

uint32_t fifo_data_to_data_stream(uint8_t *adxl_data, Buffer_t *ser_buf, uint32_t num_entries, uint32_t num_entries_init_val)
{
	uint32_t i;
	uint32_t buff_idx = 0;
	uint32_t temp_idx = 0;

	// One "sample" is data from all the axes, so read all axes, then put the entry counter
	// Can't guarantee we will get an integer multiple of NUM_AXES, so need to use the axes ID to determine the start of a "sample"
	ser_buf->num_elements = 0;
	for (i = 0; i < num_entries; i++)
	{
		if (adxl_data[i * ADXL38X_DATA_SIZE_WITH_CH] == 0)
		{
			uint32_t count = num_entries_init_val + i;
			memcpy((ser_buf->data) + ser_buf->num_elements, &count, sizeof(count));
			ser_buf->num_elements += sizeof(count);
		}
		memcpy((ser_buf->data) + ser_buf->num_elements, adxl_data + i * ADXL38X_DATA_SIZE_WITH_CH, ADXL38X_DATA_SIZE_WITH_CH);
		ser_buf->num_elements += ADXL38X_DATA_SIZE_WITH_CH;
	}
	return (ser_buf->num_elements);
}

int main()
{
	int32_t flt_code = 0;
	uint32_t total_samples_read = 0;
	uint16_t fifo_queue_depth;

	if (setup_pi_pico())
	{
		fault_handler(BOOT_ERROR);
	};
	set_led_state(1);

	// init double buffer for USB/UART interface
	buffer_init(&serialBuffers.buf[0], uart_buff0, UART_BUF_SIZE);
	buffer_init(&serialBuffers.buf[1], uart_buff1, UART_BUF_SIZE);
	serialBuffers.active = 0;

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
	empty_fifo_buffer();
	clear_data_ready_flag();
	if (flt_code) // TODO fix this.  It doesn't print, so I don't think this catches errors correctly.
		DEBUG_PRINT("ADXL is successfully initialised\n");
	else
		fault_handler(flt_code);

	// reset count
	total_samples_read = 0;
	// DEBUG_PRINT("Starting normal operation check\n");
	set_led_state(1);
	while (true)
	{
		// Read status to determine if FIFO_WATERMARK bit set. Address will show as 0x23 on logic analyzer due to R/W bit.
		flt_code = read_register(ADXL38X_STATUS0, 1, &status_reg);
		if (flt_code)
			fault_handler(SPI_COMM);

		if (status_reg & (1 << 1))
		{
			// DEBUG_PRINT("Fifo OVFLW");
			fault_handler(FIFO_OVERFLOW);
		}
		if (status_reg & (1 << 3))
		{
			// Read FIFO status and data if FIFO_WATERMARK is set
			do
			{
				flt_code = read_register(ADXL38X_FIFO_STATUS0, 2, fifo_status); // Address will show as 0x3D on logic analyzer due to R/W bit.
				if (flt_code)
					fault_handler(SPI_COMM);
				fifo_queue_depth = (fifo_status[0] | ((uint16_t)fifo_status[1] << 8));
				fifo_queue_depth = fifo_queue_depth & 0x01ff;
				// DEBUG_PRINT("Fifo entries =  %d\n", fifo_queue_depth);
				//  check if we read the number of FIFO entries wrong.
				if (fifo_queue_depth > ADXL38X_FIFO_SIZE)
					fault_handler(FIFO_UNMATCH);

				// set how many datapoints to read based on clockrate
				uint32_t num_entries_to_read = 0;
				if (fifo_queue_depth > MAX_SEQUENTIAL_FIFO_READS)
				{
					num_entries_to_read = MAX_SEQUENTIAL_FIFO_READS;
				}
				else
					num_entries_to_read = fifo_queue_depth;

				// read the data & process to USB-->UART
				critical_section_enter_blocking(&my_critical_section);
				flt_code = read_register(ADXL38X_FIFO_DATA, num_entries_to_read * ADXL38X_DATA_SIZE_WITH_CH, fifo_data); // Address will show as 0x3B on logic analyzer due to R/W bit.
				critical_section_exit(&my_critical_section);
				if (flt_code)
					fault_handler(SPI_COMM);
				else
				{
					// fifo_data_to_readable_string(fifo_data, &serialBuffers.buf[serialBuffers.active], set_fifo_queue_depth, total_samples_read);
					//  DEBUG_PRINT("%s", serialBuffers.buf[serialBuffers.active].data);
					uint32_t bytes_to_write = fifo_data_to_data_stream(fifo_data, &serialBuffers.buf[serialBuffers.active], num_entries_to_read, total_samples_read);
					// DEBUG_PRINT("%u bytes", bytes_to_write);
					// sleep_us(50);																															// This delay is critical to preventing a hardfault in the fwrite.  haven't optimized the duration.
					fwrite(serialBuffers.buf[serialBuffers.active].data, sizeof(&serialBuffers.buf[serialBuffers.active].data[0]), bytes_to_write, stdout); // not sure why bytes_to_write-1 is needed, but otherwise I get an extra byte written
					fflush(stdout);

					serialBuffers.active++;
					if (serialBuffers.active >= NUM_UART_BUFFERS)
						serialBuffers.active = 0;

					//  Update counters
					total_samples_read += num_entries_to_read;
					fifo_queue_depth -= num_entries_to_read;
				}
			} while (fifo_queue_depth > 0);
		}
	}
	DEBUG_PRINT("End\n");
	set_led_state(0);
}
