#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_system.h>
#include <esp_log.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

#pragma pack(push,1)

#define TMC5160_QUEUE_SIZE 24
#define TMC5160_NUM_CHAIN CONFIG_LMTZ_TMC5160_NUM_CHAIN
#define TMC5160_SPI_USE_DMA CONFIG_LMTZ_TMC5160_SPI_USE_DMA

#if TMC5160_SPI_USE_DMA
#define TMC5160_SPI_DMA_CHANNEL CONFIG_LMTZ_TMC5160_SPI_DMA_CHANNEL
#else
#define TMC5160_SPI_DMA_CHANNEL 0
#endif
#define TMC5160_SPI_USE_GPIO CONFIG_LMTZ_TMC5160_SPI_USE_GPIO

#if (CONFIG_LMTZ_TMC5160_SPI_HSPI)
#define TMC5160_SPI_HOST HSPI_HOST
#if !TMC5160_SPI_USE_GPIO
#define TMC5160_PIN_MOSI 13
#define TMC5160_PIN_MISO 12
#define TMC5160_PIN_SCK  14
#define TMC5160_PIN_CS   15
#endif
#elif (CONFIG_LMTZ_TMC5160_SPI_VSPI)
#define TMC5160_SPI_HOST VSPI_HOST
#if !TMC5160_SPI_USE_GPIO
#define TMC5160_PIN_MOSI 23
#define TMC5160_PIN_MISO 19
#define TMC5160_PIN_SCK  18
#define TMC5160_PIN_CS   5
#endif
#endif

#if TMC5160_SPI_USE_GPIO
#define TMC5160_PIN_MOSI CONFIG_LMTZ_TMC5160_PIN_MOSI
#define TMC5160_PIN_MISO CONFIG_LMTZ_TMC5160_PIN_MISO
#define TMC5160_PIN_SCK  CONFIG_LMTZ_TMC5160_PIN_SCK
#define TMC5160_PIN_CS   CONFIG_LMTZ_TMC5160_PIN_CS
#endif

#define TMC5160_PIN_EN CONFIG_LMTZ_TMC5160_PIN_EN
#define TMC5160_CLOCK_SPEED CONFIG_LMTZ_TMC5160_CLOCK_SPEED

typedef struct
{
	unsigned status_stop_r : 1;
	unsigned status_stop_l  : 1;
	unsigned position_reached : 1;
	unsigned velocity_reached : 1;
	unsigned standstill : 1;
	unsigned sg2 : 1; // stall guard flag active
	unsigned driver_error : 1; // clear by reading gstat
	unsigned reset_flag : 1; // clear bt reading gstat
} tmc5160_spi_status_t;

typedef struct
{
	union 
	{
		uint8_t address;
	//	tmc5160_spi_status_t status; //NOTE bit order in struct is implied!
	};

	uint32_t data;

} tmc5160_datagram_t;

#pragma pack(push,4)
typedef struct
{
	union
	{
		struct 
		{
			tmc5160_datagram_t tx[TMC5160_NUM_CHAIN];
			uint8_t _pad4[3 & -TMC5160_NUM_CHAIN];
			tmc5160_datagram_t rx[TMC5160_NUM_CHAIN];
			spi_transaction_t ref;
		};
	};
} tmc5160_transaction_t;
#pragma pack(pop)

typedef struct
{

} tmc5160_t;

typedef struct
{
	int (*init)();
	int (*deinit)();
	int (*update)();
	int (*enable)(bool en);

	int spi_host;
	int dma_channel;
	int count;

	int _queued_transaction_count;

	struct
	{
		int8_t MOSI, MISO, SCK, CS;
		int8_t STEP, DIRECTION, EN;
	} pins;

	tmc5160_transaction_t* tbuffer;

	spi_bus_config_t bus_config;
	spi_device_interface_config_t dev_config;
	spi_transaction_t transaction;
	spi_device_handle_t device;

} tmc5160_driver_t;

typedef struct
{
	unsigned recalibrate : 1;      // Zero  crossing  recalibration  during  driver  disable via ENN or via TOFF setting
	unsigned faststandstill : 1;   // 2^18 clocks, 0: 2^20 clocks to standstill detection power down
	unsigned en_pwm_mode : 1;      // StealthChopvoltage PWM mode enabled (depending  on  velocity  thresholds).  
	                               //   Switch from off to on state while instand-still and at IHOLD= nominal IRUN current, only.

	unsigned multistep_filt : 1;   // Enable   step   input   filtering   for StealthChopoptimizationwith external step source (default=1)
	unsigned shaft : 1;            // Inverse shaft direction
	unsigned diag0_error : 1;      // SD1: Enable DIAG0 active on driver errors: Over temperature (ot), short to GND (s2g), undervoltage chargepump (uv_cp)
	                               //   DIAG0 always shows the reset-status, i.e. is active low during resetconditio

	unsigned diag0_otpw : 1;       // SD1:Enable DIAG0  active  on  driver over  temperature prewarning (otpw)
	union
	{
		unsigned diag0_stall : 1;   // SD1: Enable DIAG0   activeon motor   stall(set TCOOLTHRSbefore using this feature)
		unsigned diag0_step : 1;    // SD0: Enable DIAG0 as  STEP output (half frequency, dual edge triggered)for external STEP/DIR driver
	};

	union
	{
		unsigned diag1_stall : 1;   // SD1: Enable DIAG1active   on motor   stall(set TCOOLTHRSbefore using this feature)
		unsigned diag1_dir : 1;	    // SD):0: DIAG1 outputs position compare signal.  1: Enable DIAG1as DIRoutputfor external STEP/DIR driver
	};

	unsigned diag1_index : 1;           // only SD1 Enable DIAG1active on index position (microstep look up table position 0)
	unsigned diag1_onstate : 1;         // only SD1 Enable DIAG1active when chopper is on (for the coil which is in the second half of the fullstep)

	unsigned diag1_steps_skipped: 1;    // SD1: nable  output  toggle  when  steps  are  skipped  in DcStepmode(increment  of LOST_STEPS).  
				 	    // Do  not enable in conjunction with other DIAG1 options.

	unsigned diag0_int_pushpull: 1;     // 0:SWN_DIAG0 is open collector output (active low)1:Enable SWN_DIAG0push pull output (active high)
	unsigned diag1_poscomp_pushpulll:1; // 0:SWP_DIAG1 is open collector output (active low)1:Enable SWP_DIAG1push pull output (active high)
	unsigned small__hysteresis : 1;     // 0:Hysteresis for step frequency comparison is 1/161:Hysteresis for step frequency comparison is 1/32
	unsigned stop_enable : 1;           // 0: Normal operation 1:Emergency  stop: ENCA_DCINstops  the  sequencer when  tied high(no  
	                                    // steps become executed  by the sequencer, motor goes to standstill state).

	unsigned direct_mode : 1;           // 0: Normal operation
                                            // 1:Motor coil currents and polarity directly programmed via serial interface: Register XTARGET(0x2D)
	                                    // specifies signed coil A current (bits 8..0) and coil B current (bits 24..16).In this mode, the current is scaled
	                                    // by IHOLD setting. Velocity based current regulation of StealthChopis not available in  this  mode.The  automatic 
	                                    // StealthChopcurrent regulation will work only for low stepper motor velocities.


	unsigned test_mode : 1;	            // 0: Normal operation1:Enable  analog  test  output  on  pin ENCN_DCO.IHOLD[1..0]selects  
	                                    // the  function  of ENCN_DCO: 0...2: T120, DAC, VDDHHint: Not for user, set to 0 for normal operation
} tmc5160_reg_gconf_t;

#define TMC5160_SPI_MODE 3

extern tmc5160_driver_t TMC5160;

#ifndef STEPPER
#define STEPPER TMC5160
#endif
#pragma pack(pop)
