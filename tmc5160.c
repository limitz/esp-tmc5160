#include <tmc5160.h>

#define TAG "TMC5160"

inline size_t dgbit_size() { return (sizeof(tmc5160_datagram_t) << 3) * TMC5160_NUM_CHAIN; }
inline size_t queue_size() { return TMC5160_QUEUE_SIZE; }
inline size_t trans_size() { return sizeof(tmc5160_transaction_t); }
inline size_t alloc_size() { return queue_size() * trans_size(); }

static int tmc5160_init()
{
	TMC5160.count = TMC5160_NUM_CHAIN;
	TMC5160.bus_config.max_transfer_sz = dgbit_size();
		//TMC5160.transaction.length;

	TMC5160.bus_config.miso_io_num = TMC5160.pins.MISO; 
	TMC5160.bus_config.mosi_io_num = TMC5160.pins.MOSI; 
	TMC5160.bus_config.sclk_io_num = TMC5160.pins.SCK;
	TMC5160.dev_config.spics_io_num = TMC5160.pins.CS;

	ESP_LOGI(TAG, "PIN: MOSI = %d", TMC5160.pins.MOSI);
	ESP_LOGI(TAG, "PIN: MISO = %d", TMC5160.pins.MISO);
	ESP_LOGI(TAG, "PIN: SCK  = %d", TMC5160.pins.SCK);
	ESP_LOGI(TAG, "PIN: CS   = %d", TMC5160.pins.CS);

	if (TMC5160.pins.EN >= 0)
	{
		ESP_LOGI(TAG, "PIN: EN   = %d", TMC5160.pins.EN);

		gpio_config_t config = {
			.pin_bit_mask = 1 << TMC5160.pins.EN,
			.mode = GPIO_MODE_OUTPUT,
			.pull_up_en = GPIO_PULLUP_ENABLE,
		};

		gpio_reset_pin(TMC5160.pins.EN);
		gpio_config(&config);
	}
	else ESP_LOGW(TAG, "PIN: EN NOT DEFINED. Connect to GND manually to enable the TMC5160");


#if TMC5160_SPI_USE_DMA
	TMC5160.tbuffer = heap_caps_malloc(alloc_size(), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
#else
	TMC5160.tbuffer = heap_caps_malloc(alloc_size(), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#endif
	ESP_LOGI(TAG, "Allocated 2 buffers of size %d", alloc_size());
	memset(TMC5160.tbuffer, 0x00, alloc_size());
	
	ESP_LOGI(TAG, "Using DMA: %s (%d)", TMC5160.dma_channel ? "yes" : "no", TMC5160.dma_channel);

	spi_bus_initialize(TMC5160.spi_host, &TMC5160.bus_config, TMC5160.dma_channel);
	spi_bus_add_device(TMC5160.spi_host, &TMC5160.dev_config, &TMC5160.device);

	return ESP_OK;
}

static int tmc5160_enable(bool en)
{
	if (TMC5160.pins.EN >= 0)
	{
		gpio_set_level(TMC5160.pins.EN, en ? 0 : 1);
	}
	return ESP_OK;
}

static int tmc5160_sync()
{
	int error;
	spi_transaction_t* t;
	while (TMC5160._queued_transaction_count)
	{
		error = spi_device_get_trans_result(TMC5160.device, &t, portMAX_DELAY);
		if (ESP_OK != error) return error;
		TMC5160._queued_transaction_count -= 1;
	}
	return ESP_OK;
}

static int tmc5160_update() 
{
	int error;
	static int i = 0;
	static uint8_t req[] = {
		 0x00 ,  0x01 ,  0x04 , 0x12, 
		 0x20 ,  0x21 ,  0x22 , 0x2D, 
		 0x34 ,  0x35 ,  0x36 ,
		 0x6C ,  0x6F ,
		 0x71 ,  0x72 ,  0x00,
	};

	static tmc5160_datagram_t g[] = {
		{ .address = 0x01, .data = 0x00000000 },
		{ .address = 0x8b, .data = 0x000001DD },
		{ .address = 0x90, .data = 0x00000600 },
		{ .address = 0x93, .data = 0x00000FFF },
		{ .address = 0x94, .data = 0x00000FFF },
		{ .address = 0x95, .data = 0x0000002F },
		{ .address = 0xA0, .data = 0x00000000 },
		{ .address = 0xA1, .data = 0x00000000 },
		{ .address = 0xA3, .data = 0x00000500 },
		{ .address = 0xA4, .data = 0x00000600 },
		{ .address = 0xA5, .data = 0x00018000 },
		{ .address = 0xA6, .data = 0x00000300 },
		{ .address = 0xA7, .data = 0x00030000 },
		{ .address = 0xA8, .data = 0x00000F00 },
		{ .address = 0xAA, .data = 0x00000D00 },
		{ .address = 0xAB, .data = 0x00000600 },
		{ .address = 0xAC, .data = 0x00000800 },
		{ .address = 0xAD, .data = 0x00000000 },
		{ .address = 0xB3, .data = 0x00000000 },
		{ .address = 0xB4, .data = 0x00000000 },
		{ .address = 0xEE, .data = 0x0000001C },
		{ .address = 0xEC, .data = 0x10410155 },
		{ .address = 0xA1, .data = 0x00100000 },
		{ .address = 0x00, .data = 0x00000000 },
	};

	if (!i)
	{
		for (i=0; i<24; i++)
		{
			tmc5160_transaction_t *t = TMC5160.tbuffer + i;
			t->ref.tx_buffer = t->tx;
			t->ref.rx_buffer = t->rx;
			//t->ref.user = i;
			t->ref.length = dgbit_size();	
			for (int c=0; c<TMC5160_NUM_CHAIN; c++) 
			{
				t->tx[c].address = g[i].address;
				t->tx[c].data = (0x000000FF & (g[i].data >> 030))
					      | (0x0000FF00 & (g[i].data >> 010))
					      | (0x00FF0000 & (g[i].data << 010))
					      | (0xFF000000 & (g[i].data << 030))
					      ;
	
				memset(t->rx, 0xEE, 5);
			}
			error = spi_device_queue_trans(TMC5160.device, &t->ref, 100);
			if (ESP_OK != error) {
				ESP_LOGW(TAG, "Unable to queue datagram: error %01x", error);
				return error;
			}
	
			TMC5160._queued_transaction_count += 1;
		}
		ESP_ERROR_CHECK( error = tmc5160_sync() );
		if (ESP_OK != error) return error;

		ESP_LOGW(TAG, "[REGS]");
		for (int j=1; j<24; j++)
		{
			ESP_LOGI(TAG, "%02X: %08X %02X ", 
			TMC5160.tbuffer[j-1].tx[0].address, 
			TMC5160.tbuffer[j].rx[0].data, 
			TMC5160.tbuffer[j].rx[0].address);
		}
		ESP_LOGW(TAG, "-----");
	}
//	ESP_LOG_BUFFER_HEX(TAG, TMC5160.tbuffer, alloc_size());

	return ESP_OK;
}

static int tmc5160_deinit()
{
	spi_bus_remove_device(TMC5160.device);
	spi_bus_free(TMC5160.spi_host);
	heap_caps_free(TMC5160.tbuffer);
	return ESP_OK;
}

tmc5160_driver_t TMC5160 = { 
	.device = 0, 
	.spi_host = TMC5160_SPI_HOST, 
	.dma_channel = TMC5160_SPI_DMA_CHANNEL, 
	
	.bus_config = { 
		.miso_io_num = TMC5160_PIN_MISO, 
		.mosi_io_num = TMC5160_PIN_MOSI, 
		.sclk_io_num = TMC5160_PIN_SCK, 
		.quadwp_io_num = -1, 
		.quadhd_io_num = -1, 
		.flags =  SPICOMMON_BUSFLAG_MASTER 
#if !TMC5160_SPI_USE_GPIO
			| SPICOMMON_BUSFLAG_IOMUX_PINS 
#endif
			| SPICOMMON_BUSFLAG_SCLK 
			| SPICOMMON_BUSFLAG_MOSI 
			| SPICOMMON_BUSFLAG_MISO,
#if 1
		.intr_flags = ESP_INTR_FLAG_IRAM,
#else
		.intr_flags = ESP_INTR_FLAG_SHARED,
#endif
	}, 
	
	.dev_config = { 
		.clock_speed_hz = TMC5160_CLOCK_SPEED, 
		.mode = 3, 
		.spics_io_num = TMC5160_PIN_CS,
		.queue_size = 16, 
		.flags = SPI_DEVICE_NO_DUMMY,
	},

	.pins = 
	{
		.EN   = TMC5160_PIN_EN,
		.MISO = TMC5160_PIN_MISO,
		.MOSI = TMC5160_PIN_MOSI,
		.SCK  = TMC5160_PIN_SCK,
		.CS   = TMC5160_PIN_CS,
	},

	.init = tmc5160_init,
	.enable = tmc5160_enable,
	.update = tmc5160_update,
	.deinit = tmc5160_deinit,
};

