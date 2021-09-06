/*
 * single_uart.c
 *
 *  Created on: 2021年9月3日
 *      Author: c
 */



#if BOARD_VERSION == OLM_ESP_PRO_V1X

#define BUF_SIZE (254)


void single_uart_write(uint8_t driver, uint8_t* data, uint32_t len)
{
	switch(driver)
	{
		case X_AXIS:
		{
			ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, X_DRIVER_UART_PIN, UNUSE_PIN, -1, -1));
			uart_write_bytes(UART_NUM_1, data, len);
			uart_wait_tx_done(UART_NUM_1,20/portTICK_PERIOD_MS);
			ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UNUSE_PIN, X_DRIVER_UART_PIN, -1, -1));
			break;
		}
		case Y_AXIS:
		{
			ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, Y_DRIVER_UART_PIN, UNUSE_PIN, -1, -1));
			uart_write_bytes(UART_NUM_1, data, len);
			uart_wait_tx_done(UART_NUM_1,20/portTICK_PERIOD_MS);
			ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UNUSE_PIN, Y_DRIVER_UART_PIN, -1, -1));
			break;
		}
		case Z_AXIS:
		{
			ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, Z_DRIVER_UART_PIN, UNUSE_PIN, -1, -1));
			uart_write_bytes(UART_NUM_1, data, len);
			uart_wait_tx_done(UART_NUM_1,20/portTICK_PERIOD_MS);
			ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UNUSE_PIN, Z_DRIVER_UART_PIN, -1, -1));
			break;
		}
		default:
			break;
	}
}


void single_uart_init(uint8_t driver)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    switch(driver)
    {
		case X_AXIS:
		{
			ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UNUSE_PIN, X_DRIVER_UART_PIN, -1, -1));
			break;
		}
		case Y_AXIS:
		{
			ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UNUSE_PIN, Y_DRIVER_UART_PIN, -1, -1));
			break;
		}
		case Z_AXIS:
		{
			ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UNUSE_PIN, Z_DRIVER_UART_PIN, -1, -1));
			break;
		}
		default:
			break;
    }


}

void single_uart_test(uint8_t driver)
{

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint32_t time = 0;

    while (1)
    {
    	single_uart_write(driver,(uint8_t*)"hello world.\r\n", 14);


        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        if(len > 0)
        {
			// Write data back to the UART
        	serialWriteData(data, len);
        }
        HAL_Delay(2000);
    }
}

#endif

