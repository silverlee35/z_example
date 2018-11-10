/* SoC level DTS fixup file */

#define DT_NUM_IRQ_PRIO_BITS	DT_ARM_V7M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

#define DT_ADC_0_IRQ		DT_NORDIC_NRF_SAADC_40007000_IRQ_0
#define CONFIG_ADC_0_IRQ_PRI		DT_NORDIC_NRF_SAADC_40007000_IRQ_0_PRIORITY
#define CONFIG_ADC_0_NAME		DT_NORDIC_NRF_SAADC_40007000_LABEL

#if defined(DT_NORDIC_NRF_UARTE_40002000_BASE_ADDRESS)
#define DT_UART_0_BASE		DT_NORDIC_NRF_UARTE_40002000_BASE_ADDRESS
#define DT_UART_0_IRQ_PRI		DT_NORDIC_NRF_UARTE_40002000_IRQ_0_PRIORITY
#define DT_UART_0_IRQ_NUM		DT_NORDIC_NRF_UARTE_40002000_IRQ_0
#define DT_UART_0_BAUD_RATE		DT_NORDIC_NRF_UARTE_40002000_CURRENT_SPEED
#define DT_UART_0_NAME		DT_NORDIC_NRF_UARTE_40002000_LABEL
#define DT_UART_0_TX_PIN		DT_NORDIC_NRF_UARTE_40002000_TX_PIN
#define DT_UART_0_RX_PIN		DT_NORDIC_NRF_UARTE_40002000_RX_PIN
  #if defined(DT_NORDIC_NRF_UARTE_40002000_RTS_PIN)
  #define DT_UART_0_RTS_PIN		DT_NORDIC_NRF_UARTE_40002000_RTS_PIN
  #endif
  #if defined(DT_NORDIC_NRF_UARTE_40002000_CTS_PIN)
  #define DT_UART_0_CTS_PIN		DT_NORDIC_NRF_UARTE_40002000_CTS_PIN
  #endif
#else
#define DT_UART_0_BASE		DT_NORDIC_NRF_UART_40002000_BASE_ADDRESS
#define DT_UART_0_IRQ_PRI		DT_NORDIC_NRF_UART_40002000_IRQ_0_PRIORITY
#define DT_UART_0_IRQ_NUM		DT_NORDIC_NRF_UART_40002000_IRQ_0
#define DT_UART_0_BAUD_RATE		DT_NORDIC_NRF_UART_40002000_CURRENT_SPEED
#define DT_UART_0_NAME		DT_NORDIC_NRF_UART_40002000_LABEL
#define DT_UART_0_TX_PIN		DT_NORDIC_NRF_UART_40002000_TX_PIN
#define DT_UART_0_RX_PIN		DT_NORDIC_NRF_UART_40002000_RX_PIN
  #if defined(DT_NORDIC_NRF_UART_40002000_RTS_PIN)
  #define DT_UART_0_RTS_PIN		DT_NORDIC_NRF_UART_40002000_RTS_PIN
  #endif
  #if defined(DT_NORDIC_NRF_UART_40002000_RTS_PIN)
  #define DT_UART_0_CTS_PIN		DT_NORDIC_NRF_UART_40002000_CTS_PIN
  #endif
#endif

#define DT_UART_1_BASE		DT_NORDIC_NRF_UARTE_40028000_BASE_ADDRESS
#define DT_UART_1_IRQ_PRI		DT_NORDIC_NRF_UARTE_40028000_IRQ_0_PRIORITY
#define DT_UART_1_IRQ_NUM		DT_NORDIC_NRF_UARTE_40028000_IRQ_0
#define DT_UART_1_BAUD_RATE		DT_NORDIC_NRF_UARTE_40028000_CURRENT_SPEED
#define DT_UART_1_NAME		DT_NORDIC_NRF_UARTE_40028000_LABEL
#define DT_UART_1_TX_PIN		DT_NORDIC_NRF_UARTE_40028000_TX_PIN
#define DT_UART_1_RX_PIN		DT_NORDIC_NRF_UARTE_40028000_RX_PIN
#if defined(DT_NORDIC_NRF_UARTE_40028000_RTS_PIN)
#define DT_UART_1_RTS_PIN		DT_NORDIC_NRF_UARTE_40028000_RTS_PIN
#endif
#if defined(DT_NORDIC_NRF_UARTE_40028000_CTS_PIN)
#define DT_UART_1_CTS_PIN		DT_NORDIC_NRF_UARTE_40028000_CTS_PIN
#endif

#define DT_FLASH_DEV_NAME			DT_NRF_NRF52_FLASH_CONTROLLER_4001E000_LABEL

#if defined(CONFIG_QSPI_FLASH_NRF)
  #define DT_SERIAL_FLASH_DEV_NAME	DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_LABEL
  #define DT_SERIAL_FLASH_ADDRESS	DT_SERIAL_FLASH_12000000_BASE_ADDRESS
  #define DT_SERIAL_FLASH_SIZE		DT_SERIAL_FLASH_12000000_SIZE

  #define DT_QSPI_FLASH_CSN_PIN		DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_CSN_PIN
  #define DT_QSPI_FLASH_SCK_PIN		DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_SCK_PIN
  #define DT_QSPI_FLASH_IO0_PIN		DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_IO_PINS_0
  #define DT_QSPI_FLASH_IO1_PIN		DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_IO_PINS_1
  #define DT_QSPI_FLASH_IO2_PIN		DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_IO_PINS_2
  #define DT_QSPI_FLASH_IO3_PIN		DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_IO_PINS_3
  #define DT_QSPI_FLASH_IRQ	 	DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_IRQ_0
  #define DT_QSPI_FLASH_IRQ_PRI		DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_IRQ_0_PRIORITY

  #if defined(DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_QSPI_MAX_FREQUENCY)
  #define DT_QSPI_FLASH_QSPI_FREQ_0	DT_NORDIC_NRF_QSPI_FLASH_CONTROLLER_40029000_QSPI_MAX_FREQUENCY
  #else
  #define DT_QSPI_FLASH_QSPI_FREQ_0	32000000
  #endif
#endif

#define DT_GPIO_P0_DEV_NAME		DT_NORDIC_NRF_GPIO_50000000_LABEL
#if CONFIG_HAS_HW_NRF_GPIO1
#define DT_GPIO_P1_DEV_NAME		DT_NORDIC_NRF_GPIO_50000300_LABEL
#endif
#define DT_GPIOTE_IRQ_PRI		DT_NORDIC_NRF_GPIOTE_40006000_IRQ_0_PRIORITY
#define DT_GPIOTE_IRQ		DT_NORDIC_NRF_GPIOTE_40006000_IRQ_0

#define DT_I2C_0_BASE_ADDR		DT_NORDIC_NRF_I2C_40003000_BASE_ADDRESS
#define CONFIG_I2C_0_NAME		DT_NORDIC_NRF_I2C_40003000_LABEL
#define DT_I2C_0_BITRATE		DT_NORDIC_NRF_I2C_40003000_CLOCK_FREQUENCY
#define CONFIG_I2C_0_IRQ_PRI		DT_NORDIC_NRF_I2C_40003000_IRQ_0_PRIORITY
#define DT_I2C_0_IRQ		DT_NORDIC_NRF_I2C_40003000_IRQ_0
#define DT_I2C_0_SDA_PIN		DT_NORDIC_NRF_I2C_40003000_SDA_PIN
#define DT_I2C_0_SCL_PIN		DT_NORDIC_NRF_I2C_40003000_SCL_PIN

#define DT_I2C_1_BASE_ADDR		DT_NORDIC_NRF_I2C_40004000_BASE_ADDRESS
#define CONFIG_I2C_1_NAME		DT_NORDIC_NRF_I2C_40004000_LABEL
#define DT_I2C_1_BITRATE		DT_NORDIC_NRF_I2C_40004000_CLOCK_FREQUENCY
#define CONFIG_I2C_1_IRQ_PRI		DT_NORDIC_NRF_I2C_40004000_IRQ_0_PRIORITY
#define DT_I2C_1_IRQ		DT_NORDIC_NRF_I2C_40004000_IRQ_0
#define DT_I2C_1_SDA_PIN		DT_NORDIC_NRF_I2C_40004000_SDA_PIN
#define DT_I2C_1_SCL_PIN		DT_NORDIC_NRF_I2C_40004000_SCL_PIN

#define DT_QDEC_BASE_ADDR		DT_NORDIC_NRF_QDEC_40012000_BASE_ADDRESS
#define DT_QDEC_NAME		DT_NORDIC_NRF_QDEC_40012000_LABEL
#define DT_QDEC_IRQ_PRI		DT_NORDIC_NRF_QDEC_40012000_IRQ_0_PRIORITY
#define DT_QDEC_IRQ			DT_NORDIC_NRF_QDEC_40012000_IRQ_0
#define DT_QDEC_A_PIN		DT_NORDIC_NRF_QDEC_40012000_A_PIN
#define DT_QDEC_B_PIN		DT_NORDIC_NRF_QDEC_40012000_B_PIN
#if defined(DT_NORDIC_NRF_QDEC_40012000_LED_PIN)
#define DT_QDEC_LED_PIN		DT_NORDIC_NRF_QDEC_40012000_LED_PIN
#endif
#if defined(DT_NORDIC_NRF_QDEC_40012000_ENABLE_PIN)
#define DT_QDEC_ENABLE_PIN		DT_NORDIC_NRF_QDEC_40012000_ENABLE_PIN
#endif
#define DT_QDEC_LED_PRE		DT_NORDIC_NRF_QDEC_40012000_LED_PRE
#define DT_QDEC_STEPS		DT_NORDIC_NRF_QDEC_40012000_STEPS

#define DT_SPI_0_BASE_ADDRESS       DT_NORDIC_NRF_SPI_40003000_BASE_ADDRESS
#define CONFIG_SPI_0_NAME               DT_NORDIC_NRF_SPI_40003000_LABEL
#define CONFIG_SPI_0_IRQ_PRI            DT_NORDIC_NRF_SPI_40003000_IRQ_0_PRIORITY
#define DT_SPI_0_IRQ                DT_NORDIC_NRF_SPI_40003000_IRQ_0
#define DT_SPI_0_NRF_SCK_PIN	DT_NORDIC_NRF_SPI_40003000_SCK_PIN
#define DT_SPI_0_NRF_MOSI_PIN	DT_NORDIC_NRF_SPI_40003000_MOSI_PIN
#define DT_SPI_0_NRF_MISO_PIN	DT_NORDIC_NRF_SPI_40003000_MISO_PIN
#define DT_SPI_0_NRF_CSN_PIN	DT_NORDIC_NRF_SPI_40003000_CSN_PIN

#define DT_SPI_1_BASE_ADDRESS       DT_NORDIC_NRF_SPI_40004000_BASE_ADDRESS
#define CONFIG_SPI_1_NAME               DT_NORDIC_NRF_SPI_40004000_LABEL
#define CONFIG_SPI_1_IRQ_PRI            DT_NORDIC_NRF_SPI_40004000_IRQ_0_PRIORITY
#define DT_SPI_1_IRQ                DT_NORDIC_NRF_SPI_40004000_IRQ_0
#define DT_SPI_1_NRF_SCK_PIN	DT_NORDIC_NRF_SPI_40004000_SCK_PIN
#define DT_SPI_1_NRF_MOSI_PIN	DT_NORDIC_NRF_SPI_40004000_MOSI_PIN
#define DT_SPI_1_NRF_MISO_PIN	DT_NORDIC_NRF_SPI_40004000_MISO_PIN
#define DT_SPI_1_NRF_CSN_PIN	DT_NORDIC_NRF_SPI_40004000_CSN_PIN

#define DT_SPI_2_BASE_ADDRESS       DT_NORDIC_NRF_SPI_40023000_BASE_ADDRESS
#define CONFIG_SPI_2_NAME               DT_NORDIC_NRF_SPI_40023000_LABEL
#define CONFIG_SPI_2_IRQ_PRI            DT_NORDIC_NRF_SPI_40023000_IRQ_0_PRIORITY
#define DT_SPI_2_IRQ                DT_NORDIC_NRF_SPI_40023000_IRQ_0
#define DT_SPI_2_NRF_SCK_PIN	DT_NORDIC_NRF_SPI_40023000_SCK_PIN
#define DT_SPI_2_NRF_MOSI_PIN	DT_NORDIC_NRF_SPI_40023000_MOSI_PIN
#define DT_SPI_2_NRF_MISO_PIN	DT_NORDIC_NRF_SPI_40023000_MISO_PIN
#define DT_SPI_2_NRF_CSN_PIN	DT_NORDIC_NRF_SPI_40023000_CSN_PIN

#define DT_SPI_3_BASE_ADDRESS       DT_NORDIC_NRF_SPI_4002B000_BASE_ADDRESS
#define CONFIG_SPI_3_NAME               DT_NORDIC_NRF_SPI_4002B000_LABEL
#define CONFIG_SPI_3_IRQ_PRI            DT_NORDIC_NRF_SPI_4002B000_IRQ_0_PRIORITY
#define DT_SPI_3_IRQ                DT_NORDIC_NRF_SPI_4002B000_IRQ_0
#define DT_SPI_3_NRF_SCK_PIN	DT_NORDIC_NRF_SPI_4002B000_SCK_PIN
#define DT_SPI_3_NRF_MOSI_PIN	DT_NORDIC_NRF_SPI_4002B000_MOSI_PIN
#define DT_SPI_3_NRF_MISO_PIN	DT_NORDIC_NRF_SPI_4002B000_MISO_PIN
#define DT_SPI_3_NRF_CSN_PIN	DT_NORDIC_NRF_SPI_4002B000_CSN_PIN

#define DT_USBD_NRF_IRQ		DT_NORDIC_NRF_USBD_40027000_IRQ_USBD
#define DT_USBD_NRF_IRQ_PRI		DT_NORDIC_NRF_USBD_40027000_IRQ_USBD_PRIORITY
#define DT_USBD_NRF_NUM_BIDIR_EP	DT_NORDIC_NRF_USBD_40027000_NUM_BIDIR_ENDPOINTS
#define DT_USBD_NRF_NUM_IN_EP	DT_NORDIC_NRF_USBD_40027000_NUM_IN_ENDPOINTS
#define DT_USBD_NRF_NUM_OUT_EP	DT_NORDIC_NRF_USBD_40027000_NUM_OUT_ENDPOINTS
#define DT_USBD_NRF_NUM_ISOIN_EP	DT_NORDIC_NRF_USBD_40027000_NUM_ISOIN_ENDPOINTS
#define DT_USBD_NRF_NUM_ISOOUT_EP	DT_NORDIC_NRF_USBD_40027000_NUM_ISOOUT_ENDPOINTS
#define DT_USBD_NRF_NAME		DT_NORDIC_NRF_USBD_40027000_LABEL

#define CONFIG_WDT_0_NAME		DT_NORDIC_NRF_WATCHDOG_40010000_LABEL
#define DT_WDT_NRF_IRQ		DT_NORDIC_NRF_WATCHDOG_40010000_IRQ_WDT
#define DT_WDT_NRF_IRQ_PRI		DT_NORDIC_NRF_WATCHDOG_40010000_IRQ_WDT_PRIORITY

#if defined(DT_NORDIC_NRF_CC310_5002A000_BASE_ADDRESS)
#define DT_CC310_CTL_BASE_ADDR	DT_NORDIC_NRF_CC310_5002A000_BASE_ADDRESS
#define DT_CC310_CTL_NAME		DT_NORDIC_NRF_CC310_5002A000_LABEL
#define DT_CC310_BASE_ADDR		DT_ARM_CRYPTOCELL_310_5002B000_BASE_ADDRESS
#define DT_CC310_NAME		DT_ARM_CRYPTOCELL_310_5002B000_LABEL
#define DT_CC310_IRQ		DT_ARM_CRYPTOCELL_310_5002B000_IRQ_0
#define DT_CC310_IRQ_PRI		DT_ARM_CRYPTOCELL_310_5002B000_IRQ_0_PRIORITY
#endif

#define DT_WNCM14A2A_UART_DRV_NAME			DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_BUS_NAME
#define DT_WNCM14A2A_GPIO_MDM_BOOT_MODE_SEL_NAME	DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_BOOT_MODE_SEL_GPIOS_CONTROLLER
#define DT_WNCM14A2A_GPIO_MDM_BOOT_MODE_SEL_PIN	DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_BOOT_MODE_SEL_GPIOS_PIN
#define DT_WNCM14A2A_GPIO_MDM_POWER_NAME		DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_POWER_GPIOS_CONTROLLER
#define DT_WNCM14A2A_GPIO_MDM_POWER_PIN		DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_POWER_GPIOS_PIN
#define DT_WNCM14A2A_GPIO_MDM_KEEP_AWAKE_NAME	DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_KEEP_AWAKE_GPIOS_CONTROLLER
#define DT_WNCM14A2A_GPIO_MDM_KEEP_AWAKE_PIN	DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_KEEP_AWAKE_GPIOS_PIN
#define DT_WNCM14A2A_GPIO_MDM_RESET_NAME		DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_RESET_GPIOS_CONTROLLER
#define DT_WNCM14A2A_GPIO_MDM_RESET_PIN		DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_RESET_GPIOS_PIN
#define DT_WNCM14A2A_GPIO_MDM_SHLD_TRANS_ENA_NAME	DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_SHLD_TRANS_ENA_GPIOS_CONTROLLER
#define DT_WNCM14A2A_GPIO_MDM_SHLD_TRANS_ENA_PIN	DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_SHLD_TRANS_ENA_GPIOS_PIN
#ifdef DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_SEND_OK_GPIOS_PIN
#define DT_WNCM14A2A_GPIO_MDM_SEND_OK_NAME		DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_SEND_OK_GPIOS_CONTROLLER
#define DT_WNCM14A2A_GPIO_MDM_SEND_OK_PIN		DT_NORDIC_NRF_UARTE_40028000_WNCM14A2A_MDM_SEND_OK_GPIOS_PIN
#endif

/* End of SoC Level DTS fixup file */
