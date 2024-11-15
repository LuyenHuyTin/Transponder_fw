#ifndef __MAIN_H
#define __MAIN_H

#include <vector>
#include "app_error.h"
#include "nrf_drv_uart.h"
#include "nrf_drv_gpiote.h"
#include "nrf_queue.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define PERIPHERAL_TIMER_IDX    2
#define PC_COMM_TIMER_IDX       3

#define COMM_OBJ_BUF_SIZE       256
#define MC33290_OBJ_BUF_SIZE    560
#define BLE_COM_MON_BUF_SIZE    20
#define BLE_COM_BUF_SIZE        132
#define EEPROM_24CXX_BUF_SIZE   128
#define EEPROM_93CXX_BUF_SIZE   128
#define RL78_BUF_SIZE           128
#define dout_pin                42
#define SCK_pin                 11
#define din_pin                 25

#define PC_UART_COMM_IDX        1
#define PC_UART_COMM_TX_PIN     47
#define PC_UART_COMM_RX_PIN     45
#define PC_UART_COM_BUF_SIZE    280
#define PC_UART_COMM_BAUDRATE   NRF_UART_BAUDRATE_115200

#define MC33290_UART_COMM_IDX           0
#define MC33290_COMM_TX_PIN             15
#define MC33290_COMM_RX_PIN             17
#define MC33290_DEFAULT_UART_BAUDRATE   0x002A9930UL
#define MC33290_HONDA_UART_BAUDRATE     0x002A9930UL  /* 10.4kbps */
#define MC33290_ISO14230_UART_BAUDRATE  0x002A9930UL  /* 10.4kbps */
#define MC33290_YAMAHA_UART_BAUDRATE_1  0x0040DA58UL  /* 15.8kbps */
#define MC33290_PIAGGIO_UART_BAUDRATE_1 NRF_UART_BAUDRATE_19200
#define MC33290_PIAGGIO_UART_BAUDRATE_2 NRF_UART_BAUDRATE_38400
#define MC33290_RL78_UART_BAUDRATE      NRF_UART_BAUDRATE_115200

#define UART_MUX_PIN_S0         26
#define UART_MUX_PIN_S1         4

#define SPI_FLASH_IDX           0
#define SPI_FLASH_SS_PIN        34
#define SPI_FLASH_MISO_PIN      36
#define SPI_FLASH_MOSI_PIN      38
#define SPI_FLASH_SCK_PIN       9

#define EEPROM_24CXX_I2C_IDX    1
#define EEPROM_24CXX_I2C_SCL    29
#define EEPROM_24CXX_I2C_SDA    2
#define EEPROM_24CXX_ADDR       0x50
#define EEPROM_24CXX_BUF_SIZE   128

#define EEPROM_93CXX_CS_PIN     32
#define EEPROM_93CXX_DO_PIN     22
#define EEPROM_93CXX_DI_PIN     20
#define EEPROM_93CXX_SK_PIN     24

#define EEPROM_PIC_MCLR         20
#define EEPROM_PIC_DAT          22
#define EEPROM_PIC_CLK          24

#define RL78_RES_PIN            25
#define ESY_LED_PIN             12

#define POW_OUT_5V              13
#define POW_OUT_8V              10
#define POW_OUT_12V             14
 
extern const uint8_t DEVICE_VERSION[3];

#endif /* __MAIN_H */
