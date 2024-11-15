#include <vector>
#include <stdio.h>
#include "main.h"
#include "common.h"
#include "peripheral/spi_flash.h"
#include "peripheral/i2c.h"
#include "peripheral/uart.h"
#include "eeprom/9s12.h"
#include "pc_common.h"
#include "ble_common.h"
#include "eeprom/pcf7991.h"


using namespace Transponder;
using namespace EsyPro;
using namespace EEPROM;
const uint8_t DEVICE_VERSION[3] = {1, 5, 0};

namespace Global
{
    static void InitLogModule(void) {
        ret_code_t errCode;

        errCode = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(errCode);
        NRF_LOG_DEFAULT_BACKENDS_INIT();
    }

    static void InitGpio(void) {
        nrf_gpio_cfg_output(UART_MUX_PIN_S0);
        nrf_gpio_cfg_output(UART_MUX_PIN_S1);

        nrf_gpio_cfg_input(BDM_IN, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_output(BDM_OUT);
        nrf_gpio_cfg_output(BDM_OUT_CRT);
		nrf_gpio_cfg_output(POW_OUT_5V);
        nrf_gpio_cfg_output(POW_OUT_8V);
        nrf_gpio_cfg_output(POW_OUT_12V);
        nrf_gpio_pin_set(BDM_OUT_CRT);
        nrf_gpio_pin_set(BDM_OUT);
        nrf_gpio_pin_clear(POW_OUT_5V);
        nrf_gpio_pin_set(POW_OUT_8V);
        nrf_gpio_pin_clear(POW_OUT_12V);

        nrf_gpio_cfg_output(MC33290_COMM_TX_PIN);
        nrf_gpio_pin_set(MC33290_COMM_TX_PIN);

        nrf_gpio_cfg_output(MC33290_COMM_RX_PIN);
        nrf_gpio_pin_set(MC33290_COMM_RX_PIN);

        nrf_gpio_cfg_output(PC_UART_COMM_RX_PIN);
        nrf_gpio_pin_set(PC_UART_COMM_RX_PIN);
        
        nrf_gpio_cfg_output(PC_UART_COMM_TX_PIN);
        nrf_gpio_pin_set(PC_UART_COMM_TX_PIN);
        
        nrf_gpio_cfg_output(ESY_LED_PIN);
        nrf_gpio_pin_set(ESY_LED_PIN);

        nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
        nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);

        nrf_delay_ms(500);
    }

    static void InitPeripheral(void) {
        SpiFlash::Spi0Obj *spiFlash = SpiFlash::Spi0Obj::GetInstance();
        Peripheral::Uart0Obj *uart0 = Peripheral::Uart0Obj::GetInstance();
        Peripheral::Uart1Obj *uart1 = Peripheral::Uart1Obj::GetInstance();

        nrf_delay_ms(1000);

        spiFlash->Init();
        uart0->Init((nrf_uart_baudrate_t)MC33290_DEFAULT_UART_BAUDRATE);
        uart1->Init(PC_UART_COMM_BAUDRATE);

        nrf_delay_ms(1000);
    }

    static void Run(std::vector<EsyPro::CommunicationModule> &commObjs) {
        for (unsigned int i = 0; i < commObjs.size(); i++) {
            commObjs[i].RunCommunicationModuleTask();
        }
    }
}

int main(void) {
    // Global::InitLogModule();
    // Global::InitGpio();

    // EsyPro::CommunicationObj *pc = new PcCommObj();
    // EsyPro::CommunicationObj *ble = new BleCommObj();
    // EsyPro::CommunicationModule commPC(pc, EsyPro::PC_COMM_TYPE);
    // EsyPro::CommunicationModule commBLE(ble, EsyPro::BLE_COMM_TYPE);
    // std::vector<EsyPro::CommunicationModule> commObjs{commPC, commBLE};

    // Global::InitPeripheral();

    // NRF_LOG_INFO("[ESY-PRO-V2]: INFO: Start ESY-PRO-V2: v%d.%d.%d",
    //                 DEVICE_VERSION[0], DEVICE_VERSION[1], DEVICE_VERSION[2]);

    // while (1) {
    //     Global::Run(commObjs);
    // }
    Global::InitLogModule();

    NRF_LOG_INFO("[Transponder]: INFO: Start ESY-PRO-V2: v%d.%d.%d",
                 DEVICE_VERSION[0], DEVICE_VERSION[1], DEVICE_VERSION[2]);
    while (1)
    {
        ReadAllthing();
        //nrf_delay_ms(10000);
        //WritePage();
        nrf_delay_ms(5000);
    }
    return 0;
}
