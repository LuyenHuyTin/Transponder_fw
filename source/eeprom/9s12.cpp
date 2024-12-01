#include "ble_common.h"
#include "eeprom/9s12.h"
#include "nrf_sdh_soc.h"

using namespace EsyPro;
using namespace EEPROM;
using namespace MC9S12;


static const char LOG_DGB_NAME[] = "9s12";

static volatile uint32_t memAddr = 0x00;
static volatile uint8_t bdmLogicReadLevel = 1;

static MC9S12::SetupCommand mc9s12SetupCmd;
static MC9S12::ReadCommand mc9s12ReadCmd;
static MC9S12::EraseCommand mc9s12EraseCmd;
static MC9S12::WriteCommand mc9s12WriteCmd;

static volatile bool working_enable = false;
static volatile bool keep_running = true;

static Mcs9S12Option_t option;

//////////////////////////
// #include "nrf.h"
// #include "app_error.h"
// #include "nrf_gpio.h"
// #include "nrf_sdh.h"

// /**Constants for timeslot API
// */
// static nrf_radio_request_t  m_timeslot_request;
// static uint32_t             m_slot_length;

// /**@brief Radio event handler
// */
// void RADIO_timeslot_IRQHandler(void);


// /**@brief Request next timeslot event in earliest configuration
//  */
// uint32_t request_next_event_earliest(void);


// /**@brief Configure next timeslot event in earliest configuration
//  */
// void configure_next_event_earliest(void);


// /**@brief Configure next timeslot event in normal configuration
//  */
// void configure_next_event_normal(void);


// /**@brief Timeslot signal handler
//  */
// void nrf_evt_signal_handler(uint32_t evt_id);


// /**@brief Timeslot event handler
//  */
// nrf_radio_signal_callback_return_param_t * radio_callback(uint8_t signal_type);


// /**@brief Function for initializing the timeslot API.
//  */
// uint32_t timeslot_sd_init(void);

//////////////////////////

Command *MC9S12::GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType) {
    Command *cmd = NULL;

    switch (commCmdType & 0x0F) {
    case CMD_BASIC_MEM_SETUP_REQ:
        cmd = &mc9s12SetupCmd;
        NRF_LOG_INFO("[%s]: INFO: Setup Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_READ_DATA_REQ:
        cmd = &mc9s12ReadCmd;
        NRF_LOG_INFO("[%s]: INFO: Read Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_ERASE_REQ:
        cmd = &mc9s12EraseCmd;
        NRF_LOG_INFO("[%s]: INFO: Erase Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_WRITE_DATA_REQ:
        cmd = &mc9s12WriteCmd;
        NRF_LOG_INFO("[%s]: INFO: Write Request", LOG_DGB_NAME);
        break;
    }

    return cmd;
}

#if defined ( __CC_ARM   )
static __ASM void __INLINE bdmDelayTC(uint32_t volatile numOfTC) {
loop
    SUBS    R0, R0, #1
    NOP
    NOP
    NOP
    NOP
    NOP
    BNE    loop
    BX     LR
}

#elif defined ( __ICCARM__ )
static void __INLINE bdmDelayTC(uint32_t volatile numOfTC) {
__ASM (
    "loop:\n\t"
    " SUBS R0, R0, #1\n\t"
    " NOP\n\t"
    " NOP\n\t"
    " NOP\n\t"
    " NOP\n\t"
    " NOP\n\t"
    " NOP\n\t"
    " BNE.n loop\n\t");
}

#elif defined ( _WIN32 ) || defined ( __unix ) || defined( __APPLE__ )
  __STATIC_INLINE void bdmDelayTC(uint32_t volatile numOfTC);
#ifndef CUSTOM_bdmDelayTC
  __STATIC_INLINE void bdmDelayTC(uint32_t volatile numOfTC)
{}
#endif

#elif defined ( __GNUC__ )
static void __INLINE bdmDelayTC(uint32_t volatile numOfTC) __attribute__((always_inline));
static void __INLINE bdmDelayTC(uint32_t volatile numOfTC) {
register uint32_t delay __ASM ("r0") = numOfTC;
__ASM volatile (
    "1:\n"
    " SUBS %0, %0, #1\n"
    " NOP\n"
    " NOP\n"
    " NOP\n"
    " NOP\n"
    " NOP\n"
    " NOP\n"
    " NOP\n"
    " NOP\n"
    " NOP\n"
    " BNE 1b\n"
    : "+r" (delay));
}
#endif

static void bdmTx(uint8_t level) {
    nrf_gpio_pin_clear(BDM_OUT);
    nrf_gpio_pin_clear(BDM_OUT_CRT);

    bdmDelayTC(4);

    if (level) {
        nrf_gpio_pin_set(BDM_OUT);
    }
    bdmDelayTC(8);

    nrf_gpio_pin_set(BDM_OUT);
    bdmDelayTC(4);
    nrf_gpio_pin_set(BDM_OUT_CRT);
}

static uint8_t bdmRx(void) {
    nrf_gpio_pin_clear(BDM_OUT);
    nrf_gpio_pin_clear(BDM_OUT_CRT);
    bdmDelayTC(4);

    nrf_gpio_pin_set(BDM_OUT_CRT);
    bdmDelayTC(4);
    bdmLogicReadLevel = nrf_gpio_pin_read(BDM_IN);
    bdmDelayTC(10);

    return bdmLogicReadLevel;
}

static void mc9s12SendCmd(uint8_t cmd, uint8_t ack) {
    for (uint8_t i = 0; i < 8; i++) {
        if (cmd & 0x80) {
            bdmTx(1);
        } else {
            bdmTx(0);
        }

        cmd = cmd << 1;
    }

    if (ack) {
        nrf_delay_us(25);
    }
}

static uint8_t mc9s12Rx(void) {
    uint8_t byte = bdmRx();

    return byte;
}


static void mc9s12WaitForReady()
{
    do {
      // Make sure any pending events are cleared
      __SEV();
      __WFE();

    } while(working_enable == false);
}


static uint16_t _9s12ReadCmd(uint8_t cmd_header[], uint8_t length) {
    uint8_t byteData[2] = {0};

    mc9s12WaitForReady();

    NVIC_DisableIRQ(TIMER0_IRQn);

    mc9s12SendCmd(cmd_header[0], 0);
    mc9s12SendCmd(cmd_header[1], 0);
    mc9s12SendCmd(cmd_header[2], 0);
    nrf_delay_us(40);

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j <= 7; j++) {
            byteData[i] |= (mc9s12Rx() << (7 - j));
        }
    }

    NVIC_EnableIRQ(TIMER0_IRQn);
     //NRF_LOG_INFO("9s12:  _9s12ReadCmd %02X%02X%02X_%04X", cmd_header[0], cmd_header[1], cmd_header[2], (byteData[1] & 0xFF) | (byteData[0] << 8));
    return ((byteData[1] & 0xFF) | (byteData[0] << 8));
}


static void _9s12WriteCmd(uint8_t command[]) {

    mc9s12WaitForReady();

    NVIC_DisableIRQ(TIMER0_IRQn);

    mc9s12SendCmd(command[0], 0);
    mc9s12SendCmd(command[1], 0);
    mc9s12SendCmd(command[2], 0);
    mc9s12SendCmd(command[3], 0);
    mc9s12SendCmd(command[4], 0);
    nrf_delay_us(40);

    NVIC_EnableIRQ(TIMER0_IRQn);
     //NRF_LOG_INFO("9s12:  _9s12WriteCmd %02X%02X%02X_%02X%02X", command[0], command[1], command[2], command[3], command[4]);
}


// static void time_slot_soc_evt_handler(uint32_t evt_id, void * p_context)
// {
//     nrf_evt_signal_handler(evt_id);
// }

static bool eeprom_9s12_sync(void)
{
    uint16_t timeout = 0;
    nrf_gpio_pin_set(BDM_OUT_CRT);
    nrf_gpio_pin_set(BDM_OUT);
    nrf_delay_us(600);
    nrf_gpio_pin_clear(BDM_OUT_CRT);
    nrf_gpio_pin_clear(BDM_OUT);
    nrf_delay_us(155);

    nrf_gpio_pin_set(BDM_OUT);
    bdmDelayTC(10);
    nrf_gpio_pin_set(BDM_OUT_CRT);
    nrf_gpio_pin_set(BDM_OUT);

    do
    {
        bdmLogicReadLevel = nrf_gpio_pin_read(BDM_IN);
        timeout ++;
        if(timeout > 100)
        {
            //NRF_LOG_INFO("eeprom_9s12_sync FAIL");
            return false;
        }
        bdmDelayTC(10);
    } while(bdmLogicReadLevel == 0);

    do
    {
        bdmLogicReadLevel = nrf_gpio_pin_read(BDM_IN);
        timeout ++;
        if(timeout > 100)
        {
            //NRF_LOG_INFO("eeprom_9s12_sync FAIL");
            return false;
        }
        bdmDelayTC(10);
    } while(bdmLogicReadLevel == 1);

    //NRF_LOG_INFO("eeprom_9s12_sync SUCCESS");
    return true;
}


static bool eeprom_9s12_try_to_sync()
{
    bool ret;
    uint32_t try_time = 1;

    do
    {
        ret = eeprom_9s12_sync();
        try_time ++;

        if(ret)
        {
            //NRF_LOG_INFO("eeprom_9s12_sync SUCCESS");

            return true;
        }

        nrf_delay_ms(1000);
    } while (try_time < 5);

    //NRF_LOG_INFO("eeprom_9s12_sync FAIL");
    return false;
}


static void eeprom_9s12_power_on()
{
    NRF_LOG_INFO("eeprom_9s12_power_on mcu");
    nrf_gpio_pin_clear(BDM_OUT_CRT);
    nrf_gpio_pin_clear(BDM_OUT);
    nrf_gpio_pin_set(POW_OUT_5V);
    nrf_gpio_pin_set(POW_OUT_12V);
    nrf_delay_ms(300);
}

static void eeprom_9s12_power_off()
{
    NRF_LOG_INFO("eeprom_9s12_power_off mcu");
    nrf_delay_ms(200);
    nrf_gpio_pin_set(BDM_OUT_CRT);
    nrf_gpio_pin_set(BDM_OUT);
    nrf_gpio_pin_clear(POW_OUT_5V);
    nrf_gpio_pin_clear(POW_OUT_12V);
    nrf_delay_ms(300);
}

static void eeprom_9s12_hard_reset()
{
	NRF_LOG_INFO("9s12 reset mcu");
    nrf_gpio_pin_clear(POW_OUT_5V);
    nrf_gpio_pin_clear(POW_OUT_12V);
	nrf_gpio_pin_set(POW_OUT_5V);
    nrf_gpio_pin_set(POW_OUT_12V);
	nrf_delay_ms(200);
}

static void eeprom_9s12_end_communication(void)
{
  nrf_delay_ms(10);
  nrf_gpio_pin_set(BDM_OUT_CRT);
}


static uint16_t _9s12ReadCmdUntil(uint8_t cmd_header[], uint8_t length, uint16_t expect, uint16_t *out = NULL) {
    uint16_t ret = 0;
    uint16_t data_out = 0xFFFF;

    static uint32_t timeout = 0;
    if(timeout != 0)
    {
        timeout = 0;  //reset
    }

    do
    {
        data_out = _9s12ReadCmd(cmd_header, 3);
        // NRF_LOG_INFO("9s12:  _9s12ReadCmdUntil expect %04X, %02X%02X%02X_%04X", expect, cmd_header[0], cmd_header[1], cmd_header[2], data_out);

        timeout ++;

        if(timeout > 5000)
        {
            ret = 0xFFFF;
            NRF_LOG_INFO("9s12:  _9s12ReadCmdUntil timeout expect %04X, %02X%02X%02X_%04X", expect, cmd_header[0], cmd_header[1], cmd_header[2], data_out);
            return ret;
        }
		nrf_delay_ms(1);
    } while ((data_out & expect) != expect);

    if(out != NULL)
    {
        *out = data_out;
    }

    return ret;
}


static uint16_t flash_status_wait_release()
{
    uint16_t ret = 0xffff;
    uint8_t cmd[5] = {0xe4, 0x01, 0x06, 0x08, 0x00};
    ret = _9s12ReadCmdUntil(cmd, 3, 0x8000);

    return ret;
}


static uint16_t erase_common()
{
    uint8_t cmd_w_00[5] = {0xc0, 0x00, 0x3c, 0xc0, 0x00};
    uint8_t cmd_w_01[5] = {0xc0, 0x01, 0x08, 0x03, 0x00};
    uint8_t cmd_r_00[5] = {0xe0, 0x01, 0x08, 0x03, 0x00};
    uint8_t cmd_w_02[5] = {0xc0, 0x01, 0x08, 0x1a, 0x00};
    uint8_t cmd_r_01[5] = {0xe0, 0x01, 0x08, 0x1a, 0x00};
    uint8_t cmd_r_02[5] = {0xec, 0x00, 0x1a, 0xf0, 0x80};
    uint8_t cmd_w_03[5] = {0xc4, 0xff, 0x01, 0x00, 0xc0};
    uint8_t cmd_r_03[5] = {0xe0, 0xff, 0x0f, 0xff, 0xfe};
    uint8_t cmd_w_04[5] = {0xc0, 0x38, 0x00, 0x55, 0x00};
    uint8_t cmd_w_05[5] = {0xc0, 0x38, 0x02, 0xaa, 0x00};
    uint8_t cmd_r_04[5] = {0xe0, 0x38, 0x00, 0x55, 0x0bd};
    uint8_t cmd_r_05[5] = {0xe0, 0x38, 0x02, 0xaa, 0xdf};
    uint8_t cmd_w_06[5] = {0xc4, 0x01, 0x08, 0xe4, 0x00};
    uint8_t cmd_w_07[5] = {0xc4, 0x01, 0x09, 0x00, 0xff};
    uint8_t cmd_w_08[5] = {0xc4, 0x01, 0x00, 0x0b, 0x00};
    uint8_t cmd_r_06[5] = {0xe4, 0x01, 0x00, 0x8b, 0x00};
    uint8_t cmd_w_09[5] = {0xc4, 0x00, 0x13, 0x00, 0x00};
    uint8_t cmd_w_10[5] = {0xc4, 0xff, 0x01, 0x00, 0xc0};
    uint8_t cmd_r_07[5] = {0xe0, 0xff, 0x0f, 0xff, 0xfe};
    uint8_t cmd_w_11[5] = {0xc0, 0x38, 0x00, 0x55, 0x00};
    uint8_t cmd_w_12[5] = {0xc0, 0x38, 0x02, 0xaa, 0x00};
    uint8_t cmd_r_08[5] = {0xe0, 0x38, 0x00, 0x55, 0x0bd};
    uint8_t cmd_r_09[5] = {0xe0, 0x38, 0x02, 0xaa, 0xdf};
    uint8_t cmd_w_13[5] = {0xc4, 0x01, 0x08, 0xe4, 0x00};
    uint8_t cmd_w_14[5] = {0xc4, 0x01, 0x09, 0x00, 0xff};
    uint8_t cmd_w_15[5] = {0xc4, 0x01, 0x00, 0x0b, 0x00};
    uint8_t cmd_r_10[5] = {0xe4, 0x01, 0x00, 0x8b, 0x00};
    uint8_t cmd_w_16[5] = {0xc4, 0x00, 0x13, 0x00, 0x00};
    uint8_t cmd_r_11[5] = {0xe4, 0x01, 0x06, 0x80, 0x00};
    uint8_t cmd_w_17[5] = {0xc4, 0x01, 0x06, 0x30, 0x00};
    uint8_t cmd_w_18[5] = {0xc4, 0x01, 0x02, 0x00, 0x00};
}

static uint16_t erase_eeprom()
{
    uint8_t cmd_w_00[5] = {0xcc, 0x01, 0x0a, 0x09, 0x00};
    uint8_t cmd_w_01[5] = {0xc4, 0x01, 0x02, 0x01, 0x00};
    uint8_t cmd_w_02[5] = {0xcc, 0x01, 0x0a, 0x04, 0x00};
    uint8_t cmd_w_03[5] = {0xc4, 0x01, 0x06, 0x80, 0x00};
    //......................................
    uint8_t cmd_r_01[5] = {0xe4, 0x01, 0x06, 0x80, 0x00};
    uint8_t cmd_r_02[5] = {0xe4, 0x01, 0x06, 0x80, 0x00};
}

static uint16_t erase_pflash()
{
    uint8_t cmd_w_00[5] = {0xcc, 0x01, 0x0a, 0x09, 0x03};
    uint8_t cmd_w_01[5] = {0xc4, 0x01, 0x02, 0x01, 0x00};
    uint8_t cmd_w_02[5] = {0xcc, 0x01, 0x0a, 0x00, 0x00};
    uint8_t cmd_w_03[5] = {0xc4, 0x01, 0x06, 0x80, 0x00};
    //.............................................................................
    uint8_t cmd_r_00[5] = {0xe4, 0x01, 0x06, 0x80, 0x00};
    uint8_t cmd_r_01[5] = {0xe4, 0x01, 0x06, 0x80, 0x00};
    uint8_t cmd_r_02[5] = {0xe4, 0x01, 0x06, 0x80, 0x00};
    uint8_t cmd_w_04[5] = {0xc4, 0x01, 0x06, 0x30, 0x00};
    uint8_t cmd_w_05[5] = {0xc4, 0x01, 0x02, 0x00, 0x00};
    uint8_t cmd_w_06[5] = {0xcc, 0x01, 0x0a, 0x06, 0x03};
    uint8_t cmd_w_07[5] = {0xc4, 0x01, 0x02, 0x01, 0x00};
    uint8_t cmd_w_08[5] = {0xcc, 0x01, 0x0a, 0xff, 0x08};
    uint8_t cmd_w_09[5] = {0xc4, 0x01, 0x02, 0x02, 0x00};
    uint8_t cmd_w_10[5] = {0xcc, 0x01, 0x0a, 0xff, 0xff};
    uint8_t cmd_w_11[5] = {0xc4, 0x01, 0x02, 0x03, 0x00};
    uint8_t cmd_w_12[5] = {0xcc, 0x01, 0x0a, 0xff, 0xff};
    uint8_t cmd_w_13[5] = {0xc4, 0x01, 0x02, 0x04, 0x00};
    uint8_t cmd_w_14[5] = {0xcc, 0x01, 0x0a, 0xff, 0xff};
    uint8_t cmd_w_15[5] = {0xc4, 0x01, 0x02, 0x05, 0x00};
    uint8_t cmd_w_16[5] = {0xcc, 0x01, 0x0a, 0xff, 0xfe};
    uint8_t cmd_w_17[5] = {0xc4, 0x01, 0x06, 0x80, 0x00};
    //.
    uint8_t cmd_r_03[5] = {0xe4, 0x01, 0x06, 0x80, 0x00};
    uint8_t cmd_r_04[5] = {0xe4, 0x01, 0x06, 0x80, 0x00};
}


static bool erase()
{
    uint16_t ret = 0;
    uint16_t data_out = 0;
    uint8_t cmd_w_00[5] = {0xc0, 0x00, 0x3c, 0x00, 0x00};
    uint8_t cmd_r_00[5] = {0xe0, 0x00, 0x0b, 0x00, 0x00};
    uint8_t cmd_r_01[5] = {0xe4, 0xff, 0x01, 0x00, 0xc0};
    uint8_t cmd_w_01[5] = {0xc0, 0x01, 0x00, 0x0b, 0x0b};
    uint8_t cmd_r_02[5] = {0xe0, 0x01, 0x00, 0x8b, 0x00};
    uint8_t cmd_w_02[5] = {0xc0, 0x01, 0x08, 0xff, 0xff};
    uint8_t cmd_w_03[5] = {0xc0, 0x01, 0x09, 0xff, 0xff};
    uint8_t cmd_w_04[5] = {0xc0, 0x01, 0x06, 0x30, 0x30};
    uint8_t cmd_w_05[5] = {0xc0, 0x01, 0x02, 0x00, 0x00};
    uint8_t cmd_w_06[5] = {0xc8, 0x01, 0x0a, 0x08, 0x00};
    uint8_t cmd_w_07[5] = {0xc0, 0x01, 0x06, 0x80, 0x80};
    uint8_t cmd_wait[5] = {0xe0, 0x01, 0x06, 0x08, 0x00};
    uint8_t cmd_r_03[5] = {0xe0, 0x01, 0x07, 0x00, 0x00};

    NRF_LOG_INFO("9s12:  ERASE request");

    _9s12WriteCmd(cmd_w_00);
    nrf_delay_ms(40);
    _9s12ReadCmd(cmd_r_00, 3);
    _9s12ReadCmd(cmd_r_01, 3);
    nrf_delay_ms(520);
    _9s12WriteCmd(cmd_w_01);
    nrf_delay_us(350);
    _9s12ReadCmd(cmd_r_02, 3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd_w_02);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd_w_03);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd_w_04);
    _9s12WriteCmd(cmd_w_05);
    _9s12WriteCmd(cmd_w_06);
    _9s12WriteCmd(cmd_w_07);
    ret = _9s12ReadCmdUntil(cmd_wait, 3, 0x8000);
    if(ret != EEPROM_E_OK)
        return 0xFFFF;
    _9s12ReadCmd(cmd_r_03, 3);

    //again
    nrf_delay_ms(200);
	eeprom_9s12_power_off();
	eeprom_9s12_power_on();
    eeprom_9s12_try_to_sync();

    _9s12WriteCmd(cmd_w_00);
    _9s12ReadCmd(cmd_r_00, 3);
    _9s12ReadCmd(cmd_r_01, 3);
    _9s12WriteCmd(cmd_w_01);
    _9s12ReadCmd(cmd_r_02, 3);
    _9s12WriteCmd(cmd_w_02);
    _9s12WriteCmd(cmd_w_03);
    _9s12WriteCmd(cmd_w_04);
    _9s12WriteCmd(cmd_w_05);

    uint8_t cmd_w_10[5] = {0xc8, 0x01, 0x0a, 0x06, 0x03};
    uint8_t cmd_w_11[5] = {0xc0, 0x01, 0x02, 0x01, 0x01};
    uint8_t cmd_w_12[5] = {0xc8, 0x01, 0x0a, 0xff, 0x08};
    uint8_t cmd_w_13[5] = {0xc0, 0x01, 0x02, 0x02, 0x02};
    uint8_t cmd_w_14[5] = {0xc8, 0x01, 0x0a, 0xff, 0xff};
    uint8_t cmd_w_15[5] = {0xc0, 0x01, 0x02, 0x03, 0x03};
    uint8_t cmd_w_16[5] = {0xc8, 0x01, 0x0a, 0xff, 0xff};
    uint8_t cmd_w_17[5] = {0xc0, 0x01, 0x02, 0x04, 0x04};
    uint8_t cmd_w_18[5] = {0xc8, 0x01, 0x0a, 0xff, 0xff};
    uint8_t cmd_w_19[5] = {0xc0, 0x01, 0x02, 0x05, 0x05};
    uint8_t cmd_w_20[5] = {0xc8, 0x01, 0x0a, 0xff, 0xfe};
    uint8_t cmd_w_21[5] = {0xc0, 0x01, 0x06, 0x80, 0x80};
    uint8_t cmd_r_10[5] = {0xe0, 0x01, 0x06, 0x80, 0x00};
    uint8_t cmd_r_11[5] = {0xe0, 0x01, 0x07, 0x00, 0x00};

    _9s12WriteCmd(cmd_w_10);
    _9s12WriteCmd(cmd_w_11);
    _9s12WriteCmd(cmd_w_12);
    _9s12WriteCmd(cmd_w_13);
    _9s12WriteCmd(cmd_w_14);
    _9s12WriteCmd(cmd_w_15);
    _9s12WriteCmd(cmd_w_16);
    _9s12WriteCmd(cmd_w_17);
    _9s12WriteCmd(cmd_w_18);
    _9s12WriteCmd(cmd_w_19);
    _9s12WriteCmd(cmd_w_20);
    _9s12WriteCmd(cmd_w_21);
    ret = _9s12ReadCmdUntil(cmd_wait, 3, 0x8000);
    if(ret != EEPROM_E_OK)
        return 0xFFFF;
    _9s12ReadCmd(cmd_r_10, 3);
    _9s12ReadCmd(cmd_r_11, 3);

    NRF_LOG_INFO("9s12:  ERASE request Done");
    nrf_delay_ms(200);
    eeprom_9s12_power_off();

    return ret;
}

static uint16_t setupWrite_eeprom() {
    uint8_t cmd1[] = {0xc0, 0x00, 0x3c, 0x00, 0x00};
    uint8_t cmdR0[] = {0xe0, 0x00, 0x0b}; // 00 00
    uint8_t cmdR1[] = {0xe4, 0xff, 0x01}; // 00 C0 or C8
    uint8_t cmdR2[] = {0xe0, 0x01, 0x01}; // 00 FE
    uint8_t cmd2[] = {0xc0, 0x01, 0x00, 0x0b, 0x0b};
    uint8_t cmdR3[] = {0xe0, 0x01, 0x00}; // 8B 00
    uint8_t cmd3[] = {0xc0, 0x01, 0x08, 0xff, 0xff};
    uint8_t cmd4[] = {0xc0, 0x01, 0x09, 0xff, 0xff};
    uint16_t ret = 0;
	uint16_t data_out = 0;

    NRF_LOG_INFO("setupWrite_eeprom");
    eeprom_9s12_power_on();
    eeprom_9s12_try_to_sync();
    memAddr = 0x400;

    _9s12WriteCmd(cmd1);
    nrf_delay_ms(50);
    _9s12ReadCmd(cmdR0, 3);
    nrf_delay_us(350);
    data_out = _9s12ReadCmd(cmdR1, 3);
    nrf_delay_us(350);
    if(data_out == 0)
    {
        NRF_LOG_INFO("setupWrite_eeprom Fail");
        ret = 0xFFFF;
    }
    _9s12ReadCmd(cmdR2, 3);
    nrf_delay_ms(520);
    _9s12WriteCmd(cmd2);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR3, 3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd4);
    nrf_delay_us(350);

    NRF_LOG_INFO("setupWrite_eeprom Done");

    return ret;
}

static uint16_t write_eeprom(uint32_t address, const uint8_t *eeprom_bin)
{
    uint16_t ret = 0;
    uint8_t addrH = (address >> 8) & 0xFF;
    uint8_t addrL = address & 0xFF;

    uint8_t cmd5[] = {0xc0, 0x01, 0x6, 0x30, 0x30};
    uint8_t cmd6[] = {0xc0, 0x01, 0x2, 0x00, 0x00};
    uint8_t cmd7[] = {0xc8, 0x01, 0xa, 0x12, 0x00};
    uint8_t cmd8[] = {0xc0, 0x01, 0x2, 0x01, 0x01};
    uint8_t cmd_address[] = {0xc8, 0x01, 0xa, addrH, addrL};  // 2 byte, start from 0x400
    uint8_t cmd10[] = {0xc0, 0x01, 0x6, 0x80, 0x80};
    uint8_t cmdR4[] = {0xe0, 0x01, 0x06}; // loop 08 00
    uint8_t cmdR5[] = {0xe0, 0x01, 0x07}; //00 00
    uint8_t cmd11[] = {0xc0, 0x01, 0x6, 0x30, 0x30};
    uint8_t cmd12[] = {0xc0, 0x01, 0x2, 0x00, 0x00};
    uint8_t cmd13[] = {0xc8, 0x01, 0xa, 0x11, 0x00};
    // uint8_t cmd_address[] = {0xc8, 0x01, 0xa, addrH, addrL};
    uint8_t cmd14[] = {0xc0, 0x01, 0x2, 0x01, 0x01};
    uint8_t cmd16[] = {0xc0, 0x01, 0x2, 0x2, 0x2};
    uint8_t cmd_data1[] = {0xc8, 0x01, 0xa, eeprom_bin[0], eeprom_bin[1]};   // 2 byte data
    uint8_t cmd18[] = {0xc0, 0x01, 0x2, 0x3, 0x3};
    uint8_t cmd_data2[] = {0xc8, 0x01, 0xa, eeprom_bin[2], eeprom_bin[3]};   // 2 byte data
    uint8_t cmd20[] = {0xc0, 0x01, 0x6, 0x80, 0x80};

    NRF_LOG_INFO("9s12:  write_eeprom address %X: %02X%02X%02X%02X", address,
                 eeprom_bin[0], eeprom_bin[1], eeprom_bin[2], eeprom_bin[3]);

    _9s12WriteCmd(cmd5);
    _9s12WriteCmd(cmd6);
    _9s12WriteCmd(cmd7);
    _9s12WriteCmd(cmd8);
    _9s12WriteCmd(cmd_address);
    _9s12WriteCmd(cmd10);
    ret = _9s12ReadCmdUntil(cmdR4, 3, 0x8000);
    _9s12ReadCmd(cmdR5, 3);

    _9s12WriteCmd(cmd11);
    _9s12WriteCmd(cmd12);
    _9s12WriteCmd(cmd13);
    _9s12WriteCmd(cmd14);
    _9s12WriteCmd(cmd_address);
    _9s12WriteCmd(cmd16);
    _9s12WriteCmd(cmd_data1);
    _9s12WriteCmd(cmd18);
    _9s12WriteCmd(cmd_data2);
    _9s12WriteCmd(cmd20);
    ret = _9s12ReadCmdUntil(cmdR4, 3, 0x8000);
    _9s12ReadCmd(cmdR5, 3);

    return ret;
}

static uint16_t setupRead_eeprom()
{
    uint8_t cmd1[] = {0xc0, 0x00, 0x3c, 0x00, 0x00};
    uint8_t cmdR0[] = {0xe0, 0x00, 0x0b}; // 00 00
    uint8_t cmdR1[] = {0xe4, 0xff, 0x01}; // 00 C0 or C8
    uint8_t cmdR2[] = {0xe0, 0x01, 0x01};
    uint16_t data_out = 0;
    uint16_t ret = 0;
    memAddr = 0x400;

    NRF_LOG_INFO("setupRead_eeprom");
    eeprom_9s12_power_on();
    eeprom_9s12_try_to_sync();

    _9s12WriteCmd(cmd1);
    nrf_delay_ms(50);
    _9s12ReadCmd(cmdR0, 3);
    data_out = _9s12ReadCmd(cmdR1, 3);
    if(data_out != 0x00C0)
    {
        NRF_LOG_INFO("setupRead_eeprom Fail");
        ret = 0xFFFF;
    }

    _9s12ReadCmd(cmdR2, 3);
    nrf_delay_ms(500);

    NRF_LOG_INFO("setupRead_eeprom Done");

    return ret;
}

static uint16_t read_eeprom(uint32_t address)
{
    uint16_t readData;
    uint8_t cmd[] = {0xE0, 0x00, 0x00};
    cmd[1] = address >> 8;
    cmd[2] = address & 0xFF;

    readData = _9s12ReadCmd(cmd, 3);

    return readData;
}

static uint16_t setupRead_pflash()
{
    uint8_t cmd1[] = {0xc0, 0x00, 0x3c, 0x00, 0x00};
    uint8_t cmdR0[] = {0xe0, 0x00, 0x0b}; // 00 00
    uint8_t cmdR1[] = {0xe4, 0xff, 0x01}; // 00 C0 or C8
    uint8_t cmdR2[] = {0xe0, 0x01, 0x01};


    uint16_t data_out = 0;
    uint16_t ret = 0x0;

    NRF_LOG_INFO("setupRead_pflash");
    eeprom_9s12_power_on();
    eeprom_9s12_try_to_sync();

    memAddr = 0;

    _9s12WriteCmd(cmd1);
    nrf_delay_ms(50);
    _9s12ReadCmd(cmdR0, 3);
    nrf_delay_us(350);
    // ret = _9s12ReadCmdUntil(cmdR1, 3, 0x00CF);
    // if(ret != EEPROM_E_OK)
    //  return 0xFFFF;
    data_out = _9s12ReadCmd(cmdR1, 3);
    nrf_delay_us(350);
    if(data_out != 0x00C0)
    {
        NRF_LOG_INFO("setupRead_pflash Fail");
        ret = 0xFFFF;
        return ret;
    }
    _9s12ReadCmd(cmdR2, 3);
    nrf_delay_us(350);

    NRF_LOG_INFO("setupRead_pflash Done");

    return ret;
}

static uint16_t read_pflash(uint32_t address)
{
    uint16_t readData = 0;
    uint16_t buffer_address = 0x8000;

    uint8_t cmd1[] = {0xc0, 0x00, 0x15, 0xC, 0xC};
    uint8_t cmdR0[] = {0xe0, 0, 0}; // 00 00

    /*
    Pflash phân thành 4 vùng C, D, E, F,
    Mỗi vùng địa chỉ từ 0x8000 - 0x0bFFF (size 0x4000)
    Bắt đầu đọc cần khởi tạo vùng địa chỉ cần đọc
    */
    if(address % 0x4000 == 0)

    {
        NRF_LOG_INFO("9s12read_pflash address %X", address);
        switch (address/0x4000)
        {
        case 0:
            cmd1[3] = 0x0C;
            cmd1[4] = 0x0C;
            address = address - 0x4000 * 0;
            break;
        case 1:
            cmd1[3] = 0x0D;
            cmd1[4] = 0x0D;
            address = address - 0x4000 * 1;
            break;
        case 2:
            cmd1[3] = 0x0E;
            cmd1[4] = 0x0E;
            address = address - 0x4000 * 2;
            break;
        case 3:
            cmd1[3] = 0x0F;
            cmd1[4] = 0x0F;
            address = address - 0x4000 * 3;
            break;
        default:
            NRF_LOG_INFO("9s12read_pflash out of memory");
            break;
        }
        _9s12WriteCmd(cmd1);
    }

    cmdR0[1] = (buffer_address + address % 0x4000) >> 8;
    cmdR0[2] = (buffer_address + address % 0x4000) & 0xFF;
    readData = _9s12ReadCmd(cmdR0, 3);

    return readData;
}


static uint16_t setupWrite_pflash()
{
    uint16_t ret = 0;
	uint16_t data_out = 0;
    uint8_t cmd1[] = {0xc0, 0x00, 0x3c, 0x00, 0x00};
    uint8_t cmdR0[] = {0xe0, 0x00, 0x0b}; // 00 00
    uint8_t cmdR1[] = {0xe4, 0xff, 0x01}; // 00 C0 or C8
    uint8_t cmdR2[] = {0xe0, 0x01, 0x01}; // 00 FE
    uint8_t cmd2[] = {0xc0, 0x01, 0x00, 0x0b, 0x0b};
    uint8_t cmdR3[] = {0xe0, 0x01, 0x00}; // 8B 00
    uint8_t cmd3[] = {0xc0, 0x01, 0x08, 0xff, 0xff};
    uint8_t cmd4[] = {0xc0, 0x01, 0x09, 0xff, 0xff};


    uint8_t cmd_test1[] = {0xc4, 0xff, 0x01, 0x00, 0xC0};

    memAddr = 0;

    NRF_LOG_INFO("setupWrite_pflash");
    eeprom_9s12_power_on();
    eeprom_9s12_try_to_sync();

    _9s12WriteCmd(cmd1);
    nrf_delay_ms(50);
    _9s12ReadCmd(cmdR0, 3);
    do {
        data_out = _9s12ReadCmd(cmdR1, 3);
    } while(data_out == 0xFFFF);
    if(data_out != 0x00C0)
    {
        NRF_LOG_INFO("setupWrite_pflash FAIL");
        return 0xFFFF;
    }
    _9s12ReadCmd(cmdR2, 3);
    nrf_delay_ms(520);
    _9s12WriteCmd(cmd2);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR3, 3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd4);
    nrf_delay_ms(1);

    return 0;
}

static uint16_t write_pflash_wait_success(uint32_t address)
{
    uint16_t ret = 0;
    uint8_t addrH = (address >> 8) & 0xFF;
    uint8_t addrL = address & 0xFF;

    uint8_t cmd11[] = {0xc0, 0x01, 0x6, 0x30, 0x30};
    uint8_t cmd12[] = {0xc0, 0x01, 0x2, 0x00, 0x00};
    uint8_t cmd7[] = {0xc8, 0x01, 0xa, 0xa, 0x3};
    uint8_t cmd8[] = {0xc0, 0x01, 0x2, 0x01, 0x01};
    uint8_t cmd9[] = {0xc8, 0x01, 0xa, addrH, addrL};
    uint8_t cmd19[] = {0xc0, 0x01, 0x6, 0x80, 0x80};
    uint8_t cmdR4[] = {0xe0, 0x01, 0x6}; // 8B 00
    uint8_t cmdR5[] = {0xe0, 0x01, 0x07}; //00 00

    _9s12WriteCmd(cmd11);
    _9s12WriteCmd(cmd12);
    _9s12WriteCmd(cmd7);
    _9s12WriteCmd(cmd8);
    _9s12WriteCmd(cmd9);
    _9s12WriteCmd(cmd19);

    ret = _9s12ReadCmdUntil(cmdR4, 3, 0x8000);
    if(ret != EEPROM_E_OK)
    {
        NRF_LOG_INFO("write_pflash_wait_success Fail");
        ret = 0xFFFF;
    }
    _9s12ReadCmd(cmdR5, 3);

    return ret;
}

static uint16_t write_pflash(uint32_t address, const uint8_t *pflash_bin)
{
    uint16_t ret = 0;
    uint8_t addrH = (address >> 8) & 0xFF;
    uint8_t addrL = address & 0xFF;
    uint8_t cmdR4[] = {0xe0, 0x01, 0x6}; // 8B 00
    uint8_t cmdR5[] = {0xe0, 0x01, 0x07}; //00 00
    uint8_t cmd11[] = {0xc0, 0x01, 0x6, 0x30, 0x30};
    uint8_t cmd12[] = {0xc0, 0x01, 0x2, 0x00, 0x00};
    uint8_t cmd13[] = {0xc8, 0x01, 0xa, 0x6, 0x3};
    uint8_t cmd14[] = {0xc0, 0x01, 0x2, 0x01, 0x01};
    uint8_t cmd_address[] = {0xc8, 0x01, 0xa, addrH, addrL};
    uint8_t cmd15[] = {0xc0, 0x01, 0x2, 0x2, 0x2};
    uint8_t cmd_data1[] = {0xc8, 0x01, 0xa, pflash_bin[0], pflash_bin[1]};   // 2 byte data
    uint8_t cmd16[] = {0xc0, 0x01, 0x2, 0x3, 0x3};
    uint8_t cmd_data2[] = {0xc8, 0x01, 0xa, pflash_bin[2], pflash_bin[3]};   // 2 byte data
    uint8_t cmd17[] = {0xc0, 0x01, 0x2, 0x4, 0x4};
    uint8_t cmd_data3[] = {0xc8, 0x01, 0xa, pflash_bin[4], pflash_bin[5]};   // 2 byte data
    uint8_t cmd18[] = {0xc0, 0x01, 0x2, 0x5, 0x5};
    uint8_t cmd_data4[] = {0xc8, 0x01, 0xa, pflash_bin[6], pflash_bin[7]};   // 2 byte data
    uint8_t cmd19[] = {0xc0, 0x01, 0x6, 0x80, 0x80};

    NRF_LOG_INFO("9s12:  write_pflash address %04X: some data:%02X%02X%02X%02X", address, pflash_bin[0], pflash_bin[1], pflash_bin[2], pflash_bin[3]);
    if(address % 0x200 == 0)
    {
        NRF_LOG_INFO("9s12:  write_pflash_wait_success");
        // nrf_delay_ms(10);
        write_pflash_wait_success(address);
        nrf_delay_ms(10);
    }

    _9s12WriteCmd(cmd11);
    _9s12WriteCmd(cmd12);
    _9s12WriteCmd(cmd13);
    _9s12WriteCmd(cmd14);
    _9s12WriteCmd(cmd_address);
    _9s12WriteCmd(cmd15);
    _9s12WriteCmd(cmd_data1);
    _9s12WriteCmd(cmd16);
    _9s12WriteCmd(cmd_data2);
    _9s12WriteCmd(cmd17);
    _9s12WriteCmd(cmd_data3);
    _9s12WriteCmd(cmd18);
    _9s12WriteCmd(cmd_data4);
    _9s12WriteCmd(cmd19);
    ret = _9s12ReadCmdUntil(cmdR4, 3, 0x8000);
    _9s12ReadCmd(cmdR5, 3);

    return ret;
}


void SetupCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType) {
    uint16_t ret = 0xFFFF;
    bool is_ping = false;
    nrf_gpio_cfg_output(POW_OUT_5V);
    nrf_gpio_cfg_output(POW_OUT_12V);
    //NRF_LOG_INFO("commReqPacket %02X", commReqPacket->buffer[7]);
    option = (Mcs9S12Option_t)commReqPacket->buffer[7];
    is_ping = (commReqPacket->buffer[7] >> 7);

    //mc9s12WaitForReady();
    eeprom_9s12_power_off();
    working_enable = false;

    if(is_ping)
    {
        NRF_LOG_INFO("9s12 ping request");
        //try to sync
        eeprom_9s12_power_on();
        nrf_delay_ms(500);
        if(eeprom_9s12_try_to_sync() == false)
        {
           NRF_LOG_INFO("9s12_try_to_sync FAILD");
           eeprom_9s12_hard_reset();
           ret = 0xFFFF;
        }
        else
        {
           ret = EEPROM_E_OK;
           NRF_LOG_INFO("9s12_try_to_sync SUCCESS");
        }
        nrf_delay_ms(200);
        eeprom_9s12_power_off();
    }
    else //setup message
    {
        //initialize the timeslot
        //NRF_SDH_SOC_OBSERVER(m_time_slot_soc_observer, 0, time_slot_soc_evt_handler, NULL);
        //timeslot_sd_init();

        NRF_LOG_INFO("9s12 setup request");
        // eeprom_9s12_power_on();
        // eeprom_9s12_try_to_sync();

        if (option == READ_EEPROM) {
            NRF_LOG_INFO("9s12 setup READ_EEPROM");
            ret = setupRead_eeprom();

        } else if (option == READ_PFLASH) {
            NRF_LOG_INFO("9s12 setup READ_PFLASH");
            ret = setupRead_pflash();

        } else if (option == WRITE_EEPROM) {
            NRF_LOG_INFO("9s12 setup WRITE_EEPROM");
            ret = setupWrite_eeprom();
            // ret = 0;
        } else {
            NRF_LOG_INFO("9s12 setup WRITE_PFLASH");
            ret = setupWrite_pflash();
        }
		ret = 0;
    }

    if (ret == EEPROM_E_OK) {
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
        commResPacket->buffer[0] = 0x01;
        commResPacket->bufLen = 1;

    } else {
		commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_IGNORE_RES;
        commResPacket->buffer[0] = 0x00;
        commResPacket->bufLen = 1;
    }
    this->SetCommandRepeatState(false);

}

void ReadCommand::Execute(CommPacket_t *commResPacket,
                          const CommPacket_t *commReqPacket,
                          CommunicationType_t commType) {
    uint16_t readData;

    working_enable = false;

    for (uint8_t i = 0; i < EEPROM_9S12_READ_WRITE_SIZE; i += 2) {
        if (option == READ_EEPROM) {
            readData = read_eeprom(memAddr);

        } else if (option == READ_PFLASH) {
            readData = read_pflash(memAddr);
        }

        commResPacket->buffer[i] = (readData >> 8) & 0xFF;
        commResPacket->buffer[i + 1] = readData & 0xFF;
        memAddr += 2;
    }

    //for (int i = 0; i < EEPROM_9S12_READ_WRITE_SIZE; i++) {
    //    NRF_LOG_INFO("%x", commResPacket->buffer[i]);
    //}

    commResPacket->cmd = CMD_BASIC_MEM_READ_DATA_RES;
    commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
    commResPacket->bufLen = EEPROM_9S12_READ_WRITE_SIZE;

    ////NRF_LOG_INFO("ReadCommand option %d, address %04X data %02X%02X%02X", option, memAddr - EEPROM_9S12_READ_WRITE_SIZE, commResPacket->buffer[0], commResPacket->buffer[1], commResPacket->buffer[2]);

    this->SetCommandRepeatState(false);
}

void EraseCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType) {
    uint16_t ret = 0;

    NRF_LOG_INFO("EraseCommand option %d, address %04X", option, memAddr);
    eeprom_9s12_power_off();
    eeprom_9s12_power_on();
    eeprom_9s12_try_to_sync();
    nrf_delay_ms(1);

    ret = erase();

    if(ret == EEPROM_E_OK) {
        commResPacket->cmd = CMD_BASIC_MEM_ERASE_RES;
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->bufLen = 0;
        NRF_LOG_INFO("EraseCommand DONE");
    }
    else
    {
        commResPacket->cmd = CMD_IGNORE_RES;
    }

    this->SetCommandRepeatState(false);
}

void WriteCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType) {
    uint16_t ret = 0;
	uint16_t length = 4;

	if (option == WRITE_EEPROM)
		length = 4;
	else if (option == WRITE_PFLASH)
		length = 8;

    for (uint8_t i = 0; i < EEPROM_9S12_READ_WRITE_SIZE; i += length) {
        if (option == WRITE_EEPROM) {
            // NRF_LOG_INFO("WriteCommand WRITE_EEPROM, address %04X", memAddr);
            ret = write_eeprom(memAddr, &commReqPacket->buffer[i]);
            memAddr += 4;
        } else if (option == WRITE_PFLASH) {
            // NRF_LOG_INFO("WriteCommand WRITE_PFLASH, address %04X", memAddr);
            ret = write_pflash(memAddr, &commReqPacket->buffer[i]);
            memAddr += 8;
        }

        if(ret != EEPROM_E_OK)
            break;
    }

    if(ret == EEPROM_E_OK) {
        commResPacket->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->bufLen = 0;
    }
    else
    {
        commResPacket->cmd = CMD_IGNORE_RES;
    }

    this->SetCommandRepeatState(false);
}

/////////////////////////////////


// static nrf_radio_signal_callback_return_param_t signal_callback_return_param;

// /**@brief Request next timeslot event in earliest configuration
//  */
// uint32_t request_next_event_earliest(void)
// {
//     m_slot_length                                  = 10000;
//     m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
//     m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
//     m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
//     m_timeslot_request.params.earliest.length_us   = m_slot_length;
//     m_timeslot_request.params.earliest.timeout_us  = 1000000;
//     return sd_radio_request(&m_timeslot_request);
// }


// /**@brief Configure next timeslot event in earliest configuration
//  */
// void configure_next_event_earliest(void)
// {
//     m_slot_length                                  = 10000;
//     m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
//     m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
//     m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
//     m_timeslot_request.params.earliest.length_us   = m_slot_length;
//     m_timeslot_request.params.earliest.timeout_us  = 1000000;
// }


// /**@brief Configure next timeslot event in normal configuration
//  */
// void configure_next_event_normal(void)
// {
//     m_slot_length                                 = 10000;
//     m_timeslot_request.request_type               = NRF_RADIO_REQ_TYPE_NORMAL;
//     m_timeslot_request.params.normal.hfclk        = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
//     m_timeslot_request.params.normal.priority     = NRF_RADIO_PRIORITY_HIGH;
//     m_timeslot_request.params.normal.distance_us  = 100000;
//     m_timeslot_request.params.normal.length_us    = m_slot_length;
// }


// /**@brief Timeslot signal handler
//  */
// void nrf_evt_signal_handler(uint32_t evt_id)
// {
//     uint32_t err_code;

//     switch (evt_id)
//     {
//         case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
//             //No implementation needed
//             break;
//         case NRF_EVT_RADIO_SESSION_IDLE:
//             working_enable = false;
//             if (keep_running)
//             {
//                 err_code = request_next_event_earliest();
//                 APP_ERROR_CHECK(err_code);
//             }
//             //No implementation needed
//             break;
//         case NRF_EVT_RADIO_SESSION_CLOSED:
//             //No implementation needed, session ended
//             working_enable = false;
//             break;
//         case NRF_EVT_RADIO_BLOCKED:
//             //Fall through
//         case NRF_EVT_RADIO_CANCELED:
//             if (keep_running)
//             {
//                 err_code = request_next_event_earliest();
//                 APP_ERROR_CHECK(err_code);
//             }
//             break;
//         default:
//             break;
//     }
// }


// /**@brief Timeslot event handler
//  */
// nrf_radio_signal_callback_return_param_t * radio_callback(uint8_t signal_type)
// {
//     switch(signal_type)
//     {
//         case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
//             //Start of the timeslot - set up timer interrupt
//             signal_callback_return_param.params.request.p_next = NULL;
//             signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
//             NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
//             NRF_TIMER0->CC[0] = m_slot_length - 1000;
//             NVIC_EnableIRQ(TIMER0_IRQn);
//             //nrf_gpio_pin_set(13); //Toggle LED
//             working_enable = true;
//             break;

//         case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
//             signal_callback_return_param.params.request.p_next = NULL;
//             signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
//             break;

//         case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
//             working_enable = false;
//             //nrf_gpio_pin_clear(13);
//             signal_callback_return_param.params.extend.length_us = m_slot_length;
//             if(keep_running)
//             {
//               //Timer interrupt - do graceful shutdown - schedule next timeslot
//               configure_next_event_normal();
//               signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
//             }
//             else
//             {
//               signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
//             }
//             break;
//         case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
//             //No implementation needed
//             //nrf_gpio_pin_set(13);
//             //working_enable = true;
//             break;
//         case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
//             //Try scheduling a new timeslot
//             //nrf_gpio_pin_clear(13);
//             working_enable = false;
//             configure_next_event_earliest();
//             signal_callback_return_param.params.request.p_next = &m_timeslot_request;
//             signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
//             break;
//         default:
//             //No implementation needed
//             break;
//     }
//     return (&signal_callback_return_param);
// }


// /**@brief Function for initializing the timeslot API.
//  */
// uint32_t timeslot_sd_init(void)
// {
//     uint32_t err_code;

//     keep_running = true;
//     working_enable = false;

//     nrf_delay_ms(100);

//     err_code = sd_radio_session_open(radio_callback);
//     if (err_code != NRF_SUCCESS)
//     {
//         NRF_LOG_INFO("sd_radio_session_open FAIL")
//         return err_code;
//     }

//     err_code = request_next_event_earliest();
//     if (err_code != NRF_SUCCESS)
//     {
//         (void)sd_radio_session_close();
//         NRF_LOG_INFO("sd_radio_session_close FAIL")
//         return err_code;
//     }
//     NRF_LOG_INFO("timeslot_sd_init done")

//     //some delay
//     nrf_delay_us(m_slot_length);
//     return NRF_SUCCESS;
// }
// /////////////////////////////////

