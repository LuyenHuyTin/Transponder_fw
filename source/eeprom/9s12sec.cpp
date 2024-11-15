#include "ble_common.h"
#include "eeprom/9s12sec.h"
#include "nrf_sdh_soc.h"

using namespace EsyPro;
using namespace EEPROM;
using namespace MC9S12Sec;


static const char LOG_DGB_NAME[] = "9s12sec";

static volatile uint32_t memAddr = 0x00;
static volatile uint8_t bdmLogicReadLevel = 1;

static MC9S12Sec::SetupCommand mc9s12SetupCmd;
static MC9S12Sec::ReadCommand mc9s12ReadCmd;
static MC9S12Sec::EraseCommand mc9s12EraseCmd;
static MC9S12Sec::WriteCommand mc9s12WriteCmd;

static volatile bool working_enable = false;
static volatile bool keep_running = true;

static Mcs9S12SecOption_t option;
static bool is9s12sec256 = false;

//////////////////////////
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_sdh.h"

/**Constants for timeslot API
*/
static nrf_radio_request_t  m_timeslot_request;
static uint32_t             m_slot_length;

/**@brief Radio event handler
*/
void RADIO_timeslot_IRQHandler(void);


/**@brief Request next timeslot event in earliest configuration
 */
uint32_t mc9s12Sec_request_next_event_earliest(void);


/**@brief Configure next timeslot event in earliest configuration
 */
void mc9s12Sec_configure_next_event_earliest(void);


/**@brief Configure next timeslot event in normal configuration
 */
void mc9s12Sec_configure_next_event_normal(void);


/**@brief Timeslot signal handler
 */
void mc9s12Sec_nrf_evt_signal_handler(uint32_t evt_id);


/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t * mc9s12Sec_radio_callback(uint8_t signal_type);


/**@brief Function for initializing the timeslot API.
 */
uint32_t mc9s12Sec_timeslot_sd_init(void);

//////////////////////////

Command *MC9S12Sec::GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType) {
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

    nrf_delay_us(150);
    NRF_LOG_INFO("_9s12ReadCmd: %02X%02X%02X_%04X", cmd_header[0], cmd_header[1], cmd_header[2], (byteData[1] & 0xFF) | (byteData[0] << 8));
    return ((byteData[1] & 0xFF) | (byteData[0] << 8));
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
        // NRF_LOG_INFO("9s12sec:  _9s12ReadCmdUntil expect %04X, %02X%02X%02X_%04X", expect, cmd_header[0], cmd_header[1], cmd_header[2], data_out);

        timeout ++;

        if(timeout > 5000)
        {
            ret = 0xFFFF;
            NRF_LOG_INFO("9s12sec:  _9s12ReadCmdUntil timeout expect %04X, %02X%02X%02X_%04X", expect, cmd_header[0], cmd_header[1], cmd_header[2], data_out);
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

static void _9s12WriteCmd(uint8_t command[]) {

    mc9s12WaitForReady();

    NVIC_DisableIRQ(TIMER0_IRQn);

    mc9s12SendCmd(command[0], 0);
    mc9s12SendCmd(command[1], 0);
    mc9s12SendCmd(command[2], 0);
    mc9s12SendCmd(command[3], 0);
    mc9s12SendCmd(command[4], 0);

    NVIC_EnableIRQ(TIMER0_IRQn);

    nrf_delay_us(40);
    NRF_LOG_INFO("_9s12WriteCmd: %02X%02X%02X%02X%02X", command[0], command[1], command[2], command[3], command[4]);
}


static void time_slot_soc_evt_handler(uint32_t evt_id, void * p_context)
{
    mc9s12Sec_nrf_evt_signal_handler(evt_id);
}

static bool eeprom_9s12_sync(void)
{
  uint16_t timeout = 0;
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


static bool eeprom_9s12sec_try_to_sync()
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

        nrf_delay_ms(500);
    } while (try_time < 8);

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
    nrf_delay_ms(300);
    nrf_gpio_pin_clear(POW_OUT_5V);
    nrf_gpio_pin_clear(POW_OUT_12V);
    nrf_gpio_pin_set(BDM_OUT_CRT);
    nrf_gpio_pin_set(BDM_OUT);
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
  nrf_gpio_pin_set(BDM_OUT);
}


static void erase_sec()
{
    //Not support
}

static uint16_t setupWrite_pflash_sec128()
{
    NRF_LOG_INFO("9s12sec: setupWrite_pflash_sec128 request");
    uint16_t ret = 0;
    uint8_t cmd1[] = {0xc0, 0x0, 0x3c, 0x0, 0x0};
    uint8_t cmdR0[] = {0xe0, 0x0, 0xb}; // 00 00
    uint8_t cmdR1[] = {0xe4, 0xff, 0x1}; // 00 C0
    uint8_t cmdR2[] = {0xe0, 0x01, 0x1};
    uint8_t cmd2[] = {0xc0, 0x1, 0x0, 0x9, 0x9};
    uint8_t cmdR3[] = {0xe0, 0x1, 0x0}; // 89 00
    uint8_t cmd3[] = {0xc0, 0x1, 0x8, 0xff, 0xff};
    uint8_t cmd4[] = {0xc0, 0x1, 0x9, 0xff, 0xff};
    
    memAddr = 0;

    eeprom_9s12_power_on();
    eeprom_9s12sec_try_to_sync();

    _9s12WriteCmd(cmd1);
    nrf_delay_ms(50);
    _9s12ReadCmd(cmdR0, 3);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR1, 3);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR2, 3);
    nrf_delay_ms(500);
    _9s12WriteCmd(cmd2);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR3, 3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd4);
    

    return ret;
}

static uint16_t setupWrite_pflash_sec256()
{
    NRF_LOG_INFO("9s12sec: setupWrite_pflash_sec256 request");
    uint16_t ret = 0;
    uint8_t cmd1[] = {0xc0, 0x0, 0x3c, 0x0, 0x0};
    uint8_t cmdR0[] = {0xe0, 0x0, 0xb}; // 00 00
    uint8_t cmdR1[] = {0xe4, 0xff, 0x1}; // 00 C0
    uint8_t cmdR2[] = {0xe0, 0x01, 0x1};
    uint8_t cmd2[] = {0xc0, 0x1, 0x0, 0xb, 0xb};
    uint8_t cmdR3[] = {0xe0, 0x1, 0x0}; // 8B 00
    uint8_t cmd3[] = {0xc0, 0x1, 0x8, 0xff, 0xff};
    uint8_t cmd4[] = {0xc0, 0x1, 0x9, 0xff, 0xff};

    eeprom_9s12_power_on();
    eeprom_9s12sec_try_to_sync();

    _9s12WriteCmd(cmd1);
    nrf_delay_ms(50);
    _9s12ReadCmd(cmdR0, 3);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR1, 3);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR2, 3);
    nrf_delay_ms(500);
    _9s12WriteCmd(cmd2);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR3, 3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd4);

    return ret;
}

static uint16_t write_pflash_sec_wait_success(uint32_t address)
{
    uint16_t ret = 0;
    uint8_t addrH = (address >> 8) & 0xFF;
    uint8_t addrL = address & 0xFF;
    
    uint8_t cmd11[] = {0xc0, 0x1, 0x6, 0x30, 0x30};
    uint8_t cmd12[] = {0xc0, 0x1, 0x2, 0x00, 0x00};
    uint8_t cmd13[] = {0xc8, 0x1, 0xa, 0x0a, 0x7F}; //7E0000 to 7FFFFF
    if(is9s12sec256)
    {
        cmd13[4] = 0x7C;
    } else {
        cmd13[4] = 0x7E;
    }
    cmd13[4] += address / 0x8000;

    uint8_t cmd14[] = {0xc0, 0x1, 0x2, 0x01, 0x01};
    uint8_t cmd15[] = {0xc8, 0x1, 0xa, addrH, addrL};
    uint8_t cmd16[] = {0xc0, 0x1, 0x6, 0x80, 0x80};\
    uint8_t cmdR4[] = {0xe0, 0x1, 0x6}; // 80 00
    uint8_t cmdR5[] = {0xe0, 0x1, 0x7}; //00 00
    
    // NRF_LOG_INFO("9s12sec: write_pflash_sec_wait_success add 0x%04X", address);
    
    _9s12WriteCmd(cmd11);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd12);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd13);
    nrf_delay_us(350);  
    _9s12WriteCmd(cmd14);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd15);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd16);
    ret = _9s12ReadCmdUntil(cmdR4, 3, 0x8000);
    if(ret != EEPROM_E_OK)
    {
        NRF_LOG_INFO("write_dflash_sec128_wait_success Fail");
    } else {
        NRF_LOG_INFO("write_dflash_sec128_wait_success Done");
    }
    _9s12ReadCmd(cmdR5, 3);

    return ret;
}


static uint16_t write_pflash_sec(uint32_t address, const uint8_t *pflash_bin)
{
    uint16_t ret = 0;
    uint8_t addrH = (address >> 8) & 0xFF;
    uint8_t addrL = address & 0xFF;
    uint8_t cmdR4[] = {0xe0, 0x1, 0x6}; // 80 00
    uint8_t cmdR5[] = {0xe0, 0x01, 0x07}; //00 00
    uint8_t cmd11[] = {0xc0, 0x1, 0x6, 0x30, 0x30};
    uint8_t cmd12[] = {0xc0, 0x1, 0x2, 0x00, 0x00};
    uint8_t cmd13[] = {0xc8, 0x1, 0xa, 0x6, 0x7c + (address >> 16) & 0xFF}; //7E0000 to 7FFFFF
    if(is9s12sec256 == false)
    {
        cmd13[4] = 0x7E + (address >> 16) & 0xFF;
    }
    uint8_t cmd14[] = {0xc0, 0x1, 0x2, 0x1, 0x1};
    uint8_t cmd_address[] = {0xc8, 0x1, 0xa, addrH, addrL};
    uint8_t cmd15[] = {0xc0, 0x1, 0x2, 0x2, 0x2};
    uint8_t cmd_data1[] = {0xc8, 0x1, 0xa, pflash_bin[0], pflash_bin[1]};   // 2 byte data
    uint8_t cmd16[] = {0xc0, 0x1, 0x2, 0x3, 0x3};
    uint8_t cmd_data2[] = {0xc8, 0x1, 0xa, pflash_bin[2], pflash_bin[3]};   // 2 byte data
    uint8_t cmd17[] = {0xc0, 0x1, 0x2, 0x4, 0x4};
    uint8_t cmd_data3[] = {0xc8, 0x1, 0xa, pflash_bin[4], pflash_bin[5]};   // 2 byte data
    uint8_t cmd18[] = {0xc0, 0x1, 0x2, 0x5, 0x5};
    uint8_t cmd_data4[] = {0xc8, 0x1, 0xa, pflash_bin[6], pflash_bin[7]};   // 2 byte data
    uint8_t cmd19[] = {0xc0, 0x1, 0x6, 0x80, 0x80};

    NRF_LOG_INFO("9s12sec: write address %X: %02X %02X %02X %02X", address, pflash_bin[0], pflash_bin[1], pflash_bin[2], pflash_bin[3]);
    if(address % 0x200 == 0)
    {
        NRF_LOG_INFO("9s12sec: write_pflash_sec_wait_success");
        write_pflash_sec_wait_success(address);
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


static uint16_t setupWrite_dflash_sec128() {
    uint16_t ret = 0;
    uint8_t cmd1[] = {0xc0, 0x0, 0x3c, 0x0, 0x0};
    uint8_t cmdR0[] = {0xe0, 0x0, 0xb}; // 00 00
    uint8_t cmdR1[] = {0xe4, 0xff, 0x1}; // 00 C0
    uint8_t cmdR2[] = {0xe0, 0x01, 0x1};
    uint8_t cmd2[] = {0xc0, 0x1, 0x0, 0x9, 0x9};
    uint8_t cmdR3[] = {0xe0, 0x1, 0x0}; // 0xe0 0x01 0x01 0x00 0x00 0xfe
    uint8_t cmd4[] = {0xc0, 0x1, 0x8, 0xff, 0xff};
    uint8_t cmd5[] = {0xc0, 0x1, 0x9, 0xff, 0xff};
    
    NRF_LOG_INFO("setupWrite_dflash_sec128");
    eeprom_9s12_power_on();
    eeprom_9s12sec_try_to_sync();

    _9s12WriteCmd(cmd1);
    nrf_delay_ms(50);
    _9s12ReadCmd(cmdR0, 3);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR1, 3);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR2, 3);
    nrf_delay_ms(500);
    _9s12WriteCmd(cmd2);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR3, 3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd4);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd5);
    
    return 0;
}

static uint16_t setupWrite_dflash_sec256() {
    uint16_t ret = 0;
    uint8_t cmd1[] = {0xc0, 0x0, 0x3c, 0x0, 0x0};
    uint8_t cmdR0[] = {0xe0, 0x0, 0xb}; // 00 00
    uint8_t cmdR1[] = {0xe4, 0xff, 0x1}; // 00 C0
    uint8_t cmdR2[] = {0xe0, 0x01, 0x1};
    uint8_t cmd2[] = {0xc0, 0x1, 0x0, 0xb, 0xb};
    uint8_t cmdR3[] = {0xe0, 0x1, 0x0}; // 0xe0 0x01 0x01 0x00 0x00 0xfe
    uint8_t cmd3[] = {0xc0, 0x1, 0x0, 0xb, 0xb};
    uint8_t cmd4[] = {0xc0, 0x1, 0x8, 0xff, 0xff};
    uint8_t cmd5[] = {0xc0, 0x1, 0x9, 0xff, 0xff};

    NRF_LOG_INFO("setupWrite_dflash_sec256");
    eeprom_9s12_power_on();
    eeprom_9s12sec_try_to_sync();

    _9s12WriteCmd(cmd1);
    nrf_delay_ms(50);
    _9s12ReadCmd(cmdR0, 3);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR1, 3);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR2, 3);
    nrf_delay_ms(500);
    _9s12WriteCmd(cmd2);
    nrf_delay_us(350);
    _9s12ReadCmd(cmdR3, 3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd3);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd4);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd5);

    return 0;
}

static uint16_t write_dflash_sec_wait_success(uint32_t address)
{
    uint16_t ret = 0;
    uint8_t addrH = (address >> 8) & 0xFF;
    uint8_t addrL = address & 0xFF;
    
    uint8_t cmd11[] = {0xc0, 0x1, 0x6, 0x30, 0x30};
    uint8_t cmd12[] = {0xc0, 0x1, 0x2, 0x00, 0x00};
    uint8_t cmd13[] = {0xc8, 0x1, 0xa, 0x12, 0x10};
    uint8_t cmd14[] = {0xc0, 0x1, 0x2, 0x01, 0x01};
    uint8_t cmd15[] = {0xc8, 0x1, 0xa, addrH, addrL};
    uint8_t cmd16[] = {0xc0, 0x1, 0x6, 0x80, 0x80};\
    uint8_t cmdR4[] = {0xe0, 0x1, 0x6}; // 80 00
    uint8_t cmdR5[] = {0xe0, 0x1, 0x7}; //00 00
    
    NRF_LOG_INFO("9s12sec: write_dflash_sec_wait_success add 0x%04X", address);
    
    _9s12WriteCmd(cmd11);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd12);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd13);
    nrf_delay_us(350);  
    _9s12WriteCmd(cmd14);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd15);
    nrf_delay_us(350);
    _9s12WriteCmd(cmd16);
    ret = _9s12ReadCmdUntil(cmdR4, 3, 0x8000);
    if(ret != EEPROM_E_OK)
    {
        NRF_LOG_INFO("write_dflash_sec_wait_success Fail");
    }
    _9s12ReadCmd(cmdR5, 3);

    return ret;
}


static uint16_t write_dflash_sec(uint32_t address, const uint8_t *pflash_bin)
{
    uint16_t ret = 0;
    uint8_t addrH = (address >> 8) & 0xFF;
    uint8_t addrL = address & 0xFF;
    uint8_t cmdR4[] = {0xe0, 0x1, 0x6}; // 80 00
    uint8_t cmdR5[] = {0xe0, 0x01, 0x07}; //00 00
    uint8_t cmd11[] = {0xc0, 0x1, 0x6, 0x30, 0x30};
    uint8_t cmd12[] = {0xc0, 0x1, 0x2, 0x00, 0x00};
    uint8_t cmd13[] = {0xc8, 0x1, 0xa, 0x11, 0x10};
    uint8_t cmd14[] = {0xc0, 0x1, 0x2, 0x1, 0x1};
    uint8_t cmd_address[] = {0xc8, 0x1, 0xa, addrH, addrL};
    uint8_t cmd15[] = {0xc0, 0x1, 0x2, 0x2, 0x2};
    uint8_t cmd_data1[] = {0xc8, 0x1, 0xa, pflash_bin[0], pflash_bin[1]};   // 2 byte data
    uint8_t cmd16[] = {0xc0, 0x1, 0x2, 0x3, 0x3};
    uint8_t cmd_data2[] = {0xc8, 0x1, 0xa, pflash_bin[2], pflash_bin[3]};   // 2 byte data
    uint8_t cmd17[] = {0xc0, 0x1, 0x2, 0x4, 0x4};
    uint8_t cmd_data3[] = {0xc8, 0x1, 0xa, pflash_bin[4], pflash_bin[5]};   // 2 byte data
    uint8_t cmd18[] = {0xc0, 0x1, 0x2, 0x5, 0x5};
    uint8_t cmd_data4[] = {0xc8, 0x1, 0xa, pflash_bin[6], pflash_bin[7]};   // 2 byte data
    uint8_t cmd19[] = {0xc0, 0x1, 0x6, 0x80, 0x80};

    NRF_LOG_INFO("9s12sec: write address %04X: %02X %02X %02X %02X", address, pflash_bin[0], pflash_bin[1], pflash_bin[2], pflash_bin[3]);
    
    if(address % 0x100 == 0)
    {
        write_dflash_sec_wait_success(address);
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

    NRF_LOG_INFO("commReqPacket %02X", commReqPacket->buffer[7]);
    option = (Mcs9S12SecOption_t)(commReqPacket->buffer[7] & 0x03);
    is_ping = (commReqPacket->buffer[7] >> 7);
    is9s12sec256 = (commReqPacket->buffer[7]) & 0x4; // 0x01xx

    if(is9s12sec256) {
        NRF_LOG_INFO("9s12xs256");
    } else {
        NRF_LOG_INFO("9s12xs128");
    }

    //mc9s12WaitForReady();
    eeprom_9s12_power_off();

    if(is_ping)
    {
        NRF_LOG_INFO("9s12sec ping request");
        //try to sync
        eeprom_9s12_power_on();
        nrf_delay_ms(500);
        if(eeprom_9s12sec_try_to_sync() == false)
        {
           NRF_LOG_INFO("9s12sec_try_to_sync FAILD");
           eeprom_9s12_hard_reset();
           ret = 0xFFFF;
        }
        else
        {
           ret = EEPROM_E_OK;
           NRF_LOG_INFO("9s12sec_try_to_sync SUCCESS");
        }
        // ret = EEPROM_E_OK;
        nrf_delay_ms(200);
        eeprom_9s12_power_off();
    }
    else //setup message
    {
        NRF_LOG_INFO("9s12sec setup request");
        //initialize the timeslot
        NRF_SDH_SOC_OBSERVER(m_time_slot_soc_observer, 0, time_slot_soc_evt_handler, NULL);
        mc9s12Sec_timeslot_sd_init();
        //mc9s12WaitForReady();

        working_enable = false;
        memAddr = 0;
        //NRF_LOG_INFO("is9s12sec256 %02X", commReqPacket->buffer[7]);

        if (option == READ_EEPROM) {
            NRF_LOG_INFO("9s12sec not support READ_EEPROM");
        } else if (option == READ_PFLASH) {
            NRF_LOG_INFO("9s12sec not support READ_PFLASH");
        }

        else if (option == WRITE_EEPROM) {
            if(is9s12sec256) {
                NRF_LOG_INFO("SetupCommand WRITE_EEPROM_9S12SEC256");
                ret = setupWrite_dflash_sec256();
            } else {
                NRF_LOG_INFO("SetupCommand WRITE_EEPROM_9S12SEC128");
                ret = setupWrite_dflash_sec128();
            }
            
        } else if (option == WRITE_PFLASH) {
            if(is9s12sec256) {
                NRF_LOG_INFO("SetupCommand WRITE_PFLASH_9S12SEC256");
                ret = setupWrite_pflash_sec256();
            } else {
                NRF_LOG_INFO("SetupCommand WRITE_PFLASH_9S12SEC128");
                ret = setupWrite_pflash_sec128();
            }
        }
        NRF_LOG_INFO("9s12sec setup request done");
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
    // uint16_t readData;

    // working_enable = false;

    // for (uint8_t i = 0; i < EEPROM_9S12_READ_WRITE_SIZE; i += 2) {
    //     if (option == READ_EEPROM) {
    //         readData = read_eeprom(memAddr);

    //     } else if (option == READ_PFLASH) {
    //         readData = read_pflash(memAddr);
    //     }

    //     commResPacket->buffer[i] = readData & 0xFF;
    //     commResPacket->buffer[i + 1] = (readData >> 8) & 0xFF;
    //     memAddr += 2;
    // }
    commResPacket->cmd = CMD_IGNORE_RES;
    commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
    commResPacket->bufLen = 0;

    ////NRF_LOG_INFO("ReadCommand option %d, address %04X data %02X%02X%02X", option, memAddr - EEPROM_9S12_READ_WRITE_SIZE, commResPacket->buffer[0], commResPacket->buffer[1], commResPacket->buffer[2]);

    this->SetCommandRepeatState(false);
}

void EraseCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType) {

    NRF_LOG_INFO("9s12sec not support erase");
 //   NRF_LOG_INFO("EraseCommand option %d, address %04X", option, memAddr);
    //NRF_LOG_FLUSH();

    commResPacket->cmd = CMD_IGNORE_RES;
    commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
    commResPacket->bufLen = 0;

    this->SetCommandRepeatState(false);
}

void WriteCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType) {
    uint16_t ret = 0;

    for (uint8_t i = 0; i < EEPROM_9S12_READ_WRITE_SIZE; i += 8) {
        if (option == WRITE_EEPROM) {
            ret = write_dflash_sec(memAddr, &commReqPacket->buffer[i]);
        } else if (option == WRITE_PFLASH) {
            ret = write_pflash_sec(memAddr, &commReqPacket->buffer[i]);
        }
        memAddr += 8;
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

    //NRF_LOG_INFO("WriteCommand option %d, address %04X", option, memAddr - EEPROM_9S12_READ_WRITE_SIZE);
    NRF_LOG_FLUSH();

    this->SetCommandRepeatState(false);
}

/////////////////////////////////


static nrf_radio_signal_callback_return_param_t signal_callback_return_param;

/**@brief Request next timeslot event in earliest configuration
 */
uint32_t mc9s12Sec_request_next_event_earliest(void)
{
    m_slot_length                                  = 10000;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 1000000;
    return sd_radio_request(&m_timeslot_request);
}


/**@brief Configure next timeslot event in earliest configuration
 */
void mc9s12Sec_configure_next_event_earliest(void)
{
    m_slot_length                                  = 10000;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 1000000;
}


/**@brief Configure next timeslot event in normal configuration
 */
void mc9s12Sec_configure_next_event_normal(void)
{
    m_slot_length                                 = 10000;
    m_timeslot_request.request_type               = NRF_RADIO_REQ_TYPE_NORMAL;
    m_timeslot_request.params.normal.hfclk        = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.normal.priority     = NRF_RADIO_PRIORITY_HIGH;
    m_timeslot_request.params.normal.distance_us  = 100000;
    m_timeslot_request.params.normal.length_us    = m_slot_length;
}


/**@brief Timeslot signal handler
 */
void mc9s12Sec_nrf_evt_signal_handler(uint32_t evt_id)
{
    uint32_t err_code;

    switch (evt_id)
    {
        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            //No implementation needed
            break;
        case NRF_EVT_RADIO_SESSION_IDLE:
            working_enable = false;
            if (keep_running)
            {
                err_code = mc9s12Sec_request_next_event_earliest();
                APP_ERROR_CHECK(err_code);
            }
            //No implementation needed
            break;
        case NRF_EVT_RADIO_SESSION_CLOSED:
            //No implementation needed, session ended
            working_enable = false;
            break;
        case NRF_EVT_RADIO_BLOCKED:
            //Fall through
        case NRF_EVT_RADIO_CANCELED:
            if (keep_running)
            {
                err_code = mc9s12Sec_request_next_event_earliest();
                APP_ERROR_CHECK(err_code);
            }
            break;
        default:
            break;
    }
}


/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t * mc9s12Sec_radio_callback(uint8_t signal_type)
{
    switch(signal_type)
    {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            //Start of the timeslot - set up timer interrupt
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
            NRF_TIMER0->CC[0] = m_slot_length - 1000;
            NVIC_EnableIRQ(TIMER0_IRQn);
            //nrf_gpio_pin_set(13); //Toggle LED
            working_enable = true;
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            working_enable = false;
            //nrf_gpio_pin_clear(13);
            signal_callback_return_param.params.extend.length_us = m_slot_length;
            if(keep_running)
            {
              //Timer interrupt - do graceful shutdown - schedule next timeslot
              mc9s12Sec_configure_next_event_normal();
              signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
            }
            else
            {
              signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            }
            break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
            //No implementation needed
            //nrf_gpio_pin_set(13);
            //working_enable = true;
            break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
            //Try scheduling a new timeslot
            //nrf_gpio_pin_clear(13);
            working_enable = false;
            mc9s12Sec_configure_next_event_earliest();
            signal_callback_return_param.params.request.p_next = &m_timeslot_request;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            break;
        default:
            //No implementation needed
            break;
    }
    return (&signal_callback_return_param);
}


/**@brief Function for initializing the timeslot API.
 */
uint32_t mc9s12Sec_timeslot_sd_init(void)
{
    uint32_t err_code;

    keep_running = true;
    working_enable = false;

    nrf_delay_ms(100);

    err_code = sd_radio_session_open(mc9s12Sec_radio_callback);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("sd_radio_session_open FAIL")
        return err_code;
    }

    err_code = mc9s12Sec_request_next_event_earliest();
    if (err_code != NRF_SUCCESS)
    {
        (void)sd_radio_session_close();
        NRF_LOG_INFO("sd_radio_session_close FAIL")
        return err_code;
    }
    NRF_LOG_INFO("timeslot_sd_init done")

    //some delay
    nrf_delay_us(m_slot_length);
    return NRF_SUCCESS;
}
/////////////////////////////////

