#include "ble_common.h"
#include "transponder/pcf7991.h"
#include "eeprom/9s12.h"
#include "nrf_sdh_soc.h"

using namespace PCF7991;
using namespace EsyPro;
// using namespace EEPROM;

// Note: din_pin must have external interrupt feature!
static const char LOG_DGB_NAME[] = "pcf7991";
static PCF7991::SetupCommand pcf7991SetupCmd;
static PCF7991::ReadCommand pcf7991ReadCmd;
static PCF7991::WriteCommand pcf7991WriteCmd;

PCF7991::Pcf7991 *Pcf7991::instance = nullptr;
const nrf_drv_timer_t TIMER_TEST = NRF_DRV_TIMER_INSTANCE(4);
uint32_t isrtimes[400];
uint32_t *isrtimes_ptr = isrtimes;
volatile int32_t isrCnt = 0;
int32_t rfoffset = 2;
int32_t debug = 0;
int32_t decodemode = 0;
int32_t delay_1 = 20;
int32_t delay_0 = 14;
int32_t hysteresis = 1;
bool gpiote_initialized = false;
bool timer_initialized = false;
std::vector<int> byte_to_send;
static volatile bool working_enable = false;
static volatile bool keep_running = true;

/*ABIC Settings */
//////////////////////////
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_sdh.h"
#include "nrf_soc.h"

/**Constants for timeslot API
 */
static nrf_radio_request_t m_timeslot_request;
static uint32_t m_slot_length;

/**@brief Radio event handler
 */
void RADIO_timeslot_IRQHandler(void);

/**@brief Request next timeslot event in earliest configuration
 */
uint32_t request_next_event_earliest(void);

/**@brief Configure next timeslot event in earliest configuration
 */
void configure_next_event_earliest(void);

/**@brief Configure next timeslot event in normal configuration
 */
void configure_next_event_normal(void);

/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler(uint32_t evt_id);

/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t *radio_callback(uint8_t signal_type);

/**@brief Function for initializing the timeslot API.
 */
uint32_t timeslot_sd_init(void);

void radio_session_open_example(void);
static void mcPcfWaitForReady()
{
    do
    {
        // Make sure any pending events are cleared
        __SEV();
        __WFE();

    } while (working_enable == false);
}

//////////////////////////

Command *PCF7991::GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType)
{
    Command *cmd = NULL;

    NRF_LOG_INFO("-----reqPacket.cmd: %x", commCmdType & 0x0F);
    switch (commCmdType & 0x0F)
    {
    case CMD_BASIC_TRANS_SETUP_REQ:
        cmd = &pcf7991SetupCmd;
        NRF_LOG_INFO("[%s]: INFO: Setup Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_TRANS_READ_DATA_REQ:
        cmd = &pcf7991ReadCmd;
        NRF_LOG_INFO("[%s]: INFO: Read Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_TRANS_WRITE_DATA_REQ:
        cmd = &pcf7991WriteCmd;
        NRF_LOG_INFO("[%s]: INFO: Write Request", LOG_DGB_NAME);
        break;
    }

    return cmd;
}

uint8_t readPCF991Response()
{
    uint8_t _receive = 0;

    for (int i = 0; i < 8; i++)
    {
        nrf_gpio_pin_set(SCK_pin);
        nrf_delay_us(50);

        uint8_t tmp = nrf_gpio_pin_read(din_pin);
        if (tmp == 0)
        {
            _receive &= ~(1 << (7 - i));
        }
        else
        {
            _receive |= (1 << (7 - i));
        }
        nrf_gpio_pin_clear(SCK_pin);
        nrf_delay_us(50);
    }
    return _receive;
}

void writePCF7991Reg(uint8_t send, uint8_t bits)
{
    nrf_gpio_cfg_output(dout_pin);
    nrf_gpio_pin_clear(dout_pin);
    nrf_delay_us(50);
    nrf_gpio_pin_set(SCK_pin);
    nrf_delay_us(50);
    nrf_gpio_pin_set(dout_pin);
    nrf_delay_us(50);
    nrf_gpio_pin_clear(SCK_pin);
    for (uint8_t i = 0; i < bits; i++) // 10010011
    {
        nrf_delay_us(50);
        if ((send >> (7 - i)) & 0x01)
        {
            nrf_gpio_pin_set(dout_pin);
        }
        else
        {
            nrf_gpio_pin_clear(dout_pin);
        }
        nrf_delay_us(50);
        nrf_gpio_pin_set(SCK_pin);
        nrf_delay_us(50);
        nrf_gpio_pin_clear(SCK_pin);
    }
}

uint8_t readPCF7991Reg(uint8_t addr)
{
    uint8_t readval = 0;
    writePCF7991Reg(addr, 8);
    nrf_delay_us(500);
    readval = readPCF991Response();
    return readval;
}

void adapt(int offset)
{
    uint8_t phase = readPCF7991Reg(0x08); // Read Phase
    NRF_LOG_INFO("Measured phase: %x", phase);
    uint8_t samplingT = (0x3f & (2 * phase + offset));
    NRF_LOG_INFO("adapt samplingT: %x", samplingT);
    uint8_t readval = readPCF7991Reg((1 << 7) | samplingT); // Set Sampling Time + Cmd
    NRF_LOG_INFO("adapt readval: %x", readval);
}
// void pin_ISR(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
// {
//     uint32_t travelTime = nrf_drv_timer_capture(&TIMER_TEST, NRF_TIMER_CC_CHANNEL0);
//     NRF_LOG_INFO("TRAVEL TIME: %d", travelTime);
//     nrf_drv_timer_clear(&TIMER_TEST);
//     if (nrf_gpio_pin_read(din_pin))
//     {
//         travelTime &= ~1;
//     }
//     else
//     {
//         travelTime |= 1;
//     }
//     /*Handle over flow*/

//     isrtimes_ptr[isrCnt] = travelTime;

//     if (isrCnt < 400)
//         isrCnt++;
// }

int initTransponder()
{
    AbicConf_Page0.SetPageCmd = 1;
    AbicConf_Page1.SetPageCmd = 1;
    AbicConf_Page2.SetPageCmd = 1;

    AbicConf_Page0.gain = 2;
    AbicConf_Page0.filter_h = 1;
    AbicConf_Page0.filter_l = 1;
    AbicConf_Page1.hysteresis = 0;
    AbicConf_Page1.page_nr = 1;
    AbicConf_Page2.page_nr = 2;

    // nrf_gpio_cfg_output(CLKOUT);
    nrf_gpio_cfg_output(SCK_pin);
    nrf_gpio_cfg_output(dout_pin);
    nrf_gpio_cfg_output(22);
    nrf_gpio_cfg_input(din_pin, NRF_GPIO_PIN_PULLUP);

    nrf_gpio_pin_set(SCK_pin);
    nrf_gpio_pin_set(dout_pin);
    nrf_gpio_pin_set(din_pin);
    NRF_LOG_INFO("Init done");
    writePCF7991Reg(0x40, 8); // wake up
    AbicConf_Page1.txdis = 0;
    writePCF7991Reg(AbicConf_Page1.byteval, 8); // rf on
    uint8_t readval = 0;
    for (int i = 2; i < 9; i++)
    {
        readval = readPCF7991Reg(i);
    }
    writePCF7991Reg(0x70, 8);

    int checkAntena = readPCF7991Reg(0x7);
    NRF_LOG_INFO("checkAntena: %d", checkAntena);
    return checkAntena;
}

void writeToTag(uint8_t *data, int bits)
{
    int bytes = bits / 8;
    int rembits = bits % 8;
    int bytBits = 8;
    int cnt1 = 0;
    int cnt2 = 0;
    nrf_gpio_pin_clear(dout_pin);

    writePCF7991Reg(0x19, 8);
    nrf_gpio_pin_clear(dout_pin);

    nrf_delay_us(20); // 20
    nrf_gpio_pin_set(dout_pin);
    nrf_delay_us(20);
    nrf_gpio_pin_clear(dout_pin);

    for (int by = 0; by <= bytes; by++)
    {
        if (by == bytes)
            bytBits = rembits;
        else
            bytBits = 8;

        for (int i = 0; i < bytBits; i++)
        {
            if ((data[by] >> (7 - i)) & 0x01)
            {
                for (int i = 0; i < delay_1; i++)
                {
                    nrf_delay_us(10); // 180
                }
                nrf_gpio_pin_set(dout_pin);
                nrf_delay_us(10);
                nrf_gpio_pin_clear(dout_pin);
            }
            else
            {
                // Serial.print("0");
                for (int i = 0; i < delay_0; i++)
                {
                    nrf_delay_us(10); // 120
                }
                nrf_gpio_pin_set(dout_pin);
                nrf_delay_us(10);
                nrf_gpio_pin_clear(dout_pin);
            }
        }
    }
    // NVIC_EnableIRQ(TIMER0_IRQn);
    // end of transmission
    if (decodemode == 0)
        nrf_delay_us(1200);
    else
        nrf_delay_us(400); // Why biphase needs shorter delay???
}

unsigned int fir_filter(unsigned int pulse_fil_in, unsigned int current_pulse)
{
    unsigned int pulse_fil_out;
    if (((int)pulse_fil_in - (int)current_pulse) > 3)
    {
        pulse_fil_out = pulse_fil_in - 1;
    }
    else if (((int)current_pulse - (int)pulse_fil_in) > 3)
    {
        pulse_fil_out = pulse_fil_in + 1;
    }
    else
    {
        pulse_fil_out = pulse_fil_in;
    }
    return pulse_fil_out;
}
void timer_event_handler(nrf_timer_event_t event_type, void *p_context)
{
}
void initTimer()
{
    if (!timer_initialized)
    {
        uint32_t err_code;
        nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;

        timer_cfg.mode = NRF_TIMER_MODE_TIMER; // Use TIMER mode for timing intervals
        timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
        timer_cfg.frequency = NRF_TIMER_FREQ_250kHz;

        err_code = nrf_drv_timer_init(&TIMER_TEST, &timer_cfg, NULL); // Assuming no handler needed
        APP_ERROR_CHECK(err_code);

        nrf_drv_timer_enable(&TIMER_TEST);
        timer_initialized = true;
    }
}

int processManchester()
{
    int bitcount = 0;
    int bytecount = 0;
    int mybytes[10] = {0};
    int lead = 0;
    int errorCnt = 0;
    int state = 1;
    int start = 0;
    int pulsetime_fil = 0;

    for (start = 0; start < 10; start++)
    {
        if (isrtimes_ptr[start] < 150)
            break;
    }
    start += 3;
    /* Aadapt filtered pulse time during first pulses */
    int pulsetime_accum = 0;
    for (uint8_t i = start + 1; i < start + 4; i++)
    {
        pulsetime_accum += isrtimes_ptr[i];
    }
    pulsetime_fil = pulsetime_accum / 4;

    if (((isrtimes_ptr[start]) & 1) == 0)
    {
        start--;
        pulsetime_fil = fir_filter(pulsetime_fil, isrtimes_ptr[start]);
    }

    for (int i = start; i < isrCnt; i++)
    {
        // int pulsetime_thresh = pulsetime_fil + (pulsetime_fil/2);
        int pulsetime_thresh = 188;
        int travelTime = isrtimes_ptr[i];
        // NRF_LOG_INFO("%d", travelTime);
        if (((travelTime & 1) == 1)) // high
        {
            if (travelTime > pulsetime_thresh)
            {
                // travelTime =11;
                if (state)
                {
                    state = 1;
                    if (lead < 4)
                    {
                        lead++;
                        // Serial.print("X");
                    }
                    else
                    {
                        mybytes[bytecount] |= (1 << (7 - bitcount++));
                        // Serial.print("1");
                    }
                }
                else
                {
                    // NRF_LOG_INFO("X");
                    if (bytecount < 1)
                        errorCnt++;
                }
            }
            else
            {
                pulsetime_fil = fir_filter(pulsetime_fil, travelTime);
                if (state)
                {
                    state = 0;
                    if (lead < 4)
                    {
                        lead++;
                    }
                    else
                    {
                        mybytes[bytecount] |= (1 << (7 - bitcount++));
                        // Serial.print("1");
                    }
                }
                else
                {
                    state++;
                }
            }
        }
        else
        {
            if (travelTime > pulsetime_thresh)
            {
                if (state)
                {
                    state = 1;
                    bitcount++;
                    // Serial.print("0");
                }
                else
                {
                    // NRF_LOG_INFO("X");
                    if (bytecount < 1)
                    {
                        errorCnt++;
                    }
                }
            }
            else
            {
                pulsetime_fil = fir_filter(pulsetime_fil, travelTime);
                if (state)
                {
                    state = 0;
                    bitcount++;
                    // Serial.print("0");
                }
                else
                {
                    state++;
                }
            }
        }

        if (bitcount > 7)
        {
            bitcount = 0;
            bytecount++;
        }
        if (travelTime > 310)
        {
            if (bitcount > 0)
            {
                bytecount++;
            }
            break;
        }
    }
    // NRF_LOG_INFO("bytecount: %d", bytecount);
    NRF_LOG_INFO("bytecount: %d, bitcount: %d, errorCnt: %d, state: %d", bytecount, bitcount, errorCnt, state);
    if ((bytecount == 4) && (bitcount == 0) && (errorCnt == 0))
    {
        // NRF_LOG_INFO("bytecount: %d, bitcount: %d, errorCnt: %d, state: %d", bytecount, bitcount, errorCnt, state);
        //  if(bytecount < 4) {
        //      doAllthing();
        //  }
        char hash[20];
        std::string result = "";
        NRF_LOG_INFO("RESP:");
        if (errorCnt || bytecount == 0)
        {
            NRF_LOG_INFO("NORESP\n");
        }
        else
        {
            for (int s = 0; s < bytecount && s < 20; s++)
            {
                byte_to_send.push_back(mybytes[s]);
                NRF_LOG_INFO("0x%x", mybytes[s]);
            }
        }
        NRF_LOG_INFO("\n");
        return bytecount;
    }
    else
    {
        if((bytecount == 2) && (errorCnt == 0) && (bitcount == 2)) {
            for (int s = 0; s < bytecount && s < 20; s++)
            {
                byte_to_send.push_back(mybytes[s]);
                NRF_LOG_INFO("0x%x", mybytes[s]);
            }
        }
        else {
            for (int s = 0; s < 4 && s < 20; s++)
            {
                byte_to_send.push_back(0);
                //NRF_LOG_INFO("0x%x", mybytes[s]);
            }
        }
    }
    // NRF_LOG_INFO("bytecount: %d", res);
    return 0;
}

static void time_slot_soc_evt_handler(uint32_t evt_id, void *p_context)
{
    nrf_evt_signal_handler(evt_id);
}

//void gpio_init_interrupt(void)
//{
//    if (!gpiote_initialized)
//    {
//        ret_code_t err_code;

//        err_code = nrf_drv_gpiote_init();
//        APP_ERROR_CHECK(err_code);

//        nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
//        in_config.pull = NRF_GPIO_PIN_PULLUP;

//        err_code = nrf_drv_gpiote_in_init(din_pin, &in_config, pin_ISR);
//        APP_ERROR_CHECK(err_code);

//        nrf_drv_gpiote_in_event_enable(din_pin, true);
//        gpiote_initialized = true;
//    }
//}

void readTagResp(void)
{
    writePCF7991Reg(0xe0, 3);
    isrCnt = 0;
        //nrf_gpio_cfg_output(22);
        bool prev_din_pin_state = nrf_gpio_pin_read(din_pin);
        bool pin_22_state = prev_din_pin_state;
        nrf_gpio_pin_write(22, pin_22_state);
        int prev_time = 0;
        // for(volatile int i=0;i<150;i++){
            for(volatile int j=0;j<9100;j++){
                if(nrf_gpio_pin_read(din_pin) != prev_din_pin_state) {
                    NRF_TIMER0->TASKS_CAPTURE[3] = 1;
                    uint32_t current_time = NRF_TIMER0->CC[3];
                    int32_t travelTime = current_time - prev_time;
                    //NRF_LOG_INFO("travelTime: %d", travelTime);
                    if (nrf_gpio_pin_read(din_pin))
                    {
                        travelTime &= ~1;
                    }
                    else
                    {
                        travelTime |= 1;
                    }
                    if(isrCnt < 400) {
                        /*caculate the interval time between 2 capture and store into isrtimes_ptr*/
                        isrtimes_ptr[isrCnt] = travelTime;
                        //NRF_LOG_INFO("captured_time: %d", isrtimes_ptr[isrCnt]);
                        prev_time = current_time;
                        isrCnt++;
                    }
                    // pin_22_state = nrf_gpio_pin_read(din_pin);
                    // // Toggle the state
                    // pin_22_state = !pin_22_state;
                    
                    // // Write new state to pin 22
                    // nrf_gpio_pin_write(22, pin_22_state);
                    prev_din_pin_state = nrf_gpio_pin_read(din_pin);
                }
            }
        // }
            ;
    

    //NVIC_EnableIRQ(TIMER0_IRQn);

    if (isrCnt < 400 && isrCnt > 3)
    {
        isrtimes_ptr[isrCnt - 1] = isrtimes_ptr[isrCnt - 2] + 201;
        isrCnt++;
    }
    working_enable = false;
}

int communicateTag(uint8_t *tagcmd, unsigned int cmdLengt)
{
    int result;
    isrtimes_ptr = isrtimes;
    mcPcfWaitForReady();
    writeToTag(tagcmd, cmdLengt);
    readTagResp();

    NRF_LOG_INFO("ISRcnt: 0x%d", isrCnt);

    if (debug)
    {
        //char hash[5];
         for(int s=0; s<10; s++)
         {
           NRF_LOG_INFO("%d, ", isrtimes[s]);
         }
    }
    if (decodemode == 0)
        result = processManchester();
    else
        result = processManchester();

    working_enable = false;
    return result;
}

void tester()
{
    isrCnt = 0;
    //gpio_init_interrupt();

    uint8_t phase = readPCF7991Reg(0x08);

    // STOP_TIMER;
    NRF_LOG_INFO("Measured phase: 0x%x", phase);
    NRF_LOG_INFO("ISRcnt: %x", isrCnt);
}

std::string convertToString(char *a, int size)
{
    return std::string(a, size);
}

uint8_t serialToByte(std::string &value)
{
    if (value.length() < 2)
    {
        NRF_LOG_INFO("Input string must have at least 2 characters.");
        return 0; // Handle the error as appropriate
    }

    uint8_t retval = 0;

    for (int i = 0; i < 2; i++)
    {
        retval <<= 4; // Shift left by 4 bits (one hex digit)
        char raw = value[i];
        if (raw >= '0' && raw <= '9')
            retval |= raw - '0'; // Convert character to integer
        else if (raw >= 'A' && raw <= 'F')
            retval |= raw - 'A' + 10; // Convert hex character to integer
        else if (raw >= 'a' && raw <= 'f')
            retval |= raw - 'a' + 10; // Convert hex character to integer
        else
        {
            NRF_LOG_INFO("Invalid character in input string: ");
            return 0; // Handle the error as appropriate
        }
    }

    // Erase the first 2 characters from the value string
    value.erase(0, 2);
    return retval; // Return the resulting integer
}

std::string hexToString(const uint8_t *data, size_t length)
{
    std::string result;
    for (size_t i = 0; i < length; i++)
    {
        result += static_cast<char>(data[i]);
    }
    return result;
}

void SetupCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType)
{
    int m_Antenna = initTransponder();
    AbicConf_Page1.txdis = 0;
    writePCF7991Reg(AbicConf_Page0.byteval, 8);
    writePCF7991Reg(AbicConf_Page2.byteval, 8);
    writePCF7991Reg(AbicConf_Page1.byteval, 8); // rf on

    adapt(rfoffset);
    NRF_LOG_INFO("m_Antenna: %d", m_Antenna);
    if((((m_Antenna >> 5) & 0x01) == 1) && (m_Antenna != 255)) {
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_TRANS_SETUP_RES;
        commResPacket->buffer[0] = 1;
        commResPacket->bufLen = 1;
    }
    else
    {
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_TRANS_SETUP_RES;
        commResPacket->buffer[0] = 0;
        commResPacket->bufLen = 1;
    }
}

void ReadCommand::Execute(CommPacket_t *commResPacket,
                          const CommPacket_t *commReqPacket,
                          CommunicationType_t commType)
{
    byte_to_send.clear();
    NRF_SDH_SOC_OBSERVER(m_time_slot_soc_observer, 0, time_slot_soc_evt_handler, NULL);
    timeslot_sd_init();
    working_enable = false;
    std::vector<std::string> test = req_read_set;
    for (int k = 0; k < test.size(); k++)
    {
        uint8_t cmdlength = serialToByte(test[k]);
        uint8_t authcmd[300] = {0};
        if (cmdlength > 1 && cmdlength < 200)
        {
            for (int i = 0; i < (cmdlength + 7) / 8; i++)
            {
                authcmd[i] = serialToByte(test[k]);
            }
        }

        int result = communicateTag(authcmd, cmdlength);
        // nrf_delay_ms(20);
    }
    // NRF_LOG_INFO("byte_to_send.size(): %d", byte_to_send.size());
    // NRF_LOG_INFO("VALID_RESPONSE_SIZE_TRANS: %d", VALID_RESPONSE_SIZE_TRANS);
    if (byte_to_send.size() == VALID_READ_RESPONSE_SIZE_TRANS)
    {
        for (int i = 0; i < byte_to_send.size(); i++)
        {
            commResPacket->buffer[i] = byte_to_send[i];
        }
        commResPacket->cmd = CMD_BASIC_TRANS_READ_DATA_RES;
        commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
        commResPacket->bufLen = VALID_READ_RESPONSE_SIZE_TRANS;
    }
    this->SetCommandRepeatState(false);
}

void WriteCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType)
{
    byte_to_send.clear();
    uint8_t receiver[commReqPacket->bufLen]; // Create an array to hold 4 bytes
    // NRF_LOG_INFO("length: %d", commReqPacket->bufLen);
    memcpy(receiver, commReqPacket->buffer, commReqPacket->bufLen);
    std::string data_to_write = hexToString(receiver, sizeof(receiver));
    std::vector<std::string> test = req_write_set;
    test.push_back(data_to_write);
    for (int k = 0; k < test.size(); k++)
    {
        uint8_t cmdlength = serialToByte(test[k]);
        uint8_t authcmd[300] = {0};
        if (cmdlength > 1 && cmdlength < 200)
        {
            for (int i = 0; i < (cmdlength + 7) / 8; i++)
            {
                authcmd[i] = serialToByte(test[k]);
            }
        }

        int result = communicateTag(authcmd, cmdlength);
        nrf_delay_ms(20);
    }
    // NRF_LOG_INFO("byte_to_send.size(): %d", byte_to_send.size());
    if (byte_to_send.size() == VALID_WRITE_RESPONSE_SIZE_TRANS)
    {
        commResPacket->buffer[0] = 1;
        commResPacket->cmd = CMD_BASIC_TRANS_WRITE_DATA_RES;
        commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
        commResPacket->bufLen = 1;
    }
    this->SetCommandRepeatState(false);
}
void PCF7991::Setup(void)
{
    working_enable = false;
    // sd_radio_session_open(radio_callback);
    // adio_session_open_example();

    PCF7991::SetupCommand setupCommand;
    // PCF7991::ReadCommand readCommand;

    // Create instances of CommPacket_t (ensure you have proper constructors or initializations)
    CommPacket_t commResPacket;
    CommPacket_t commReqPacket;

    setupCommand.Execute(&commResPacket, &commReqPacket, PC_COMM_TYPE);
    // readCommand.Execute(&commResPacket, &commReqPacket, PC_COMM_TYPE);

    NRF_LOG_INFO("EOF-----------------------------------------------\n");
}

/////////////////////////////////

static nrf_radio_signal_callback_return_param_t signal_callback_return_param;

/**@brief Request next timeslot event in earliest configuration
 */
uint32_t request_next_event_earliest(void)
{
    m_slot_length                                  = 25000;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 1000000;
    return sd_radio_request(&m_timeslot_request);
}


/**@brief Configure next timeslot event in earliest configuration
 */
void configure_next_event_earliest(void)
{
    m_slot_length                                  = 25000;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 1000000;
}


/**@brief Configure next timeslot event in normal configuration
 */
void configure_next_event_normal(void)
{
    m_slot_length                                 = 25000;
    m_timeslot_request.request_type               = NRF_RADIO_REQ_TYPE_NORMAL;
    m_timeslot_request.params.normal.hfclk        = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.normal.priority     = NRF_RADIO_PRIORITY_HIGH;
    m_timeslot_request.params.normal.distance_us  = 100000;
    m_timeslot_request.params.normal.length_us    = m_slot_length;
}

/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler(uint32_t evt_id)
{
    NRF_LOG_INFO("evt_id %d", evt_id);
    uint32_t err_code;

    switch (evt_id)
    {
    case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
        // No implementation needed
        break;
    case NRF_EVT_RADIO_SESSION_IDLE:
        working_enable = false;
        if (keep_running)
        {
            err_code = request_next_event_earliest();
            APP_ERROR_CHECK(err_code);
        }
        // No implementation needed
        break;
    case NRF_EVT_RADIO_SESSION_CLOSED:
        // No implementation needed, session ended
        working_enable = false;
        break;
    case NRF_EVT_RADIO_BLOCKED:
        // Fall through
    case NRF_EVT_RADIO_CANCELED:
        if (keep_running)
        {
            err_code = request_next_event_earliest();
            APP_ERROR_CHECK(err_code);
        }
        break;
    default:
        break;
    }
}

/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t *radio_callback(uint8_t signal_type)
{
    // NRF_LOG_INFO("signal_type %d", signal_type);
    switch (signal_type)
    {
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
        // Start of the timeslot - set up timer interrupt
        signal_callback_return_param.params.request.p_next = NULL;
        signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
        NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
        NRF_TIMER0->CC[0] = m_slot_length - 1000;
        NVIC_EnableIRQ(TIMER0_IRQn);
        // nrf_gpio_pin_set(13); //Toggle LED
        working_enable = true;
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
        signal_callback_return_param.params.request.p_next = NULL;
        signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
        break;

    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
        working_enable = false;
        // nrf_gpio_pin_clear(13);
        signal_callback_return_param.params.extend.length_us = m_slot_length;
        if (keep_running)
        {
            // Timer interrupt - do graceful shutdown - schedule next timeslot
            configure_next_event_normal();
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
        }
        else
        {
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        }
        break;
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
        // No implementation needed
        // nrf_gpio_pin_set(13);
        // working_enable = true;
        NRF_TIMER0->TASKS_CLEAR = 1;
        break;
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
        // Try scheduling a new timeslot
        // nrf_gpio_pin_clear(13);
        working_enable = false;
        configure_next_event_earliest();
        signal_callback_return_param.params.request.p_next = &m_timeslot_request;
        signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        break;
    default:
        // No implementation needed
        break;
    }
    return (&signal_callback_return_param);
}

/**@brief Function for initializing the timeslot API.
 */
uint32_t timeslot_sd_init(void)
{
    uint32_t err_code;

    keep_running = true;
    working_enable = false;

    nrf_delay_ms(100);

    err_code = sd_radio_session_open(radio_callback);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("sd_radio_session_open FAIL: %d", err_code); // Log the error code
        return err_code;
    }

    err_code = request_next_event_earliest();
    if (err_code != NRF_SUCCESS)
    {
        (void)sd_radio_session_close();
        NRF_LOG_INFO("sd_radio_session_close FAIL")
        return err_code;
    }
    NRF_LOG_INFO("timeslot_sd_init done")

    // some delay
    nrf_delay_us(m_slot_length);
    return NRF_SUCCESS;
}

/////////////////////////////////