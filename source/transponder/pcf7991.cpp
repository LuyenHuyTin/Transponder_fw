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
uint32_t isrtimes[400];
uint32_t *isrtimes_ptr = isrtimes;
volatile int32_t isrCnt = 0;
uint8_t rfoffset = 2;
uint8_t decodemode = 0;
uint32_t delay_1 = 20;
uint32_t delay_0 = 14;
uint32_t waiting_time = 9100;
uint32_t byte_to_send[VALID_READ_RESPONSE_SIZE_TRANS] = {0};
uint8_t byte_to_send_count = 0;
uint8_t sampcont=0x17;
static volatile bool working_enable = false;
static volatile bool keep_running = true;
static bool status = false;

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
uint32_t request_next_event_earliest_pcf7991(void);

/**@brief Configure next timeslot event in earliest configuration
 */
void configure_next_event_earliest_pcf7991(void);

/**@brief Configure next timeslot event in normal configuration
 */
void configure_next_event_normal_pcf7991(void);

/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler_pcf7991(uint32_t evt_id);

/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t *radio_callback_pcf7991(uint8_t signal_type);

/**@brief Function for initializing the timeslot API.
 */
uint32_t timeslot_sd_init_pcf7991(void);

void radio_session_open_example_pcf7991(void);
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
    switch (commCmdType & 0x0F)
    {
    case CMD_BASIC_TRANS_SETUP_REQ:
        cmd = &pcf7991SetupCmd;
        NRF_LOG_INFO("[%s]: INFO: Ping (Setup) Request", LOG_DGB_NAME);
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

    for (uint32_t i = 0; i < 8; i++)
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

void adapt(uint8_t offset)
{
    uint8_t phase = readPCF7991Reg(0x08); // Read Phase
    //NRF_LOG_INFO("Measured phase: %x", phase);
    uint8_t samplingT = (0x3f & (2 * phase + offset));
    //NRF_LOG_INFO("adapt samplingT: %x", samplingT);
    uint8_t readval = readPCF7991Reg((1 << 7) | samplingT); // Set Sampling Time + Cmd
    //NRF_LOG_INFO("adapt readval: %x", readval);
}

uint8_t initTransponder()
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
    for(int i = 2;i<9;i++)
    {
        readval = readPCF7991Reg(i);
        //NRF_LOG_INFO("i:%d, val:%d", i, readval);
    }
    //NRF_LOG_INFO("set sampling time");
    int samplingT = (1<<7)|(0x3f&sampcont);
    readval = readPCF7991Reg(samplingT);
    //NRF_LOG_INFO("samplingT:%d, val:%d", samplingT, readval);

    //NRF_LOG_INFO("done registers\n");

    /*Config for oscillator 4Mhz*/
    writePCF7991Reg(0x70, 8);

    /*Check antenna*/
    uint8_t m_checkAntenna = readPCF7991Reg(0x7);
    NRF_LOG_INFO("checkAntena: %d", m_checkAntenna);
    return m_checkAntenna;
}

void writeToTag(uint8_t *data, uint32_t bits)
{
    uint8_t bytes = bits / 8;
    uint8_t rembits = bits % 8;
    uint8_t bytBits = 8;
    uint8_t cnt1 = 0;
    uint8_t cnt2 = 0;
    nrf_gpio_pin_clear(dout_pin);

    writePCF7991Reg(0x19, 8);
    nrf_gpio_pin_clear(dout_pin);

    nrf_delay_us(20); // 20
    nrf_gpio_pin_set(dout_pin);
    nrf_delay_us(20);
    nrf_gpio_pin_clear(dout_pin);

    for (uint32_t by = 0; by <= bytes; by++)
    {
        if (by == bytes)
            bytBits = rembits;
        else
            bytBits = 8;

        for (uint32_t i = 0; i < bytBits; i++)
        {
            if ((data[by] >> (7 - i)) & 0x01)
            {
                for (uint32_t i = 0; i < delay_1; i++)
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
                for (uint32_t i = 0; i < delay_0; i++)
                {
                    nrf_delay_us(10); // 120
                }
                nrf_gpio_pin_set(dout_pin);
                nrf_delay_us(10);
                nrf_gpio_pin_clear(dout_pin);
            }
        }
    }
    // end of transmission
    if (decodemode == 0)
        nrf_delay_us(1200);
    else
        nrf_delay_us(400); // Why biphase needs shorter delay???
}

uint32_t fir_filter(uint32_t pulse_fil_in, uint32_t current_pulse)
{
    uint32_t pulse_fil_out;
    if (((uint32_t)pulse_fil_in - (uint32_t)current_pulse) > 3)
    {
        pulse_fil_out = pulse_fil_in - 1;
    }
    else if (((uint32_t)current_pulse - (uint32_t)pulse_fil_in) > 3)
    {
        pulse_fil_out = pulse_fil_in + 1;
    }
    else
    {
        pulse_fil_out = pulse_fil_in;
    }
    return pulse_fil_out;
}

void processManchester(void)
{
    uint8_t bitcount = 0;
    uint8_t bytecount = 0;
    uint8_t lead = 0;
    uint8_t errorCnt = 0;
    uint8_t state = 1;
    uint8_t start = 0;
    uint32_t pulsetime_fil = 0;
    uint32_t mybytes[10] = {0};

    for (start = 0; start < 10; start++)
    {
        if (isrtimes_ptr[start] < 150)
            break;
    }
    start += 3;
    /* Aadapt filtered pulse time during first pulses */
    uint32_t pulsetime_accum = 0;
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

    for (uint32_t i = start; i < isrCnt; i++)
    {
        // uint32_t pulsetime_thresh = pulsetime_fil + (pulsetime_fil/2);
        uint32_t pulsetime_thresh = 188;
        uint32_t travelTime = isrtimes_ptr[i];
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
    // NRF_LOG_INFO("bytecount: %d, bitcount: %d, errorCnt: %d, state: %d", bytecount, bitcount, errorCnt, state);
    if ((bytecount == 4) && (bitcount == 0) && (errorCnt == 0))
    {
        char hash[20];
        std::string result = "";
        //NRF_LOG_INFO("RESP:");
        if (errorCnt || bytecount == 0)
        {
            NRF_LOG_INFO("NORESP\n");
        }
        else
        {
            for (uint32_t s = 0; s < bytecount && s < 20; s++)
            {
                byte_to_send[byte_to_send_count] = mybytes[s];
                byte_to_send_count++;
                NRF_LOG_INFO("0x%x", mybytes[s]);
            }
        }
        NRF_LOG_INFO("\n");
    }
    else
    {
        if ((bytecount == 2) && (errorCnt == 0) && (bitcount == 2))
        {
            for (uint32_t s = 0; s < bytecount && s < 20; s++)
            {
                byte_to_send[byte_to_send_count] = mybytes[s];
                byte_to_send_count++;
                //NRF_LOG_INFO("0x%x", mybytes[s]);
            }
        }
        else
        {
            for (uint32_t s = 0; s < 4 && s < 20; s++)
            {
                byte_to_send[byte_to_send_count] = 0;
                byte_to_send_count++;
                //NRF_LOG_INFO("0x%x", mybytes[s]);
            }
        }
    }
}

static void time_slot_soc_evt_handler(uint32_t evt_id, void *p_context)
{
    nrf_evt_signal_handler_pcf7991(evt_id);
}

void readTagResp(void)
{
    writePCF7991Reg(0xe0, 3);
    isrCnt = 0;
    bool prev_din_pin_state = nrf_gpio_pin_read(din_pin);
    nrf_gpio_pin_write(22, prev_din_pin_state);
    uint32_t prev_time = 0;
    for (volatile uint32_t j = 0; j < waiting_time; j++)
    {
        if (nrf_gpio_pin_read(din_pin) != prev_din_pin_state)
        {
            NRF_TIMER0->TASKS_CAPTURE[3] = 1;
            uint32_t current_time = NRF_TIMER0->CC[3];
            uint32_t travelTime = current_time - prev_time;
            if (nrf_gpio_pin_read(din_pin))
            {
                travelTime &= ~1;
            }
            else
            {
                travelTime |= 1;
            }
            if (isrCnt < 400)
            {
                /*caculate the interval time between 2 capture and store into isrtimes_ptr*/
                isrtimes_ptr[isrCnt] = travelTime;
                // NRF_LOG_INFO("captured_time: %d", isrtimes_ptr[isrCnt]);
                prev_time = current_time;
                isrCnt++;
            }
            nrf_gpio_pin_write(22, nrf_gpio_pin_read(din_pin));
            prev_din_pin_state = nrf_gpio_pin_read(din_pin);
        }
    }

    if (isrCnt < 400 && isrCnt > 3)
    {
        isrtimes_ptr[isrCnt - 1] = isrtimes_ptr[isrCnt - 2] + 201;
        isrCnt++;
    }
}

void communicateTag(uint8_t *tagcmd, uint32_t cmdLengt)
{
    isrtimes_ptr = isrtimes;
    mcPcfWaitForReady();
    writeToTag(tagcmd, cmdLengt);
    readTagResp();

    NRF_LOG_INFO("ISRcnt: 0x%d", isrCnt);

    if (decodemode == 0)
        processManchester();

    /*else for cdp encoding*/
    working_enable = false;
}

void statusTimeSlot()
{
    if (status == false)
    {
        NRF_SDH_SOC_OBSERVER(m_time_slot_soc_observer, 0, time_slot_soc_evt_handler, NULL);
        timeslot_sd_init_pcf7991();
        status = true;
    }
}

void hexToString(const uint8_t *data, size_t length, char *output)
{
    for (size_t i = 0; i < length; i++)
    {
        output[i] = static_cast<char>(data[i]);
    }
    output[length] = '\0'; // Ensure null-termination
}

uint8_t serialToByte(const char *value, uint32_t &index)
{
    uint8_t retval = 0;

    // Ensure we have at least two characters to process
    if (value[index] == '\0' || value[index + 1] == '\0')
    {
        NRF_LOG_INFO("Input string must have at least 2 characters.");
        return 0;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        retval <<= 4; // Shift left by 4 bits (one hex digit)
        char raw = value[index++];
        if (raw >= '0' && raw <= '9')
            retval |= raw - '0'; // Convert character to integer
        else if (raw >= 'A' && raw <= 'F')
            retval |= raw - 'A' + 10; // Convert hex character to integer
        else if (raw >= 'a' && raw <= 'f')
            retval |= raw - 'a' + 10; // Convert hex character to integer
        else
        {
            NRF_LOG_INFO("Invalid character in input string.");
            return 0;
        }
    }
    return retval;
}

void SetupCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType)
{
    uint8_t m_Antenna = initTransponder();
    AbicConf_Page1.txdis = 0;
    writePCF7991Reg(AbicConf_Page0.byteval, 8);
    writePCF7991Reg(AbicConf_Page2.byteval, 8);
    writePCF7991Reg(AbicConf_Page1.byteval, 8); // rf on

    adapt(rfoffset);
    if ((((m_Antenna >> 5) & 0x01) == 1) && (m_Antenna != 255))
    {
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
    byte_to_send[VALID_READ_RESPONSE_SIZE_TRANS] = {0};
    byte_to_send_count = 0;
    //uint32_t delay_time[] = {8700, }
    statusTimeSlot();
    working_enable = false;
    for (uint32_t k = 0; k < MAX_COMMANDS; k++)
    {
        const char *currentCmd = mesReqRead[k];
        uint32_t index = 0; // Tracks the current position in the command string

        uint8_t cmdlength = serialToByte(currentCmd, index);
        uint8_t authcmd[300] = {0};

        if (cmdlength > 1 && cmdlength < 200)
        {
            for (uint32_t i = 0; i < (cmdlength + 7) / 8; i++)
            {
                authcmd[i] = serialToByte(currentCmd, index);
            }
        }
        communicateTag(authcmd, cmdlength);
    }
    AbicConf_Page1.txdis = 1;
    writePCF7991Reg(AbicConf_Page1.byteval, 8);//rf off

    for (uint32_t i = 0; i < VALID_READ_RESPONSE_SIZE_TRANS; i++)
    {
        commResPacket->buffer[i] = byte_to_send[i];
    }
    commResPacket->cmd = CMD_BASIC_TRANS_READ_DATA_RES;
    commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
    commResPacket->bufLen = VALID_READ_RESPONSE_SIZE_TRANS;
    this->SetCommandRepeatState(false);
}

void WriteCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType)
{
    byte_to_send[VALID_READ_RESPONSE_SIZE_TRANS] = {0};
    byte_to_send_count = 0;
    statusTimeSlot();
    working_enable = false;

    uint8_t receiver[commReqPacket->bufLen - 1]; // Create an array to hold 4 bytes
    uint8_t positionPage;
    memcpy(receiver, commReqPacket->buffer, commReqPacket->bufLen - 1);
    memcpy(&positionPage, &commReqPacket->buffer[commReqPacket->bufLen - 1], 1);
    positionPage = positionPage - '0';

    // Convert the receiver array to a hex string
    char data_to_write[MAX_COMMAND_LENGTH] = {0};
    hexToString(receiver, sizeof(receiver), data_to_write);

    // Create a static array to hold the commands for this operation
    const char *commands[4]; // Max commands: 2 from mesReqWrite, 1 from mesEachPage, 1 data_to_write
    uint8_t commandCount = 0;

    // Add commands from mesReqWrite
    for (uint8_t i = 0; i < MAX_WRITE_COMMANDS; i++)
    {
        commands[commandCount++] = mesReqWrite[i];
    }

    // Add the selected page command
    if (positionPage < MAX_PAGE_COMMANDS)
    {
        commands[commandCount++] = mesEachPage[positionPage];
    }

    // Add the data to write
    commands[commandCount++] = data_to_write;

    // Process each command in the static array
    for (uint32_t k = 0; k < commandCount; k++)
    {
        const char *currentCmd = commands[k];
        uint32_t index = 0; // Index for parsing the string

        uint8_t cmdlength = serialToByte(currentCmd, index);
        uint8_t authcmd[300] = {0};

        if (cmdlength > 1 && cmdlength < 200)
        {
            for (uint32_t i = 0; i < (cmdlength + 7) / 8; i++)
            {
                authcmd[i] = serialToByte(currentCmd, index);
            }
        }

        communicateTag(authcmd, cmdlength);
    }
    AbicConf_Page1.txdis = 1;
    writePCF7991Reg(AbicConf_Page1.byteval, 8);//rf off


    for (uint32_t i = 0; i < 2; i++)
    {
        commResPacket->buffer[i] = byte_to_send[i + 8];
    }
    commResPacket->cmd = CMD_BASIC_TRANS_WRITE_DATA_RES;
    commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
    commResPacket->bufLen = 2;

    this->SetCommandRepeatState(false);
}

/////////////////////////////////

static nrf_radio_signal_callback_return_param_t signal_callback_return_param;

/**@brief Request next timeslot event in earliest configuration
 */
uint32_t request_next_event_earliest_pcf7991(void)
{
    m_slot_length = 30000;
    m_timeslot_request.request_type = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us = 1000000;
    return sd_radio_request(&m_timeslot_request);
}

/**@brief Configure next timeslot event in earliest configuration
 */
void configure_next_event_earliest_pcf7991(void)
{
    m_slot_length = 30000;
    m_timeslot_request.request_type = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us = 1000000;
}

/**@brief Configure next timeslot event in normal configuration
 */
void configure_next_event_normal_pcf7991(void)
{
    m_slot_length = 30000;
    m_timeslot_request.request_type = NRF_RADIO_REQ_TYPE_NORMAL;
    m_timeslot_request.params.normal.hfclk = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.normal.priority = NRF_RADIO_PRIORITY_HIGH;
    m_timeslot_request.params.normal.distance_us = 100000;
    m_timeslot_request.params.normal.length_us = m_slot_length;
}

/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler_pcf7991(uint32_t evt_id)
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
            err_code = request_next_event_earliest_pcf7991();
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
            err_code = request_next_event_earliest_pcf7991();
            APP_ERROR_CHECK(err_code);
        }
        break;
    default:
        break;
    }
}

/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t *radio_callback_pcf7991(uint8_t signal_type)
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
            configure_next_event_normal_pcf7991();
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
        configure_next_event_earliest_pcf7991();
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
uint32_t timeslot_sd_init_pcf7991(void)
{
    uint32_t err_code;

    keep_running = true;
    working_enable = false;

    nrf_delay_ms(100);

    err_code = sd_radio_session_open(radio_callback_pcf7991);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("sd_radio_session_open FAIL: %d", err_code); // Log the error code
        return err_code;
    }

    err_code = request_next_event_earliest_pcf7991();
    if (err_code != NRF_SUCCESS)
    {
        (void)sd_radio_session_close();
        NRF_LOG_INFO("sd_radio_session_close FAIL");
        return err_code;
    }
    NRF_LOG_INFO("timeslot_sd_init_pcf7991 done");

    // some delay
    nrf_delay_us(m_slot_length);
    return NRF_SUCCESS;
}

/////////////////////////////////