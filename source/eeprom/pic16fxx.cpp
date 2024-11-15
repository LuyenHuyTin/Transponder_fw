#include "ble_common.h"
#include "eeprom/pic16fxx.h"

using namespace EsyPro;
using namespace EEPROM;
using namespace PIC16FXX;

static uint32_t curAddr = 0;
static Pic16fxx_t pic16fxxDevice;
static const char LOG_DGB_NAME[] = "pic16fxx";

static PIC16FXX::SetupCommand pic16fxxSetupCmd;
static PIC16FXX::ReadCommand pic16fxxReadCmd;
static PIC16FXX::WriteCommand pic16fxxWriteCmd;

static void Pic16fIspSend(uint16_t data, uint8_t numBits) {
    for (uint8_t i = 0; i < numBits; i++) {
        (data & 0x01) ? nrf_gpio_pin_write(EEPROM_PIC_DAT, 1) 
                      : nrf_gpio_pin_write(EEPROM_PIC_DAT, 0);
        nrf_delay_us(ISP_CLK_DELAY);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
        data = data >> 1;
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
        nrf_gpio_pin_write(EEPROM_PIC_DAT, 0);
    }
}

static uint16_t Pic16fIspSendRead(uint8_t cmd) {
    uint16_t data = 0x0000;
    Pic16fIspSend(cmd, 6);
    nrf_gpio_cfg_input(EEPROM_PIC_DAT, NRF_GPIO_PIN_PULLDOWN);
    nrf_delay_us(ISP_CLK_DELAY);
    
    for (uint8_t i = 0; i < 16; i++) {
        data >>= 1;
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
        nrf_delay_us(ISP_CLK_DELAY);
        if (nrf_gpio_pin_read(EEPROM_PIC_DAT)) {
            data |= 0x8000;
        }
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
        nrf_delay_us(ISP_CLK_DELAY);
    }

    nrf_gpio_cfg_output(EEPROM_PIC_DAT);
    nrf_delay_us(ISP_CLK_DELAY);
    return ((data >> 1) & 0x3FFF);
}

static void Pic16fSendWrite(uint8_t cmd, uint16_t data) {
    Pic16fIspSend(cmd, 6);
    nrf_delay_us(ISP_CLK_DELAY);
    Pic16fIspSend(data, 16);
    nrf_delay_us(ISP_CLK_DELAY);
}

static void Pic16fEnterProgmode(bool isHighVoltage = true) {
    uint8_t progmodeKey[] = {0x50, 0x48, 0x43, 0x4D};

    if (!isHighVoltage) {
        nrf_gpio_pin_set(POW_OUT_5V);
        nrf_delay_ms(200);

        nrf_gpio_pin_write(EEPROM_PIC_MCLR, 1);
        nrf_gpio_pin_write(EEPROM_PIC_DAT, 0);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
        nrf_delay_us(600);
        nrf_gpio_pin_write(EEPROM_PIC_MCLR, 0);

        nrf_delay_ms(1);

        for (uint8_t i = 0; i < sizeof(progmodeKey); i++) {
            Pic16fIspSend(progmodeKey[i], 8);
        }
        nrf_delay_us(ISP_CLK_DELAY);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);

    } else {
        nrf_gpio_pin_clear(POW_OUT_5V);
        nrf_gpio_pin_write(EEPROM_PIC_DAT, 0);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
        nrf_delay_ms(200);
        nrf_gpio_pin_set(POW_OUT_8V);
        nrf_delay_ms(200);

        nrf_gpio_pin_clear(POW_OUT_8V);
        nrf_delay_ms(200);
        nrf_gpio_pin_set(POW_OUT_5V);
        nrf_delay_ms(200);
    }
}

static uint16_t Pic16fReadDeviceID(void) {
    Pic16fEnterProgmode();
    nrf_delay_ms(1);
    Pic16fSendWrite(CMD_LOAD_CONFIG, 0x00);

    for (int i = 0; i < 6; i++) {
        Pic16fIspSend(CMD_INCR_ADDR, 6);
        nrf_delay_us(ISP_CLK_DELAY);
    }

    uint16_t id = Pic16fIspSendRead(CMD_READ_PROG_MEM);
    return (id >> 5);
}

static void Pic16fxxReadFlashData(uint8_t *data, uint8_t dataLen) {
    if (pic16fxxDevice.startAddr == curAddr) {
        Pic16fEnterProgmode();
        nrf_delay_ms(1);
    }

    for (int i = 0; i < dataLen / 2; i++) {
        uint16_t tmpData = Pic16fIspSendRead(CMD_READ_PROG_MEM);
        data[2 * i] = tmpData & 0xFF;
        data[2 * i + 1] = (tmpData >> 8) & 0xFF;
        Pic16fIspSend(CMD_INCR_ADDR, 6);
        nrf_delay_us(ISP_CLK_DELAY);
        curAddr++;
    }
}

static void Pic16fxxReadEepromData(uint8_t *data, uint8_t dataLen) {
    if (pic16fxxDevice.startAddr == curAddr) {
        Pic16fEnterProgmode();
        nrf_delay_ms(1);
    }

    for (int i = 0; i < dataLen; i++) {
        uint16_t tmpData = Pic16fIspSendRead(CMD_READ_DATA_MEM);
        data[i] = tmpData & 0xFF;
        Pic16fIspSend(CMD_INCR_ADDR, 6);
        nrf_delay_us(ISP_CLK_DELAY);
        curAddr++;
    }
}

static void Pic16fxxMultipleWriteFlashData(const uint8_t *data) {
    for (int i = 0; i < MAX_WORD_WRITE_SIZE; i++) {
        uint16_t tmpData = (data[2 * i] | (data[2 * i + 1] << 8)) & 0x3FFF;

        Pic16fSendWrite(CMD_LOAD_PROGRAM_MEMORY, (tmpData << 1));
        if (i == MAX_WORD_WRITE_SIZE - 1) {
            Pic16fIspSend(CMD_BEGIN_PROGRAM_INT, 6);
            nrf_delay_ms(5);
        }

        Pic16fIspSend(CMD_INCR_ADDR, 6);
        nrf_delay_us(ISP_CLK_DELAY);

        curAddr++;
    }
} 

static void Pic16fxxWriteFlashData(const uint8_t *data, uint8_t dataLen) {
    if (pic16fxxDevice.startAddr == curAddr) {
        Pic16fEnterProgmode();
        nrf_delay_ms(1);

        Pic16fSendWrite(CMD_LOAD_CONFIG, 0x00);
        Pic16fIspSend(CMD_BULK_ERASE_PROG, 6);
        
        nrf_delay_ms(10);
        Pic16fEnterProgmode();
    }

    for (int i = 0; i < dataLen / (MAX_WORD_WRITE_SIZE * 2); i++) {
        Pic16fxxMultipleWriteFlashData(&data[i * 16]);
    }
}

static void Pic16fxxWriteEepromData(const uint8_t *data, uint8_t dataLen) {
    if (pic16fxxDevice.startAddr == curAddr) {
        Pic16fEnterProgmode();
        nrf_delay_ms(10);
    }

    for (int i = 0; i < dataLen; i++) {
        uint16_t tmpData = data[i] & 0x00FF;

        Pic16fSendWrite(CMD_LOAD_EEPROM_MEMORY, (tmpData << 1));
        Pic16fIspSend(CMD_BEGIN_PROGRAM_INT, 6);
        nrf_delay_ms(5);

        Pic16fIspSend(CMD_INCR_ADDR, 6);
        nrf_delay_us(ISP_CLK_DELAY);

        curAddr++;
    }
}

Command *PIC16FXX::GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType) {
    Command *cmd = NULL;

    switch (commCmdType & 0x0F) {
    case CMD_BASIC_MEM_SETUP_REQ:
        cmd = &pic16fxxSetupCmd;
        NRF_LOG_INFO("[%s]: INFO: Setup Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_READ_DATA_REQ:
        cmd = &pic16fxxReadCmd;
        NRF_LOG_INFO("[%s]: INFO: Read Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_WRITE_DATA_REQ:
        cmd = &pic16fxxWriteCmd;
        NRF_LOG_INFO("[%s]: INFO: Write Request", LOG_DGB_NAME);
        break;    
    }

    return cmd;    
}

void SetupCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType) {
    pic16fxxDevice.memType = commReqPacket->buffer[7] 
                    ? PIC16FXX_EEPROM_MEM : PIC16FXX_PROG_MEM;
    memcpy(&pic16fxxDevice.startAddr, &commReqPacket->buffer[0], 3);
    memcpy(&pic16fxxDevice.endAddr, &commReqPacket->buffer[4], 3);

    nrf_gpio_cfg_output(EEPROM_PIC_MCLR);
    nrf_gpio_cfg_output(EEPROM_PIC_DAT);
    nrf_gpio_cfg_output(EEPROM_PIC_CLK);
    nrf_gpio_pin_write(EEPROM_PIC_MCLR, 0);
    nrf_gpio_pin_write(EEPROM_PIC_DAT, 0);
    nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);

    uint16_t id = Pic16fReadDeviceID();
    commResPacket->buffer[0] = (id == 0 || id == 0x01FF) ? 0 : 1;
    commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
    commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
    commResPacket->buffer[1] = id & 0xFF;
    commResPacket->buffer[2] = (id >> 8) & 0xFF;
    commResPacket->bufLen = 3;

    curAddr = pic16fxxDevice.startAddr;
    nrf_delay_ms(200);
    this->SetCommandRepeatState(false);
}

void ReadCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                          const EsyPro::CommPacket_t *commReqPacket,
                          EsyPro::CommunicationType_t commType) {
    if (curAddr >= pic16fxxDevice.endAddr) {
        commResPacket->bufLen = 0;
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_END_READ_DATA_RES;

    } else {
        if (pic16fxxDevice.memType == PIC16FXX_PROG_MEM) {
            Pic16fxxReadFlashData(commResPacket->buffer, PIC_PROG_BUF_SIZE);
            commResPacket->bufLen = PIC_PROG_BUF_SIZE;
            commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_READ_DATA_RES;

        } else if (pic16fxxDevice.memType == PIC16FXX_EEPROM_MEM) {
            Pic16fxxReadEepromData(commResPacket->buffer, PIC_EEPROM_BUF_SIZE);
            commResPacket->bufLen = PIC_EEPROM_BUF_SIZE;
            commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_READ_DATA_RES;
        }
    }
    
    this->SetCommandRepeatState(false);
}

void WriteCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                           const EsyPro::CommPacket_t *commReqPacket,
                           EsyPro::CommunicationType_t commType) {
    if (pic16fxxDevice.memType == PIC16FXX_PROG_MEM) {
        Pic16fxxWriteFlashData(commReqPacket->buffer, commReqPacket->bufLen);
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
        commResPacket->bufLen = 0;
    
    } else if (pic16fxxDevice.memType == PIC16FXX_EEPROM_MEM) {
        Pic16fxxWriteEepromData(commReqPacket->buffer, commReqPacket->bufLen);
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
        commResPacket->bufLen = 0;
    }

    this->SetCommandRepeatState(false);
}