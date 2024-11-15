#include "ble_common.h"
#include "eeprom/at93cxx.h"

using namespace EsyPro;
using namespace EEPROM;
using namespace AT93CXX;

static const char LOG_DGB_NAME[] = "at93cxx";
static uint16_t curAddr = 0;
static At93cxx_t at93cxxDevice;

static AT93CXX::SetupCommand at93cxxSetupCmd;
static AT93CXX::ReadCommand at93cxxReadCmd;
static AT93CXX::WriteCommand at93cxxWriteCmd;

Command *AT93CXX::GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType) {
    Command *cmd = NULL;

    switch (commCmdType & 0x0F) {
    case CMD_BASIC_MEM_SETUP_REQ:
        cmd = &at93cxxSetupCmd;
        NRF_LOG_INFO("[%s]: INFO: Setup Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_READ_DATA_REQ:
        cmd = &at93cxxReadCmd;
        NRF_LOG_INFO("[%s]: INFO: Read Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_WRITE_DATA_REQ:
        cmd = &at93cxxWriteCmd;
        NRF_LOG_INFO("[%s]: INFO: Write Request", LOG_DGB_NAME);
        break;
    }

    return cmd;    
}

static bool SetupAT93cxx(At93cxx_t *device) {
    nrf_gpio_cfg_output(EEPROM_93CXX_CS_PIN);
    nrf_gpio_cfg_output(EEPROM_93CXX_SK_PIN);
    nrf_gpio_cfg_output(EEPROM_93CXX_DI_PIN);
    nrf_gpio_cfg_input(EEPROM_93CXX_DO_PIN, NRF_GPIO_PIN_NOPULL);
    curAddr = 0;

    if (device->org == AT93CXX_ORG_8) {
        device->size = 128;
        device->addrBits = 7;
        device->mask = 0x7F; 
    } else if (device->org == AT93CXX_ORG_16) {
        device->size = 64;
        device->addrBits = 6;
        device->mask = 0x3F;
    }

    if (device->model == EEPROM_93CXX_56) {
        device->size *= 2;
        device->addrBits += 2;
        device->mask = (device->mask << 2) + 0x03;
    } else if (device->model == EEPROM_93CXX_66) {
        device->size *= 4;
        device->addrBits += 2;
        device->mask = (device->mask << 2) + 0x03;
    } else if (device->model == EEPROM_93CXX_76) {
        device->size *= 8;
        device->addrBits += 4;
        device->mask = (device->mask << 4) + 0x0F;
    } else if (device->model == EEPROM_93CXX_86) {
        device->size *= 16;
        device->addrBits += 4;
        device->mask = (device->mask << 4) + 0x0F;
    }

    return true;
}

static void SendBits(uint16_t value, int len) {
    for (int i = len - 1; i >= 0; i--) {
        bool toSend = (value & (1 << i));

        if (toSend) {
            nrf_gpio_pin_write(EEPROM_93CXX_DI_PIN, 1);
        } else {
            nrf_gpio_pin_write(EEPROM_93CXX_DI_PIN, 0);
        }
        nrf_delay_us(1);
        nrf_gpio_pin_write(EEPROM_93CXX_SK_PIN, 1);
        nrf_delay_us(1);
        nrf_gpio_pin_write(EEPROM_93CXX_SK_PIN, 0);
        nrf_delay_us(1);
    }
}

static uint16_t ReadAT93cxxCmd(At93cxx_t *device, uint16_t addr) {
	uint16_t val = 0;
    int amtBits;
    uint16_t cmdVal = (OPCODE_READ << device->addrBits) | (addr & device->mask);

    nrf_gpio_pin_write(EEPROM_93CXX_CS_PIN, 1);
    SendBits(1, 1);
    if (device->org == AT93CXX_ORG_16) {
        SendBits(cmdVal, device->addrBits + 2);
        amtBits = 16;
    } else {
        SendBits(cmdVal, device->addrBits + 2);
        amtBits = 8;
    }

    for (int i = amtBits; i > 0; i--) {
        nrf_gpio_pin_write(EEPROM_93CXX_SK_PIN, 1);
        nrf_delay_us(1);
        int gpioVal = nrf_gpio_pin_read(EEPROM_93CXX_DO_PIN);
        nrf_gpio_pin_write(EEPROM_93CXX_SK_PIN, 0);
        nrf_delay_us(1);
        val |= (gpioVal << (i - 1));
    }
	nrf_gpio_pin_write(EEPROM_93CXX_CS_PIN, 0);

	return val;    
}

static bool ReadAT93cxx(At93cxx_t *device, uint8_t *data, uint8_t dataLen) {
    for (int i = 0; i < dataLen; i++) {
        uint16_t bytes;

        bytes = ReadAT93cxxCmd(device, curAddr);
        data[i] = bytes & 0xFF;
        if (device->org == AT93CXX_ORG_16) {
            data[++i] = (bytes >> 8) & 0xFF;
        }
        curAddr++;
    }

    return true;
}

static void AT93cxxEwEnable(At93cxx_t *device) {
    uint16_t cmdVal = (OPCODE_CTRL << device->addrBits) 
                    | (OPCODE_EW_ENABLE << (device->addrBits - 2));

    nrf_gpio_pin_write(EEPROM_93CXX_CS_PIN, 1);
    SendBits(1, 1);
    SendBits(cmdVal, device->addrBits + 2);
    nrf_gpio_pin_write(EEPROM_93CXX_CS_PIN, 0);
}

static void AT93cxxEwDisable(At93cxx_t *device) {
    uint16_t cmdVal = (OPCODE_CTRL << device->addrBits) 
                    | (OPCODE_EW_DISABLE << (device->addrBits - 2));

    nrf_gpio_pin_write(EEPROM_93CXX_CS_PIN, 1);
    SendBits(1, 1);
    SendBits(cmdVal, device->addrBits + 2);
    nrf_gpio_pin_write(EEPROM_93CXX_CS_PIN, 0);
}

static void AT93cxxWaitReady(void) {
    nrf_gpio_pin_write(EEPROM_93CXX_CS_PIN, 1);
    while (nrf_gpio_pin_read(EEPROM_93CXX_DO_PIN) != 1) {
        nrf_delay_us(1);
    }
    nrf_gpio_pin_write(EEPROM_93CXX_CS_PIN, 0);
}

static void WriteAT93cxxCmd(At93cxx_t *device, uint16_t addr, uint16_t bytes) {
    uint16_t cmdVal = (OPCODE_WRITE << device->addrBits) | (addr & device->mask);

    nrf_gpio_pin_write(EEPROM_93CXX_CS_PIN, 1);
    SendBits(1, 1);
    if (device->org == AT93CXX_ORG_16) {
        SendBits(cmdVal, device->addrBits + 2);
        SendBits(bytes & 0xFFFF, 16);
    } else {
        SendBits(cmdVal, device->addrBits + 2);
        SendBits(bytes & 0xFF, 8);
    }
    nrf_gpio_pin_write(EEPROM_93CXX_CS_PIN, 0);
    AT93cxxWaitReady();
}

static bool WriteAT93cxx(At93cxx_t *device, const uint8_t *data, uint8_t dataLen) {
    AT93cxxEwEnable(device);

    for (int i = 0; i < dataLen; i++) {
        uint16_t writeBytes;

        writeBytes = data[i] & 0xFF;
        if (device->org == AT93CXX_ORG_16) {
            writeBytes |= (data[++i] << 8);
        }
        WriteAT93cxxCmd(device, curAddr, writeBytes);
        curAddr++;
    }

    AT93cxxEwDisable(device);
    return true;
}

void SetupCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType) {
    bool ret;
    
    at93cxxDevice.model = (EEPROMType_t)commReqPacket->buffer[3];
    at93cxxDevice.org = (AT93cxxOrg_t)commReqPacket->buffer[2];

    ret = SetupAT93cxx(&at93cxxDevice);
    if (ret) {
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
        commResPacket->buffer[0] = 0x01;
        commResPacket->bufLen = 1;
    } else {
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
        commResPacket->buffer[0] = 0x00;
        commResPacket->bufLen = 1;
    }

    this->SetCommandRepeatState(false);
}

void ReadCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                          const EsyPro::CommPacket_t *commReqPacket,
                          EsyPro::CommunicationType_t commType) {
    bool ret;

    ret = ReadAT93cxx(&at93cxxDevice, commResPacket->buffer, EEPROM_93CXX_BUF_SIZE);
    if (ret) {
        commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_READ_DATA_RES;
        commResPacket->bufLen = EEPROM_93CXX_BUF_SIZE;
    } else {
        commResPacket->cmd = CMD_IGNORE_RES;
    }

    this->SetCommandRepeatState(false);
}

void WriteCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                           const EsyPro::CommPacket_t *commReqPacket,
                           EsyPro::CommunicationType_t commType) {
    bool ret;

    ret = WriteAT93cxx(&at93cxxDevice, commReqPacket->buffer, EEPROM_93CXX_BUF_SIZE);
    if (ret) {
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
        commResPacket->bufLen = 0;
    } else {
        commResPacket->cmd = CMD_IGNORE_RES;
    }

    this->SetCommandRepeatState(false);
}
