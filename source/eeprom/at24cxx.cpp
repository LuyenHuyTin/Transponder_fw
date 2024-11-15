#include "ble_common.h"
#include "eeprom/at24cxx.h"
#include "peripheral/i2c.h"
#include "peripheral/timer.h"

using namespace EsyPro;
using namespace EEPROM;
using namespace AT24CXX;

static const char LOG_DGB_NAME[] = "at24cxx";
static uint8_t pageSize = 0;
static uint16_t curAddr = 0;
static uint8_t at24cxxBaseAddr;
static EEPROMType_t at24cxxType;

static AT24CXX::SetupCommand at24cxxSetupCmd;
static AT24CXX::ReadCommand at24cxxReadCmd;
static AT24CXX::WriteCommand at24cxxWriteCmd;

Command *AT24CXX::GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType) {
    Command *cmd = NULL;

    switch (commCmdType & 0x0F) {
    case CMD_BASIC_MEM_SETUP_REQ:
        cmd = &at24cxxSetupCmd;
        NRF_LOG_INFO("[%s]: INFO: Setup Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_READ_DATA_REQ:
        cmd = &at24cxxReadCmd;
        NRF_LOG_INFO("[%s]: INFO: Read Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_WRITE_DATA_REQ:
        cmd = &at24cxxWriteCmd;
        NRF_LOG_INFO("[%s]: INFO: Write Request", LOG_DGB_NAME);
        break;
    }

    return cmd;    
}

static bool SetupAT24cxx(EEPROMType_t type) {
    uint8_t dummyByte;
    int sda_cnt = 0;
    uint8_t availAddr[] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};
    Peripheral::I2C1Obj *at24cxx = Peripheral::I2C1Obj::GetInstance();

    curAddr = 0;

    if (type <= EEPROM_24CXX_2K) {
        pageSize = 8;

    } else if (type >= EEPROM_24CXX_4K && type <= EEPROM_24CXX_16K) {
        pageSize = 16;
    
    } else {
        pageSize = 32;
    }

    at24cxx->DeInit();
    
    nrf_gpio_cfg_output(EEPROM_24CXX_I2C_SCL);
    nrf_gpio_cfg_output(EEPROM_24CXX_I2C_SDA);
    nrf_gpio_pin_write(EEPROM_24CXX_I2C_SCL, 0);
    nrf_gpio_pin_write(EEPROM_24CXX_I2C_SDA, 0);
    nrf_delay_ms(100);
    nrf_gpio_pin_write(EEPROM_24CXX_I2C_SCL, 1);
    nrf_gpio_pin_write(EEPROM_24CXX_I2C_SDA, 1);
    nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_NOPULL);

    for (int i = 0; i < 9; i++) {
        nrf_gpio_pin_write(EEPROM_24CXX_I2C_SCL, 1);
        nrf_delay_us(5);
        if (nrf_gpio_pin_read(EEPROM_24CXX_I2C_SDA)) {
            sda_cnt++;
            if (sda_cnt == 2) {
                break;
            }
        }
        nrf_gpio_pin_write(EEPROM_24CXX_I2C_SCL, 0);
        nrf_delay_us(5);
    }

    nrf_gpio_pin_write(EEPROM_24CXX_I2C_SCL, 0);
    nrf_gpio_cfg_output(EEPROM_24CXX_I2C_SDA);
    nrf_gpio_pin_write(EEPROM_24CXX_I2C_SDA, 0);
    nrf_delay_us(5);
    nrf_gpio_pin_write(EEPROM_24CXX_I2C_SCL, 1);
    nrf_gpio_pin_write(EEPROM_24CXX_I2C_SDA, 1);

    nrf_delay_ms(10);
    
    if (at24cxxBaseAddr == 0x00) {
        at24cxx->Init();

        for (unsigned int i = 0; i < sizeof(availAddr); i++) {
            bool ret;
            ret = at24cxx->ReceiveByte(availAddr[i], &dummyByte, 1, 100);
            if (ret) {
                at24cxxBaseAddr = availAddr[i];
                NRF_LOG_INFO("Automatic finding address: %x", at24cxxBaseAddr);
                at24cxx->DeInit();
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
                return true;
            }
        }

    } else {
        NRF_LOG_INFO("Force select address: %x", at24cxxBaseAddr);
        nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
        nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
        return true;
    }

    nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
    return false;
}

static bool ReadAT24cxx(EEPROMType_t type, uint8_t *data, uint8_t dataLen) {
    bool ret;
    Peripheral::I2C1Obj *at24cxx = Peripheral::I2C1Obj::GetInstance();

    at24cxx->Init();

    if ((type == EEPROM_24CXX_1K) || (type == EEPROM_24CXX_2K)) {
        uint8_t memAddr = curAddr & 0xFF;
        ret = at24cxx->SendBytes(at24cxxBaseAddr, &memAddr, 1, true, EEPROM_TIMEOUT);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read EEPROM_24CXX_1/EEPROM_24CXX_2K-1", LOG_DGB_NAME);
            at24cxx->DeInit();
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
            return false;
        }
        ret = at24cxx->ReceiveByte(at24cxxBaseAddr, data, dataLen, EEPROM_TIMEOUT);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read EEPROM_24CXX_1/EEPROM_24CXX_2K-2", LOG_DGB_NAME);
            at24cxx->DeInit();
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
            return false;
        }

    } else if (type == EEPROM_24CXX_4K) {
        uint8_t memAddr = curAddr & 0xFF;
        ret = at24cxx->SendBytes(at24cxxBaseAddr | ((curAddr & 0x0100) >> 8), 
                                 &memAddr, 1, true, EEPROM_TIMEOUT);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read EEPROM_24CXX_4K-1", LOG_DGB_NAME);
            at24cxx->DeInit();
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
            return false;
        }
        ret = at24cxx->ReceiveByte(at24cxxBaseAddr | ((curAddr & 0x0100) >> 8),
                                   data, dataLen, EEPROM_TIMEOUT);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read EEPROM_24CXX_4K-2", LOG_DGB_NAME);
            at24cxx->DeInit();
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
            return false;
        }

    } else if (type == EEPROM_24CXX_8K) {
        uint8_t memAddr = curAddr & 0xFF;
        ret = at24cxx->SendBytes(at24cxxBaseAddr | ((curAddr & 0x0300) >> 8),
                                 &memAddr, 1, true, EEPROM_TIMEOUT);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read EEPROM_24CXX_8K-1", LOG_DGB_NAME);
            at24cxx->DeInit();
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
            return false;
        }
        ret = at24cxx->ReceiveByte(at24cxxBaseAddr | ((curAddr & 0x0300) >> 8), 
                                   data, dataLen, EEPROM_TIMEOUT);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read EEPROM_24CXX_8K-2", LOG_DGB_NAME);
            at24cxx->DeInit();
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
            return false;
        }

    } else if (type == EEPROM_24CXX_16K) {
        uint8_t memAddr = curAddr & 0xFF;
        ret = at24cxx->SendBytes(at24cxxBaseAddr | ((curAddr & 0x0700) >> 8),
                                 &memAddr, 1, true, EEPROM_TIMEOUT);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read EEPROM_24CXX_16K-1", LOG_DGB_NAME);
            at24cxx->DeInit();
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
            return false;
        }
        ret = at24cxx->ReceiveByte(at24cxxBaseAddr | ((curAddr & 0x0700) >> 8), 
                                   data, dataLen, EEPROM_TIMEOUT);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read EEPROM_24CXX_16K-2", LOG_DGB_NAME);
            at24cxx->DeInit();
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
            return false;
        }

    } else {
        uint8_t memAddr[2];

        memAddr[0] = (curAddr >> 8) & 0xFF;
        memAddr[1] = curAddr & 0xFF;
        ret = at24cxx->SendBytes(at24cxxBaseAddr, memAddr, 2, true, EEPROM_TIMEOUT);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read EEPROM_24CXX_32-1", LOG_DGB_NAME);
            at24cxx->DeInit();
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
            return false;
        }
        ret = at24cxx->ReceiveByte(at24cxxBaseAddr, data, dataLen, EEPROM_TIMEOUT);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read EEPROM_24CXX_32-2", LOG_DGB_NAME);
            at24cxx->DeInit();
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
            nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
            return false;
        }
    }

    curAddr += dataLen;
    at24cxx->DeInit();
    nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);

    return true;
}

static bool WriteAT24cxx(EEPROMType_t type, const uint8_t *data, uint8_t dataLen) {
    uint8_t bytesLeft = EEPROM_24CXX_BUF_SIZE;
    Peripheral::I2C1Obj *at24cxx = Peripheral::I2C1Obj::GetInstance();
    uint8_t buffer[50];
    bool ret;

    at24cxx->Init();

    while (bytesLeft > 0) {
        uint8_t bytesToWrites;

        bytesToWrites = pageSize - (curAddr % pageSize);
        if (bytesToWrites > bytesLeft) {
            bytesToWrites = bytesLeft;
        }

        if ((type == EEPROM_24CXX_1K) || (type == EEPROM_24CXX_2K)) {
            buffer[0] = curAddr & 0xFF;
            memcpy(&buffer[1], &data[EEPROM_24CXX_BUF_SIZE - bytesLeft], bytesToWrites);
            ret = at24cxx->SendBytes(at24cxxBaseAddr, buffer, bytesToWrites + 1,
                                     false, EEPROM_TIMEOUT);
            if (!ret) {
                at24cxx->DeInit();
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
                return false;
            }

        } else if (type == EEPROM_24CXX_4K) {
            buffer[0] = curAddr & 0xFF;
            memcpy(&buffer[1], &data[EEPROM_24CXX_BUF_SIZE - bytesLeft], bytesToWrites);
            ret = at24cxx->SendBytes(at24cxxBaseAddr | ((curAddr & 0x0100) >> 8), 
                                    buffer, bytesToWrites + 1, false, EEPROM_TIMEOUT);            
            if (!ret) {
                at24cxx->DeInit();
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
                return false;
            }

        } else if (type == EEPROM_24CXX_8K) {
            buffer[0] = curAddr & 0xFF;
            memcpy(&buffer[1], &data[EEPROM_24CXX_BUF_SIZE - bytesLeft], bytesToWrites);
            ret = at24cxx->SendBytes(at24cxxBaseAddr | ((curAddr & 0x0300) >> 8), 
                                    buffer, bytesToWrites + 1, false, EEPROM_TIMEOUT);            
            if (!ret) {
                at24cxx->DeInit();
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
                return false;
            }

        } else if (type == EEPROM_24CXX_16K) {
            buffer[0] = curAddr & 0xFF;
            memcpy(&buffer[1], &data[EEPROM_24CXX_BUF_SIZE - bytesLeft], bytesToWrites);
            ret = at24cxx->SendBytes(at24cxxBaseAddr | ((curAddr & 0x0700) >> 8),
                                    buffer, bytesToWrites + 1, false, EEPROM_TIMEOUT);
            if (!ret) {
                at24cxx->DeInit();
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
                return false;
            }

        } else {
            buffer[0] = (curAddr >> 8) & 0xFF;
            buffer[1] = curAddr & 0xFF;
            memcpy(&buffer[2], &data[EEPROM_24CXX_BUF_SIZE - bytesLeft], bytesToWrites);
            ret = at24cxx->SendBytes(at24cxxBaseAddr, buffer, bytesToWrites + 2,
                                     false, EEPROM_TIMEOUT);
            if (!ret) {
                at24cxx->DeInit();
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
                nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
                return false;
            }
        }

        curAddr += bytesToWrites;
        bytesLeft -= bytesToWrites;
        nrf_delay_ms(10);
    }

    at24cxx->DeInit();
    nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SDA, NRF_GPIO_PIN_PULLDOWN);
    nrf_gpio_cfg_input(EEPROM_24CXX_I2C_SCL, NRF_GPIO_PIN_PULLDOWN);
    return true;
}

void SetupCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType) {
    bool ret;
    at24cxxType = (EEPROMType_t)commReqPacket->buffer[3];    
    at24cxxBaseAddr = commReqPacket->buffer[7];

    ret = SetupAT24cxx(at24cxxType);
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

    ret = ReadAT24cxx(at24cxxType, commResPacket->buffer, EEPROM_24CXX_BUF_SIZE);
    if (ret) {
        commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_READ_DATA_RES;
        commResPacket->bufLen = EEPROM_24CXX_BUF_SIZE;
    } else {
        commResPacket->cmd = CMD_IGNORE_RES;
    }

    this->SetCommandRepeatState(false);
}

void WriteCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                           const EsyPro::CommPacket_t *commReqPacket,
                           EsyPro::CommunicationType_t commType) {
    bool ret;

    ret = WriteAT24cxx(at24cxxType, commReqPacket->buffer, EEPROM_24CXX_BUF_SIZE);
    if (ret) {
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
        commResPacket->bufLen = 0;
    } else {
        commResPacket->cmd = CMD_IGNORE_RES;
    }

    this->SetCommandRepeatState(false);
}
