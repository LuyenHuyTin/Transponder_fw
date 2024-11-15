#include "ble_common.h"
#include "eeprom/pic18fxx.h"

using namespace EsyPro;
using namespace EEPROM;
using namespace PIC18FXX;

static uint32_t curAddr = 0;
static Pic18fxx_t pic18fxxDevice;
static const char LOG_DGB_NAME[] = "pic18fxx";

static PIC18FXX::SetupCommand pic18fxxSetupCmd;
static PIC18FXX::ReadCommand pic18fxxReadCmd;
static PIC18FXX::WriteCommand pic18fxxWriteCmd;

static void Pic18fIspSendMsbFirst(uint16_t data, uint8_t numBits) {
    for (uint8_t i = 0; i < numBits; i++) {
        (data & (1 << (numBits - i - 1))) ? nrf_gpio_pin_write(EEPROM_PIC_DAT, 1)
                        : nrf_gpio_pin_write(EEPROM_PIC_DAT, 0);
        nrf_delay_us(ISP_CLK_DELAY);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
        nrf_gpio_pin_write(EEPROM_PIC_DAT, 0);
    }
}

static void Pic18fIspSendLsbFirst(uint16_t data, uint8_t numBits) {
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

static uint8_t Pic18fReadFlash(uint32_t addr) {
    uint8_t data = 0x00;

    Pic18fIspSendLsbFirst(0x0, 4);  // MOVLW ADDR[21:16]
    Pic18fIspSendLsbFirst((0x0E00 | ((addr >> 16) & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TBLPTRU
    Pic18fIspSendLsbFirst(0x6EF8, 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVLW ADDR[15:8]
    Pic18fIspSendLsbFirst((0x0E00 | ((addr >> 8) & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TBLPTRH
    Pic18fIspSendLsbFirst(0x6EF7, 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVLW ADDR[7:0]
    Pic18fIspSendLsbFirst((0x0E00 | (addr  & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TBLPTRL
    Pic18fIspSendLsbFirst(0x6EF6, 16);

    Pic18fIspSendLsbFirst(0x08, 4); // Table Read
    nrf_gpio_cfg_input(EEPROM_PIC_DAT, NRF_GPIO_PIN_PULLDOWN);
    for (int i = 0; i < 8; i++) {
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
    }
    for (int i = 0; i < 8; i++) {
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
        nrf_delay_us(ISP_CLK_DELAY);
        if (nrf_gpio_pin_read(EEPROM_PIC_DAT)) {
            data += (1 << i);
        }

        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
        nrf_delay_us(ISP_CLK_DELAY);
    }

    nrf_gpio_cfg_output(EEPROM_PIC_DAT);
    return data;
}

static uint8_t Pic18fReadEeprom(uint16_t addr) {
    uint8_t data = 0x00;

    Pic18fIspSendLsbFirst(0x0, 4);  // BCF EECON1, EEPGD
    Pic18fIspSendLsbFirst(0x9E7F, 16);
    Pic18fIspSendLsbFirst(0x0, 4);  // BCF EECON1, CFGS
    Pic18fIspSendLsbFirst(0x9C7F, 16);

    Pic18fIspSendLsbFirst(0x0, 4);  // MOVLW ADDR[7:0]
    Pic18fIspSendLsbFirst((0x0E00 | (addr  & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF EEADR
    Pic18fIspSendLsbFirst(0x6E62, 16);
    Pic18fIspSendLsbFirst(0x0, 4);  // MOVLW ADDR[15:8]
    Pic18fIspSendLsbFirst((0x0E00 | ((addr >> 8) & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF EEADRH
    Pic18fIspSendLsbFirst(0x6E63, 16);

    Pic18fIspSendLsbFirst(0x0, 4);  // BSF EECON1, RD
    Pic18fIspSendLsbFirst(0x807F, 16);

    Pic18fIspSendLsbFirst(0x0, 4);  // MOVF EEDATA, W, 0
    Pic18fIspSendLsbFirst(0x5061, 16);
    Pic18fIspSendLsbFirst(0x0, 4);  // MOVWF TABLAT
    Pic18fIspSendLsbFirst(0x6EF5, 16);
    Pic18fIspSendLsbFirst(0x0, 4);  // NOP
    Pic18fIspSendLsbFirst(0x0000, 16);

    Pic18fIspSendLsbFirst(0x02, 4); // Shift data
    nrf_gpio_cfg_input(EEPROM_PIC_DAT, NRF_GPIO_PIN_PULLDOWN);
    for (int i = 0; i < 8; i++) {
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
    }
    for (int i = 0; i < 8; i++) {
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
        nrf_delay_us(ISP_CLK_DELAY);
        if (nrf_gpio_pin_read(EEPROM_PIC_DAT)) {
            data += (1 << i);
        }

        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
        nrf_delay_us(ISP_CLK_DELAY);
    }

    nrf_gpio_cfg_output(EEPROM_PIC_DAT);
    return data;
}

static void Pic18fWriteEeprom(uint16_t addr, uint8_t data) {
    uint8_t flag = 0x02;

    Pic18fIspSendLsbFirst(0x0, 4);  // BCF EECON1, EEPGD
    Pic18fIspSendLsbFirst(0x9E7F, 16);
    Pic18fIspSendLsbFirst(0x0, 4);  // BCF EECON1, CFGS
    Pic18fIspSendLsbFirst(0x9C7F, 16);

    Pic18fIspSendLsbFirst(0x0, 4);  // MOVLW ADDR[7:0]
    Pic18fIspSendLsbFirst((0x0E00 | (addr  & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF EEADR
    Pic18fIspSendLsbFirst(0x6E62, 16);
    Pic18fIspSendLsbFirst(0x0, 4);  // MOVLW ADDR[15:8]
    Pic18fIspSendLsbFirst((0x0E00 | ((addr >> 8) & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF EEADRH
    Pic18fIspSendLsbFirst(0x6E63, 16);

    Pic18fIspSendLsbFirst(0x0, 4);  // MOVLW DATA
    Pic18fIspSendLsbFirst((0x0E00 | (data  & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF EEDATA
    Pic18fIspSendLsbFirst(0x6E61, 16);

    Pic18fIspSendLsbFirst(0x0, 4); // BSF EECON1, WREN
    Pic18fIspSendLsbFirst(0x847F, 16);

    Pic18fIspSendLsbFirst(0x0, 4); // BSF EECON1, WR
    Pic18fIspSendLsbFirst(0x827F, 16);
    
    while (flag & 0x02) {
        uint8_t tmpData = 0;

        Pic18fIspSendLsbFirst(0x0, 4); // MOVF EECON1, W, 0
        Pic18fIspSendLsbFirst(0x507F, 16);
        Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TABLAT
        Pic18fIspSendLsbFirst(0x6EF5, 16);
        Pic18fIspSendLsbFirst(0x0, 4); // NOP
        Pic18fIspSendLsbFirst(0x0000, 16);

        Pic18fIspSendLsbFirst(0x02, 4); // Shift data
        nrf_gpio_cfg_input(EEPROM_PIC_DAT, NRF_GPIO_PIN_PULLDOWN);
        for (int i = 0; i < 8; i++) {
            nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
            nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
        }
        for (int i = 0; i < 8; i++) {
            nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
            nrf_delay_us(ISP_CLK_DELAY);
            if (nrf_gpio_pin_read(EEPROM_PIC_DAT)) {
                tmpData += (1 << i);
            }

            nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
            nrf_delay_us(ISP_CLK_DELAY);
        }
        
        flag = tmpData;
        nrf_gpio_cfg_output(EEPROM_PIC_DAT);
    }

    nrf_delay_us(100);
    Pic18fIspSendLsbFirst(0x0, 4); // BCF EECON1, WREN
    Pic18fIspSendLsbFirst(0x94F7, 16);
}

static void Pic18fEnterProgmode(bool isHighVoltage = true) {
    uint8_t progmodeKey[] = {0x4D, 0x43, 0x48, 0x50};

    if (!isHighVoltage) {
        nrf_gpio_pin_write(EEPROM_PIC_MCLR, 1);
        nrf_gpio_pin_write(EEPROM_PIC_DAT, 0);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
        nrf_delay_us(600);
        nrf_gpio_pin_write(EEPROM_PIC_MCLR, 0);
        nrf_delay_ms(1);

        for (uint8_t i = 0; i < sizeof(progmodeKey); i++) {
            Pic18fIspSendMsbFirst(progmodeKey[i], 8);
        }
        nrf_delay_us(ISP_CLK_DELAY);
        nrf_gpio_pin_write(EEPROM_PIC_MCLR, 1);

    } else {
        nrf_gpio_pin_write(EEPROM_PIC_DAT, 0);
        nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
        nrf_gpio_pin_set(POW_OUT_8V);
        nrf_delay_ms(200);

        nrf_gpio_pin_clear(POW_OUT_8V);
        nrf_delay_ms(200);

        for (uint8_t i = 0; i < sizeof(progmodeKey); i++) {
            Pic18fIspSendMsbFirst(progmodeKey[i], 8);
        }
    }
}

static void Pic18fEraseBlock(uint16_t cmdErase1, uint16_t cmdErase2) {
    Pic18fIspSendLsbFirst(0x0, 4); // MOVLW 3Ch
    Pic18fIspSendLsbFirst(0x0E3C, 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TBLPTRU
    Pic18fIspSendLsbFirst(0x6EF8, 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVLW 00h
    Pic18fIspSendLsbFirst(0x0E00, 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TBLPTRH
    Pic18fIspSendLsbFirst(0x6EF7, 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVLW 04h
    Pic18fIspSendLsbFirst(0x0E04, 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TBLPTRL
    Pic18fIspSendLsbFirst(0x6EF6, 16);

    Pic18fIspSendLsbFirst(0x0C, 4);
    Pic18fIspSendLsbFirst(cmdErase1, 16);

    Pic18fIspSendLsbFirst(0x00, 4); // MOVLW 05h
    Pic18fIspSendLsbFirst(0x0E05, 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TBLPTRL
    Pic18fIspSendLsbFirst(0x6EF6, 16);
    Pic18fIspSendLsbFirst(0x0C, 4);
    Pic18fIspSendLsbFirst(cmdErase2, 16);
    
    Pic18fIspSendLsbFirst(0x0, 4); // MOVLW 06h
    Pic18fIspSendLsbFirst(0x0E06, 16);
    Pic18fIspSendLsbFirst(0x00, 4); // MOVWF TBLPTRL
    Pic18fIspSendLsbFirst(0x6EF6, 16);
    Pic18fIspSendLsbFirst(0x0C, 4); // Write 80h to 3C0006h to erase block 0
    Pic18fIspSendLsbFirst(0x8080, 16);

    Pic18fIspSendLsbFirst(0x0, 4); // NOP
    Pic18fIspSendLsbFirst(0x0000, 16);
    Pic18fIspSendLsbFirst(0x0, 4); // NOP
    Pic18fIspSendLsbFirst(0x0000, 16);
    nrf_delay_ms(500);
    nrf_gpio_pin_write(EEPROM_PIC_DAT, 1);
}

static uint16_t Pic18fReadDeviceID(void) {
    uint16_t id;
    Pic18fEnterProgmode();
    nrf_delay_ms(1);

    uint8_t devID1 = Pic18fReadFlash(0x3FFFFE) & 0xE0;
    uint8_t devID2 = Pic18fReadFlash(0x3FFFFF);

    id = (devID2 << 3) | (devID1 >> 5);

    return id;
}

static void Pic18fxxReadFlashData(uint8_t *data, uint8_t dataLen) {
    if (pic18fxxDevice.startAddr == curAddr) {
        Pic18fEnterProgmode();
        nrf_delay_ms(1);
    }

    for (int i = 0; i < dataLen; i++) {
        data[i] = Pic18fReadFlash(curAddr);
        curAddr++;
    }
}

static void Pic18fxxReadEepromData(uint8_t *data, uint8_t dataLen) {
    if (pic18fxxDevice.startAddr == curAddr) {
        Pic18fEnterProgmode();
        nrf_delay_ms(1);
    }

    for (int i = 0; i < dataLen; i++) {
        data[i] = Pic18fReadEeprom(curAddr);
        curAddr++;
    }
}

static void Pic18fxxWriteEepromData(const uint8_t *data, uint8_t dataLen) {
    if (pic18fxxDevice.startAddr == curAddr) {
        Pic18fEnterProgmode();
        nrf_delay_ms(1);
    }

    for (int i = 0; i < dataLen; i++) {
        Pic18fWriteEeprom((curAddr & 0xFFFF), data[i]);
        curAddr++;
    }
    
}

static void Pic18fEraseFlashData(void) {
    Pic18fEraseBlock(0x0505, 0x0000);

    Pic18fEraseBlock(0x0404, 0x0101);
    Pic18fEraseBlock(0x0404, 0x0202);
    Pic18fEraseBlock(0x0404, 0x0404);
    Pic18fEraseBlock(0x0404, 0x0808);

    if (pic18fxxDevice.endAddr > 0xFFFF) {
        Pic18fEraseBlock(0x0404, 0x1010);
        Pic18fEraseBlock(0x0404, 0x2020);
        Pic18fEraseBlock(0x0404, 0x4040);
        Pic18fEraseBlock(0x0404, 0x8080);
    }
}


static void Pic18fWriteFlash(uint32_t addr, const uint8_t *data, uint8_t dataLen) {
    uint8_t i;

    Pic18fIspSendLsbFirst(0x0, 4);  // BSF EECON1, EEPGD
    Pic18fIspSendLsbFirst(0x8E7F, 16);
    Pic18fIspSendLsbFirst(0x0, 4);  // BCF EECON1, CFGS
    Pic18fIspSendLsbFirst(0x9C7F, 16);
    Pic18fIspSendLsbFirst(0x0, 4);  // BSF EECON1, WREN
    Pic18fIspSendLsbFirst(0x847F, 16);

    Pic18fIspSendLsbFirst(0x0, 4);  // MOVLW ADDR[21:16]
    Pic18fIspSendLsbFirst((0x0E00 | ((addr >> 16) & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TBLPTRU
    Pic18fIspSendLsbFirst(0x6EF8, 16);

    Pic18fIspSendLsbFirst(0x0, 4);  // MOVLW ADDR[15:8]
    Pic18fIspSendLsbFirst((0x0E00 | ((addr >> 8) & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TBLPTRH
    Pic18fIspSendLsbFirst(0x6EF7, 16);

    Pic18fIspSendLsbFirst(0x0, 4);  // MOVLW ADDR[7:0]
    Pic18fIspSendLsbFirst((0x0E00 | (addr & 0xFF)), 16);
    Pic18fIspSendLsbFirst(0x0, 4); // MOVWF TBLPTRL
    Pic18fIspSendLsbFirst(0x6EF6, 16);

    for (i = 0; i < dataLen / 2 - 1; i++) {
        uint16_t tmpData = (data[i * 2 + 1] << 8) | data[i * 2];
        Pic18fIspSendLsbFirst(0x0D, 4); // Write 2 bytes + incr addr by 2
        Pic18fIspSendLsbFirst(tmpData, 16);
    }

    Pic18fIspSendLsbFirst(0x0F, 4); // Write last 2 bytes
    Pic18fIspSendLsbFirst((data[i * 2 + 1] << 8) | data[i * 2], 16);

    nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
    nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
    nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
    nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
    nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
    nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
    nrf_delay_us(1);

    nrf_gpio_pin_write(EEPROM_PIC_CLK, 1);
    nrf_delay_ms(1);
    nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
    nrf_delay_us(200);

    Pic18fIspSendLsbFirst(0x00, 16);
}

static void Pic18fxxWriteFlashData(const uint8_t *data, uint8_t dataLen) {
    if (pic18fxxDevice.startAddr == curAddr) {
        Pic18fEnterProgmode();
        nrf_delay_ms(1);
        Pic18fEraseFlashData();
    }

    for (int i = 0; i < dataLen / PIC_MAX_FLASH_BUF_SIZE; i++) {
        Pic18fWriteFlash(curAddr, &data[i * PIC_MAX_FLASH_BUF_SIZE], 
                         PIC_MAX_FLASH_BUF_SIZE);

        curAddr += PIC_MAX_FLASH_BUF_SIZE;
    }
}

Command *PIC18FXX::GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType) {
    Command *cmd = NULL;

    switch (commCmdType & 0x0F) {
    case CMD_BASIC_MEM_SETUP_REQ:
        cmd = &pic18fxxSetupCmd;
        NRF_LOG_INFO("[%s]: INFO: Setup Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_READ_DATA_REQ:
        cmd = &pic18fxxReadCmd;
        NRF_LOG_INFO("[%s]: INFO: Read Request", LOG_DGB_NAME);
        break;

    case CMD_BASIC_MEM_WRITE_DATA_REQ:
        cmd = &pic18fxxWriteCmd;
        NRF_LOG_INFO("[%s]: INFO: Write Request", LOG_DGB_NAME);
        break;    
    }

    return cmd;    
}

void SetupCommand::Execute(CommPacket_t *commResPacket,
                           const CommPacket_t *commReqPacket,
                           CommunicationType_t commType) {
    pic18fxxDevice.memType = commReqPacket->buffer[7] 
                    ? PIC18FXX_EEPROM_MEM : PIC18FXX_PROG_MEM;
    memcpy(&pic18fxxDevice.startAddr, &commReqPacket->buffer[0], 3);
    memcpy(&pic18fxxDevice.endAddr, &commReqPacket->buffer[4], 3);

    nrf_gpio_pin_write(EEPROM_PIC_MCLR, 0);
    nrf_gpio_pin_write(EEPROM_PIC_DAT, 0);
    nrf_gpio_pin_write(EEPROM_PIC_CLK, 0);
    nrf_gpio_cfg_output(EEPROM_PIC_MCLR);
    nrf_gpio_cfg_output(EEPROM_PIC_DAT);
    nrf_gpio_cfg_output(EEPROM_PIC_CLK);

    //Power-on
    nrf_gpio_pin_set(POW_OUT_5V);
    nrf_delay_ms(200); //small delay

    uint16_t id = Pic18fReadDeviceID();
    commResPacket->buffer[0] = (id == 0 || id == 0x01FF) ? 0 : 1;
    commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
    commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
    commResPacket->buffer[1] = id & 0xFF;
    commResPacket->buffer[2] = (id >> 8) & 0xFF;
    commResPacket->bufLen = 3;

    curAddr = pic18fxxDevice.startAddr;
    nrf_delay_ms(200);
    this->SetCommandRepeatState(false);
}

void ReadCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                          const EsyPro::CommPacket_t *commReqPacket,
                          EsyPro::CommunicationType_t commType) {
    if (curAddr >= pic18fxxDevice.endAddr) {
        commResPacket->bufLen = 0;
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_END_READ_DATA_RES;

    } else {
        if (pic18fxxDevice.memType == PIC18FXX_PROG_MEM) {
            Pic18fxxReadFlashData(commResPacket->buffer, PIC_PROG_BUF_SIZE);
            commResPacket->bufLen = PIC_PROG_BUF_SIZE;
            commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_READ_DATA_RES;

        } else if (pic18fxxDevice.memType == PIC18FXX_EEPROM_MEM) {
            Pic18fxxReadEepromData(commResPacket->buffer, PIC_EEPROM_BUF_SIZE);
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
    if (pic18fxxDevice.memType == PIC18FXX_PROG_MEM) {
        Pic18fxxWriteFlashData(commReqPacket->buffer, commReqPacket->bufLen);
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
        commResPacket->bufLen = 0;
    
    } else if (pic18fxxDevice.memType == PIC18FXX_EEPROM_MEM) {
        Pic18fxxWriteEepromData(commReqPacket->buffer, commReqPacket->bufLen);
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
        commResPacket->bufLen = 0;
    }

    this->SetCommandRepeatState(false);
}