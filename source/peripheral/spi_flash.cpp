#include "peripheral/spi_flash.h"
#include "peripheral/timer.h"
#include "ble_common.h"
#include "crc32.h"

using namespace EsyPro;

static const char LOG_DGB_NAME[] = "spi_flash";
static const nrf_drv_spi_t spi0 = NRF_DRV_SPI_INSTANCE(SPI_FLASH_IDX);
static uint32_t fileSize = 0;
static uint32_t crc32Calc = 0;
static uint32_t flashAddr = 0;

static SpiFlash::EraseCommand spiFlashEraseCmd;
static SpiFlash::WriteCommand spiFlashWriteCmd;
static SpiFlash::VerifyCommand spiFlashVerifyCmd;
static SpiFlash::ReadCommand spiFlashReadCmd;

namespace SpiFlash {
    Spi0Obj *Spi0Obj::pInstance = NULL;

    Spi0Obj::Spi0Obj(void) {
        spiDrv = spi0;
        transferDone = false;
    }

    Spi0Obj *Spi0Obj::GetInstance(void) {
        if (pInstance == NULL) {
            pInstance = new Spi0Obj();
        }

        return pInstance;
    }

    void Spi0Obj::SetTransferDone(bool state) {
        transferDone = state;
    }

    static void Spi0EventHandler(const nrf_drv_spi_evt_t *pEvt, void *pContext) {
        Spi0Obj *spiFlash = Spi0Obj::GetInstance();

        switch (pEvt->type) {
        case NRF_DRV_SPI_EVENT_DONE:
            spiFlash->SetTransferDone(true);
            break;

        default:
            break;
        }
    }

    void Spi0Obj::Init(void) {
        ret_code_t errCode;
        nrf_drv_spi_config_t spiCfg = NRF_DRV_SPI_DEFAULT_CONFIG;

        spiCfg.ss_pin = SPI_FLASH_SS_PIN;
        spiCfg.miso_pin = SPI_FLASH_MISO_PIN;
        spiCfg.mosi_pin = SPI_FLASH_MOSI_PIN;
        spiCfg.sck_pin = SPI_FLASH_SCK_PIN;
        spiCfg.frequency = NRF_DRV_SPI_FREQ_4M;
        spiCfg.mode = NRF_DRV_SPI_MODE_0;
        errCode = nrf_drv_spi_init(&spiDrv, &spiCfg, Spi0EventHandler, NULL);
        APP_ERROR_CHECK(errCode);
    }

    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        switch (commCmdType & 0x0F) {
        case CMD_QSPI_FLASH_ERASE_REQ:
            cmd = &spiFlashEraseCmd;
            NRF_LOG_INFO("[%s]: INFO: SPI Flash Erase Request", LOG_DGB_NAME);
            break;

        case CMD_QSPI_FLASH_WRITE_REQ:
            cmd = &spiFlashWriteCmd;
            NRF_LOG_INFO("[%s]: INFO: SPI Flash Write Request", LOG_DGB_NAME);
            break;

        case CMD_QSPI_FLASH_VERIFY_REQ:
            cmd = &spiFlashVerifyCmd;
            NRF_LOG_INFO("[%s]: INFO: SPI Flash Verify Request", LOG_DGB_NAME);
            break;

        case CMD_QSPI_FLASH_READ_REQ:
            cmd = &spiFlashReadCmd;
            NRF_LOG_INFO("[%s]: INFO: SPI Flash Read Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    bool Spi0Obj::TransferBytes(const uint8_t *sendBytes, uint16_t sendLen,
                                uint8_t *recvBytes, uint16_t recvLen) {
        ret_code_t errCode;
        Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();

        transferDone = false;
        timer->Start(SPI_FLASH_TIMEOUT);
        errCode = nrf_drv_spi_transfer(&spiDrv, sendBytes, sendLen, recvBytes, recvLen);
        APP_ERROR_CHECK(errCode);
        while (!timer->IsTimeout()) {
            if (transferDone) {
                return true;
            }
        }

        return false;
    }

    bool Spi0Obj::WaitForWriteEnd(void) {
        uint8_t cmd = CMD_RDSR1;
        uint8_t byteToRecv[2];

        do {
            nrf_delay_ms(1);
            if (!TransferBytes(&cmd, 1, byteToRecv, 2)) {
                return false;
            }
        } while ((byteToRecv[1] & 0x01) == 0x01);

        return true;
    }

    bool Spi0Obj::WriteEnable(void) {
        uint8_t cmd = CMD_WREN;
        uint8_t dummyBytes[1];
    
        if (!TransferBytes(&cmd, 1, dummyBytes, 1)) {
            return false;
        }
        nrf_delay_ms(1);

        return true;
    }

    bool Spi0Obj::EraseBlock64K(uint32_t addr) {
        uint8_t bytesToSend[4];
        uint8_t dummyBytes[4];

        if (!WaitForWriteEnd()) {
            return false;
        }
        if (!WriteEnable()) {
            return false;
        }

        bytesToSend[0] = CMD_ERASE_64K;
        bytesToSend[1] = (addr >> 16) & 0xFF;
        bytesToSend[2] = (addr >> 8) & 0xFF;
        bytesToSend[3] = addr & 0xFF;
        if (!TransferBytes(bytesToSend, 4, dummyBytes, 4)) {
            return false;
        }
        if (!WaitForWriteEnd()) {
            return false;
        }
        nrf_delay_ms(1);

        return true;
    }

    bool Spi0Obj::WriteToFlash(uint32_t addr, const uint8_t *data, uint16_t dataLen) {
        uint8_t bytesToSend[SPI_FLASH_MAX_WRITE_DATA_SIZE];
        uint8_t dummyBytes[SPI_FLASH_MAX_WRITE_DATA_SIZE];

        if (!WaitForWriteEnd()) {
            return false;
        }
        if (!WriteEnable()) {
            return false;
        }

        bytesToSend[0] = CMD_PP;
        bytesToSend[1] = (addr & 0xFF0000) >> 16;
        bytesToSend[2] = (addr & 0xFF00) >> 8;
        bytesToSend[3] = addr & 0xFF;
        memcpy(&bytesToSend[4], data, dataLen);
        if (!TransferBytes(bytesToSend, dataLen + 4, dummyBytes, dataLen + 4)) {
            return false;
        }
        if (!WaitForWriteEnd()) {
            return false;
        }
        nrf_delay_ms(1);

        return true;
    }

    bool Spi0Obj::ReadFromFlash(uint32_t addr, uint8_t *data, uint16_t dataLen) {
        uint8_t bytesToSend[4];

        bytesToSend[0] = CMD_READ;
        bytesToSend[1] = (addr & 0xFF0000) >> 16;
        bytesToSend[2] = (addr & 0xFF00) >> 8;
        bytesToSend[3] = addr & 0xFF;
        if (!TransferBytes(bytesToSend, 4, data, dataLen + 4)) {
            return false;
        }
        nrf_delay_ms(1);
        for (int i = 0; i < dataLen; i++) {
            data[i] = data[i + 4];
        }

        return true;
    }

    static bool EraseFlash(uint32_t size) {
        uint32_t blockSize = 65536;
        uint32_t tmpAddr = 0;
        uint16_t totalBlocks = (size / blockSize) + 1;
        Spi0Obj *spiFlash = Spi0Obj::GetInstance();

        for (int i = 0; i < totalBlocks; i++) {
            if (!spiFlash->EraseBlock64K(tmpAddr)) {
                return false;
            }
            tmpAddr += blockSize;
        }

        return true;
    }

    static bool WriteFlash(const uint8_t *data, uint16_t dataLen) {
        uint8_t readBackBytes[SPI_FLASH_MAX_WRITE_DATA_SIZE];
        Spi0Obj *spiFlash = Spi0Obj::GetInstance();

        if (!spiFlash->WriteToFlash(flashAddr, data, dataLen)) {
            return false;
        }

        if (!spiFlash->ReadFromFlash(flashAddr, readBackBytes, dataLen)) {
            return false;
        }

        if (memcmp(data, readBackBytes, dataLen)) {
            return false;
        }

        crc32Calc = crc32_compute(readBackBytes, dataLen, &crc32Calc);
        flashAddr += dataLen;
        
        return true;
    }

    void EraseCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                               const EsyPro::CommPacket_t *commReqPacket,
                               EsyPro::CommunicationType_t commType) {
        bool ret;

        memcpy(&fileSize, commReqPacket->buffer, 4);

        ret = EraseFlash(fileSize);
        if (ret) {
            crc32Calc = 0;
            flashAddr = 0;
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_QSPI_FLASH_ERASE_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void WriteCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                               const EsyPro::CommPacket_t *commReqPacket,
                               EsyPro::CommunicationType_t commType) {
        bool ret;

        ret = WriteFlash(commReqPacket->buffer, commReqPacket->bufLen);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_QSPI_FLASH_WRITE_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void VerifyCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                const EsyPro::CommPacket_t *commReqPacket,
                                EsyPro::CommunicationType_t commType) {
        if (flashAddr == fileSize) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_QSPI_FLASH_VERIFY_RES;
            commResPacket->bufLen = 4;
            commResPacket->buffer[0] = crc32Calc & 0x000000FF;
            commResPacket->buffer[1] = (crc32Calc & 0x0000FF00) >> 8;
            commResPacket->buffer[2] = (crc32Calc & 0x00FF0000) >> 16;
            commResPacket->buffer[3] = (crc32Calc & 0xFF000000) >> 24;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false); 
    }

    void ReadCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                              const EsyPro::CommPacket_t *commReqPacket,
                              EsyPro::CommunicationType_t commType) {
        uint8_t dataLen;
        uint32_t readAddr;
        bool ret;
        Spi0Obj *spiFlash = Spi0Obj::GetInstance();

        dataLen = commReqPacket->buffer[0];
        memcpy(&readAddr, &commReqPacket->buffer[1], 4);

        ret = spiFlash->ReadFromFlash(readAddr, commResPacket->buffer, dataLen);
        if (ret) {
            commResPacket->cmd = CMD_QSPI_FLASH_READ_RES;
            commResPacket->bufLen = dataLen;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }
}
