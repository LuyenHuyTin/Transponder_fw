#include "ecu/honda_update.h"
#include "peripheral/uart.h"
#include "peripheral/spi_flash.h"
#include "ble_common.h"
#include "crc32.h"

using namespace EsyPro;
using namespace ECU;

static const char LOG_DGB_NAME[] = "honda_update";

static uint32_t spiFlashAddr = 0;
static uint32_t flashSize = 0;
static uint32_t flashOffset = 0;
static uint8_t pageSize = 0;
static uint16_t startPageOffset = 0;
static uint16_t endPageOffset = 0;
static uint16_t eepromAddr = 0;

static HondaKeihinUpdate::InitWriteCommand hondaKeihinInitWriteCmd;
static HondaKeihinUpdate::EraseCommand hondaKeihinEraseCmd;
static HondaKeihinUpdate::WriteFlashCommand hondaKeihinWriteFlashCmd;
static HondaKeihinUpdate::WriteFinishCommand hondaKeihinWriteFinishCmd;
static HondaKeihinUpdate::WriteEepromCommand hondaKeihinWriteEepromCmd;
static HondaKeihinUpdate::TurnOffCommand hondaKeihinOffCmd;
static HondaKeihinUpdate::TurnOnCommand hondaKeihinOnCmd;
static HondaKeihinUpdate::ReadEepromCommand hondaKeihinReadEepromCmd;
static HondaKeihinUpdate::FormatEepromCommand hondaKeihinFormatEepromCmd;

static HondaShindengenUpdate::InitWriteCommand hondaShindengenInitWriteCmd;
static HondaShindengenUpdate::SeedkeyCommand hondaShindengenSeedkeyCmd;
static HondaShindengenUpdate::EraseCommand hondaShindengenEraseCmd;
static HondaShindengenUpdate::WriteFlashCommand hondaShindengenWriteFlashCmd;
static HondaShindengenUpdate::WriteFinishCommand hondaShindengenWriteFinishCmd;
static HondaShindengenUpdate::WriteEepromCommand hondaShindengenWriteEepromCmd;
static HondaShindengenUpdate::EndWriteEepromCommand hondaShindengenEndwriteEepromCmd;
static HondaShindengenUpdate::TurnOffCommand hondaShindengenOffCmd;
static HondaShindengenUpdate::TurnOnCommand hondaShindengenOnCmd;
static HondaShindengenUpdate::ReadEepromCommand hondaShindengenReadEepromCmd;

static HondaCommonObj ecuObj;

namespace HondaCmd {
    static bool WakeUpHondaPgmfi(void) {
        bool ret;
        uint8_t srcCmd[] = {0xFE};
        uint8_t srcBuffer[] = {0x72};

        ecuObj.InitKlineBus();
        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();

        return ret;
    }

    static bool KeihinCheckInitWrite(void) {
        bool ret;
        uint8_t initWrite1Cmd[] = {0x7D};
        uint8_t initWrite1Buffer[] = {0x01, 0x02, 0x50, 0x47, 0x4D};
        uint8_t initWrite2Cmd[] = {0x7D};
        uint8_t initWrite2Buffer[] = {0x01, 0x03, 0x2D, 0x46, 0x49};

        ecuObj.SetReqEcuPacket(initWrite1Cmd, sizeof(initWrite1Cmd),
                             initWrite1Buffer, sizeof(initWrite1Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Init-Write-1", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(initWrite2Cmd, sizeof(initWrite1Cmd),
                             initWrite2Buffer, sizeof(initWrite2Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Init-Write-2", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

   static bool KeihinCheckInitWrite2(void) {
        bool ret;
        uint8_t initWrite1Cmd[] = {0x7B};
        uint8_t initWrite1Buffer[] = {0x00, 0x01, 0x04};
        uint8_t initWrite2Cmd[] = {0x7B};
        uint8_t initWrite2Buffer[] = {0x00, 0x02, 0x76, 0x03, 0x17};
        uint8_t initWrite3Cmd[] = {0x7B};
        uint8_t initWrite3Buffer[] = {0x00, 0x03, 0x75, 0x05, 0x13};
        uint8_t initWrite4Cmd[] = {0x72};
        uint8_t initWrite4Buffer[] = {0x00, 0xF1};
        uint8_t initWrite5Cmd[] = {0x27};
        uint8_t initWrite5Buffer[] = {0x00, 0x01, 0x00};

        ecuObj.SetReqEcuPacket(initWrite1Cmd, sizeof(initWrite1Cmd),
                             initWrite1Buffer, sizeof(initWrite1Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin-2 Init-Write-1", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(initWrite2Cmd, sizeof(initWrite2Cmd),
                             initWrite2Buffer, sizeof(initWrite2Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin-2 Init-Write-2", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(initWrite3Cmd, sizeof(initWrite3Cmd),
                             initWrite3Buffer, sizeof(initWrite3Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin-2 Init-Write-3", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(initWrite4Cmd, sizeof(initWrite4Cmd),
                             initWrite4Buffer, sizeof(initWrite4Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin-2 Init-Write-4", LOG_DGB_NAME);
            return false;
        }

        nrf_delay_ms(1000);

        ecuObj.SetReqEcuPacket(initWrite5Cmd, sizeof(initWrite5Cmd),
                             initWrite5Buffer, sizeof(initWrite5Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin-2 Init-Write-5", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool CheckStateWriteKeihin() {
        bool ret;
        uint8_t srcCmd[] = {0x7E};
        uint8_t srcBuffer[] = {0x01, 0x01, 0x00};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Check State Write", LOG_DGB_NAME);
            return false;
        }

        return true;   
    }

    static bool CheckStateWriteKeihin(uint8_t compareFlag) {
        bool ret;
        uint8_t srcCmd[] = {0x7E};
        uint8_t srcBuffer[] = {0x01, 0x01, 0x00};
        uint8_t flag[10];
        uint8_t dataLen;

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Check State Write-1", LOG_DGB_NAME);
            return false;
        }
        ecuObj.GetEcuPacketResData(&flag[0], &dataLen, 1);
        if (compareFlag == flag[0]) {
            return true;
        }

        NRF_LOG_INFO("[%s]: ERROR: Keihin Check State Write-2", LOG_DGB_NAME);
        return false;
    }

    static bool KeihinCheckEraseFlash(void) {
        bool ret;
        uint8_t eraseFlag[10];
        uint8_t dataLen;

        uint8_t erase1Cmd[] = {0x7E};
        uint8_t erase1Buffer[] = {0x01, 0x02};
        uint8_t erase2Cmd[] = {0x7E};
        uint8_t erase2Buffer[] = {0x01, 0x03, 0x00, 0x00};
        uint8_t erase3Cmd[] = {0x7E};
        uint8_t erase3Buffer[] = {0x01, 0x0B, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF};
        uint8_t erase4Cmd[] = {0x7E};
        uint8_t erase4Buffer[] = {0x01, 0x0E, 0x01, 0x90};
        uint8_t erase5Cmd[] = {0x7E};
        uint8_t erase5Buffer[] = {0x01, 0x04, 0xFF};
        uint8_t erase6Cmd[] = {0x7E};
        uint8_t erase6Buffer[] = {0x01, 0x05};

        if (!CheckStateWriteKeihin()) {
            return false;
        }
        nrf_delay_ms(11000);

        ecuObj.SetReqEcuPacket(erase1Cmd, sizeof(erase1Cmd),
                             erase1Buffer, sizeof(erase1Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Erase-1", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(erase2Cmd, sizeof(erase2Cmd),
                             erase2Buffer, sizeof(erase2Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Erase-2", LOG_DGB_NAME);
            return false;
        }
        if (!CheckStateWriteKeihin()) {
            return false;
        }

        ecuObj.SetReqEcuPacket(erase3Cmd, sizeof(erase3Cmd),
                             erase3Buffer, sizeof(erase3Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Erase-3", LOG_DGB_NAME);
            return false;
        }
        if (!CheckStateWriteKeihin()) {
            return false;
        }

        ecuObj.SetReqEcuPacket(erase4Cmd, sizeof(erase4Cmd),
                             erase4Buffer, sizeof(erase4Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Erase-4", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(40);

        ecuObj.SetReqEcuPacket(erase5Cmd, sizeof(erase5Cmd),
                               erase5Buffer, sizeof(erase5Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        ecuObj.GetEcuPacketResData(&eraseFlag[0], &dataLen, 1);
        if (!ret || eraseFlag[0] != 0x00) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Erase-5", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(2000);

        do {
            ecuObj.SetReqEcuPacket(erase6Cmd, sizeof(erase6Cmd),
                                   erase6Buffer, sizeof(erase6Buffer));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            ecuObj.GetEcuPacketResData(&eraseFlag[0], &dataLen, 1);
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Keihin Erase-6.0", LOG_DGB_NAME);
                return false;
            }
        } while (eraseFlag[0] != 0x00);

        ret = CheckStateWriteKeihin(0x30);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Erase-6.1", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool UpdateKeihinFlash(uint8_t *buffer, uint8_t bufLen) {
        uint8_t srcCmd[] = {0x7E};
        uint8_t tmpByte[20]; 
        uint8_t dataLen;
        int retryCnt = 5;

        for (int i = 0; i < retryCnt; i++) {
            ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), buffer, bufLen);
            ecuObj.SendRequestToEcu();
            bool ret = ecuObj.ReceiveRequestFromEcu();
            ecuObj.GetEcuPacketResData(tmpByte, &dataLen, 1);
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Keihin Flash", LOG_DGB_NAME);
                return false;
            }

            if ((dataLen + 3) != 5) {
                NRF_LOG_INFO("[%s]: RETRY: Keihin Flash", LOG_DGB_NAME);
                continue;

            } else {
                return true;
            }
        }

        return false;
    }

    static bool UpdateKeihinFlash64Ack(void) {
        bool ret;
        uint8_t srcCmd[] = {0x7E};
        uint8_t srcBuffer[] = {0x01, 0x07};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Flash-64 Ack", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool CheckFinishKeihinUpdate(void) {
        bool ret;
        uint8_t finishFlag[10];
        uint8_t dataLen;
        uint8_t finish1Cmd[] = {0x7E};
        uint8_t finish1Buffer[] = {0x01, 0x08};
        uint8_t finish2Cmd[] = {0x7E};
        uint8_t finish2Buffer[] = {0x01, 0x09};
        uint8_t finish3Cmd[] = {0x7E};
        uint8_t finish3Buffer[] = {0x01, 0x0A};
        uint8_t finish4Cmd[] = {0x7E};
        uint8_t finish4Buffer[] = {0x01, 0x0C};
        uint8_t finish5Cmd[] = {0x7E};
        uint8_t finish5Buffer[] = {0x01, 0x0D};

        ecuObj.SetReqEcuPacket(finish1Cmd, sizeof(finish1Cmd),
                             finish1Buffer, sizeof(finish1Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Finish-1", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(500);
        if (!CheckStateWriteKeihin()) {
            return false;
        }
    
        ecuObj.SetReqEcuPacket(finish2Cmd, sizeof(finish2Cmd),
                             finish2Buffer, sizeof(finish2Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Finish-2", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(500);
        if (!CheckStateWriteKeihin()) {
            return false;
        }

        ecuObj.SetReqEcuPacket(finish3Cmd, sizeof(finish3Cmd),
                             finish3Buffer, sizeof(finish3Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Finish-3", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(500);
        if (!CheckStateWriteKeihin()) {
            return false;
        }

        ecuObj.SetReqEcuPacket(finish4Cmd, sizeof(finish4Cmd),
                             finish4Buffer, sizeof(finish4Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Finish-4.0", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(500);
        ret = CheckStateWriteKeihin(0x0F);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Finish-4.1", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(finish5Cmd, sizeof(finish5Cmd),
                             finish5Buffer, sizeof(finish5Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        ecuObj.GetEcuPacketResData(&finishFlag[0], &dataLen, 1);
        if (!ret && (finishFlag[0] != 0x0F)) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Finish-5", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool ShindengenCheckInitWrite(void) {
        bool ret;
        uint8_t initWrite1Cmd[] = {0x7B};
        uint8_t initWrite1Buffer[] = {0x02, 0x02, 0x50, 0x19, 0x76};
        uint8_t initWrite2Cmd[] = {0x7B};
        uint8_t initWrite2Buffer[] = {0x02, 0x03, 0x25, 0x37, 0x86};
        uint8_t initWrite3Cmd[] = {0x7B};
        uint8_t initWrite3Buffer[] = {0x02, 0x01, 0x01};

        ecuObj.SetReqEcuPacket(initWrite1Cmd, sizeof(initWrite1Cmd),
                             initWrite1Buffer, sizeof(initWrite1Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen Init-Write-1", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(initWrite2Cmd, sizeof(initWrite1Cmd),
                             initWrite2Buffer, sizeof(initWrite2Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen Init-Write-2", LOG_DGB_NAME);
            return false;
        }

        for (int i = 0; i < 5; i++) {
            ecuObj.SetReqEcuPacket(initWrite3Cmd, sizeof(initWrite3Cmd),
                                   initWrite3Buffer, sizeof(initWrite3Buffer));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Shindengen Init-Write-3", LOG_DGB_NAME);
                return false;
            }
            nrf_delay_ms(2000);
        }

        return true;
    }

    static bool ReadEcuSeedkey(uint8_t *data) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcCmd[] = {0x7B};
        uint8_t srcBuffer[] = {0x02, 0x04};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd),
                             srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen Seedkey Get", LOG_DGB_NAME);
            return false;
        }
        ecuObj.GetEcuPacketResData(data, &dataLen, 2);
        return true;
    }

    static bool ShindengenCheckEraseFlash(const uint8_t *srcSeedkey, uint8_t *ecuType) {
        bool ret;
        uint8_t flag;
        uint8_t dataLen;

        uint8_t seedkeyCmd[] = {0x7B};
        uint8_t seedkeyBuf[] = {0x02, 0x05, srcSeedkey[0], srcSeedkey[1]};
        uint8_t erase1Cmd[] = {0x7C};
        uint8_t erase1Buf[] = {0x02, 0x0B, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF};
        uint8_t erase2Cmd[] = {0x7C};
        uint8_t erase2Buf[] = {0x02, 0x0E, 0x01, 0x00};
        uint8_t erase3Cmd[] = {0x7C};
        uint8_t erase3Buf[] = {0x02, 0x04, 0xFF};
        uint8_t erase4Cmd[] = {0x7C};
        uint8_t erase4Buf[] = {0x02, 0x05};
        uint8_t erase5Cmd[] = {0x7C};
        uint8_t erase5Buf[] = {0x02, 0x01, 0x01};

        ecuObj.SetReqEcuPacket(seedkeyCmd, sizeof(seedkeyCmd),
                               seedkeyBuf, sizeof(seedkeyBuf));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen Seedkey Set", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(500);

        ecuObj.SetReqEcuPacket(erase1Cmd, sizeof(erase1Cmd),
                               erase1Buf, sizeof(erase1Buf));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen Erase-1", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(erase2Cmd, sizeof(erase2Cmd),
                             erase2Buf, sizeof(erase2Buf));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen Erase-2", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(erase3Cmd, sizeof(erase3Cmd),
                             erase3Buf, sizeof(erase3Buf));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen Erase-3", LOG_DGB_NAME);
            return false;
        }

        do {
            ecuObj.SetReqEcuPacket(erase4Cmd, sizeof(erase4Cmd),
                                   erase4Buf, sizeof(erase4Buf));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Shindengen Erase-4", LOG_DGB_NAME);
                return false;
            }
            ecuObj.GetEcuPacketResData(&flag, &dataLen, 1);
            nrf_delay_ms(200);
        } while (flag != 0x00);


        ecuObj.SetReqEcuPacket(erase5Cmd, sizeof(erase5Cmd),
                               erase5Buf, sizeof(erase5Buf));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen Erase-5", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(1000);

        return true;
    }

    static bool UpdateShindengenFlash(uint8_t *buffer, uint8_t bufLen) {
        bool ret;
        uint8_t srcCmd[] = {0x7C};
        uint8_t tmpByte[5];
        uint8_t dataLen;

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), buffer, bufLen);
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        ecuObj.GetEcuPacketResData(tmpByte, &dataLen, 0);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen Flash-1", LOG_DGB_NAME);
            return false;
        }

        if (tmpByte[0] != 0x06 || (tmpByte[1] != 0x00) && (tmpByte[1] != 0x02)) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen Flash-1", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool CheckFinishShindengenUpdate(void) {
        bool ret;
        int retried = 25;
        uint8_t finishFlag[10];
        uint8_t dataLen;
        uint8_t finish1Cmd[] = {0x7C};
        uint8_t finish1Buf[] = {0x02, 0x08};
        uint8_t finish2Cmd[] = {0x7C};
        uint8_t finish2Buf[] = {0x02, 0x09};
        uint8_t finish3Cmd[] = {0x7C};
        uint8_t finish3Buf[] = {0x02, 0x0A};
        uint8_t finish4Cmd[] = {0x7C};
        uint8_t finish4Buf[] = {0x02, 0x0C};
        uint8_t finish5Cmd[] = {0x7C};
        uint8_t finish5Buf[] = {0x02, 0x01, 0x00};

        for (int i = 0; i < retried; i++) {
            nrf_delay_ms(20);

            ecuObj.SetReqEcuPacket(finish1Cmd, sizeof(finish1Cmd), 
                                   finish1Buf, sizeof(finish1Buf));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Shindengen Finish Write-1", LOG_DGB_NAME);
                return false;
            }
            ecuObj.GetEcuPacketResData(&finishFlag[0], &dataLen, 1);
            if (finishFlag[0] == 0x00) {
                break;
            }
        }

        for (int i = 0; i < retried; i++) {
            nrf_delay_ms(20);

            ecuObj.SetReqEcuPacket(finish2Cmd, sizeof(finish2Cmd), 
                                 finish2Buf, sizeof(finish2Buf));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Shindengen Finish Write-2", LOG_DGB_NAME);
                return false;
            }
            ecuObj.GetEcuPacketResData(&finishFlag[0], &dataLen, 1);
            if (finishFlag[0] == 0x00) {
                break;
            }   
        }

        for (int i = 0; i < retried; i++) {
            nrf_delay_ms(20);

            ecuObj.SetReqEcuPacket(finish3Cmd, sizeof(finish3Cmd), 
                                 finish3Buf, sizeof(finish3Buf));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Shindengen Finish Write-3", LOG_DGB_NAME);
                return false;
            }
            ecuObj.GetEcuPacketResData(&finishFlag[0], &dataLen, 1);
            if (finishFlag[0] == 0x00) {
                break;
            }
        } 

        for (int i = 0; i < retried; i++) {
            nrf_delay_ms(20);

            ecuObj.SetReqEcuPacket(finish4Cmd, sizeof(finish4Cmd), 
                                 finish4Buf, sizeof(finish4Buf));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Shindengen Finish Write-4", LOG_DGB_NAME);
                return false;
            }
            ecuObj.GetEcuPacketResData(&finishFlag[0], &dataLen, 1);
            if (finishFlag[0] == 0x00) {
                break;
            }
        }

        for (int i = 0; i < 10; i++) {
            ecuObj.SetReqEcuPacket(finish5Cmd, sizeof(finish5Cmd), 
                                 finish5Buf, sizeof(finish5Buf));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Shindengen Finish Write-5", LOG_DGB_NAME);
                return false;
            }
        }

        return true;
    }

    static bool ShindengenEepromCheckInitWrite(void) {
        bool ret;
        uint8_t initWrite1Cmd[] = {0x72};
        uint8_t initWrite1Buffer[] = {0x00, 0xF0};
        uint8_t initWrite2Cmd[] = {0x7B};
        uint8_t initWrite2Buffer[] = {0x02, 0x01, 0x03};
        uint8_t initWrite3Cmd[] = {0x91, 0x91};
        uint8_t initWrite3Buffer[] = {0xDF, 0x9E, 0x8D, 0x9A, 0x86, 0x90, 0x8A, 0x8C, 0x9B};
        uint8_t initWrite4Cmd[] = {0x91, 0x91};
        uint8_t initWrite4Buffer[] = {0xDF, 0x92, 0x9E, 0x86, 0x96, 0x8B, 0x8D, 0x86, 0xC0};

        ecuObj.SetReqEcuPacket(initWrite1Cmd, sizeof(initWrite1Cmd),
                             initWrite1Buffer, sizeof(initWrite1Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen EEPROM Init-Write-1", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(initWrite2Cmd, sizeof(initWrite2Cmd),
                               initWrite2Buffer, sizeof(initWrite2Buffer));
        ecuObj.SendRequestToEcu();        
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen EEPROM Init-Write-2", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(initWrite3Cmd, sizeof(initWrite3Cmd),
                               initWrite3Buffer, sizeof(initWrite3Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen EEPROM Init-Write-3", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(initWrite4Cmd, sizeof(initWrite4Cmd),
                               initWrite4Buffer, sizeof(initWrite4Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen EEPROM Init-Write-4", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool UpdateShindengenEeprom(uint16_t addr, const uint8_t *buffer) {
        bool ret;
        uint8_t srcCmd[] = {0x91, 0x91};
        uint8_t srcBuffer[5] = {0x41, 0x00, 0x00, 0x00, 0x00};

        srcBuffer[1] = addr & 0xFF;
        srcBuffer[2] = (addr >> 8) & 0xFF;
        srcBuffer[3] = buffer[0];
        srcBuffer[4] = buffer[1];

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen WriteEEPROM", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool ShindengenEndWriteEeprom(void) {
        bool ret;
        uint8_t srcCmd1[] = {0x72};
        uint8_t srcBuffer1[] = {0x72, 0x00, 0x00, 0x01};
        uint8_t srcCmd2[] = {0x72};
        uint8_t srcBuffer2[] = {0x60, 0x03};
        uint8_t flag[10];
        uint8_t dataLen;

        ecuObj.SetReqEcuPacket(srcCmd1, sizeof(srcCmd1), srcBuffer1, sizeof(srcBuffer1));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen End-Write-1", LOG_DGB_NAME);
            return false;
        }

        do {
            ecuObj.SetReqEcuPacket(srcCmd2, sizeof(srcCmd2), srcBuffer2, sizeof(srcBuffer2));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Shindengen End-Write-2", LOG_DGB_NAME);
                return false;
            }
            ecuObj.GetEcuPacketResData(flag, &dataLen, 1);
            nrf_delay_ms(50);

        } while (flag[0] != 0x00);

        return true;
    }

    static bool CheckShindengenTurnOff(void) {
        bool ret;
        uint8_t srcCmd[] = {0x72};
        uint8_t srcBuffer[] = {0x00, 0xF0};

        ecuObj.InitKlineBus();
        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();

        return !ret;  
    }

    static bool EnterShindengenEepromMode(void) {
        uint8_t wakeCmd[] = {0xFE};
        uint8_t wakeBuffer[] = {0x72};

        ecuObj.InitKlineBus();
        ecuObj.SetReqEcuPacket(wakeCmd, sizeof(wakeCmd), wakeBuffer, sizeof(wakeBuffer));
        ecuObj.SendRequestToEcu();
        ecuObj.ReceiveRequestFromEcu();

        if (!ShindengenEepromCheckInitWrite()) {
            return false;
        }

        return true;
    }

    static bool ReadShindengenEeprom(uint16_t addr, uint8_t *buffer) {
        bool ret;
        uint8_t srcCmd[] = {0x91, 0x91};
        uint8_t srcBuffer[3] = {0x40, 0x00, 0x00};
        uint8_t dataLen = 0;

        srcBuffer[1] = addr & 0xFF;
        srcBuffer[2] = (addr >> 8) & 0xFF;

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Shindengen ReadEEPROM", LOG_DGB_NAME);
            return false;
        }
        ecuObj.GetEcuPacketResData(buffer, &dataLen, 1);

        return true;     
    }

    static bool CheckKeihinTurnOff(void) {
        bool ret;
        uint8_t srcCmd[] = {0xFE};
        uint8_t srcBuffer[] = {0x72};

        ecuObj.InitKlineBus();
        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();

        return !ret;  
    }

    static bool KeihinEepromCheckInitWrite(void) {
        bool ret;
        uint8_t secure1Cmd[] = {0x27};
        uint8_t secure1Buffer[] = {0xE0, 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x48, 0x6F};
        uint8_t secure2Cmd[] = {0x27};
        uint8_t secure2Buffer[] = {0xE0, 0x77, 0x41, 0x72, 0x65, 0x59, 0x6f, 0x75};

        ecuObj.SetReqEcuPacket(secure1Cmd, sizeof(secure1Cmd),
                             secure1Buffer, sizeof(secure1Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Check Secure-1", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(secure2Cmd, sizeof(secure2Cmd),
                             secure2Buffer, sizeof(secure2Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin Check Secure-2", LOG_DGB_NAME);
            return false;
        }

        return true;        
    }

    static bool CheckKeihinTurnOn(void) {
        bool ret;
        uint8_t wakeCmd[] = {0xFE};
        uint8_t wakeBuffer[] = {0x72};
        uint8_t stateOkCmd[] = {0x72};
        uint8_t stateOkBuffer[] = {0x00, 0xF0};

        ecuObj.InitKlineBus();
        ecuObj.SetReqEcuPacket(wakeCmd, sizeof(wakeCmd), wakeBuffer, sizeof(wakeBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            return false;
        }

        ecuObj.SetReqEcuPacket(stateOkCmd, sizeof(stateOkCmd),
                             stateOkBuffer, sizeof(stateOkBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            return false;
        }

        return true;
    }

    static bool EnterKeihinEepromMode(void) {
        if (!CheckKeihinTurnOn()) { 
            return false;
        }

        if (!KeihinEepromCheckInitWrite()) {
            return false;
        }

        nrf_delay_ms(500);

        return true;
    }

    static bool ReadKeihinEeprom(uint16_t addr, uint8_t *buffer) {
        bool ret;
        uint8_t srcCmd[] = {0x82, 0x82, 0x10};
        uint8_t srcBuffer[1];
        uint8_t dataLen = 0;

        srcBuffer[0] = addr & 0xFF;
        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin ReadEEPROM", LOG_DGB_NAME);
            return false;
        }
        ecuObj.GetEcuPacketResData(buffer, &dataLen, 0);

        return true;     
    }

    static bool UpdateKeihinEeprom(uint16_t addr, const uint8_t *buffer) {
        bool ret;
        uint8_t srcCmd[] = {0x82, 0x82, 0x14};
        uint8_t srcBuffer[3] = {0x00, 0x00, 0x00};

        srcBuffer[0] = addr & 0xFF;
        srcBuffer[1] = buffer[0];
        srcBuffer[2] = buffer[1];

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Keihin WriteEEPROM", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool FormatKeihinEeprom(uint8_t formatType) {
        uint8_t srcCmdFF[] = {0x82, 0x82, 0x18};
        uint8_t srcCmd00[] = {0x82, 0x82, 0x19};
        bool ret;

        if (formatType) {
            ecuObj.SetReqEcuPacket(srcCmdFF, sizeof(srcCmdFF), NULL, 0);
        } else {
            ecuObj.SetReqEcuPacket(srcCmd00, sizeof(srcCmd00), NULL, 0);
        }

        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Format Keihin EEPROM", LOG_DGB_NAME);
            return false;
        }

        return true;
    }
}

namespace HondaKeihinUpdate {
    static BleFlashState_t bleFlashState;

    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_HONDA_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_ECU_HONDA_KEIHIN_INIT_WRITE_REQ:
            cmd = &hondaKeihinInitWriteCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin Init Write Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_KEIHIN_ERASE_REQ:
            cmd = &hondaKeihinEraseCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin Erase-Only Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_KEIHIN_WRITE_REQ:
            cmd = &hondaKeihinWriteFlashCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin Write Flash Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_KEIHIN_WRITE_FINISH_REQ:
            cmd = &hondaKeihinWriteFinishCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin Write Finish Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_KEIHIN_WRITE_EEPROM_REQ:
           cmd = &hondaKeihinWriteEepromCmd;
           NRF_LOG_INFO("[%s]: INFO: Keihin Write EEPROM Request", LOG_DGB_NAME);
           break;

        case CMD_ECU_HONDA_KEIHIN_EEPROM_OFF_REQ:
            cmd = &hondaKeihinOffCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin OFF Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_KEIHIN_EEPROM_ON_REQ:
            cmd = &hondaKeihinOnCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin ON Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_KEIHIN_READ_EEPROM_REQ:
            cmd = &hondaKeihinReadEepromCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin Read EEPROM Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_KEIHIN_EEEPROM_FORMAT_REQ:
            cmd = &hondaKeihinFormatEepromCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin fill EEPROM Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    static void InitFlashParams(const EsyPro::CommPacket_t *commReqPacket) {
        memcpy(&flashSize, &commReqPacket->buffer[0], 4);
        memcpy(&flashOffset, &commReqPacket->buffer[4], 4);
        flashOffset = flashOffset / 16;
        pageSize = 128;
        startPageOffset = flashOffset;
        endPageOffset = flashOffset + pageSize / 16;
        spiFlashAddr = 0;
    }

    static bool InitWriteEcu(void) {        
        if (!HondaCmd::KeihinCheckInitWrite()) {
            return false;
        }
        nrf_delay_ms(500);
        if (!HondaCmd::KeihinCheckEraseFlash()) {
            return false;
        }

        return true;
    }

    static bool InitWriteEcu2(void) {
        if (!HondaCmd::KeihinCheckInitWrite2()) {
            return false;
        }
        nrf_delay_ms(500);
        if (!HondaCmd::KeihinCheckEraseFlash()) {
            return false;
        }

        return true;
    }

    static void CalcUpdateFlashChecksum(const uint8_t *buffer, uint8_t bufLen,
                                        uint8_t *nChecksum, uint8_t *eChecksum) {
        uint32_t sum = 0;

        for (int i = 0; i < bufLen; i++) {
            sum += buffer[i];
        }
        *nChecksum = 0xFF - ((sum - 1) >> 8);
        *eChecksum = ((sum ^ 0xFF) + 1) & 0x000000FF;
    }

    static bool UpdateFlash(const uint8_t *data) {
        uint8_t buffer[150];
        uint8_t nChecksum, eChecksum;

        buffer[0] = 0x01;
        buffer[1] = 0x06;
        buffer[2] = (startPageOffset >> 8) & 0xFF;
        buffer[3] = startPageOffset & 0xFF;
        memcpy(&buffer[4], data, pageSize);
        buffer[4 + pageSize] = (endPageOffset >> 8) & 0xFF;
        buffer[5 + pageSize] = endPageOffset & 0xFF;
        CalcUpdateFlashChecksum(&buffer[2], pageSize + 4, &nChecksum, &eChecksum);
        buffer[6 + pageSize] = nChecksum;
        buffer[7 + pageSize] = eChecksum;

        if (!HondaCmd::UpdateKeihinFlash(buffer, (pageSize + 8))) {
            return false;
        }

        if ((pageSize == 64) && ((endPageOffset % 0x08) == 0)) {
            if (!HondaCmd::UpdateKeihinFlash64Ack()) {
                return false;
            }
            nrf_delay_ms(200);
        }

        return true;
    }

    static bool WriteEcuFlash(const uint8_t *data, uint16_t dataLen) {
        for (int i = 0; i < (dataLen / pageSize); i++) {
            bool ret;

            endPageOffset = (endPageOffset == (flashSize / 16 + flashOffset)) 
                                                ? 0 : endPageOffset;
            ret = UpdateFlash(&data[i * pageSize]);
            if (!ret) {
                nrf_delay_ms(1);
                if (startPageOffset == flashOffset) {
                    pageSize = 64;
                    endPageOffset = startPageOffset + pageSize / 16;
                    ret = UpdateFlash(&data[i * pageSize]);
                    if (!ret) {
                        return false;
                    }
                } else {
                    return false;
                }
            }

            startPageOffset = endPageOffset;
            endPageOffset = startPageOffset + pageSize / 16;
        }

        return true;
    }

    static bool WriteEcuFlash(CommunicationCmd_t *resCmd, BleFlashState_t *state) {
        bool ret;
        uint8_t spiFlashData[150];
        SpiFlash::Spi0Obj *spiFlash = SpiFlash::Spi0Obj::GetInstance();

        ret = spiFlash->ReadFromFlash(spiFlashAddr, spiFlashData, pageSize);
        if (!ret) {
            return false;
        }

        endPageOffset = (endPageOffset == (flashSize / 16 + flashOffset)) 
                                            ? 0 : endPageOffset;
        ret = UpdateFlash((const uint8_t *)&spiFlashData);
        if (!ret) {
            nrf_delay_ms(1);
            if (startPageOffset == flashOffset) {
                pageSize = 64;
                endPageOffset = startPageOffset + pageSize / 16;
                ret = UpdateFlash((const uint8_t *)&spiFlashData);
                if (!ret) {
                    return false;
                }
            } else {
                return false;
            }
        }

        startPageOffset = endPageOffset;
        endPageOffset = startPageOffset + pageSize / 16;
        spiFlashAddr += pageSize;
        *resCmd = (pageSize == 64) ? CMD_ECU_HONDA_KEIHIN_WRITE_64B_RES 
                                    : CMD_ECU_HONDA_KEIHIN_WRITE_RES;
        if (flashSize <= spiFlashAddr) {
            *state = END_FLASHING;
        }

        return true;
    }

    static bool BleUpdateFlashProgress(InitWriteCommand *cmd, BleFlashState_t *state,
                                       CommunicationCmd_t *resCmd) {
        bool ret;

        switch (*state) {
        case INIT_WRITE_ECU:
            ret = InitWriteEcu();
            if (!ret) {
                return false;
            }
            *state = FLASHING_ECU;
            *resCmd = EsyPro::CMD_ECU_HONDA_KEIHIN_INIT_WRITE_RES;
            cmd->SetCommandRepeatState(true);
            NRF_LOG_INFO("[%s]: INFO: BLE Keihin Init Write", LOG_DGB_NAME);
            break;

        case ERASE_ECU:
            if ((HondaCmd::KeihinCheckEraseFlash() == false) && (InitWriteEcu2() == false)) {
                return false;
            }
            *state = FLASHING_ECU;
            *resCmd = EsyPro::CMD_ECU_HONDA_KEIHIN_INIT_WRITE_RES;
            cmd->SetCommandRepeatState(true);
            NRF_LOG_INFO("[%s]: INFO: BLE Keihin Erase", LOG_DGB_NAME);
            break;

        case FLASHING_ECU:
            ret = WriteEcuFlash(resCmd, state);
            if (!ret) {
                return false;
            }
            cmd->SetCommandRepeatState(true);
            NRF_LOG_INFO("[%s]: INFO: BLE Keihin Flashing", LOG_DGB_NAME);
            break;

        case END_FLASHING:
            nrf_delay_ms(50);
            ret = HondaCmd::CheckFinishKeihinUpdate();
            if (!ret) {
                return false;
            }
            *resCmd = CMD_ECU_HONDA_KEIHIN_WRITE_FINISH_RES;
            cmd->SetCommandRepeatState(false);
            NRF_LOG_INFO("[%s]: INFO: BLE Keihin End", LOG_DGB_NAME);
            break;
        }

        return true;
    }

    void InitWriteCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                   const EsyPro::CommPacket_t *commReqPacket,
                                   EsyPro::CommunicationType_t commType) {
        bool ret;

        if (commType == BLE_COMM_TYPE) {
            if (!this->GetCommandRepeatState()) {
                HondaCmd::WakeUpHondaPgmfi();
                bleFlashState = INIT_WRITE_ECU;
                InitFlashParams(commReqPacket);
            }

            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->bufLen = 0;
            ret = BleUpdateFlashProgress(this, &bleFlashState,
                                        (CommunicationCmd_t *)(&commResPacket->cmd));
            if (!ret) {
                commResPacket->cmd = CMD_IGNORE_RES;
                this->SetCommandRepeatState(false);
            }

        } else {
            HondaCmd::WakeUpHondaPgmfi();
            InitFlashParams(commReqPacket);
            ret = InitWriteEcu();
            if (ret) {
                commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_INIT_WRITE_RES;
                commResPacket->bufLen = 0;
            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }

            this->SetCommandRepeatState(false);
        }        
    }

    void EraseCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                               const EsyPro::CommPacket_t *commReqPacket,
                               EsyPro::CommunicationType_t commType) {
        bool ret;

        if (commType == BLE_COMM_TYPE) {
            if (!this->GetCommandRepeatState()) {
                HondaCmd::WakeUpHondaPgmfi();
                bleFlashState = ERASE_ECU;
                InitFlashParams(commReqPacket);
            }
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->bufLen = 0;
            ret = BleUpdateFlashProgress(reinterpret_cast<InitWriteCommand *>(this),
                            &bleFlashState, (CommunicationCmd_t *)(&commResPacket->cmd));
            if (!ret) {
                commResPacket->cmd = CMD_IGNORE_RES;
                this->SetCommandRepeatState(false);
            }

        } else {
            HondaCmd::WakeUpHondaPgmfi();
            InitFlashParams(commReqPacket);

            if (HondaCmd::KeihinCheckEraseFlash() || InitWriteEcu2()) {
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_INIT_WRITE_RES;
                commResPacket->bufLen = 0;
            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }

            this->SetCommandRepeatState(false);
        }
    }

    void WriteFlashCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                    const EsyPro::CommPacket_t *commReqPacket,
                                    EsyPro::CommunicationType_t commType) {
        bool ret;

        ret = WriteEcuFlash(commReqPacket->buffer, commReqPacket->bufLen);
        if (ret) {
            commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_WRITE_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void WriteFinishCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                     const EsyPro::CommPacket_t *commReqPacket,
                                     EsyPro::CommunicationType_t commType) {
        bool ret;

        ret = HondaCmd::CheckFinishKeihinUpdate();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_WRITE_FINISH_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void WriteEepromCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                    const EsyPro::CommPacket_t *commReqPacket,
                                    EsyPro::CommunicationType_t commType) {
       bool ret;
  
       for (int i = 0; i < commReqPacket->bufLen / 2; i++) {
           ret = HondaCmd::UpdateKeihinEeprom(eepromAddr, &commReqPacket->buffer[i * 2]);
           if (!ret) {
               commResPacket->cmd = CMD_IGNORE_RES;
               this->SetCommandRepeatState(false);
               return;
           }
           eepromAddr++;
        }

        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_WRITE_EEPROM_RES;
        commResPacket->bufLen = 0;
  
        this->SetCommandRepeatState(false);
    }

    void TurnOffCommand::Execute(CommPacket_t *commResPacket,
                                 const CommPacket_t *commReqPacket,
                                 CommunicationType_t commType) {
        bool ret;

        eepromAddr = 0;

        ret = HondaCmd::CheckKeihinTurnOff();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_EEPROM_OFF_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x01;
        } else {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_EEPROM_OFF_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x00;
        }
        this->SetCommandRepeatState(false);
    }

    void TurnOnCommand::Execute(CommPacket_t *commResPacket,
                                const CommPacket_t *commReqPacket,
                                CommunicationType_t commType) {
        bool ret;

        eepromAddr = 0x0000;

        ret = HondaCmd::EnterKeihinEepromMode();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_EEPROM_ON_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x01;           
        } else {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_EEPROM_ON_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x00;
        }
        this->SetCommandRepeatState(false);
    }

    void ReadEepromCommand::Execute(CommPacket_t *commResPacket,
                                    const CommPacket_t *commReqPacket,
                                    CommunicationType_t commType) {
        bool ret;
  
        ret = HondaCmd::ReadKeihinEeprom(eepromAddr, &commResPacket->buffer[0]);
        if (!ret) {
            commResPacket->cmd = CMD_IGNORE_RES;
            this->SetCommandRepeatState(false);
            return;
        }
  
        eepromAddr++;
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_READ_EEPROM_RES;
        commResPacket->bufLen = 2;
  
        this->SetCommandRepeatState(false);
    }

    void FormatEepromCommand::Execute(CommPacket_t *commResPacket,
                                     const CommPacket_t *commReqPacket,
                                     CommunicationType_t commType) {
        bool ret = HondaCmd::FormatKeihinEeprom(commReqPacket->buffer[0]);
        if (!ret) {
            commResPacket->cmd = CMD_IGNORE_RES;
            this->SetCommandRepeatState(false);
            return;
        }

        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_ECU_HONDA_KEIHIN_EEEPROM_FORMAT_RES;
        commResPacket->buffer[0] = 0x01;
        commResPacket->bufLen = 1;

        this->SetCommandRepeatState(false);
    }
}

namespace HondaShindengenUpdate {
    static BleFlashState_t bleFlashState;

    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_HONDA_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_ECU_HONDA_SHINDENGEN_INIT_WRITE_REQ:
            cmd = &hondaShindengenInitWriteCmd;
            NRF_LOG_INFO("[%s]: INFO: Shindengen Init Write Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_SHINDENGEN_SEEDKEY_REQ:
            cmd = &hondaShindengenSeedkeyCmd;
            NRF_LOG_INFO("[%s]: INFO: Shindengen Seedkey Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_SHINDENGEN_ERASE_REQ:
            cmd = &hondaShindengenEraseCmd;
            NRF_LOG_INFO("[%s]: INFO: Shindengen Erase Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_SHINDENGEN_WRITE_REQ:
            cmd = &hondaShindengenWriteFlashCmd;
            NRF_LOG_INFO("[%s]: INFO: Shindengen Write Flash Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_SHINDENGEN_WRITE_FINISH_REQ:
            cmd = &hondaShindengenWriteFinishCmd;
            NRF_LOG_INFO("[%s]: INFO: Shindengen Write Finish Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_SHINDENGEN_WRITE_EEPROM_REQ:
           cmd = &hondaShindengenWriteEepromCmd;
           NRF_LOG_INFO("[%s]: INFO: Shindengen Write EEPROM Request", LOG_DGB_NAME);
           break;

        case CMD_ECU_HONDA_SHINDENGEN_END_WRITE_EEPROM_REQ:
           cmd = &hondaShindengenEndwriteEepromCmd;
           NRF_LOG_INFO("[%s]: INFO: Shindengen Finish EEPROM Request", LOG_DGB_NAME);
           break;

        case CMD_ECU_HONDA_SHINDENGEN_EEPROM_OFF_REQ:
            cmd = &hondaShindengenOffCmd;
            NRF_LOG_INFO("[%s]: INFO: Shindengen OFF Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_SHINDENGEN_EEPROM_ON_REQ:
            cmd = &hondaShindengenOnCmd;
            NRF_LOG_INFO("[%s]: INFO: Shindengen ON Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_SHINDENGEN_READ_EEPROM_REQ:
            cmd = &hondaShindengenReadEepromCmd;
            NRF_LOG_INFO("[%s]: INFO: Shindengen Read EEPROM Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    static void CalcUpdateFlashChecksum(const uint8_t *buffer, uint8_t bufLen,
                                        uint8_t *nChecksum, uint8_t *eChecksum) {
        uint32_t sum = 0;

        for (int i = 0; i < bufLen; i++) {
            sum += buffer[i];
        }
        *nChecksum = 0xFF - ((sum - 1) >> 8);
        *eChecksum = ((sum ^ 0xFF) + 1) & 0x000000FF;
    }

    static void InitFlashParams(const EsyPro::CommPacket_t *commReqPacket,
                                uint8_t *seedkey, uint8_t *ecuType) {
        spiFlashAddr = 0;
        flashOffset = 0;
        pageSize = 128;
        startPageOffset = flashOffset;
        endPageOffset = flashOffset + pageSize / 16;
        memcpy(&flashSize, &commReqPacket->buffer[0], 4);
        seedkey[0] = commReqPacket->buffer[4];
        seedkey[1] = commReqPacket->buffer[5];
        memcpy(ecuType, &commReqPacket->buffer[8], 3);
    }

    static bool WriteEcuFlash(const uint8_t *data, uint16_t dataLen) {
        uint8_t buffer[150];
        uint8_t nChecksum, eChecksum;

        endPageOffset = (endPageOffset == (flashSize / 16 + flashOffset)) 
                                            ? 0 : endPageOffset;
        buffer[0] = 0x02;
        buffer[1] = 0x06;
        buffer[2] = (startPageOffset >> 8) & 0xFF;
        buffer[3] = startPageOffset & 0xFF;
        memcpy(&buffer[4], data, dataLen);
        buffer[4 + dataLen] = (endPageOffset >> 8) & 0xFF;
        buffer[5 + dataLen] = endPageOffset & 0xFF;
        CalcUpdateFlashChecksum(&buffer[2], dataLen + 4, &nChecksum, &eChecksum);
        buffer[6 + dataLen] = nChecksum;
        buffer[7 + dataLen] = eChecksum;

        if (!HondaCmd::UpdateShindengenFlash(buffer, (dataLen + 8))) {
            return false;
        }

        startPageOffset = endPageOffset;
        endPageOffset = startPageOffset + dataLen / 16;

        return true;
    }

    static bool WriteEcuFlash(BleFlashState_t *state) {
        uint8_t spiFlashData[136];
        uint8_t nChecksum, eChecksum;
        SpiFlash::Spi0Obj *spiFlash = SpiFlash::Spi0Obj::GetInstance();

        endPageOffset = (endPageOffset == (flashSize / 16 + flashOffset)) 
                                            ? 0 : endPageOffset;

        spiFlashData[0] = 0x02;
        spiFlashData[1] = 0x06;
        spiFlashData[2] = (startPageOffset >> 8) & 0xFF;
        spiFlashData[3] = startPageOffset & 0xFF;
        if (!spiFlash->ReadFromFlash(spiFlashAddr, &spiFlashData[4],
                                      SPI_FLASH_SHINDEGEN_DATA_SIZE)) {
            return false;
        }   

        spiFlashData[4 + SPI_FLASH_SHINDEGEN_DATA_SIZE] = (endPageOffset >> 8) & 0xFF;
        spiFlashData[5 + SPI_FLASH_SHINDEGEN_DATA_SIZE] = endPageOffset & 0xFF;
        CalcUpdateFlashChecksum(&spiFlashData[2], (SPI_FLASH_SHINDEGEN_DATA_SIZE + 4), 
                                &nChecksum, &eChecksum);
        spiFlashData[6 + SPI_FLASH_SHINDEGEN_DATA_SIZE] = nChecksum;
        spiFlashData[7 + SPI_FLASH_SHINDEGEN_DATA_SIZE] = eChecksum;


        if (!HondaCmd::UpdateShindengenFlash(spiFlashData, 
                                            (SPI_FLASH_SHINDEGEN_DATA_SIZE + 8))) {
            return false;
        }

        startPageOffset = endPageOffset;
        endPageOffset = startPageOffset + SPI_FLASH_SHINDEGEN_DATA_SIZE / 16;

        spiFlashAddr += SPI_FLASH_SHINDEGEN_DATA_SIZE;
        if (flashSize <= spiFlashAddr) {
            *state = END_FLASHING;
        }

        return true;
    }

    static bool BleUpdateFlashProgress(EraseCommand *cmd, BleFlashState_t *state, uint8_t *ecuType,
                                       uint8_t *srcSeedkey, CommunicationCmd_t *resCmd) {
        bool ret;

        switch (*state) {
        case ERASE_ECU:
            ret = HondaCmd::ShindengenCheckEraseFlash(srcSeedkey, ecuType);
            if (!ret) {
                return false;
            }
            *state = FLASHING_ECU;
            *resCmd = EsyPro::CMD_ECU_HONDA_SHINDENGEN_ERASE_RES;
            cmd->SetCommandRepeatState(true);
            NRF_LOG_INFO("[%s]: INFO: BLE Shindengen Erase", LOG_DGB_NAME);
            break;

        case FLASHING_ECU:
            ret = WriteEcuFlash(state);
            if (!ret) {
                return false;
            }
            *resCmd = EsyPro::CMD_ECU_HONDA_SHINDENGEN_WRITE_RES;
            cmd->SetCommandRepeatState(true);
            NRF_LOG_INFO("[%s]: INFO: BLE Shindengen Flashing", LOG_DGB_NAME);
            break;

        case END_FLASHING:
            nrf_delay_ms(50);
            ret = HondaCmd::CheckFinishShindengenUpdate();
            if (!ret) {
                return false;
            }
            *resCmd = CMD_ECU_HONDA_SHINDENGEN_WRITE_FINISH_RES;
            cmd->SetCommandRepeatState(false);
            NRF_LOG_INFO("[%s]: INFO: BLE Shindengen End", LOG_DGB_NAME);
            break;
        }

        return true;
    }

    void InitWriteCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                   const EsyPro::CommPacket_t *commReqPacket,
                                   EsyPro::CommunicationType_t commType) {
        bool ret;

        HondaCmd::WakeUpHondaPgmfi();
        ret = HondaCmd::ShindengenCheckInitWrite();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_INIT_WRITE_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void SeedkeyCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                 const EsyPro::CommPacket_t *commReqPacket,
                                 EsyPro::CommunicationType_t commType) {
        bool ret;

        ret = HondaCmd::ReadEcuSeedkey(commResPacket->buffer);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_SEEDKEY_RES;
            commResPacket->bufLen = 2;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void EraseCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                               const EsyPro::CommPacket_t *commReqPacket,
                               EsyPro::CommunicationType_t commType) {
        bool ret;
        uint8_t seedkey[2];
        uint8_t ecuType[3];

        if (commType == BLE_COMM_TYPE) {
            if (!this->GetCommandRepeatState()) {
                bleFlashState = ERASE_ECU;
                InitFlashParams(commReqPacket, seedkey, ecuType);
            }
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->bufLen = 0;
            ret = BleUpdateFlashProgress(this, &bleFlashState, ecuType,
                                         seedkey, (CommunicationCmd_t *)&commResPacket->cmd);
            if (!ret) {
                commResPacket->cmd = CMD_IGNORE_RES;
                this->SetCommandRepeatState(false);
            }

        } else {
            InitFlashParams(commReqPacket, seedkey, ecuType);
            ret = HondaCmd::ShindengenCheckEraseFlash(seedkey, ecuType);
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_ERASE_RES;
                commResPacket->bufLen = 0;
            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }

            this->SetCommandRepeatState(false);
        }
    }

    void WriteFlashCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                    const EsyPro::CommPacket_t *commReqPacket,
                                    EsyPro::CommunicationType_t commType) {
        bool ret;

        ret = WriteEcuFlash(commReqPacket->buffer, commReqPacket->bufLen);
        if (ret) {
            commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_WRITE_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void WriteFinishCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                     const EsyPro::CommPacket_t *commReqPacket,
                                     EsyPro::CommunicationType_t commType) {
        bool ret;

        ret = HondaCmd::CheckFinishShindengenUpdate();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_WRITE_FINISH_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void WriteEepromCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                    const EsyPro::CommPacket_t *commReqPacket,
                                    EsyPro::CommunicationType_t commType) {
       bool ret;
  
       for (int i = 0; i < commReqPacket->bufLen / 2; i++) {
           ret = HondaCmd::UpdateShindengenEeprom(eepromAddr, &commReqPacket->buffer[i * 2]);
           if (!ret) {
               commResPacket->cmd = CMD_IGNORE_RES;
               this->SetCommandRepeatState(false);
               return;
           }
           eepromAddr++;
        }

        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_WRITE_EEPROM_RES;
        commResPacket->bufLen = 0;
  
        this->SetCommandRepeatState(false);
    }

    void EndWriteEepromCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                        const EsyPro::CommPacket_t *commReqPacket,
                                        EsyPro::CommunicationType_t commType) {
        bool ret;

        nrf_delay_ms(1000);

        while (!HondaCmd::WakeUpHondaPgmfi()) {}

        ret = HondaCmd::ShindengenEndWriteEeprom();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_END_WRITE_EEPROM_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void TurnOffCommand::Execute(CommPacket_t *commResPacket,
                                 const CommPacket_t *commReqPacket,
                                 CommunicationType_t commType) {
        bool ret;

        eepromAddr = 0;

        ret = HondaCmd::CheckShindengenTurnOff();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_EEPROM_OFF_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x01;
        } else {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_EEPROM_OFF_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x00;
        }
        this->SetCommandRepeatState(false);
    }

    void TurnOnCommand::Execute(CommPacket_t *commResPacket,
                                const CommPacket_t *commReqPacket,
                                CommunicationType_t commType) {
        bool ret;

        eepromAddr = 0x0000;

        ret = HondaCmd::EnterShindengenEepromMode();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_EEPROM_ON_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x01;           
        } else {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_EEPROM_ON_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x00;
        }
        this->SetCommandRepeatState(false);
    }

    void ReadEepromCommand::Execute(CommPacket_t *commResPacket,
                                    const CommPacket_t *commReqPacket,
                                    CommunicationType_t commType) {
        bool ret;
  
        ret = HondaCmd::ReadShindengenEeprom(eepromAddr, &commResPacket->buffer[0]);
        if (!ret) {
            commResPacket->cmd = CMD_IGNORE_RES;
            this->SetCommandRepeatState(false);
            return;
        }
  
        eepromAddr++;
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_ECU_HONDA_SHINDENGEN_READ_EEPROM_RES;
        commResPacket->bufLen = 2;
  
        this->SetCommandRepeatState(false);
    }
}
