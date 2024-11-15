#include "ecu/piaggio_flash.h"
#include "peripheral/uart.h"
#include "ble_common.h"

using namespace EsyPro;
using namespace ECU;

static const char LOG_DGB_NAME[] = "piaggio_flash";

static PiaggioMIUFlash::AccessSecureCommand piaggioMIUAccessSecureCmd;
static PiaggioMIUFlash::ReadCommand piaggioMIUReadFlashCmd;
static PiaggioMIUFlash::WriteCommand piaggioMIUWriteFlashCmd;
static PiaggioMIUFlash::EndWriteCommand piaggioMIUWriteFlashEndCmd;

static PiaggioPGFlash::SetupCommand piaggioPGSetupCmd;
static PiaggioPGFlash::ReadCommand piaggioPGReadFlashCmd;
static PiaggioPGFlash::ReadKeyCommand piaggioPGReadKeyCmd;

static PiaggioAC19xFlash::SetupCommand piaggioAC19xSetupCmd;
static PiaggioAC19xFlash::ReadCommand piaggioAC19xReadFlashCmd;
static PiaggioAC19xFlash::WriteCommand piaggioAC19xWriteFlashCmd;

static PiaggioRISSFlash::ReadCommand piaggioRISSReadFlashCmd;
static PiaggioRISSFlash::SetupCommand piaggioRISSSetupCmd;
static PiaggioRISSFlash::ReadKeyCommand piaggioRISSReadKeyCmd;

static PiaggioCommonObj ecuObj;
static uint32_t ecuCurFlashAddr = 0;
static uint32_t ecuEndFlashAddr = 0;
static int flashType = MIU_EEPROM;

namespace PiaggioCmd {
    static bool MIUFlashWriteTranscieveObd2Packet(uint8_t *srcBuf, uint8_t bufLen,
                                                  uint8_t *tmpData, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[4] = {0x80, PIAGGIO_TARGET_BYTE, PIAGGIO_SOURCE_BYTE, 0x00};
        
        srcCmd[3] = bufLen;
        ecuObj.SetReqEcuPacket(srcCmd, 4, srcBuf, bufLen);

        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        ecuObj.GetEcuPacketResData(tmpData, dataLen, 0);
        if (!ret || (tmpData[0] != (srcBuf[0] | 0x40))) {
            return false;
        }

        return true;
    }

    static bool MIUTranscieveObd2Packet(uint8_t *srcBuf, uint8_t bufLen,
                                     uint8_t *tmpData, uint8_t *dataLen,
                                     bool isHeaderFour) {
        bool ret;
        uint8_t srcCmd[4] = {0x00, PIAGGIO_TARGET_BYTE, PIAGGIO_SOURCE_BYTE, 0x00};
        
        if (!isHeaderFour) {
            srcCmd[0] = 0x80 | bufLen;
            ecuObj.SetReqEcuPacket(srcCmd, 3, srcBuf, bufLen);

        } else {
            srcCmd[0] = 0x80;
            srcCmd[2] = 0x01;
            srcCmd[3] = bufLen;
            ecuObj.SetReqEcuPacket(srcCmd, 4, srcBuf, bufLen);
        }

        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        ecuObj.GetEcuPacketResData(tmpData, dataLen, 0);
        if (!ret || (tmpData[0] != (srcBuf[0] | 0x40))) {
            return false;
        }

        return true;
    }

    static bool MIUEnterSecureModeRequest(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBufStart[] = {0x81};
        uint8_t srcBufSetup[] = {0x10, 0x84, 0x04};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ecuObj.InitKlineBus();
        ret = MIUTranscieveObd2Packet(srcBufStart, sizeof(srcBufStart), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Send SetupSecureMode-1 Errors", LOG_DGB_NAME);
            return false;
        }

        ret = MIUTranscieveObd2Packet(srcBufSetup, sizeof(srcBufSetup), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Send SetupSecureMode-2 Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool MIUSendSeedkeyRequest(uint8_t *seedkey) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x27, 0x09};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, true);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Send SeedRequest Errors", LOG_DGB_NAME);
            return false;
        }

        memcpy(seedkey, &tmpData[2], 2);
        return true;
    }

    static bool MIUSendKeyRespond(const uint8_t *seedkey) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x27, 0x0A, 0x00, 0x00, 0x37, 0x54};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcBuf[2] = seedkey[0];
        srcBuf[3] = seedkey[1];

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, true);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Send KeyRespond Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool MIURequestUpload(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x35, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcBuf[1] = (ecuCurFlashAddr < MIU_FLASH_START_ADDR) ? 0x22 : 0x23;
        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, true);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Request Upload Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool MIUReadEcuFlash(const uint32_t *addr, uint8_t *data) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x36, 0x00, 0x00, 0x00, 0x00, 0x80};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcBuf[1] = (ecuCurFlashAddr < MIU_FLASH_START_ADDR) ? 0x22 : 0x23;
        srcBuf[2] = (*addr >> 16) & 0xFF;
        srcBuf[3] = (*addr >> 8) & 0xFF;
        srcBuf[4] = *addr & 0xFF;

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, true);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read-Flash[%x] Errors", LOG_DGB_NAME, srcBuf[1]);
            return false;
        }

        memcpy(data, &tmpData[6], 128);
        return true;
    }

    static bool MIURequestDownload(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x34, 0x24};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, true);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Request Download Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool MIUWriteEcuEeprom(const uint32_t *addr, const uint8_t *data) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcHeader[] = {0x36, 0x24, 0x00, 0x00, 0x00, 0x20};
        uint8_t srcBuf[38] = {0};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcHeader[2] = (*addr >> 16) & 0xFF;
        srcHeader[3] = (*addr >> 8) & 0xFF;
        srcHeader[4] = *addr & 0xFF;

        memcpy(&srcBuf[0], srcHeader, sizeof(srcHeader));
        memcpy(&srcBuf[6], data, 32);

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, true);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Write-Eeprom Errors", LOG_DGB_NAME);
            return false;
        }        

        return true;
    }

    static bool MIUStartDiagService(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x10, 0x85, 0x03};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Start Diagnostic Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool MIUSwitchSession(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x83, 0x03, 0x32, 0x01, 0xDC, 0x01, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Switch session Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool MIUWriteDataLocalIdService(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf1[] = {0x3B, 0x99, 0x20, 0x24, 0x03, 0x13};
        uint8_t srcBuf2[] = {0x3B, 0x98, 0x57, 0x4C, 0x6F, 0x61, 0x64, 0x31, 0x30, 0x33, 0x39, 0x54};

        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = MIUTranscieveObd2Packet(srcBuf1, sizeof(srcBuf1), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Write LocalID-1 Errors", LOG_DGB_NAME);
            return false;
        }

        ret = MIUTranscieveObd2Packet(srcBuf2, sizeof(srcBuf2), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Write LocalID-2 Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool MIUStartRoutineLocalIDService(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x31, 0x02, 0xC2, 0x00, 0x00, 0xC7, 0xFF, 0xFF};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Start routine LocalID Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool MIUFlashSendSeedkeyRequest(int16_t *seedkey1, int16_t *seedkey2) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x27, 0x01};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Send Seed-Flash Request Errors", LOG_DGB_NAME);
            return false;
        }

        *seedkey1 = (tmpData[2] << 8) | tmpData[3];
        *seedkey2 = (tmpData[4] << 8) | tmpData[5];
        return true;
    }

    static bool MIUFlashSendKeyRespond(const int16_t seedkey1, const int16_t seedkey2) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x27, 0x02, 0x00, 0x00, 0x00, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcBuf[2] = (seedkey1 >> 8) & 0xFF;
        srcBuf[3] = seedkey1 & 0xFF;
        srcBuf[4] = (seedkey2 >> 8) & 0xFF;
        srcBuf[5] = seedkey2 & 0xFF;

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Send KeyRespond Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool MIUFlashEraseRequest(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x33, 0x02};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};
        Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, false);
        nrf_delay_ms(1100);
        mc33290->FlushBytes();

        return true;
    }

    static bool MIUFlashRequestDownload(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x34, 0xC2, 0x00, 0x00, 0x03, 0x06, 0x00, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = MIUTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Request download Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool MIUFlashTransferRequest(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x36, 0xDA, 0x67, 0x78, 0x39, 0xF6, 0x5B, 0x15, 0x94};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = MIUFlashWriteTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Request transfer Errors", LOG_DGB_NAME);
            return false;
        }

        return true;   
    }

    static bool MIUWriteEcuFlash(const uint8_t *data, uint8_t bufLen) {
        bool ret;
        uint8_t tmpLen;
        uint8_t srcBuf[129] = {0};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcBuf[0] = 0x36;
        memcpy(&srcBuf[1], data, bufLen);

        ret = MIUFlashWriteTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &tmpLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Write-Flash Errors", LOG_DGB_NAME);
            return false;
        }        

        return true;
    }

    static bool MIUWriteEcuFlashEnd(const uint16_t checksum) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf1[] = {0x37};
        uint8_t srcBuf2[] = {0x31, 0x01, 0xC2, 0x00, 0x00, 0xC7, 0xFF, 0xFF, 0x00, 0x00};
        uint8_t srcBuf3[] = {0x33, 0x01};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};
        Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();

        ret = MIUFlashWriteTranscieveObd2Packet(srcBuf1, sizeof(srcBuf1), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Request transfer exit-1 Errors", LOG_DGB_NAME);
            return false;
        }

        srcBuf2[8] = checksum & 0xFF;
        srcBuf2[9] = (checksum >> 8) & 0xFF;
        ret = MIUTranscieveObd2Packet(srcBuf2, sizeof(srcBuf2), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Request transfer exit-2 Errors", LOG_DGB_NAME);
            return false;
        }

        ret = MIUTranscieveObd2Packet(srcBuf3, sizeof(srcBuf3), tmpData, &dataLen, false);
        nrf_delay_ms(1000);
        mc33290->FlushBytes();

        return true;
    }

    static bool PGMapTranscieveObd2Packet(uint8_t *srcBuf, uint8_t bufLen,
                                       uint8_t *tmpData, uint8_t *dataLen,
                                       bool isHeaderFour) {
        bool ret;
        uint8_t srcCmd[4] = {0x00, PGFLASH_PIAGGIO_TARGET_BYTE, PIAGGIO_SOURCE_BYTE, 0x00};
        
        if (!isHeaderFour) {
            srcCmd[0] = 0x80 | bufLen;
            ecuObj.SetReqEcuPacket(srcCmd, 3, srcBuf, bufLen);

        } else {
            srcCmd[0] = 0x80;
            srcCmd[2] = 0xF1;
            srcCmd[3] = bufLen;
            ecuObj.SetReqEcuPacket(srcCmd, 4, srcBuf, bufLen);
        }

        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        ecuObj.GetEcuPacketResData(tmpData, dataLen, 0);
        if (!ret || (tmpData[0] != (srcBuf[0] | 0x40))) {
            return false;
        }

        return true;
    }

    static bool PGStopCommunication(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf1Cmd[] = {0x20};
        uint8_t srcBuf2Cmd[] = {0x82};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = MIUTranscieveObd2Packet(srcBuf1Cmd, sizeof(srcBuf1Cmd), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: PG Stop-1 Errors", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(100);

        ret = MIUTranscieveObd2Packet(srcBuf2Cmd, sizeof(srcBuf2Cmd), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: PG Stop-2 Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool PGCheckInitReadFlash(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf1Cmd[] = {0xA0, 0x02};
        uint8_t srcBuf2Cmd[] = {0x31, 0xFA};
        uint8_t srcBuf3Cmd[] = {0x31, 0xFB, 0x00, 0x00, 0x00, 0x00, 0x06};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};
        Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();

        PGStopCommunication();
        nrf_delay_ms(100);

        ecuObj.InitKlineBus();

        ret = PGMapTranscieveObd2Packet(srcBuf1Cmd, sizeof(srcBuf1Cmd), tmpData, &dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: InitReadFlash-1 Errors", LOG_DGB_NAME);
            return false;
        }

        mc33290->DeInit();
        mc33290->Init((nrf_uart_baudrate_t)MC33290_PIAGGIO_UART_BAUDRATE_1);
        nrf_delay_ms(100);
        ret = PGMapTranscieveObd2Packet(srcBuf2Cmd, sizeof(srcBuf2Cmd), tmpData, &dataLen, true);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: InitReadFlash-2 Errors", LOG_DGB_NAME);
            return false;
        }
        ecuObj.GetEcuPacketResData(tmpData, &dataLen, 0);
        memcpy(&srcBuf3Cmd[2], &tmpData[2], 4);

        ret = PGMapTranscieveObd2Packet(srcBuf3Cmd, sizeof(srcBuf3Cmd), tmpData, &dataLen, true);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: InitReadFlash-3 Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool PGReadEcuFlash(const uint32_t *addr, uint8_t *data) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x31, 0xFB, 0x00, 0x00, 0x00, 0x00, 0x20};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcBuf[4] = (*addr >> 8) & 0xFF;
        srcBuf[5] = *addr & 0xFF;

        ret = PGMapTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen, true);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read-Flash Errors", LOG_DGB_NAME);
            return false;
        }

        memcpy(data, &tmpData[2], 32);
        return true;
    }

    static bool PGReadEcuKey(uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcBuf1Cmd[] = {0x21, 0x0F};
        uint8_t srcBuf2Cmd[] = {0x21, 0x10};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};
        int curIdx = 0;

        ret = MIUTranscieveObd2Packet(srcBuf1Cmd, sizeof(srcBuf1Cmd), tmpData, dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read-Key-1 Errors", LOG_DGB_NAME);
            return false;
        }
        memcpy(&data[curIdx], &tmpData[2], *dataLen - 2);
        curIdx += *dataLen - 2;

        ret = MIUTranscieveObd2Packet(srcBuf2Cmd, sizeof(srcBuf2Cmd), tmpData, dataLen, false);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read-Key-1 Errors", LOG_DGB_NAME);
            return false;
        }
        memcpy(&data[curIdx], &tmpData[2], *dataLen - 2);
        curIdx += *dataLen - 2;

        *dataLen = curIdx;
        return true;
    }

    static bool RISSReadEcuKey(uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcBuf[] = {0x31, 0xFB, 0x00, 0x00, 0x40, 0x65, 0x1B};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = PGMapTranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, dataLen, true);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read-Flash Errors", LOG_DGB_NAME);
            return false;
        }

        memcpy(data, &tmpData[2], *dataLen - 2);
        *dataLen -= 2;
        return true;
    }

    static bool AC19xCheckInitFlash(void) {
        bool ret;
        uint8_t srcCmd[] = {0x33, 0x80, 0x01};
        uint8_t srcBuf[] = {0xFF, 0xAA, 0x2A, 0x55, 0xAA, 0x5A};
        Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();

        mc33290->DeInit();
        mc33290->Init((nrf_uart_baudrate_t)MC33290_PIAGGIO_UART_BAUDRATE_1);
        nrf_delay_ms(100);

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuf, sizeof(srcBuf));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Init-Flash Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool AC19xReadEcuFlash(const uint32_t *addr, uint8_t *data) {
        bool ret;
        uint8_t srcCmd[] = {0x33, 0x02, 0x02};
        uint8_t srcBuf[] = {0x00, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE];
        uint8_t dataLen;

        srcBuf[0] = (*addr >> 8) & 0xFF;
        srcBuf[1] = *addr & 0xFF;
        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuf, sizeof(srcBuf));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Read-Flash Errors", LOG_DGB_NAME);
            return false;
        }
        ecuObj.GetEcuPacketResData(tmpData, &dataLen, 0);
        memcpy(data, tmpData, 2);
        return true;
    }

    static bool AC19xWriteEcuFlash(const uint32_t *addr, const uint8_t *data) {
        bool ret;
        uint8_t srcCmd[] = {0x33, 0x12, 0x00};
        uint8_t srcBuf[] = {0x00, 0x00, 0x00, 0x00};

        srcBuf[0] = (*addr >> 8) & 0xFF;
        srcBuf[1] = *addr & 0xFF;
        srcBuf[2] = data[0];
        srcBuf[3] = data[1];

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuf, sizeof(srcBuf));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Write-Flash Errors", LOG_DGB_NAME);
            return false;
        }
        
        return true;
    }
}

namespace PiaggioMIUFlash {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_PIAGGIO_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_BASIC_MEM_SETUP_REQ:
            cmd = &piaggioMIUAccessSecureCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Access-Security Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_READ_DATA_REQ:
            cmd = &piaggioMIUReadFlashCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Read Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_WRITE_DATA_REQ:
            cmd = &piaggioMIUWriteFlashCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Write Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_END_WRITE_DATA_REQ:
            cmd = &piaggioMIUWriteFlashEndCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU End Write Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

   static void CalculateKey(uint8_t x, uint8_t y, uint8_t *a, uint8_t *b) {
        float temp = (y + 1) * 1.44f;
        int32_t temp2 = ((int32_t)((x * 256 + y) / 200)) * 200;

        *a = (uint8_t)temp + 1;
        temp2 = temp2 & 0xFF;
        temp2 = y - temp2;
        *b = temp2 & 0xFF;
    }

    static bool AccessEcuSecureMode1(int flashMode) {
        int count = 0;
        const int totalRetries = 10;
        
        uint8_t ecuSeed[2], ecuKey[2];

        while (count++ < totalRetries) {
            bool ret;

            ret = PiaggioCmd::MIUEnterSecureModeRequest();
            if (!ret) {
                nrf_delay_ms(100);
                continue;
            }

            nrf_delay_ms(10);
            ret = PiaggioCmd::MIUSendSeedkeyRequest(ecuSeed);
            if (!ret) {
                nrf_delay_ms(100);
                continue;
            }

            nrf_delay_ms(10);

            CalculateKey(ecuSeed[0], ecuSeed[1], &ecuKey[0], &ecuKey[1]);
            ret = PiaggioCmd::MIUSendKeyRespond(ecuKey);
            if (ret) {
                if (flashMode == MIU_FLASH_MODE_READ) {
                    ret = PiaggioCmd::MIURequestUpload();
                } else {
                    ret = PiaggioCmd::MIURequestDownload();
                }
                
                if (ret) {
                    return true;
                }
                break;
            }

            nrf_delay_ms(5000);
        }

        return false;
    }

    int16_t hamEA4(int16_t seed) {
        int16_t word_42D520, word_42D522, word_42D524, word_42D526, word_42D528, word_42D52C;
        word_42D520 = seed;

        word_42D524 = 177;
        word_42D522 = word_42D520 / word_42D524;
        word_42D526 = 177 * word_42D522;
        word_42D520 = word_42D520 - word_42D526;
        word_42D526 = -word_42D522 - word_42D522;
        word_42D528 = 171;
        word_42D522 = 171 * word_42D520;
        word_42D526 += 171 * word_42D520;
        word_42D520 = word_42D526;
        
        if ( word_42D526 < 0 )
            word_42D520 += 30269;
        
        word_42D52C = word_42D520;
        return word_42D52C;
    }

    int16_t hamB76(int16_t seed) {
        int16_t word_42D520, word_42D522, word_42D524, word_42D526, word_42D528, word_42D52C;
        word_42D520 = seed;
    
        word_42D524 = 178;
        word_42D522 = word_42D520 / word_42D524;
        word_42D520 %= 178;
        word_42D528 = -63 * word_42D522;
        word_42D522 = 170;
        word_42D526 = 170 * word_42D520;
        word_42D528 += 170 * word_42D520;
        word_42D520 = word_42D528;
        
        if (word_42D528 < 0) {
            word_42D520 += 30323;
        }
        
        word_42D52C = word_42D520;
        return word_42D52C;
    }

    int16_t hamA6D(int16_t magic, int16_t seed) {
        int16_t so36 = hamB76(magic);
        so36 += hamEA4(seed);
   
        int16_t so34 = hamB76(seed);
        so34 += hamEA4(so36);

        return so34;
    }

    static uint32_t caculateFlashSeedKey(int16_t seedReal1, int16_t seedReal2) {
        int16_t out_A6D  =  hamA6D(0x1759, seedReal1);
        int16_t out_A6D2  =  hamA6D(0x1215, seedReal2);
        uint32_t seedkey = (out_A6D & 0xFFFF) << 16 | out_A6D2 & 0xFFFF;
        
        return seedkey;
    }

    static bool AccessEcuSecureMode2(void) {
        bool ret;
        Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();
        int16_t flashSeed1, flashSeed2;
        uint32_t keyRespond;

        ret = PiaggioCmd::MIUStartDiagService();
        if (!ret) {
            return false;
        }

        mc33290->DeInit();
        mc33290->Init((nrf_uart_baudrate_t)MC33290_PIAGGIO_UART_BAUDRATE_2);
        nrf_delay_ms(100);

        ret = PiaggioCmd::MIUSwitchSession();
        if (!ret) {
            return false;
        }

        //Seedkey processing
        ret = PiaggioCmd::MIUFlashSendSeedkeyRequest(&flashSeed1, &flashSeed2);
        if (!ret) {
            return false;
        }

        keyRespond = caculateFlashSeedKey(flashSeed1, flashSeed2);

        ret = PiaggioCmd::MIUFlashSendKeyRespond(((keyRespond >> 16) & 0xFFFF), (keyRespond & 0xFFFF));
        if (!ret) {
            return false;
        }

        ret = PiaggioCmd::MIUWriteDataLocalIdService();
        if (!ret) {
            return false;
        }

        ret = PiaggioCmd::MIUStartRoutineLocalIDService();
        if (!ret) {
            return false;
        }

        ret = PiaggioCmd::MIUFlashEraseRequest();
        if (!ret) {
            return false;
        }

        ret = PiaggioCmd::MIUFlashRequestDownload();
        if (!ret) {
            return false;
        }

        ret = PiaggioCmd::MIUFlashTransferRequest();
        if (!ret) {
            return false;
        }

        return true;
    }

    void AccessSecureCommand::Execute(CommPacket_t *commResPacket,
                                      const CommPacket_t *commReqPacket,
                                      CommunicationType_t commType) {
        bool ret;
        memcpy(&ecuCurFlashAddr, &commReqPacket->buffer[0], 3);
        memcpy(&ecuEndFlashAddr, &commReqPacket->buffer[4], 3);

        int flashMode = commReqPacket->buffer[7];
        flashType = commReqPacket->buffer[8];

        if (flashMode == MIU_FLASH_MODE_READ || flashType == MIU_EEPROM) {
            ret = AccessEcuSecureMode1(flashMode);
        
        } else {
            ret = AccessEcuSecureMode2();
        }
        
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

    void ReadCommand::Execute(CommPacket_t *commResPacket,
                              const CommPacket_t *commReqPacket,
                              CommunicationType_t commType) {
        if (ecuCurFlashAddr >= ecuEndFlashAddr) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_END_READ_DATA_RES;
            commResPacket->bufLen = 0;
        
        } else {
            bool ret = PiaggioCmd::MIUReadEcuFlash(&ecuCurFlashAddr, 
                                                    commResPacket->buffer);
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
                commResPacket->bufLen = 128;
                ecuCurFlashAddr += commResPacket->bufLen;
                commResPacket->cmd = CMD_BASIC_MEM_READ_DATA_RES;
            
            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }
        }

        this->SetCommandRepeatState(false);
    }

    void WriteCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        bool ret;

        if (flashType == MIU_EEPROM) {
            ret = PiaggioCmd::MIUWriteEcuEeprom(&ecuCurFlashAddr,
                                                commReqPacket->buffer);
        } else {
            ret = PiaggioCmd::MIUWriteEcuFlash(commReqPacket->buffer,
                                               commReqPacket->bufLen);
        }

        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
            commResPacket->bufLen = 0;
            ecuCurFlashAddr += commReqPacket->bufLen;

        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void EndWriteCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                  const EsyPro::CommPacket_t *commReqPacket,
                                  EsyPro::CommunicationType_t commType) {
        bool ret;
        uint16_t checksum;

        memcpy(&checksum, &commReqPacket->buffer[0], 2);

        ret = PiaggioCmd::MIUWriteEcuFlashEnd(checksum);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_END_WRITE_DATA_RES;
            commResPacket->bufLen = 0;

        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }  
}

namespace PiaggioPGFlash {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_PIAGGIO_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_BASIC_MEM_SETUP_REQ:
            cmd = &piaggioPGSetupCmd;
            NRF_LOG_INFO("[%s]: INFO: PG Setup Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_READ_DATA_REQ:
            cmd = &piaggioPGReadFlashCmd;
            NRF_LOG_INFO("[%s]: INFO: PG Read Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_READ_KEY_REQ:
            cmd = &piaggioPGReadKeyCmd;
            NRF_LOG_INFO("[%s]: INFO: PG Read-Key Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    void SetupCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        memcpy(&ecuCurFlashAddr, &commReqPacket->buffer[0], 3);
        memcpy(&ecuEndFlashAddr, &commReqPacket->buffer[4], 3);

        if (ecuCurFlashAddr) {
            bool ret = PiaggioCmd::PGCheckInitReadFlash();
            if (!ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
                commResPacket->buffer[0] = 0x00;
                commResPacket->bufLen = 1;
                this->SetCommandRepeatState(false);
                return;
            }
        }

        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
        commResPacket->buffer[0] = 0x01;
        commResPacket->bufLen = 1;

        this->SetCommandRepeatState(false);
    }

    void ReadCommand::Execute(CommPacket_t *commResPacket,
                              const CommPacket_t *commReqPacket,
                              CommunicationType_t commType) {        
        if (ecuCurFlashAddr >= ecuEndFlashAddr) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_END_READ_DATA_RES;
            commResPacket->bufLen = 0;      

        } else {
            bool ret = PiaggioCmd::PGReadEcuFlash(&ecuCurFlashAddr, 
                                            commResPacket->buffer);
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
                commResPacket->bufLen = 32;
                ecuCurFlashAddr += commResPacket->bufLen;
                commResPacket->cmd = CMD_BASIC_MEM_READ_DATA_RES;
            
            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }
        }

        this->SetCommandRepeatState(false);
    }

    void ReadKeyCommand::Execute(CommPacket_t *commResPacket,
                                 const CommPacket_t *commReqPacket,
                                 CommunicationType_t commType) {
        uint8_t dataLen = 0;

        bool ret = PiaggioCmd::PGReadEcuKey(commResPacket->buffer, &dataLen);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
            commResPacket->bufLen = dataLen;
            commResPacket->cmd = CMD_BASIC_MEM_READ_KEY_RES;
           
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }
}

namespace PiaggioRISSFlash {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_PIAGGIO_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_BASIC_MEM_SETUP_REQ:
            cmd = &piaggioRISSSetupCmd;
            NRF_LOG_INFO("[%s]: INFO: RISS Setup Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_READ_DATA_REQ:
            cmd = &piaggioRISSReadFlashCmd;
            NRF_LOG_INFO("[%s]: INFO: RISS Read Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_READ_KEY_REQ:
            cmd = &piaggioRISSReadKeyCmd;
            NRF_LOG_INFO("[%s]: INFO: RISS Read-Key Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    void SetupCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        memcpy(&ecuCurFlashAddr, &commReqPacket->buffer[0], 3);
        memcpy(&ecuEndFlashAddr, &commReqPacket->buffer[4], 3);

        bool ret = PiaggioCmd::PGCheckInitReadFlash();
        if (!ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
            commResPacket->buffer[0] = 0x00;
            commResPacket->bufLen = 1;
        
        } else {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
            commResPacket->buffer[0] = 0x01;
            commResPacket->bufLen = 1;
        }

        this->SetCommandRepeatState(false);
    }

    void ReadCommand::Execute(CommPacket_t *commResPacket,
                              const CommPacket_t *commReqPacket,
                              CommunicationType_t commType) {    
        if (ecuCurFlashAddr >= ecuEndFlashAddr) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_END_READ_DATA_RES;
            commResPacket->bufLen = 0;      

        } else {
            bool ret = PiaggioCmd::PGReadEcuFlash(&ecuCurFlashAddr, 
                                            commResPacket->buffer);
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
                commResPacket->bufLen = 32;
                ecuCurFlashAddr += commResPacket->bufLen;
                commResPacket->cmd = CMD_BASIC_MEM_READ_DATA_RES;

            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }
        }

        this->SetCommandRepeatState(false);
    }

    void ReadKeyCommand::Execute(CommPacket_t *commResPacket,
                                 const CommPacket_t *commReqPacket,
                                 CommunicationType_t commType) {
        uint8_t dataLen = 0;

        bool ret = PiaggioCmd::RISSReadEcuKey(commResPacket->buffer, &dataLen);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
            commResPacket->bufLen = dataLen;
            commResPacket->cmd = CMD_BASIC_MEM_READ_KEY_RES;
           
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }
}

namespace PiaggioAC19xFlash {
    Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_PIAGGIO_MUX);
    
        switch (commCmdType & 0x0F) {
        case CMD_BASIC_MEM_SETUP_REQ:
            cmd = &piaggioAC19xSetupCmd;
            NRF_LOG_INFO("[%s]: INFO: AC19x Setup Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_READ_DATA_REQ:
            cmd = &piaggioAC19xReadFlashCmd;
            NRF_LOG_INFO("[%s]: INFO: AC19x Read Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_WRITE_DATA_REQ:
            cmd = &piaggioAC19xWriteFlashCmd;
            NRF_LOG_INFO("[%s]: INFO: AC19x Write Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    void SetupCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        memcpy(&ecuCurFlashAddr, &commReqPacket->buffer[0], 3);
        memcpy(&ecuEndFlashAddr, &commReqPacket->buffer[4], 3);

        bool ret = PiaggioCmd::AC19xCheckInitFlash();

        commResPacket->bufLen = 1;
        commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
        commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
        commResPacket->buffer[0] = (ret) ? 0x01 : 0x00;

        this->SetCommandRepeatState(false);
    }

    void ReadCommand::Execute(CommPacket_t *commResPacket,
                              const CommPacket_t *commReqPacket,
                              CommunicationType_t commType) {
        if (ecuCurFlashAddr >= ecuEndFlashAddr) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_END_READ_DATA_RES;
            commResPacket->bufLen = 0;
          
        } else {
            bool ret = PiaggioCmd::AC19xReadEcuFlash(&ecuCurFlashAddr, 
                                                     commResPacket->buffer);
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
                commResPacket->bufLen = 2;
                ecuCurFlashAddr += commResPacket->bufLen;
                commResPacket->cmd = CMD_BASIC_MEM_READ_DATA_RES;
            
            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }            
        }

        this->SetCommandRepeatState(false);
    }

    void WriteCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        bool ret;

        ret = PiaggioCmd::AC19xWriteEcuFlash(&ecuCurFlashAddr,
                                             commReqPacket->buffer);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
            commResPacket->bufLen = 0;
            ecuCurFlashAddr += 2;

        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }
}
