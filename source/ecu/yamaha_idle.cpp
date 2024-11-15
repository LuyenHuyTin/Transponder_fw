#include "ecu/yamaha_idle.h"
#include "peripheral/uart.h"
#include "peripheral/timer.h"
#include "ble_common.h"

using namespace EsyPro;
using namespace ECU;

static const char LOG_DGB_NAME[] = "yamaha_idle";

static uint8_t statisticTable[MAX_YAMAHA_SENSORS_TABLE] = {0};
static uint8_t totalStatisticTable = 0;

static YamahaCommon::ErrorState_t errorState = YamahaCommon::ErrorState_t::SEND_ERROR_REQ;

static YamahaCommon::ErrorCommand yamahaCommonErrorCmd;
static YamahaCommon::PingCommand yamahaCommonPingCmd;
static YamahaCommon::StatisticCommand yamahaCommonStatisticCmd;
static YamahaCommon::DiagnosticCommand yamahaCommonDiagnosticCommand;
static YamahaCommon::EraseCommand yamahaCommonEraseCommand;

static Yamaha::CheckSyncCommand yamahaCheckSyncCmd;
static YamahaSMK::CheckSyncCommand yamahaSMKCheckSyncCmd;
static YamahaObd2::CheckSyncCommand yamahaObd2CheckSyncCmd;

static YamahaObd2::StatisticCommand yamahaObd2StatisticCmd;
static YamahaObd2::DiagnosticCommand yamahaObd2DiagnosticCommand;

static YamahaCommonObj ecuObj;

namespace YamahaCmd {
    static bool TranscieveObd2Packet(uint8_t *srcBuf, uint8_t bufLen, 
                                     uint8_t *tmpData, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[3] = {0x00, YAMAHA_TARGET_BYTE, YAMAHA_SOURCE_BYTE};
        
        srcCmd[0] = 0x80 | bufLen;
        ecuObj.SetReqEcuPacket(srcCmd, 3, srcBuf, bufLen);
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        ecuObj.GetEcuPacketResData(tmpData, dataLen, 0);
        if (!ret || (tmpData[0] != (srcBuf[0] | 0x40))) {
            return false;
        }

        nrf_delay_ms(100);
        return true;        
    }

    static bool EcuGetDiagIndex(uint8_t *diagIdx) {
        uint8_t dataLen;
        uint8_t requestCA[] = {0xCA};
        uint8_t tmpData[10] = {0};

        do {
            bool ret;

            ecuObj.SetReqEcuPacket(requestCA, 1);
            ret = ecuObj.ProcessRequestEcu15800();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: ECU Get DiagIndex", LOG_DGB_NAME);
                return false;
            }
            ecuObj.GetEcuPacketResData(tmpData, &dataLen, 0);
        } while ((tmpData[0] & 0x40) == 0);

        *diagIdx = tmpData[1];

        return true;
    }

    static bool EcuActiveDiagObd2(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf1[] = {0x31, 0x70, 0xCE, 0x20};
        uint8_t srcBuf2[] = {0x31, 0x70, 0xCA};
        uint8_t srcBuf3[] = {0x31, 0x70, 0xCE, 0x08};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};
        
        ret = TranscieveObd2Packet(srcBuf1, sizeof(srcBuf1), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Active Diagnostic OBD2-1 Errors", LOG_DGB_NAME);
            return false;
        }
        
        ret = TranscieveObd2Packet(srcBuf2, sizeof(srcBuf2), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Active Diagnostic OBD2-2 Errors", LOG_DGB_NAME);
            return false;
        }

        ret = TranscieveObd2Packet(srcBuf3, sizeof(srcBuf3), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Active Diagnostic OBD2-3 Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool EcuSyncStageFE(void) {
        uint8_t waitingEcuResCmd[] = {0xFE};
        Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();
        Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();

        mc33290->DeInit();
        mc33290->Init((nrf_uart_baudrate_t)MC33290_YAMAHA_UART_BAUDRATE_1);

        ecuObj.SetReqEcuPacket(waitingEcuResCmd, sizeof(waitingEcuResCmd));

        timer->Start(ECU_YAMAHA_SYNC_TIMEOUT_1);
        while (!timer->IsTimeout()) {
            bool ret = ecuObj.ProcessRequestEcu15800();
            if (ret) {
                return true;
            }
        }

        NRF_LOG_INFO("[%s]: ERROR: ECU Sync-FE", LOG_DGB_NAME);
        return false;
    }

    static bool EcuSyncStagePing01(void) {
        uint8_t dataLen;
        uint8_t tmpData[10] = {0};
        uint8_t waitingEcuSyncCmd[] = {0xF7};
        uint8_t finishEcuSyncResNorm[] = {0xAB, 0xAB};
        uint8_t finishEcuSyncResCuJu[] = {0x40, 0xFF};
        Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();

        ecuObj.SetReqEcuPacket(waitingEcuSyncCmd, sizeof(waitingEcuSyncCmd));

        timer->Start(ECU_YAMAHA_SYNC_TIMEOUT_2);
        while (!timer->IsTimeout()) {
            bool ret;
            
            ret = ecuObj.ProcessRequestEcu15800();
            ecuObj.GetEcuPacketResData(tmpData, &dataLen, 0);
            if (ret) {
                if (!memcmp(tmpData, finishEcuSyncResNorm, 2)) {
                    ecuObj.SetEcuPingType(YamahaCommonObj::ECU_NORMAL_TYPE);
                    nrf_delay_ms(500);
                    return true;

                } else if (!memcmp(tmpData, finishEcuSyncResCuJu, 2)) {
                    ecuObj.SetEcuPingType(YamahaCommonObj::ECU_KEY_CUJU);
                    return true;
                }
            }
        }

        NRF_LOG_INFO("[%s]: ERROR: ECU SyncPing-01", LOG_DGB_NAME);
        return false;
    }

    static bool EcuSyncStageStatistic01(uint8_t idx) {
        bool ret;
        uint8_t dataLen;
        uint8_t tmpData[10] = {0};
        uint8_t cmdIdx[1];
        
        cmdIdx[0] = idx;
        ecuObj.SetReqEcuPacket(cmdIdx, 1);
        ret = ecuObj.ProcessRequestEcu15800();
        ecuObj.GetEcuPacketResData(tmpData, &dataLen, 0);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: ECU SyncStatistic01-1", LOG_DGB_NAME);
            return false;
        }

        if ((tmpData[0] & 0x40) || (idx == 0x01)) {
            Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();

            if (idx == 0x01) {
                statisticTable[totalStatisticTable] = 0x01;
                totalStatisticTable++;
            }

            cmdIdx[0] = 0x01;
            ecuObj.SetReqEcuPacket(cmdIdx, 1);

            timer->Start(ECU_YAMAHA_SYNC_TIMEOUT_2);
            while (!timer->IsTimeout()) {
                ret = ecuObj.ProcessRequestEcu15800();
                if (ret) {
                    return true;
                }
            }

        } else {
            statisticTable[totalStatisticTable] = idx;
            totalStatisticTable++;
            if (totalStatisticTable > MAX_YAMAHA_SENSORS_TABLE) {
                NRF_LOG_INFO("[%s]: ERROR: ECU SyncStatistic01-2", LOG_DGB_NAME);
                return false;
            }
            return true;
        }

        NRF_LOG_INFO("[%s]: ERROR: ECU SyncStatistic01-2", LOG_DGB_NAME);
        return false;
    }

    static bool EcuSyncDiag01(void) {
        uint8_t dataLen;
        uint8_t waitSyncCE[] = {0xCE};
        uint8_t waitSyncCA[] = {0xCA};
        uint8_t recvData[10] = {0};
        uint8_t cmpData[10] = {0};
        int totalTry = 50;

        ecuObj.SetReqEcuPacket(waitSyncCE, 1);
        while (totalTry) {
            bool ret;

            ret = ecuObj.ProcessRequestEcu15800();
            ecuObj.GetEcuPacketResData(recvData, &dataLen, 0);
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: ECU SyncDiag01-1", LOG_DGB_NAME);
                return false;
            }
            if (totalTry == 50) {
                memcpy(cmpData, recvData, 2);

            } else if (memcmp(recvData, cmpData, 2)) {
                ecuObj.SetReqEcuPacket(waitSyncCA, 1);
                ret = ecuObj.ProcessRequestEcu15800();
                if (!ret) {
                    NRF_LOG_INFO("[%s]: ERROR: ECU SyncDiag01-2", LOG_DGB_NAME);
                    return false;
                }

                return true;
            }
            
            totalTry--;
        }

        NRF_LOG_INFO("[%s]: ERROR: ECU SyncDiag01-3", LOG_DGB_NAME);
        return false;
    }

    static bool EcuSyncDiag02(void) {
        uint8_t dataLen;
        uint8_t firstData[10] = {0};
        uint8_t endData[10] = {0};
        uint8_t waitSyncCE[] = {0xCE};
        int totalTry = 50;

        ecuObj.SetReqEcuPacket(waitSyncCE, 1);
        while (totalTry--) {
            for (int i = 0; i < 8; i++) {
                bool ret;

                ret = ecuObj.ProcessRequestEcu15800();
                if (!ret) {
                    NRF_LOG_INFO("[%s]: ERROR: ECU SyncDiag02-1", LOG_DGB_NAME);
                    return false;
                }
                if (i == 0) {
                    ecuObj.GetEcuPacketResData(firstData, &dataLen, 0);
                } else {
                    ecuObj.GetEcuPacketResData(endData, &dataLen, 0);
                }
            }

            if (memcmp(firstData, endData, dataLen)) {
                return true;
            }
        }

        NRF_LOG_INFO("[%s]: ERROR: ECU SyncDiag02-2", LOG_DGB_NAME);
        return false;
    }

    static bool EcuSyncDiag03(uint8_t *diagTable, uint8_t *tableSize) {
        uint8_t requestCB[] = {0xCB};
        uint8_t diagIndex;

        *tableSize = 0;

        for (int i = 0; i < MAX_YAMAHA_DIAG_TABLE; i++) {
            bool ret;

            ret = EcuGetDiagIndex(&diagIndex);
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: ECU SyncDiag03-1", LOG_DGB_NAME);
                return false;
            }

            for (int j = 0; j < i; j++) {
                if (diagTable[j] == diagIndex) {
                    return true;
                }
            }
            diagTable[i] = diagIndex;
            *tableSize += 1;

            ecuObj.SetReqEcuPacket(requestCB, 1);
            ret = ecuObj.ProcessRequestEcu15800();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: ECU SyncDiag03-2", LOG_DGB_NAME);
                return false;
            }
        }

        NRF_LOG_INFO("[%s]: ERROR: ECU SyncDiag03-3", LOG_DGB_NAME);
        return false;
    }

    static bool EcuSyncDiagObd2(uint8_t *diagTable, uint8_t *tableSize) {
        uint8_t srcBufCA[] = {0x31, 0x70, 0xCA};
        uint8_t srcBufCB[] = {0x31, 0x70, 0xCB};

        *tableSize = 0;

        for (int i = 0; i < MAX_YAMAHA_DIAG_TABLE; i++) {
            bool ret;
            uint8_t dataLen;
            uint8_t diagIdx;
            uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

            ret = TranscieveObd2Packet(srcBufCA, sizeof(srcBufCA), tmpData, &dataLen);
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: SyncDiag OBD2-1 Errors", LOG_DGB_NAME);
                return false;
            }

            diagIdx = tmpData[3];
            for (int j = 0; j < i; j++) {
                if (diagTable[j] == diagIdx) {
                    return true;
                }
            }
            diagTable[i] = diagIdx;
            *tableSize += 1;

            ret = TranscieveObd2Packet(srcBufCB, sizeof(srcBufCB), tmpData, &dataLen);
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: SyncDiag OBD2-2 Errors", LOG_DGB_NAME);
                return false;
            }
        }

        NRF_LOG_INFO("[%s]: ERROR: ECU SyncDiag OBD2-3", LOG_DGB_NAME);
        return false;        
    }

    static bool EcuSyncStageSMK(void) {
        uint8_t dataLen;
        uint8_t firstData[10] = {0};
        uint8_t secData[10] = {0};
        int totalTry = 50;
        int syncCnt = 0;
        bool isChecked = false;

        while (totalTry--) {
            bool ret;

            ret = ecuObj.ProcessRequestEcuSMK();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: ECU SyncSMK-1", LOG_DGB_NAME);
                return false;
            }
            if (!isChecked) {
                isChecked = true;
                ecuObj.GetEcuPacketResData(firstData, &dataLen, 0);
            } else {
                isChecked = false;
                ecuObj.GetEcuPacketResData(secData, &dataLen, 0);
                if (!memcmp(firstData, secData, dataLen)) {
                    syncCnt++;
                    if (syncCnt == 2) {
                        return true;
                    }
                }
            }
        }

        NRF_LOG_INFO("[%s]: ERROR: ECU SyncSMK-2", LOG_DGB_NAME);
        return false;
    }

    static bool StartCommunication(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf1[] = {0x81};
        uint8_t srcBuf2[] = {0x10, 0x83};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ecuObj.InitKlineBus();

        ret = TranscieveObd2Packet(srcBuf1, sizeof(srcBuf1), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: ECU Start Communication", LOG_DGB_NAME);
            return false;
        }

        ret = TranscieveObd2Packet(srcBuf2, sizeof(srcBuf2), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: ECU Start Communication-2", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool GetTableStatistics(uint8_t idx, uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t tmpData[10] = {0};
        uint8_t tmpCmd[10] = {0};
        uint8_t cmdIdx[1];
        
        cmdIdx[0] = idx;
        ecuObj.SetReqEcuPacket(cmdIdx, 1);
        ret = ecuObj.ProcessRequestEcu15800();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Get Statistics: %x", LOG_DGB_NAME, idx);
            return false;
        }
        ecuObj.GetEcuPacketResData(tmpData, dataLen, 0);
        ecuObj.GetEcuCmdResData(tmpCmd);
        data[0] = idx;
        memcpy(&data[1], tmpCmd, 2);
        memcpy(&data[3], tmpData, *dataLen);
        *dataLen += 3;

        return true;
    }

    static bool GetStatisticObd2(uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcBuf[] = {0x21, 0x01};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};     

        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Get Statistic-OBD2", LOG_DGB_NAME);
            return false;
        }
        *dataLen -= 2;
        memcpy(data, &tmpData[2], *dataLen);

        return true;
    }

    static bool GetInfo(uint8_t idx,  uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcBuf[] = {0x1A, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcBuf[1] = idx;
        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Get ECU Info: %x", LOG_DGB_NAME, idx);
            return false;
        }
        *dataLen -= 2;
        memcpy(data, &tmpData[2], *dataLen);

        return true;
    }

    static bool GetErrorStatics(uint8_t *data) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x18, 0x00, 0xFF, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Get Errors", LOG_DGB_NAME);
            return false;
        }
        dataLen -= 2;
        data[0] = tmpData[1];
        memcpy(&data[1], &tmpData[2], dataLen);

        return true;
    }

    static bool GetDiagnosticsData(uint8_t isActive, uint8_t cmdIdx,
                                   uint8_t *data, uint8_t *dataLen) {
        uint8_t totalQuerries = 7;
        uint8_t requestCA[] = {0xCA};
        uint8_t requestCB[] = {0xCB};
        uint8_t requestCE[] = {0xCE};
        uint8_t diagIdx;

        for (int i = 0; i < MAX_YAMAHA_DIAG_TABLE; i++) {
            bool ret;

            ret = EcuGetDiagIndex(&diagIdx);
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Get Diagnostic-1", LOG_DGB_NAME);
                return false;
            }

            if (diagIdx != cmdIdx) {
                totalQuerries = 6;
                ecuObj.SetReqEcuPacket(requestCB, 1);
                ret = ecuObj.ProcessRequestEcu15800();
                if (!ret) {
                    NRF_LOG_INFO("[%s]: ERROR: Get Diagnostic-2", LOG_DGB_NAME);
                    return false;
                }
            
            } else {
                if (isActive) {
                    for (int j = 0; j < 9; j++) {
                        ecuObj.SetReqEcuPacket(requestCE, 1);
                        ret = ecuObj.ProcessRequestEcu15800();
                        if (!ret) {
                            NRF_LOG_INFO("[%s]: ERROR: Active Actuator", LOG_DGB_NAME);
                            return false;
                        }
                    }
                    *dataLen = 2;
                    memset(data, 0, *dataLen);

                } else {
                    for (int j = 0; j < totalQuerries; j++) {
                        ecuObj.SetReqEcuPacket(requestCA, 1);
                        ret = ecuObj.ProcessRequestEcu15800();
                        if (!ret) {
                            NRF_LOG_INFO("[%s]: ERROR: Get Diagnostic-3", LOG_DGB_NAME);
                            return false;
                        }
                        ecuObj.GetEcuPacketResData(data, dataLen, 0);
                    }
                }

                return true;
            }
        }

        NRF_LOG_INFO("[%s]: ERROR: Get Diagnostic-4", LOG_DGB_NAME);
        return false;
    }

    static bool GetDiagnosticsObd2Data(uint8_t isActive, uint8_t cmdIdx, 
                                       uint8_t *data, uint8_t *dataLen) {
        uint8_t srcBufCA[] = {0x31, 0x70, 0xCA};
        uint8_t srcBufCB[] = {0x31, 0x70, 0xCB};
        uint8_t srcBufCE[] = {0x31, 0x70, 0xCE, 0x08};

        for (int i = 0; i < MAX_YAMAHA_DIAG_TABLE; i++) {
            bool ret;
            uint8_t diagIdx;
            uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

            ret = TranscieveObd2Packet(srcBufCA, sizeof(srcBufCA), tmpData, dataLen);
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Get Diagnostic OBD2-1 Errors", LOG_DGB_NAME);
                return false;
            }

            diagIdx = tmpData[3];
            if (diagIdx != cmdIdx) {
                ret = TranscieveObd2Packet(srcBufCB, sizeof(srcBufCB), tmpData, dataLen);
                if (!ret) {
                    NRF_LOG_INFO("[%s]: ERROR: Get Diagnostic OBD2-2 Errors", LOG_DGB_NAME);
                    return false;
                }
 
            } else {
                if (isActive) {
                    ret = TranscieveObd2Packet(srcBufCE, sizeof(srcBufCE), tmpData, dataLen);
                    if (!ret) {
                        NRF_LOG_INFO("[%s]: ERROR: Get Diagnostic OBD2-3 Errors", LOG_DGB_NAME);
                        return false;
                    }
                    *dataLen = 3;
                    memset(data, 0, *dataLen);

                } else {
                    *dataLen = 3;
                    memcpy(data, &tmpData[4], *dataLen);
                }
                return true;
            }
        }

        NRF_LOG_INFO("[%s]: ERROR: Get Diagnostic OBD2-4 Errors", LOG_DGB_NAME);
        return false;
    }

    static bool EraseErrors(uint8_t *errorData, uint8_t totalPastErr) {
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x14, 0x00, 0x00, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        for (int i = 0; i < totalPastErr; i++) {
            memcpy(&srcBuf[1], &errorData[i * 3], 3);
            bool ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen);
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: Erase Errors", LOG_DGB_NAME);
                return false;
            }
        }

        return true;
    }

    static bool CuJuDetectPing(void) {
        bool ret;
        uint8_t pingCmd[] = {0x01};

        ecuObj.SetReqEcuPacket(pingCmd, 1);
        ret = ecuObj.ProcessRequestEcu15800();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: CuJu Detect Ping", LOG_DGB_NAME);
            return false;
        }

        return true;
    }
}

namespace YamahaCommon {
    static bool EcuPingSyncProcess(void) {
        bool ret;

        ret = YamahaCmd::EcuSyncStageFE();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncStagePing01();
        if (!ret) {
            return false;
        }

        return true;
    }

    static bool GetErrors(EsyPro::CommPacket_t *packet,
                          ErrorCommand *cmd, ErrorState_t *state,
                          EsyPro::CommunicationType_t commType) {
        static uint8_t errorsData[MAX_ECU_COMM_BUF_SIZE] = {0};
        static uint8_t curErrIdx, passErrIdx, errIdx;
        uint8_t totalErrors;

        switch (*state) {
        case SEND_ERROR_REQ:
            if (ecuObj.GetEcuPingType() == YamahaCommonObj::ECU_NORMAL_TYPE) {
                bool ret = YamahaCmd::GetErrorStatics(errorsData);
                if (!ret) {
                    return false;
                }
                *state = SEND_ERROR_RES;
            
            } else {
                *state = END_READING_ERROR;
            }
            
            curErrIdx = 0;
            passErrIdx = 0;
            errIdx = 0;
            packet->cmd = CMD_IGNORE_RES;
            cmd->SetCommandRepeatState(true);
            break;

        case SEND_ERROR_RES:
            totalErrors = errorsData[0];
            if (errIdx < totalErrors) {
                packet->bleUUID = CUSTOM_VALUE_ERRORS_CHAR_UUID;
                packet->bufLen = 4;
                packet->buffer[1] = errorsData[3 * errIdx + 1];
                packet->buffer[2] = errorsData[3 * errIdx + 2];
                packet->buffer[3] = errorsData[3 * errIdx + 3];
                if (errorsData[3 * errIdx + 3] == 0xA1) {
                    packet->cmd = CMD_ERROR_CUR_RES;
                    packet->buffer[0] = curErrIdx++;
                    if (commType == BLE_COMM_TYPE) {
                        packet->buffer[3] = BLE_CUR_ERROR_TYPE;
                    }
                } else {
                    packet->cmd = CMD_ERROR_PAST_RES;
                    packet->buffer[0] = passErrIdx++;
                    if (commType == BLE_COMM_TYPE) {
                        packet->buffer[3] = BLE_PAST_ERROR_TYPE;
                    }
                }
                errIdx++;                
            } else {
                *state = END_READING_ERROR;
                packet->cmd = CMD_IGNORE_RES;
            }
            cmd->SetCommandRepeatState(true);
            break;

        case END_READING_ERROR:
            packet->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            packet->cmd = CMD_END_ERROR_RES;
            packet->bufLen = 0;
            cmd->SetCommandRepeatState(false);
            break;
        }

        return true;
    }

    static bool GetAllInfo(uint8_t *data, uint8_t *dataLen) {
        uint8_t infoIdx[] = { 0x8A, 0x8C, 0x90, 0x92, 0x94,
                              0x95, 0x97, 0x98, 0x99, 0xA0 };
        uint8_t offset = 0;
        uint8_t totalLen = 0;

        for (unsigned int i = 0; i < sizeof(infoIdx); i++) {
            if (!YamahaCmd::GetInfo(infoIdx[i], &data[offset + 1], dataLen)) {
                return false;
            }
            data[offset] = *dataLen;
            offset += (1 + *dataLen);
            totalLen += (1 + *dataLen);
        }

        *dataLen = totalLen;
        return true;
    }

    static void DetectInfo(uint8_t *data, uint8_t *dataLen) {
        bool ret;
        data[0] = YamahaState_t::ECU_STATE_UNKNOWN;
        
        if (ecuObj.GetEcuPingType() == YamahaCommonObj::ECU_NORMAL_TYPE) {
            ret = GetAllInfo(&data[1], dataLen);
            if (!ret) {
                if (!YamahaCmd::StartCommunication()) {
                    *dataLen = 1;
                    return;
                }
                nrf_delay_ms(1);
                if (!GetAllInfo(&data[1], dataLen)) {
                    *dataLen = 1;
                    return;
                }
            }
            *dataLen += 1;
            data[0] = YamahaState_t::ECU_STATE_OK; 
        
        } else {
            *dataLen = 1;
            ret = YamahaCmd::CuJuDetectPing();      
            if (ret) {
                data[0] = YamahaState_t::ECU_STATE_OK; 
            }
        } 
    }

    static bool GetStatistics(uint8_t *dataLen, uint8_t statisticCmd, uint8_t *data) {
        if (YamahaCmd::GetTableStatistics(statisticCmd, data, dataLen)) {
            return true;
        }

        return false;
    }

    static bool GetAndEraseErrors(void) {
        bool ret;
        uint8_t totalErrors;
        uint8_t totalPastErr = 0, pastErrIdx = 0;
        uint8_t errorsData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = YamahaCmd::GetErrorStatics(errorsData);
        if (!ret) {
            return false;
        }

        totalErrors = errorsData[0];
        for (int i = 0; i < totalErrors; i++) {
            if (errorsData[(i * 3) + 3] != 0xA1) {
                memcpy(&errorsData[pastErrIdx], &errorsData[(i * 3) + 1], 3);
                pastErrIdx += 3;
                totalPastErr++;
            }
        }

        ret = YamahaCmd::EraseErrors(errorsData, totalPastErr);
        if (!ret) {
            return false;
        }

        return true;
    }

    void PingCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                              const EsyPro::CommPacket_t *commReqPacket,
                              EsyPro::CommunicationType_t commType) {
        uint8_t dataLen = 0;
        CommunicationType_t mainComm = GetMainCommunication();

        for (int i = 0; i < 4; i++) {
            commResPacket->buffer[i] = (NRF_FICR->DEVICEID[0] >> (8 * i)) & 0xFF;
            commResPacket->buffer[i + 4] = (NRF_FICR->DEVICEID[1] >> (8 * i)) & 0xFF;
        }

        memcpy(&commResPacket->buffer[8], DEVICE_VERSION, 3);
        if (mainComm == commType) {
            if ((EcuType_t)commReqPacket->buffer[0] != EcuType_t::ECU_YAMAHA) {
                ecuObj.SetEcuPingType(YamahaCommonObj::ECU_NORMAL_TYPE);
            }
            DetectInfo(&commResPacket->buffer[11], &dataLen);

        } else {
            dataLen = 1;
            commResPacket->buffer[11] = YamahaCommon::COMM_STATE_SUB;
        }

        commResPacket->bleUUID = CUSTOM_VALUE_PING_RES_CHAR_UUID;
        commResPacket->cmd = CMD_PING_RES;
        commResPacket->bufLen = dataLen + 11;
        this->SetCommandRepeatState(false);
    }

    void StatisticCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                   const EsyPro::CommPacket_t *commReqPacket,
                                   EsyPro::CommunicationType_t commType) {
        uint8_t dataLen;
        static int tableIdx = 0;

        if (tableIdx < totalStatisticTable) {
            bool ret;

            ret = GetStatistics(&dataLen, statisticTable[tableIdx], &commResPacket->buffer[1]);
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_STATISTICS_CHAR_UUID;
                commResPacket->cmd = CMD_STATISTIC_RES;
                commResPacket->bufLen = dataLen + 1;
                commResPacket->buffer[0] = tableIdx++;
                this->SetCommandRepeatState(true);
            } else {
                tableIdx = 0;
                commResPacket->cmd = CMD_IGNORE_RES;
                this->SetCommandRepeatState(false);
            }
        } else {
            tableIdx = 0;
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_END_STATISTIC_RES;
            commResPacket->bufLen = 0;
            this->SetCommandRepeatState(false);
        }
    }

    void ErrorCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                               const EsyPro::CommPacket_t *commReqPacket,
                               EsyPro::CommunicationType_t commType) {
        bool ret;

        if (!this->GetCommandRepeatState()) {
            errorState = SEND_ERROR_REQ;
        }

        ret = GetErrors(commResPacket, this, &errorState, commType);
        if (!ret) {
            commResPacket->cmd = CMD_IGNORE_RES;
            this->SetCommandRepeatState(false);
        }
    }

    void DiagnosticCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                    const EsyPro::CommPacket_t *commReqPacket,
                                    EsyPro::CommunicationType_t commType) {
        bool ret;
        uint8_t diagData[10];
        uint8_t dataLen;
        bool isActive = (commReqPacket->buffer[2]) ? 1 : 0;

        ret = YamahaCmd::GetDiagnosticsData(isActive, commReqPacket->buffer[1],
                                            diagData, &dataLen);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_STATISTICS_CHAR_UUID;
            commResPacket->cmd = CMD_DIAGNOSTIC_REQ;
            commResPacket->bufLen = dataLen;
            memcpy(commResPacket->buffer, diagData, dataLen);
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void EraseCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                               const EsyPro::CommPacket_t *commReqPacket,
                               EsyPro::CommunicationType_t commType) {
        bool ret = GetAndEraseErrors();

        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ERASE_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }
    
        this->SetCommandRepeatState(false);
    }
}

namespace Yamaha {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_YAMAHA_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_PING_REQ:
            cmd = &yamahaCommonPingCmd;
            NRF_LOG_INFO("[%s]: INFO: Ping Request", LOG_DGB_NAME);
            break;

        case CMD_CHECK_SYNC_REQ:
            cmd = &yamahaCheckSyncCmd;
            NRF_LOG_INFO("[%s]: INFO: Check Sync Request", LOG_DGB_NAME);
            break;

        case CMD_STATISTIC_REQ:
            cmd = &yamahaCommonStatisticCmd;
            NRF_LOG_INFO("[%s]: INFO: Statistics Request", LOG_DGB_NAME);
            break;

        case CMD_ERROR_REQ:
            cmd = &yamahaCommonErrorCmd;
            NRF_LOG_INFO("[%s]: INFO: Error Request", LOG_DGB_NAME);
            break;

        case CMD_DIAGNOSTIC_REQ:
            cmd = &yamahaCommonDiagnosticCommand;
            NRF_LOG_INFO("[%s]: INFO: Diagnostic Request", LOG_DGB_NAME);
            break;

        case CMD_ERASE_REQ:
            cmd = &yamahaCommonEraseCommand;
            NRF_LOG_INFO("[%s]: INFO: Erase Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    static bool EcuStatisticSyncProcess(void) {
        bool ret;

        ret = YamahaCmd::EcuSyncStageFE();
        if (!ret) {
            return false;
        }

        totalStatisticTable = 0;
        memset(statisticTable, 0, MAX_YAMAHA_SENSORS_TABLE);

        for (uint8_t i = 0x01; i <= 0xBF; i++) {
            ret = YamahaCmd::EcuSyncStageStatistic01(i);
            if (!ret) {
                return false;
            }
        }

        return true;
    }

    static bool EcuDiagSyncProcess(uint8_t *diagTable, uint8_t *tableSize) {
        bool ret;

        ret = YamahaCmd::EcuSyncStageFE();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncDiag01();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncDiag02();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncDiag03(diagTable, tableSize);
        if (!ret) {
            return false;
        }

        return true;
    }

    void CheckSyncCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                   const EsyPro::CommPacket_t *commReqPacket,
                                   EsyPro::CommunicationType_t commType) {
        bool ret;
        YamahaCommon::CheckSyncType_t syncType = 
                (YamahaCommon::CheckSyncType_t)(commReqPacket->buffer[1]);

        if (syncType == YamahaCommon::PING_SYNC_TYPE) {
            ret = YamahaCommon::EcuPingSyncProcess();
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->cmd = CMD_CHECK_SYNC_RES;
                commResPacket->bufLen = 0;
            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }

        } else if (syncType == YamahaCommon::STATISTIC_SYNC_TYPE) {
            ret = EcuStatisticSyncProcess();
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->cmd = CMD_CHECK_SYNC_RES;
                commResPacket->bufLen = 0;
            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }

        } else if (syncType == YamahaCommon::DIAG_SYNC_TYPE) {
            uint8_t diagTableLen;
            uint8_t diagTable[MAX_YAMAHA_DIAG_TABLE] = {0};

            ret = EcuDiagSyncProcess(diagTable, &diagTableLen);
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->cmd = CMD_CHECK_SYNC_RES;
                if (commType == BLE_COMM_TYPE) {
                    commResPacket->bufLen = diagTableLen + 1;
                    memcpy(&commResPacket->buffer[1], diagTable, diagTableLen);
                    commResPacket->buffer[0] = CMD_CHECK_SYNC_RES;
                } else {
                    commResPacket->bufLen = diagTableLen;
                    memcpy(commResPacket->buffer, diagTable, diagTableLen);
                }
            }
        }

        this->SetCommandRepeatState(false);
    }
}

namespace YamahaSMK {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_YAMAHA_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_PING_REQ:
            cmd = &yamahaCommonPingCmd;
            NRF_LOG_INFO("[%s]: INFO: SMK Ping Request", LOG_DGB_NAME);
            break;

        case CMD_CHECK_SYNC_REQ:
            cmd = &yamahaSMKCheckSyncCmd;
            NRF_LOG_INFO("[%s]: INFO: SMK Check Sync Request", LOG_DGB_NAME);
            break;

        case CMD_STATISTIC_REQ:
            cmd = &yamahaCommonStatisticCmd;
            NRF_LOG_INFO("[%s]: INFO: SMK Statistic Request", LOG_DGB_NAME);
            break;

        case CMD_ERROR_REQ:
            cmd = &yamahaCommonErrorCmd;
            NRF_LOG_INFO("[%s]: INFO: SMK Error Request", LOG_DGB_NAME);
            break;

        case CMD_DIAGNOSTIC_REQ:
            cmd = &yamahaCommonDiagnosticCommand;
            NRF_LOG_INFO("[%s]: INFO: Diagnostic Request", LOG_DGB_NAME);
            break;

        case CMD_ERASE_REQ:
            cmd = &yamahaCommonEraseCommand;
            NRF_LOG_INFO("[%s]: INFO: Erase Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    static bool EcuStatisticSyncProcess(void) {
        bool ret;

        ret = YamahaCmd::EcuSyncStageFE();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncStageSMK();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncStageFE();
        if (!ret) {
            return false;
        }

        totalStatisticTable = 0;
        memset(statisticTable, 0, MAX_YAMAHA_SENSORS_TABLE);

        for (uint8_t i = 0x01; i <= 0xBF; i++) {
            ret = YamahaCmd::EcuSyncStageStatistic01(i);
            if (!ret) {
                return false;
            }
        }

        return true;
    }

    static bool EcuDiagSyncProcess(uint8_t *diagTable, uint8_t *tableSize) {
        bool ret;

        ret = YamahaCmd::EcuSyncStageFE();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncStageSMK();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncStageFE();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncDiag01();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncDiag02();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncDiag03(diagTable, tableSize);
        if (!ret) {
            return false;
        }

        return true;
    }

    void CheckSyncCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                   const EsyPro::CommPacket_t *commReqPacket,
                                   EsyPro::CommunicationType_t commType) {
        bool ret;
        YamahaCommon::CheckSyncType_t syncType = 
                (YamahaCommon::CheckSyncType_t)(commReqPacket->buffer[1]);

        if (syncType == YamahaCommon::PING_SYNC_TYPE) {
            ret = YamahaCommon::EcuPingSyncProcess();
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->cmd = CMD_CHECK_SYNC_RES;
                commResPacket->bufLen = 0;
            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }

        } else if (syncType == YamahaCommon::STATISTIC_SYNC_TYPE) {
            ret = EcuStatisticSyncProcess();
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->cmd = CMD_CHECK_SYNC_RES;
                commResPacket->bufLen = 0;
            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }

        } else if (syncType == YamahaCommon::DIAG_SYNC_TYPE) {
            uint8_t diagTableLen;
            uint8_t diagTable[MAX_YAMAHA_DIAG_TABLE] = {0};

            ret = EcuDiagSyncProcess(diagTable, &diagTableLen);
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->cmd = CMD_CHECK_SYNC_RES;
                if (commType == BLE_COMM_TYPE) {
                    commResPacket->bufLen = diagTableLen + 1;
                    memcpy(&commResPacket->buffer[1], diagTable, diagTableLen);
                    commResPacket->buffer[0] = CMD_CHECK_SYNC_RES;
                } else {
                    commResPacket->bufLen = diagTableLen;
                    memcpy(commResPacket->buffer, diagTable, diagTableLen);
                }

            }
        }

        this->SetCommandRepeatState(false);
    }
}

namespace YamahaObd2 {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_YAMAHA_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_PING_REQ:
            cmd = &yamahaCommonPingCmd;
            NRF_LOG_INFO("[%s]: INFO: OBD2 Ping Request", LOG_DGB_NAME);
            break;

        case CMD_ERROR_REQ:
            cmd = &yamahaCommonErrorCmd;
            NRF_LOG_INFO("[%s]: INFO: OBD2 Error Request", LOG_DGB_NAME);
            break;

        case CMD_ERASE_REQ:
            cmd = &yamahaCommonEraseCommand;
            NRF_LOG_INFO("[%s]: INFO: OBD2 Erase Request", LOG_DGB_NAME);
            break;

        case CMD_STATISTIC_REQ:
            cmd = &yamahaObd2StatisticCmd;
            NRF_LOG_INFO("[%s]: INFO: OBD2 Statistic Request", LOG_DGB_NAME);
            break;

        case CMD_CHECK_SYNC_REQ:
           cmd = &yamahaObd2CheckSyncCmd;
           NRF_LOG_INFO("[%s]: INFO: OBD2 Diagnostic Sync Request", LOG_DGB_NAME);
           break;

        case CMD_DIAGNOSTIC_REQ:
           cmd = &yamahaObd2DiagnosticCommand;
           NRF_LOG_INFO("[%s]: INFO: Diagnostic Request", LOG_DGB_NAME);
           break;
        }

        return cmd;
    }

    static bool EcuDiagSyncProcess(uint8_t *diagTable, uint8_t *tableSize) {
        bool ret;

        ret = YamahaCmd::EcuActiveDiagObd2();
        if (!ret) {
            return false;
        }

        ret = YamahaCmd::EcuSyncDiagObd2(diagTable, tableSize);
        if (!ret) {
            return false;
        }

        return true;
    }

    void StatisticCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                   const EsyPro::CommPacket_t *commReqPacket,
                                   EsyPro::CommunicationType_t commType) {
        uint8_t dataLen;
        static int tableIdx = 0;

        if (tableIdx < MAX_YAMAHA_OBD2_TABLE) {
            bool ret;

            ret = YamahaCmd::GetStatisticObd2(&commResPacket->buffer[1], &dataLen);
            if (ret) {
                commResPacket->buffer[0] = tableIdx++;
                commResPacket->bleUUID = CUSTOM_VALUE_STATISTICS_CHAR_UUID;
                commResPacket->bufLen = dataLen + 1;
                commResPacket->cmd = CMD_STATISTIC_RES;
                this->SetCommandRepeatState(true);
            } else {
                tableIdx = 0;
                commResPacket->cmd = CMD_IGNORE_RES;
                this->SetCommandRepeatState(false);
            }

        } else {
            tableIdx = 0;
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_END_STATISTIC_RES;
            commResPacket->bufLen = 0;
            this->SetCommandRepeatState(false);
        }
    }

    void CheckSyncCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                   const EsyPro::CommPacket_t *commReqPacket,
                                   EsyPro::CommunicationType_t commType) {
        YamahaCommon::CheckSyncType_t syncType = 
                (YamahaCommon::CheckSyncType_t)(commReqPacket->buffer[1]);

        if (syncType == YamahaCommon::DIAG_SYNC_TYPE) {
            uint8_t diagTableLen;
            uint8_t diagTable[MAX_YAMAHA_DIAG_TABLE] = {0};

            bool ret = EcuDiagSyncProcess(diagTable, &diagTableLen);
            if (ret) {
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->cmd = CMD_CHECK_SYNC_RES;
                if (commType == BLE_COMM_TYPE) {
                    commResPacket->bufLen = diagTableLen + 1;
                    memcpy(&commResPacket->buffer[1], diagTable, diagTableLen);
                    commResPacket->buffer[0] = CMD_CHECK_SYNC_RES;
                } else {
                    commResPacket->bufLen = diagTableLen;
                    memcpy(commResPacket->buffer, diagTable, diagTableLen);
                }

            } else {
                commResPacket->cmd = CMD_IGNORE_RES;
            }
        }

        this->SetCommandRepeatState(false);
    }

    void DiagnosticCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                    const EsyPro::CommPacket_t *commReqPacket,
                                    EsyPro::CommunicationType_t commType) {
        bool ret;
        uint8_t diagData[10];
        uint8_t dataLen;
        bool isActive = (commReqPacket->buffer[2]) ? 1 : 0;

        ret = YamahaCmd::GetDiagnosticsObd2Data(isActive, commReqPacket->buffer[1],
                                               diagData, &dataLen);
        if (ret) {
           commResPacket->bleUUID = CUSTOM_VALUE_STATISTICS_CHAR_UUID;
           commResPacket->cmd = CMD_DIAGNOSTIC_REQ;
           commResPacket->bufLen = dataLen;
           memcpy(commResPacket->buffer, diagData, dataLen);
        } else {
           commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }
}
