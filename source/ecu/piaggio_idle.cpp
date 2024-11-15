#include "ecu/piaggio_idle.h"
#include "ble_common.h"

using namespace EsyPro;
using namespace ECU;

static const char LOG_DGB_NAME[] = "piaggio_idle";

static uint8_t miuActuatorID[] = {0x02, 0x26, 0x7C, 0x08, 0x11, 0x0D, 0x7B, 0x05};
static uint8_t pgActuatorID[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

static PiaggioCommon::ErrorState_t errorState = PiaggioCommon::ErrorState_t::SEND_ERROR_REQ;

static PiaggioCommon::ErrorCommand piaggioErrorCmd;

static PiaggioMIU::PingCommand piaggioMIUPingCmd;
static PiaggioMIU::StatisticCommand piaggioMIUStatisticCmd;
static PiaggioMIU::EraseCommand piaggioMIUEraseCmd;
static PiaggioMIU::ResetCommand piaggioMIUResetCmd;
static PiaggioMIU::ActuatorCommand piaggioMIUActuatorCmd;
static PiaggioMIU::ResetParamsCommand piaggioMIUResetParamsCmd;
static PiaggioMIU::BackupParamsCommand piaggioMIUBackupParamsCmd;

static PiaggioPG::PingCommand piaggioPGPingCmd;
static PiaggioPG::StatisticCommand piaggioPGStatisticCmd;
static PiaggioPG::EraseCommand piaggioPGEraseCmd;
static PiaggioPG::ResetCommand piaggioPGResetCmd;
static PiaggioPG::ActuatorCommand piaggioPGActuatorCmd;
static PiaggioPG::ResetParamsCommand piaggioPGResetParamsCmd;

static PiaggioCommonObj ecuObj;

namespace PiaggioCmd {
    static bool TranscieveObd2Packet(uint8_t *srcBuf, uint8_t bufLen, 
                                     uint8_t *tmpData, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[3] = {0x00, PIAGGIO_TARGET_BYTE, PIAGGIO_SOURCE_BYTE};
        
        srcCmd[0] = 0x80 | bufLen;
        ecuObj.SetReqEcuPacket(srcCmd, 3, srcBuf, bufLen);
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        ecuObj.GetEcuPacketResData(tmpData, dataLen, 0);
        if (!ret || (tmpData[0] != (srcBuf[0] | 0x40))) {
            return false;
        }

        return true;        
    }

    static bool StartCommunication(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x81};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ecuObj.InitKlineBus();
        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Start Communication", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool GetInfo(uint8_t idx, uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcBuf[] = {0x1A, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcBuf[1] = idx;
        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Get ECU Info", LOG_DGB_NAME);
            return false;
        }
        *dataLen -= 2;
        memcpy(data, &tmpData[2], *dataLen);

        return true;
    }

    static bool GetTableStatistics(uint8_t idx, uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcBuf[] = {0x21, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcBuf[1] = idx;
        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, dataLen);
        if (!ret) {
            if (tmpData[0] == 0x7F) {
                NRF_LOG_INFO("[%s]: ERROR: Get Statistics: %x", LOG_DGB_NAME, tmpData[0]);
                *dataLen = 0;
            } else {
                NRF_LOG_INFO("[%s]: ERROR: Get Statistics", LOG_DGB_NAME);
                return false;
            }

        } else {
            *dataLen -= 2;
            memcpy(data, &tmpData[2], *dataLen);
            data[*dataLen] = idx;
            *dataLen += 1;
        }

        return true;
    }

    static bool GetErrorStatics(uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcBuf[] = {0x18, 0x00, 0xFF, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Get Errors", LOG_DGB_NAME);
            return false;
        }
        *dataLen -= 2;
        data[0] = tmpData[1];
        memcpy(&data[1], &tmpData[2], *dataLen);

        return true;
    }

    static bool EraseErrors(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x14, 0xFF, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Erase Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool EraseMIUMemoryError(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x14, 0xFF, 0x24};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Erase MIU Memory Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool ResetMIUTPS(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf1[] = {0x31, 0x21};
        uint8_t srcBuf2[] = {0x33, 0x21};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = TranscieveObd2Packet(srcBuf1, sizeof(srcBuf1), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Reset MIU TPS-1 Errors", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(3000);

        ret = TranscieveObd2Packet(srcBuf2, sizeof(srcBuf2), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Reset MIU TPS-2 Errors", LOG_DGB_NAME);
            return false;
        } 
    
        return true;
    }

    static bool ResetPGZeroTPS(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf1[] = {0x31, 0x01};
        uint8_t srcBuf2[] = {0x33, 0x01};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = TranscieveObd2Packet(srcBuf1, sizeof(srcBuf1), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Reset PG-Zero TPS-1 Errors", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(1000);

        ret = TranscieveObd2Packet(srcBuf2, sizeof(srcBuf2), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Reset PG-Zero TPS-2 Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool ResetPGMaxTPS(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf1[] = {0x31, 0x02};
        uint8_t srcBuf2[] = {0x33, 0x02};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = TranscieveObd2Packet(srcBuf1, sizeof(srcBuf1), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Reset PG-Max TPS-1 Errors", LOG_DGB_NAME);
            return false;
        }
        nrf_delay_ms(1000);

        ret = TranscieveObd2Packet(srcBuf2, sizeof(srcBuf2), tmpData, &dataLen);
        if (!ret || (tmpData[2] != 0xFF)) {
            NRF_LOG_INFO("[%s]: ERROR: Reset PG-Max TPS-2 Errors", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool EnableActuator(uint8_t id, PiaggioCommon::ActuatorMode_t mode) {
        bool ret = false;
        uint8_t dataLen;
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        if (mode == PiaggioCommon::MODE_ON) {
            uint8_t srcBuf1[] = {0x30, 0x00, 0x07};

            srcBuf1[1] = id;
            ret = TranscieveObd2Packet(srcBuf1, sizeof(srcBuf1), tmpData, &dataLen);
        } else {
            uint8_t srcBuf2[] = {0x30, 0x00, 0x00};

            srcBuf2[1] = id;
            ret = TranscieveObd2Packet(srcBuf2, sizeof(srcBuf2), tmpData, &dataLen);
        }

        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Enable Actuator. Mode = %d", LOG_DGB_NAME, mode);
            return false;
        }

        return true;
    }

    static bool ResetMIUParams(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x30, 0x7E, 0x04};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Reset MIU Params", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool ResetPGParams(void) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x30, 0xFE, 0x04};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Reset PG Params", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool BackupMiuParams(uint8_t *totalMessages) {
        bool ret;
        uint8_t dataLen;
        uint8_t srcBuf[] = {0x31, 0x24};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, &dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Backup MIU Params-1", LOG_DGB_NAME);
            return false;
        }
        *totalMessages = tmpData[2];

        return true;
    }

    static bool BackupMiuParams(uint8_t *dataLen, uint8_t *data, uint8_t idx) {
        bool ret;
        uint8_t srcBuf[] = {0x33, 0x24, 0x00};
        uint8_t tmpData[MAX_ECU_COMM_BUF_SIZE] = {0};

        srcBuf[2] = idx;
        ret = TranscieveObd2Packet(srcBuf, sizeof(srcBuf), tmpData, dataLen);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Backup MIU Params-2.%d", LOG_DGB_NAME, idx);
            return false;
        }
        *dataLen -= 3;
        memcpy(data, &tmpData[3], *dataLen);
        nrf_delay_ms(1);

        return true;
    }
}

namespace PiaggioCommon {
    static bool GetErrors(EsyPro::CommPacket_t *packet,
                          ErrorCommand *cmd, ErrorState_t *state,
                          EsyPro::CommunicationType_t commType) {
        bool ret;
        static uint8_t errorsData[MAX_ECU_COMM_BUF_SIZE];
        static uint8_t curErrIdx, passErrIdx, errIdx;
        uint8_t dataLen, totalErrors;

        switch (*state) {
        case SEND_ERROR_REQ:
            ret = PiaggioCmd::GetErrorStatics(errorsData, &dataLen);
            if (!ret) {
                return false;
            }
            *state = SEND_ERROR_RES;
            curErrIdx = 0;
            passErrIdx = 0;
            errIdx = 0;
            packet->cmd = CMD_IGNORE_RES;
            cmd->SetCommandRepeatState(true);
            break;

        case SEND_ERROR_RES:
            totalErrors = errorsData[0];
            if (errIdx < totalErrors) {
                packet->bufLen = 3;
                packet->bleUUID = CUSTOM_VALUE_ERRORS_CHAR_UUID;
                packet->buffer[1] = errorsData[3 * errIdx + 1];
                packet->buffer[2] = errorsData[3 * errIdx + 2];
                if (errorsData[3 * errIdx + 3] & 0x40) {
                    packet->cmd = CMD_ERROR_CUR_RES;
                    packet->buffer[0] = curErrIdx++;
                    if (commType == BLE_COMM_TYPE) {
                        packet->bufLen = 4;
                        packet->buffer[3] = BLE_CUR_ERROR_TYPE;
                    }
                    
                } else {
                    packet->cmd = CMD_ERROR_PAST_RES;
                    packet->buffer[0] = passErrIdx++;
                    if (commType == BLE_COMM_TYPE) {
                        packet->bufLen = 4;
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
}

namespace PiaggioMIU {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_PIAGGIO_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_PING_REQ:
            cmd = &piaggioMIUPingCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Ping Request", LOG_DGB_NAME);
            break;

        case CMD_STATISTIC_REQ:
            cmd = &piaggioMIUStatisticCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Statistic Request", LOG_DGB_NAME);
            break;

        case CMD_ERROR_REQ:
            cmd = &piaggioErrorCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Error Request", LOG_DGB_NAME);
            break;

        case CMD_ERASE_REQ:
            cmd = &piaggioMIUEraseCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Erase Request", LOG_DGB_NAME);
            break;

        case CMD_RESET_REQ:
            cmd = &piaggioMIUResetCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Reset TPS Request", LOG_DGB_NAME);
            break;

        case CMD_EN_ACTUATOR_REQ:
            cmd = &piaggioMIUActuatorCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Enable Actuator Request", LOG_DGB_NAME);
            break;

        case CMD_RESET_PARAMS_REQ:
            cmd = &piaggioMIUResetParamsCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Reset Params Request", LOG_DGB_NAME);
            break;

        case CMD_BACKUP_PARAMS_REQ:
            cmd = &piaggioMIUBackupParamsCmd;
            NRF_LOG_INFO("[%s]: INFO: MIU Backup Params Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    static bool GetAllInfo(uint8_t *data, uint8_t *dataLen) {
        uint8_t infoIdx[] = {0x91, 0x92, 0x94, 0x96, 0x97, 0x98, 0x99};
        uint8_t offset = 0;
        uint8_t totalLen = 0;

        for (unsigned int i = 0; i < sizeof(infoIdx); i++) {
            if (!PiaggioCmd::GetInfo(infoIdx[i], &data[offset + 1], dataLen)) {
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
        data[0] = PiaggioCommon::ECU_STATE_UNKNOWN;
        
        ret = GetAllInfo(&data[1], dataLen);
        if (!ret) {
            if (!PiaggioCmd::StartCommunication()) {
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
        data[0] = PiaggioCommon::ECU_STATE_OK;
    }

    static bool GetStatistics(uint8_t *dataLen, uint8_t tableIdx, uint8_t *data) {
        uint8_t statisticCmd[MAX_PIAGGIO_MIU_SENSORS_TABLE] =
            { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x37, 0x39, 0x3A, 0x3C,
              0x45, 0x47, 0x49, 0x4B, 0x53, 0x54, 0x57, 0x58, 0x5C, 0x60,
              0x62, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x70, 0x71, 0x72, 0x74,
              0x75, 0x76, 0x77, 0x79, 0x7A, 0x7B, 0x7D };

        if (PiaggioCmd::GetTableStatistics(statisticCmd[tableIdx], data, dataLen)) {
            return true;
        }

        return false;
    }

    static bool GetBackupParams(BackupParamsCommand *cmd,
                                uint8_t *dataLen, uint8_t *data) {
        static uint8_t idx = 0x01;
        static uint8_t totalMessages = 0;

        if (idx == 0x01) {
            if (!PiaggioCmd::BackupMiuParams(&totalMessages)) {
                idx == 0x01;
                return false;
            }
            cmd->SetCommandRepeatState(true);
        }

        if (idx <= totalMessages) {
            if (!PiaggioCmd::BackupMiuParams(dataLen, data, idx)) {
                idx == 0x01;
                return false;
            }
            idx++;
            cmd->SetCommandRepeatState(true);
        } else {
            *dataLen = 0;
            idx = 0x01;
            cmd->SetCommandRepeatState(false);
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
            DetectInfo(&commResPacket->buffer[11], &dataLen);
        } else {
            dataLen = 1;
            commResPacket->buffer[11] = PiaggioCommon::COMM_STATE_SUB;
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

        if (tableIdx < MAX_PIAGGIO_MIU_SENSORS_TABLE) {
            bool ret;

            ret = GetStatistics(&dataLen, tableIdx, &commResPacket->buffer[1]);
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

    void EraseCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                               const EsyPro::CommPacket_t *commReqPacket,
                               EsyPro::CommunicationType_t commType) {
        bool ret = false;
        ErrorEraseType_t eraseType = (ErrorEraseType_t)commReqPacket->buffer[1];

        if (eraseType == ERROR_TYPE_1) {
            ret = PiaggioCmd::EraseErrors();
        } else if (eraseType == ERROR_TYPE_2) {
            ret = PiaggioCmd::EraseMIUMemoryError();
        }

        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ERASE_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void ResetCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                               const EsyPro::CommPacket_t *commReqPacket,
                               EsyPro::CommunicationType_t commType) {
        bool ret;

        ret = PiaggioCmd::ResetMIUTPS();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_RESET_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void ActuatorCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                  const EsyPro::CommPacket_t *commReqPacket,
                                  EsyPro::CommunicationType_t commType) {
        bool ret;
        uint8_t id = miuActuatorID[commReqPacket->buffer[1]];
        PiaggioCommon::ActuatorMode_t mode = 
                        (PiaggioCommon::ActuatorMode_t)commReqPacket->buffer[2];

        ret = PiaggioCmd::EnableActuator(id, mode);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_EN_ACTUATOR_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void ResetParamsCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                     const EsyPro::CommPacket_t *commReqPacket,
                                     EsyPro::CommunicationType_t commType) {
        bool ret;

        ret = PiaggioCmd::ResetMIUParams();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_RESET_PARAMS_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void BackupParamsCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                     const EsyPro::CommPacket_t *commReqPacket,
                                     EsyPro::CommunicationType_t commType) {
        bool ret;
        uint8_t dataLen;

        ret = GetBackupParams(this, &dataLen, commResPacket->buffer);
        if (ret) {
            commResPacket->bufLen = dataLen;
            commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
            commResPacket->cmd = CMD_BACKUP_PARAMS_RES;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
            this->SetCommandRepeatState(false);
        }
    }
}

namespace PiaggioPG {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_PIAGGIO_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_PING_REQ:
            cmd = &piaggioPGPingCmd;
            NRF_LOG_INFO("[%s]: INFO: PG Ping Request", LOG_DGB_NAME);
            break;

        case CMD_STATISTIC_REQ:
            cmd = &piaggioPGStatisticCmd;
            NRF_LOG_INFO("[%s]: INFO: PG Statistic Request", LOG_DGB_NAME);
            break;

        case CMD_ERROR_REQ:
            cmd = &piaggioErrorCmd;
            NRF_LOG_INFO("[%s]: INFO: PG Error Request", LOG_DGB_NAME);
            break;

        case CMD_ERASE_REQ:
            cmd = &piaggioPGEraseCmd;
            NRF_LOG_INFO("[%s]: INFO: PG Erase Request", LOG_DGB_NAME);
            break;

        case CMD_RESET_REQ:
            cmd = &piaggioPGResetCmd;
            NRF_LOG_INFO("[%s]: INFO: PG Reset TPS Request", LOG_DGB_NAME);
            break;

        case CMD_EN_ACTUATOR_REQ:
            cmd = &piaggioPGActuatorCmd;
            NRF_LOG_INFO("[%s]: INFO: PG Enable Actuator Request", LOG_DGB_NAME);
            break;

        case CMD_RESET_PARAMS_REQ:
            cmd = &piaggioPGResetParamsCmd;
            NRF_LOG_INFO("[%s]: INFO: PG Reset Params Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    static bool GetAllInfo(uint8_t *data, uint8_t *dataLen) {
        uint8_t infoIdx[] = {0x91, 0x92, 0x94, 0x95, 0x96, 0x98, 0x9A, 0x9B};
        uint8_t offset = 0;
        uint8_t totalLen = 0;

        for (unsigned int i = 0; i < sizeof(infoIdx); i++) {
            if (!PiaggioCmd::GetInfo(infoIdx[i], &data[offset + 1], dataLen)) {
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

        data[0] = PiaggioCommon::ECU_STATE_UNKNOWN;        
        ret = GetAllInfo(&data[1], dataLen);
        if (!ret) {
            if (!PiaggioCmd::StartCommunication()) {
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
        data[0] = PiaggioCommon::ECU_STATE_OK;
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
            DetectInfo(&commResPacket->buffer[11], &dataLen);
        } else {
            commResPacket->buffer[11] = PiaggioCommon::COMM_STATE_SUB;
        }
        commResPacket->bleUUID = CUSTOM_VALUE_PING_RES_CHAR_UUID;
        commResPacket->cmd = CMD_PING_RES;
        commResPacket->bufLen = dataLen + 11;
        this->SetCommandRepeatState(false);    
    }

    static bool GetStatistics(uint8_t *dataLen, uint8_t tableIdx, uint8_t *data) {
        uint8_t statisticCmd[MAX_PIAGGIO_PG_SENSORS_TABLE] =
            { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
              0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,
              0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 
              0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x26, 0x27, 0x28,
              0x29, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31,
              0x32, 0x33, 0x34, 0x39 };

        if (PiaggioCmd::GetTableStatistics(statisticCmd[tableIdx], data, dataLen)) {
            return true;
        }

        return false;
    }

    void StatisticCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                   const EsyPro::CommPacket_t *commReqPacket,
                                   EsyPro::CommunicationType_t commType) {
        uint8_t dataLen;
        static int tableIdx = 0;

        if (tableIdx < MAX_PIAGGIO_PG_SENSORS_TABLE) {
            bool ret;

            ret = GetStatistics(&dataLen, tableIdx, &commResPacket->buffer[1]);
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

    void EraseCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                               const EsyPro::CommPacket_t *commReqPacket,
                               EsyPro::CommunicationType_t commType) {
        bool ret = false;

        ret = PiaggioCmd::EraseErrors();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ERASE_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }
    
        this->SetCommandRepeatState(false);
    }

    void ResetCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                               const EsyPro::CommPacket_t *commReqPacket,
                               EsyPro::CommunicationType_t commType) {
        bool ret = false;
        TPSResetType_t resetType = (TPSResetType_t)commReqPacket->buffer[1];

        if (resetType == ZERO_THROTTLE) {
            ret = PiaggioCmd::ResetPGZeroTPS();
        } else if (resetType == MAX_THROTTLE) {
            ret = PiaggioCmd::ResetPGMaxTPS();
        }

        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_RESET_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }
    
        this->SetCommandRepeatState(false);
    }

    void ActuatorCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                  const EsyPro::CommPacket_t *commReqPacket,
                                  EsyPro::CommunicationType_t commType) {
        bool ret;
        uint8_t id = pgActuatorID[commReqPacket->buffer[1]];
        PiaggioCommon::ActuatorMode_t mode = 
                        (PiaggioCommon::ActuatorMode_t)commReqPacket->buffer[2];

        ret = PiaggioCmd::EnableActuator(id, mode);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_EN_ACTUATOR_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void ResetParamsCommand::Execute(EsyPro::CommPacket_t *commResPacket,
                                     const EsyPro::CommPacket_t *commReqPacket,
                                     EsyPro::CommunicationType_t commType) {
        bool ret;

        ret = PiaggioCmd::ResetPGParams();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_RESET_PARAMS_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }
    
        this->SetCommandRepeatState(false);
    }
}