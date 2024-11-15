#include "ecu/honda_idle.h"
#include "ble_common.h"

using namespace EsyPro;
using namespace ECU;

static const char LOG_DGB_NAME[] = "honda_idle";

static HondaCommon::PingCommand hondaCommonPingCmd;

static HondaPgmfi::StatisticCommand hondaPgmfiStatisticCmd;
static HondaPgmfi::ErrorCommand hondaPgmfiErrorCmd;
static HondaPgmfi::EraseCommand hondaPgmfiEraseCmd;
static HondaPgmfi::ResetCommand hondaPgmfiResetCmd;

static HondaAbs::StatisticCommand hondaAbsStatisticCmd;
static HondaAbs::ErrorCommand hondaAbsErrorCmd;
static HondaAbs::EraseCommand hondaAbsEraseCmd;

static HondaDCT::StatisticCommand hondaDctStatisticCmd;
static HondaDCT::ErrorCommand hondaDctErrorCmd;
static HondaCommonObj ecuObj;

namespace HondaCmd {
    static bool AbsWakeup(void) {
        bool ret;
        uint8_t srcCmd[] = {0xFE};
        uint8_t srcBuffer[] = {0xFF};

        ecuObj.InitKlineBus();
        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();

        return ret;
    }

    static bool IsAbsStateOK(uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[] = {0x72};
        uint8_t srcBuffer[] = {0x94, 0x72, 0x00, 0x00, 0x05};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));        
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: ABS State OK", LOG_DGB_NAME);
        }
        ecuObj.GetEcuPacketResData(data, dataLen, 4);
        return ret;
    }

    static bool PgmfiWakeup(void) {
        bool ret;
        uint8_t srcCmd[] = {0xFE};
        uint8_t srcBuffer[] = {0x72};

        ecuObj.InitKlineBus();
        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();

        return ret;  
    }

    static bool IsPgmfiStateOK(uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[] = {0x72};
        uint8_t srcBuffer[] = {0x71, 0x00};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: PGM-FI State OK", LOG_DGB_NAME);
        }
        ecuObj.GetEcuPacketResData(data, dataLen, 2);

        return ret;
    }

    static bool IsPgmfiStateRecover(void) {
        bool ret;
        uint8_t srcCmd[] = {0x7D};
        uint8_t srcBuffer[] = {0x01, 0x01, 0x03};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: PGM-FI State Recover", LOG_DGB_NAME);
        }

        return ret;
    }

    static bool IsPgmfiStateRead(void) {
        bool ret;
        uint8_t srcCmd[] = {0x82, 0x82, 0x10};
        uint8_t srcBuffer[] = {0x00};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: PGM-FI State Read", LOG_DGB_NAME);
        }

        return ret;
    }

    static bool IsPgmfiStateWrite(void) {
        bool ret;
        uint8_t srcCmd[] = {0x7E};
        uint8_t srcBuffer[] = {0x01, 0x01, 0x00};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: PGM-FI State Write", LOG_DGB_NAME);
        }

        return ret;
    }

    static bool PgmfiGetStatistics(uint8_t *srcBuffer, uint8_t bufLen,
                                   uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[] = {0x72};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, bufLen);
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: PGM-FI Get Statistics", LOG_DGB_NAME);
        }
        ecuObj.GetEcuPacketResData(data, dataLen, 2);
        *dataLen -= 2;

        return ret;
    }

    static bool AbsGetStatistics(uint8_t *srcBuffer, uint8_t bufLen,
                                 uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[] = {0x72};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, bufLen);
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: ABS Get Statistics", LOG_DGB_NAME);
        }
        ecuObj.GetEcuPacketResData(data, dataLen, 3);
        *dataLen -= 3;

        return ret;
    }

    static bool DctGetStatistics(uint8_t *srcBuffer, uint8_t bufLen,
                                 uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[] = {0x72};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, bufLen);
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: DCT Get Statistics", LOG_DGB_NAME);
        }
        ecuObj.GetEcuPacketResData(data, dataLen, 3);
        *dataLen -= 3;

        return ret;
    }

    static bool PgmfiGetErrors(uint8_t *srcBuffer, uint8_t bufLen,
                               uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[] = {0x72};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, bufLen);
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: PGM-FI Get Errors", LOG_DGB_NAME);
        }
        ecuObj.GetEcuPacketResData(data, dataLen, 3);
        *dataLen -= 3;

        return ret;
    }

    static bool AbsGetErrors(uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[] = {0x72};
        uint8_t srcBuffer[] = {0x94, 0x73, 0x00};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: ABS Get Errors", LOG_DGB_NAME);
        }
        ecuObj.GetEcuPacketResData(data, dataLen, 3);
        *dataLen -= 3;

        return ret;
    }

    static bool DctGetErrors(uint8_t *srcBuffer, uint8_t bufLen,
                             uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[] = {0x72};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, bufLen);
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: DCT Get Errors", LOG_DGB_NAME);
        }
        ecuObj.GetEcuPacketResData(data, dataLen, 3);
        *dataLen -= 3;

        return ret;
    }

    static bool PgmfiErase(void) {
        uint8_t srcCmd[] = {0x72};
        uint8_t srcBuffer[] = {0x60, 0x01};
        uint8_t eraseDoneFlag[5];
        uint8_t dataLen;

        do {
            bool ret;

            ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: PGM-FI Erase", LOG_DGB_NAME);
                return false;
            }
            ecuObj.GetEcuPacketResData(&eraseDoneFlag[0], &dataLen, 1);
        } while (eraseDoneFlag[0] != 0x00);

        return true;
    }

    static bool AbsErase(void) {
        bool ret;
        uint8_t srcCmd_1[] = {0x72};
        uint8_t srcBuffer_1[] = {0x60, 0x01};
        uint8_t srcCmd_2[] = {0xFE};
        uint8_t srcBuffer_2[] = {0x72, 0x94};
        uint8_t srcCmd_3[] = {0x72};
        uint8_t srcBuffer_3[] = {0x60, 0x00};        
        uint8_t eraseDoneFlag[5];
        uint8_t dataLen;

        ecuObj.SetReqEcuPacket(srcCmd_1, sizeof(srcCmd_1), srcBuffer_1, sizeof(srcBuffer_1));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: ABS Do Erase-1", LOG_DGB_NAME);
            return false;
        }

        ecuObj.InitKlineBus();
        ecuObj.SetReqEcuPacket(srcCmd_2, sizeof(srcCmd_2), srcBuffer_2, sizeof(srcBuffer_2));
        ecuObj.SendRequestToEcu();
        ecuObj.ReceiveRequestFromEcu();

        do {
            ecuObj.SetReqEcuPacket(srcCmd_3, sizeof(srcCmd_3), srcBuffer_3, sizeof(srcBuffer_3));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: ABS Do Erase-3", LOG_DGB_NAME);
                return false;
            }
            ecuObj.GetEcuPacketResData(&eraseDoneFlag[0], &dataLen, 1);
        } while (eraseDoneFlag[0] != 0x00);

        return true;
    }

    static bool PgmfiReset(void) {
        uint8_t srcCmd[] = {0x72};
        uint8_t srcBuffer[] = {0x60, 0x03};
        uint8_t eraseDoneFlag[5];
        uint8_t dataLen;

        do {
            bool ret;

            ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, sizeof(srcBuffer));
            ecuObj.SendRequestToEcu();
            ret = ecuObj.ReceiveRequestFromEcu();
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: PGM-FI Do Reset", LOG_DGB_NAME);
                return false;
            }
            ecuObj.GetEcuPacketResData(&eraseDoneFlag[0], &dataLen, 1);
        } while (eraseDoneFlag[0] != 0x00);

        return true;
    }
}

namespace HondaCommon {
    static void AbsDetectState(uint8_t *data) {
        bool ret;
        uint8_t dataLen;
        uint8_t tmpBuffer[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = HondaCmd::IsAbsStateOK(tmpBuffer, &dataLen);
        if (!ret) {
            HondaCmd::AbsWakeup();
            ret = HondaCmd::IsAbsStateOK(tmpBuffer, &dataLen);
        }
        if (ret) {
            memcpy(data, tmpBuffer, 5);
            data[5] = ECU_STATE_OK;
        } else {
            data[5] = ECU_STATE_UNKNOWN;
        }
    }

    static void PgmfiDetectState(uint8_t *data) {
        bool ret;
        uint8_t dataLen;
        uint8_t tmpBuffer[MAX_ECU_COMM_BUF_SIZE] = {0};

        ret = HondaCmd::IsPgmfiStateOK(tmpBuffer, &dataLen);
        if (!ret) {
            if (HondaCmd::PgmfiWakeup()) {
                ret = HondaCmd::IsPgmfiStateOK(tmpBuffer, &dataLen);
            } else {
                data[5] = ECU_STATE_UNKNOWN;
            }
        }
        if (ret) {
            memcpy(data, tmpBuffer, 5);

            if (tmpBuffer[3] == 0x00 && tmpBuffer[4] == 0x00) {
                data[5] = ECU_STATE_WRITE;
                if (HondaCmd::IsPgmfiStateRecover()) {
                    data[5] = ECU_STATE_RECOVER;
                }
            } else {
                data[5] = ECU_STATE_OK;
            }

        } else if (HondaCmd::IsPgmfiStateRead()) {
            data[5] = ECU_STATE_READ;

        } else if (HondaCmd::IsPgmfiStateWrite()) {
            data[5] = ECU_STATE_WRITE;
        }
    }

    void PingCommand::Execute(CommPacket_t *commResPacket,
                              const CommPacket_t *commReqPacket,
                              CommunicationType_t commType) {
        CommunicationType_t mainComm = GetMainCommunication();

        for (int i = 0; i < 4; i++) {
            commResPacket->buffer[i] = (NRF_FICR->DEVICEID[0] >> (8 * i)) & 0xFF;
            commResPacket->buffer[i + 4] = (NRF_FICR->DEVICEID[1] >> (8 * i)) & 0xFF;
        }

        if (mainComm == commType) {
            memset(&commResPacket->buffer[8], 0, 12);
            AbsDetectState(&commResPacket->buffer[14]);
            PgmfiDetectState(&commResPacket->buffer[8]);
        } else {
            commResPacket->buffer[13] = COMM_STATE_SUB;
        }

        memcpy(&commResPacket->buffer[20], DEVICE_VERSION, 3);
        commResPacket->bleUUID = CUSTOM_VALUE_PING_RES_CHAR_UUID;
        commResPacket->cmd = CMD_PING_RES;
        commResPacket->bufLen = 23;
        this->SetCommandRepeatState(false);
    }
}

namespace HondaAbs {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_HONDA_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_PING_REQ:
            cmd = &hondaCommonPingCmd;
            NRF_LOG_INFO("[%s]: INFO: ABS Ping Request", LOG_DGB_NAME);
            break;

        case CMD_STATISTIC_REQ:
            cmd = &hondaAbsStatisticCmd;
            NRF_LOG_INFO("[%s]: INFO: ABS Statistic Request", LOG_DGB_NAME);
            break;

        case CMD_ERROR_REQ:
            cmd = &hondaAbsErrorCmd;
            NRF_LOG_INFO("[%s]: INFO: ABS Error Request", LOG_DGB_NAME);
            break;

        case CMD_ERASE_REQ:
            cmd = &hondaAbsEraseCmd;
            NRF_LOG_INFO("[%s]: INFO: ABS Erase Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    static bool GetStatistics(uint8_t *dataLen, uint8_t tableIdx, uint8_t *data) {
        uint8_t statisticBuf[MAX_ABS_SENSORS_TABLE][3]  = 
                        { {0x94, 0x71, 0x10}, {0x94, 0x71, 0x20} };

        if (HondaCmd::AbsGetStatistics(statisticBuf[tableIdx], 3, data, dataLen)) {
            return true;            
        }

        return false;
    }

    static bool GetErrors(uint8_t *dataLen, uint8_t *data) {
        if (HondaCmd::AbsGetErrors(data, dataLen)) {
            return true;
        }

        return false;
    }

    void StatisticCommand::Execute(CommPacket_t *commResPacket,
                                   const CommPacket_t *commReqPacket,
                                   CommunicationType_t commType) {
        static int tableIdx = 0;
        bool ret;
        uint8_t dataLen;

        ret = GetStatistics(&dataLen, tableIdx, &commResPacket->buffer[1]);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_STATISTICS_CHAR_UUID;
            commResPacket->cmd = CMD_STATISTIC_RES;
            commResPacket->bufLen = dataLen + 1;
            commResPacket->buffer[0] = tableIdx++;
            if (tableIdx < MAX_ABS_SENSORS_TABLE) {
                this->SetCommandRepeatState(true);
            } else if (tableIdx == MAX_ABS_SENSORS_TABLE) {
                tableIdx = 0;
                this->SetCommandRepeatState(false);
            } 
        } else {
            tableIdx = 0;
            commResPacket->cmd = CMD_IGNORE_RES;
            this->SetCommandRepeatState(false);            
        }
    }

    void ErrorCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        static int tableIdx = 0;
        bool ret;
        uint8_t dataLen;

        if (tableIdx == MAX_ABS_ERRORS_TABLE) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_END_ERROR_RES;
            commResPacket->bufLen = 0;
            tableIdx = 0;
            this->SetCommandRepeatState(false);
            return;
        }

        ret = GetErrors(&dataLen, &commResPacket->buffer[1]);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_ERRORS_CHAR_UUID;
            commResPacket->cmd = CMD_ERROR_CUR_RES;
            commResPacket->buffer[0] = tableIdx++;
            commResPacket->bufLen = dataLen + 1;
            this->SetCommandRepeatState(true);

        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
            commResPacket->bufLen = 0;
            tableIdx = 0;
            this->SetCommandRepeatState(false);
        }
    }

    void EraseCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        bool ret;

        ret = HondaCmd::AbsErase();
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

namespace HondaPgmfi {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_HONDA_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_PING_REQ:
            cmd = &hondaCommonPingCmd;
            NRF_LOG_INFO("[%s]: INFO: PGM-FI Ping Request", LOG_DGB_NAME);
            break;

        case CMD_STATISTIC_REQ:
            cmd = &hondaPgmfiStatisticCmd;
            NRF_LOG_INFO("[%s]: INFO: PGM-FI Statistic Request", LOG_DGB_NAME);
            break;

        case CMD_ERROR_REQ:
            cmd = &hondaPgmfiErrorCmd;
            NRF_LOG_INFO("[%s]: INFO: PGM-FI Error Request", LOG_DGB_NAME);
            break;

        case CMD_ERASE_REQ:
            cmd = &hondaPgmfiEraseCmd;
            NRF_LOG_INFO("[%s]: INFO: PGM-FI Erase Request", LOG_DGB_NAME);
            break;

        case CMD_RESET_REQ:
            cmd = &hondaPgmfiResetCmd;
            NRF_LOG_INFO("[%s]: INFO: PGM-FI Reset Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    static bool GetStatistics(uint8_t *dataLen, uint8_t tableIdx, uint8_t *data) {
        uint8_t statisticBuf[MAX_PGM_FI_SENSORS_TABLE][2]  = 
                        { {0x71, 0x10}, {0x71, 0x11}, {0x71, 0x13},
                          {0x71, 0x17}, {0x71, 0x20}, {0x71, 0x21},
                          {0x71, 0x60}, {0x71, 0x61}, {0x71, 0x63},
                          {0x71, 0x67}, {0x71, 0x70}, {0x71, 0x71},
                          {0x71, 0xD0}, {0x71, 0xD1}, {0x71, 0x16},
                          {0x71, 0x15}, {0x71, 0x66} };

        if (HondaCmd::PgmfiGetStatistics(statisticBuf[tableIdx], 2, data, dataLen)) {
            return true;
        }

        return false;
    }

    static bool GetErrors(uint8_t *dataLen, uint8_t tableIdx, uint8_t *data) {
        uint8_t errBuf[MAX_PGM_FI_ERRORS_TABLE / 2][2] = 
                    { {0x00, 0x01}, {0x00, 0x02}, {0x00, 0x03}, 
                      {0x00, 0x04}, {0x00, 0x05}, {0x00, 0x06},
                      {0x00, 0x07}, {0x00, 0x08}, {0x00, 0x09},
                      {0x00, 0x0A}, {0x00, 0x0B} };

        if (tableIdx >= (MAX_PGM_FI_ERRORS_TABLE / 2)) {
            tableIdx -= (MAX_PGM_FI_ERRORS_TABLE / 2);
            errBuf[tableIdx][0] = 0x74;
        } else {
            errBuf[tableIdx][0] = 0x73;
        }

        if (HondaCmd::PgmfiGetErrors(errBuf[tableIdx], 2, data, dataLen)) {
            return true;
        }

        return false;
    }

    void StatisticCommand::Execute(CommPacket_t *commResPacket,
                                   const CommPacket_t *commReqPacket,
                                   CommunicationType_t commType) {
        static int tableIdx = 0;
        bool ret;
        uint8_t dataLen;

        ret = GetStatistics(&dataLen, tableIdx, &commResPacket->buffer[1]);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_STATISTICS_CHAR_UUID;
            commResPacket->cmd = CMD_STATISTIC_RES;
            commResPacket->bufLen = dataLen + 1;
            commResPacket->buffer[0] = tableIdx++;
            if (tableIdx < MAX_PGM_FI_SENSORS_TABLE) {
                this->SetCommandRepeatState(true);
            } else if (tableIdx == MAX_PGM_FI_SENSORS_TABLE) {
                tableIdx = 0;
                this->SetCommandRepeatState(false);
            }
        } else {
            tableIdx = 0;
            commResPacket->cmd = CMD_IGNORE_RES;
            this->SetCommandRepeatState(false);
        }
    }

    void ErrorCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        static int tableIdx = 0;
        static bool isGetErrorSuccess = false;
        bool ret;
        uint8_t dataLen;

        if  (tableIdx == MAX_PGM_FI_ERRORS_TABLE) {
            isGetErrorSuccess = false;
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_END_ERROR_RES;
            commResPacket->bufLen = 0;
            tableIdx = 0;
            this->SetCommandRepeatState(false);
            return;        
        }

        ret = GetErrors(&dataLen, tableIdx, &commResPacket->buffer[1]);
        if (ret) {
            isGetErrorSuccess = true;

            commResPacket->bleUUID = CUSTOM_VALUE_ERRORS_CHAR_UUID;
            commResPacket->bufLen = dataLen + 1;
            commResPacket->cmd = (tableIdx >= (MAX_PGM_FI_ERRORS_TABLE / 2)) 
                                ? CMD_ERROR_PAST_RES : CMD_ERROR_CUR_RES;

            if (commType == BLE_COMM_TYPE) {
                commResPacket->buffer[0] = tableIdx;
            } else {
                commResPacket->buffer[0] = (tableIdx >= (MAX_PGM_FI_ERRORS_TABLE / 2))
                            ? (tableIdx - (MAX_PGM_FI_ERRORS_TABLE / 2)) : tableIdx;
            }

            if (tableIdx++ < MAX_PGM_FI_ERRORS_TABLE) {
                this->SetCommandRepeatState(true);
            }

        } else {
            if (tableIdx < (MAX_PGM_FI_ERRORS_TABLE / 2)) {
                commResPacket->cmd = CMD_IGNORE_RES;
                tableIdx = (MAX_PGM_FI_ERRORS_TABLE / 2);
                this->SetCommandRepeatState(true);

            } else {
                commResPacket->cmd = isGetErrorSuccess ? CMD_END_ERROR_RES : CMD_IGNORE_RES;
                commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
                commResPacket->bufLen = 0;
                tableIdx = 0;
                isGetErrorSuccess = false;
                this->SetCommandRepeatState(false);
            }
        }
    }

    void EraseCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        bool ret;

        ret = HondaCmd::PgmfiErase();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ERASE_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }
    
        this->SetCommandRepeatState(false);
    }

    void ResetCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        bool ret;

        ret = HondaCmd::PgmfiReset();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_RESET_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }
    
        this->SetCommandRepeatState(false);       
    }
}

namespace HondaDCT {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_HONDA_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_PING_REQ:
            cmd = &hondaCommonPingCmd;
            NRF_LOG_INFO("[%s]: INFO: DCT Ping Request", LOG_DGB_NAME);
            break;

        case CMD_STATISTIC_REQ:
            cmd = &hondaDctStatisticCmd;
            NRF_LOG_INFO("[%s]: INFO: DCT Statistic Request", LOG_DGB_NAME);
            break;

        case CMD_ERROR_REQ:
            cmd = &hondaDctErrorCmd;
            NRF_LOG_INFO("[%s]: INFO: DCT Error Request", LOG_DGB_NAME);
            break;

        case CMD_ERASE_REQ:
            cmd = &hondaPgmfiEraseCmd;
            NRF_LOG_INFO("[%s]: INFO: DCT Erase Request", LOG_DGB_NAME);
            break;

        case CMD_RESET_REQ:
            cmd = &hondaPgmfiResetCmd;
            NRF_LOG_INFO("[%s]: INFO: DCT Reset Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    static bool GetStatistics(uint8_t *dataLen, uint8_t tableIdx, uint8_t *data) {
        uint8_t statisticBuf[MAX_DCT_SENSORS_TABLE][3]  = 
                        { {0x48, 0x71, 0xD0}, {0x48, 0x71, 0xD1},
                          {0x48, 0x71, 0x15}, {0x48, 0x71, 0x65} };

        if (HondaCmd::DctGetStatistics(statisticBuf[tableIdx], 3, data, dataLen)) {
            return true;            
        }

        return false;
    }

    static bool GetErrors(uint8_t *dataLen, uint8_t tableIdx, uint8_t *data) {
        uint8_t errBuf[MAX_DCT_ERRORS_TABLE / 2][3] = 
                    { {0x48, 0x00, 0x01}, {0x48, 0x00, 0x02}, 
                      {0x48, 0x00, 0x03}, {0x48, 0x00, 0x04}, 
                      {0x48, 0x00, 0x05}, {0x48, 0x00, 0x06}, 
                      {0x48, 0x00, 0x07}, {0x48, 0x00, 0x08}, 
                      {0x48, 0x00, 0x09}, {0x48, 0x00, 0x0A}, 
                      {0x48, 0x00, 0x0B} };

        if (tableIdx >= (MAX_DCT_ERRORS_TABLE / 2)) {
            tableIdx -= (MAX_DCT_ERRORS_TABLE / 2);
            errBuf[tableIdx][1] = 0x73;
        } else {
            errBuf[tableIdx][1] = 0x74;
        }

        if (HondaCmd::DctGetErrors(errBuf[tableIdx], 3, data, dataLen)) {
            return true;
        }

        return false;
    }

    void StatisticCommand::Execute(CommPacket_t *commResPacket,
                                   const CommPacket_t *commReqPacket,
                                   CommunicationType_t commType) {
        static int tableIdx = 0;
        bool ret;
        uint8_t dataLen;

        ret = GetStatistics(&dataLen, tableIdx, &commResPacket->buffer[1]);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_STATISTICS_CHAR_UUID;
            commResPacket->cmd = CMD_STATISTIC_RES;
            commResPacket->bufLen = dataLen + 1;
            commResPacket->buffer[0] = tableIdx++;
            if (tableIdx < MAX_DCT_SENSORS_TABLE) {
                this->SetCommandRepeatState(true);

            } else if (tableIdx == MAX_DCT_SENSORS_TABLE) {
                tableIdx = 0;
                this->SetCommandRepeatState(false);
            }
        } else {
            tableIdx = 0;
            commResPacket->cmd = CMD_IGNORE_RES;
            this->SetCommandRepeatState(false);
        }
    }

    void ErrorCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        static int tableIdx = 0;
        bool ret;
        uint8_t dataLen;

        if  (tableIdx == MAX_DCT_ERRORS_TABLE) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_END_ERROR_RES;
            commResPacket->bufLen = 0;
            tableIdx = 0;
            this->SetCommandRepeatState(false);
            return;        
        }

        ret = GetErrors(&dataLen, tableIdx, &commResPacket->buffer[1]);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_ERRORS_CHAR_UUID;
            commResPacket->bufLen = dataLen + 1;
            commResPacket->cmd = (tableIdx >= (MAX_DCT_ERRORS_TABLE / 2)) 
                                ? CMD_ERROR_PAST_RES : CMD_ERROR_CUR_RES;

            if (commType == BLE_COMM_TYPE) {
                commResPacket->buffer[0] = tableIdx;
            } else {
                commResPacket->buffer[0] = (tableIdx >= (MAX_DCT_ERRORS_TABLE / 2))
                            ? (tableIdx - (MAX_DCT_ERRORS_TABLE / 2)) : tableIdx;
            }

            if (tableIdx++ < MAX_DCT_ERRORS_TABLE) {
                this->SetCommandRepeatState(true);
            }

        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
            commResPacket->bufLen = 0;
            tableIdx = 0;
            this->SetCommandRepeatState(false);
        }
    }
}