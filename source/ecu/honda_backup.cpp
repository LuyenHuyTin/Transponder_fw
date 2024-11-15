#include "ecu/honda_backup.h"
#include "ble_common.h"

using namespace EsyPro;
using namespace ECU;

static const char LOG_DGB_NAME[] = "honda_backup";
static uint32_t ecuFlashOffset = 0;

static HondaKeihinBackup::TurnOffCommand hondaKeihinTurnOffCmd;
static HondaKeihinBackup::TurnOnCommand hondaKeihinTurnOnCmd;
static HondaKeihinBackup::StartReadCommand hondaKeihinStartReadCmd;
static HondaKeihinBackup::ReadCommand hondaKeihinReadCmd;

static HondaCommonObj ecuObj;

namespace HondaCmd {
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

    static bool CheckKeihinTurnOn(void) {
        bool ret;
        uint8_t wakeCmd[] = {0xFE};
        uint8_t wakeBuffer[] = {0x72};
        uint8_t stateOkCmd[] = {0x72};
        uint8_t stateOkBuffer[] = {0x71, 0x00};

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

    static bool CheckKeihinSecureKeys(void) {
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
            NRF_LOG_INFO("[%s]: ERROR: KEIHIN Check Secure-1", LOG_DGB_NAME);
            return false;
        }

        ecuObj.SetReqEcuPacket(secure2Cmd, sizeof(secure2Cmd),
                             secure2Buffer, sizeof(secure2Buffer));
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: KEIHIN Check Secure-2", LOG_DGB_NAME);
            return false;
        }

        return true;        
    }

    static bool ReadKeihinFlash(uint8_t *srcBuffer, uint8_t srcBufLen,
                                uint8_t *data, uint8_t *dataLen) {
        bool ret;
        uint8_t srcCmd[] = {0x82, 0x82, 0x00};

        ecuObj.SetReqEcuPacket(srcCmd, sizeof(srcCmd), srcBuffer, srcBufLen);
        ecuObj.SendRequestToEcu();
        ret = ecuObj.ReceiveRequestFromEcu();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: KEIHIN Read Flash", LOG_DGB_NAME);
            return false;
        }
        ecuObj.GetEcuPacketResData(data, dataLen, 0);

        return true;
    }
}

namespace HondaKeihinBackup {
    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(MC33290_HONDA_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_ECU_HONDA_TURNOFF_REQ:
            cmd = &hondaKeihinTurnOffCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin Turn-off Request", LOG_DGB_NAME);
            break;
        
        case CMD_ECU_HONDA_TURNON_REQ:
            cmd = &hondaKeihinTurnOnCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin Turn-on Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_START_READ_REQ:
            cmd = &hondaKeihinStartReadCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin Start Read Request", LOG_DGB_NAME);
            break;

        case CMD_ECU_HONDA_READ_ACK_REQ:
            cmd = &hondaKeihinReadCmd;
            NRF_LOG_INFO("[%s]: INFO: Keihin Read ACK Request", LOG_DGB_NAME);
            break;
        }

        return cmd;
    }

    static bool EnterReadMode(void) {
        if (!HondaCmd::CheckKeihinTurnOn()) {
            return false;
        }

        if (!HondaCmd::CheckKeihinSecureKeys()) {
            return false;
        }

        return true;
    }

    static bool ReadEcuFlash(uint32_t offset, uint8_t *dataLen, uint8_t *data) {
        uint8_t buffer[4];
        uint8_t numReadBytes = 12;

        while (numReadBytes) {
            bool ret;

            buffer[0] = (offset >> 16) & 0xFF;
            buffer[1] = offset & 0xFF;
            buffer[2] = (offset >> 8) & 0xFF;
            buffer[3] = numReadBytes;

            ret = HondaCmd::ReadKeihinFlash(buffer, 4, data, dataLen);
            if (ret) {
                return true;
            }
            numReadBytes--;
        }

        return false;
    }

    void TurnOffCommand::Execute(CommPacket_t *commResPacket,
                                 const CommPacket_t *commReqPacket,
                                 CommunicationType_t commType) {
        bool ret;

        ret = HondaCmd::CheckKeihinTurnOff();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_TURNOFF_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x01;
        } else {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_TURNOFF_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x00;
        }
        this->SetCommandRepeatState(false);
    }

    void TurnOnCommand::Execute(CommPacket_t *commResPacket,
                                const CommPacket_t *commReqPacket,
                                CommunicationType_t commType) {
        bool ret;

        ret = EnterReadMode();
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_TURNON_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x01;           
        } else {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_ECU_HONDA_TURNON_RES;
            commResPacket->bufLen = 1;
            commResPacket->buffer[0] = 0x00;
        }
        this->SetCommandRepeatState(false);
    }

    void StartReadCommand::Execute(CommPacket_t *commResPacket,
                                   const CommPacket_t *commReqPacket,
                                   CommunicationType_t commType) {
        bool ret;
        uint8_t dataLen;

        memcpy(&ecuFlashOffset, commReqPacket->buffer, 4);
        ret = ReadEcuFlash(ecuFlashOffset, &dataLen, commResPacket->buffer);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
            commResPacket->bufLen = dataLen;
            commResPacket->cmd = CMD_ECU_HONDA_READ_CONTINOUS_RES;
            ecuFlashOffset += dataLen;
        } else {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->bufLen = 0;
            commResPacket->cmd = CMD_ECU_HONDA_READ_FINISH_RES;            
        }
        this->SetCommandRepeatState(false);
    }

    void ReadCommand::Execute(CommPacket_t *commResPacket,
                              const CommPacket_t *commReqPacket,
                              CommunicationType_t commType) {
        bool ret;
        uint8_t dataLen;

        ret = ReadEcuFlash(ecuFlashOffset, &dataLen, commResPacket->buffer);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
            commResPacket->bufLen = dataLen;
            commResPacket->cmd = CMD_ECU_HONDA_READ_CONTINOUS_RES;
            ecuFlashOffset += dataLen;
        } else {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->bufLen = 0;
            commResPacket->cmd = CMD_ECU_HONDA_READ_FINISH_RES;            
        }
        this->SetCommandRepeatState(false);
    }
}
