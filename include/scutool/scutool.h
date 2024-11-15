#ifndef __SCUTOOL_COMMON_H
#define __SCUTOOL_COMMON_H

#include "main.h"
#include "common.h"

#define MAX_SCUTOOL_COMM_BUF_SIZE       1024
#define SCUTOOL_TIMEOUT                 1000
#define SCUTOOL_PROGMODE_TIMEOUT        7000
#define MAX_SCUTOOL_RECV_BUF_SIZE       5
#define SCUTOOL_SOF                     0x01
#define SCUTOOL_ID                      0x02
#define ESYTOOL_ID                      0x04
#define SCUTOOL_ENTERPROGMODE_OK        0x43
#define SCUTOOL_FIRMWARE_PACKET_SIZE    1024
#define SCUTOOL_BOOTLOADER_SOF          0x02
#define SCUTOOL_BOOTLOADER_RSP_OK       0x06
#define SCUTOOL_BOOTLOADER_RSP_RETRY    0x15

namespace SCUTool {
    struct ScutoolPacket_t {
        uint32_t bufLen;
        uint8_t buffer[MAX_SCUTOOL_COMM_BUF_SIZE];
    };

    class ScutoolObj {
    private:
        ScutoolPacket_t reqPacket;
        ScutoolPacket_t resPacket;
        enum ReceiveStateType_t {
            RECEIVE_SOF,
            RECEIVE_ID,
            RECEIVE_DATALEN,
            RECEIVE_CMD,
            RECEIVE_DATA,
        };

    public:
        ~ScutoolObj() = default;
        ScutoolObj() = default;
        void SendRequestToScutool(uint8_t cmd, bool isAppendZeros);
        void SetReqScutoolPacket(uint8_t *buffer, uint8_t bufLen);
        bool ReceiveRequestFromScutool(const uint8_t expectCmd);
        bool  SendRequestToScutoolResponse(uint8_t cmd, bool isAppendZeros, const uint8_t expectCmd);
        void GetScutoolPacketResData(uint8_t *data, uint8_t *dataLen) const;
    };
    void GetCmdFromScutoolRequest(EsyPro::CommunicationObj *commPtr);

    EsyPro::Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType);

    class PingCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class EnterProgmodeCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class WriteFirmwareCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };                                        
}

#endif /* __SCUTOOL_COMMON_H */
