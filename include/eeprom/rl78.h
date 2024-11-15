#ifndef __RL78_H
#define __RL78_H

#include "main.h"
#include "eeprom/eeprom_common.h"

#define MAX_RL78_COMM_BUF_SIZE  256
#define RL78_PAGE_SIZE          256
#define RL78_BLOCK_SIZE         1024

namespace RL78 {
    enum RL78PacketType_t {
        COMMAND_TYPE,
        DATA_TYPE
    };

    enum NumResMessages_t {
        ONE_MSGS,
        TWO_MSGS
    };

    enum WriteStateType_t {
        RECV_FIRST_BLOCK,
        RECV_SECOND_BLOCK,
        FLASHING_DATA
    };

    enum ReadStateType_t {
        READ_RL78,
        SEND_FIRST_BLOCK,
        SEND_SECOND_BLOCK,
        END_READ_RL78
    };

    enum ReceiveStateType_t {
        RECEIVE_ECHO,
        RECEIVE_HEADER,
        RECEIVE_LEN,
        RECEIVE_DATA,
        RECEIVE_CKSUM,
        RECEIVE_END
    };

    struct RL78Packet_t {
        RL78PacketType_t packetType;
        uint8_t cmd;
        uint8_t cmdLen;
        uint16_t bufLen;
        uint8_t buffer[MAX_RL78_COMM_BUF_SIZE];
    };

    class RL78Obj {
    protected:
        RL78Packet_t reqPacket;
        RL78Packet_t resPacket;
        uint8_t CalcChecksum(const RL78Packet_t *packet) const;
    
    public:
        void SendRequestToRL78(void);
        bool ReceiveRequestFromRL78(NumResMessages_t numResMsgs);
        void GetPacketResData(uint8_t *data, uint16_t *dataLen, uint8_t offset) const;
        void SetPacketReqData(uint8_t cmd, uint8_t cmdLen, uint8_t *buffer,
                              uint16_t bufLen, RL78PacketType_t type);
    };

    EsyPro::Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType);

    class SetupCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class ReadCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class WriteCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };
}

#endif /* __RL78_H */
