#ifndef __YAMAHA_IDLE_H
#define __YAMAHA_IDLE_H

#include "main.h"
#include "ecu/ecu_common.h"

#define YAMAHA_TARGET_BYTE          0x10
#define YAMAHA_SOURCE_BYTE          0xF0
#define MAX_YAMAHA_SENSORS_TABLE    30
#define MAX_YAMAHA_DIAG_TABLE       30
#define MAX_YAMAHA_OBD2_TABLE       1

namespace YamahaCommon {
    enum YamahaState_t {
        ECU_STATE_OK = 0x01,
        ECU_STATE_UNKNOWN = 0x03,
        COMM_STATE_SUB = 0x05
    };

    enum ErrorState_t {
        SEND_ERROR_REQ,
        SEND_ERROR_RES,
        END_READING_ERROR
    };

    enum ErrorTypeBle_t {
        BLE_PAST_ERROR_TYPE,
        BLE_CUR_ERROR_TYPE
    };

    enum CheckSyncType_t {
        PING_SYNC_TYPE,
        STATISTIC_SYNC_TYPE,
        DIAG_SYNC_TYPE
    };

    class PingCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class ErrorCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class StatisticCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class DiagnosticCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;        
    };

    class EraseCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };
}

namespace Yamaha {
    EsyPro::Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType);

    class CheckSyncCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };
}

namespace YamahaSMK {
    EsyPro::Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType);

    class CheckSyncCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };
}

namespace YamahaObd2 {
    EsyPro::Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType);

    class StatisticCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class DiagnosticCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;        
    };

    class CheckSyncCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };
}

#endif /* __YAMAHA_IDLE_H */
