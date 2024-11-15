#ifndef __HONDA_BACKUP_H
#define __HONDA_BACKUP_H

#include "main.h"
#include "ecu/ecu_common.h"

namespace HondaKeihinBackup {
    EsyPro::Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType);

    class TurnOffCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class TurnOnCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class StartReadCommand : public EsyPro::Command {
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
}

#endif /* __HONDA_BACKUP_H */
