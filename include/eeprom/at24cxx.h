#ifndef __AT24CXX_H
#define __AT24CXX_H

#include "main.h"
#include "eeprom/eeprom_common.h"

namespace AT24CXX {
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

#endif /* __AT24CXX_H */
