#ifndef __AT93CXX_H
#define __AT93CXX_H

#include "main.h"
#include "eeprom/eeprom_common.h"

#define OPCODE_CTRL         0x00
#define OPCODE_WRITE        0x01
#define OPCODE_READ         0x02
#define OPCODE_EW_DISABLE   0x00
#define OPCODE_EW_ENABLE    0x03

namespace AT93CXX {
    enum AT93cxxOrg_t {
        AT93CXX_ORG_8,
        AT93CXX_ORG_16
    };
    
    struct At93cxx_t {
        AT93cxxOrg_t org;
        EEPROM::EEPROMType_t model;
        uint16_t size;
        uint16_t addrBits;
        uint16_t mask;
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

#endif /* __AT93CXX_H */
