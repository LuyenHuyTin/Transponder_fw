#ifndef __PIAGGIO_FLASH_H
#define __PIAGGIO_FLASH_H

#include "main.h"
#include "ecu/ecu_common.h"

#define MIU_FLASH_START_ADDR        0xC00000
#define MIU_PIAGGIO_TARGET_BYTE     0xEF
#define MIU_FLASH_MODE_READ         0x00
#define MIU_FLASH_MODE_WRITE        0x01
#define MIU_EEPROM                  0x00
#define MIU_FLASH                   0x01

namespace PiaggioMIUFlash {
    EsyPro::Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType);

    class AccessSecureCommand : public EsyPro::Command {
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

    class EndWriteCommand : public EsyPro::Command {
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;     
    };
}

namespace PiaggioPGFlash {
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

    class ReadKeyCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };
}

namespace PiaggioRISSFlash {
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

    class ReadKeyCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };
}

namespace PiaggioAC19xFlash {
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

#endif /* __PIAGGIO_FLASH_H */
