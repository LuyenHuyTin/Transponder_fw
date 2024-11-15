#ifndef __9S12Sec_H
#define __9S12Sec_H

#include "main.h"
#include "eeprom/eeprom_common.h"

#define EEPROM_9S12_READ_WRITE_SIZE     128
#define EEPROM_9S12_TYPE                11
#define GPIO_PIN_CNF_OFFSET             0x700
#define GPIO_PIN_OUTSET_OFFSET          0x508
#define GPIO_PIN_OUTCLR_OFFSET          0x50C
#define GPIO_PIN_OUTSTATE_OFFSET        0x504
#define GPIO_PIN_IN                     0x510
#define GPIO_PIN_DIRSET                 0x518
#define GPIO_PIN_DIRCLR                 0x51C
#define BDM_IN                          33
#define BDM_OUT                         41
#define BDM_OUT_CRT                     8


namespace MC9S12Sec {
    enum Mcs9S12SecOption_t {
        READ_PFLASH,
        WRITE_PFLASH,
        READ_EEPROM,
        WRITE_EEPROM
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

    class EraseCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };
}

#endif /* __9S12Sec_H */