#ifndef __PIC16FXX_H
#define __PIC16FXX_H

#include "main.h"
#include "eeprom/eeprom_common.h"

#define ISP_CLK_DELAY           1
#define CMD_LOAD_CONFIG         0x00
#define CMD_INCR_ADDR           0x06
#define CMD_READ_PROG_MEM       0x04
#define CMD_READ_DATA_MEM       0x05
#define CMD_BULK_ERASE_PROG     0x09
#define CMD_LOAD_PROGRAM_MEMORY 0x02
#define CMD_LOAD_EEPROM_MEMORY  0x03
#define CMD_BEGIN_PROGRAM_INT   0x08

#define MAX_WORD_WRITE_SIZE     8

#define PIC_PROG_BUF_SIZE   128   
#define PIC_EEPROM_BUF_SIZE 64

namespace PIC16FXX {
    enum pic16fxxMemType_t {
        PIC16FXX_PROG_MEM,
        PIC16FXX_EEPROM_MEM
    };

    struct Pic16fxx_t {
        uint32_t endAddr;
        uint32_t startAddr;
        pic16fxxMemType_t memType;
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

#endif /* __PIC16FXX_H */
