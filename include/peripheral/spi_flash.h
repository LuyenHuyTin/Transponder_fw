#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include "main.h"
#include "common.h"

#define SPI_FLASH_TIMEOUT               500
#define SPI_FLASH_MAX_WRITE_DATA_SIZE   136

#define CMD_WRSR            0x01
#define CMD_PP              0x02
#define CMD_READ            0x03
#define CMD_WRDI            0x04
#define CMD_RDSR1           0x05
#define CMD_WREN            0x06
#define CMD_FAST_READ       0x0B
#define CMD_ERASE_4K        0x20
#define CMD_RDSR2           0x35
#define CMD_ERASE_32K       0x52
#define CMD_JEDEC_ID        0x9F
#define CMD_ERASE_FULL      0xC7
#define CMD_ERASE_64K       0xD8

namespace SpiFlash {
    class Spi0Obj {
    private:
        volatile bool transferDone;
        nrf_drv_spi_t spiDrv;
        static Spi0Obj *pInstance;
        Spi0Obj();
        bool TransferBytes(const uint8_t *sendBytes, uint16_t sendLen,
                           uint8_t *recvBytes, uint16_t recvLen);
        bool WaitForWriteEnd();
        bool WriteEnable();

    public:
        Spi0Obj(Spi0Obj *obj) = delete;
        void operator=(const Spi0Obj *) = delete;
        static Spi0Obj *GetInstance();

        void Init();
        void SetTransferDone(bool state);
        bool EraseBlock64K(uint32_t addr);
        bool WriteToFlash(uint32_t addr, const uint8_t *data, uint16_t dataLen);
        bool ReadFromFlash(uint32_t addr, uint8_t *data, uint16_t dataLen);
    };

    EsyPro::Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType);

    class EraseCommand : public EsyPro::Command {
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

    class VerifyCommand : public EsyPro::Command {
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

#endif /* __SPI_FLASH_H */