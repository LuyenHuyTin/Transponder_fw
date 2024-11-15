#ifndef __ECU_COMMON_H
#define __ECU_COMMON_H

#include "main.h"
#include "common.h"

#define MAX_ECU_COMM_BUF_SIZE       255
#define MAX_ECU_COMM_CMD_SIZE       4
#define ECU_HONDA_TIMEOUT           200
#define ECU_PIAGGIO_TIMEOUT         500
#define ECU_YAMAHA_TIMEOUT          500
#define ECU_YAMAHA_SYNC_DELAY       15
#define ECU_YAMAHA_SYNC_TIMEOUT_1   5000
#define ECU_YAMAHA_SYNC_TIMEOUT_2   2000

#define PIAGGIO_TARGET_BYTE         0x10
#define PGFLASH_PIAGGIO_TARGET_BYTE 0xEF
#define PIAGGIO_SOURCE_BYTE         0xF1

#define PIAGGIO_AC19X_HEADER        0x33

namespace ECU {
    enum Iso14230ReceiveStateType_t {
        RECEIVE_ECHO,
        RECEIVE_CMD_1,
        RECEIVE_CMD_CHECK,
        RECEIVE_CMD_3,
        RECEIVE_CMD_4,
        RECEIVE_DATA,
        RECEIVE_CRC
    };

    enum AC19xReceiveStateType_t {
        AC19X_RECEIVE_ECHO,
        AC19X_RECEIVE_CMD,
        AC19X_RECEIVE_DATA,
        AC19X_RECEIVE_CRC
    };

    enum EcuType_t {
        ECU_HONDA_PGMFI = 1,
        ECU_HONDA_ABS = 2,
        ECU_HONDA_DCT = 3,
        ECU_PIAGGIO_MIU = 4,
        ECU_PIAGGIO_PG = 7,
        ECU_YAMAHA = 10,
        ECU_YAMAHA_SMK = 13,
        ECU_YAMAHA_OBD2 = 16 
    };

    struct EcuPacket_t {
        uint16_t bufLen;
        uint8_t cmd[MAX_ECU_COMM_CMD_SIZE];
        uint8_t cmdLen;
        uint8_t buffer[MAX_ECU_COMM_BUF_SIZE];
    };

    class EcuCommonObj {
    protected:
        EcuPacket_t reqPacket;
        EcuPacket_t resPacket;

    public:
        virtual ~EcuCommonObj() {}
        virtual void SendRequestToEcu() = 0;
        virtual bool ReceiveRequestFromEcu() = 0;
        void GetEcuCmdResData(uint8_t *cmd) const;
        void GetEcuPacketResData(uint8_t *data, uint8_t *dataLen,
                                 uint8_t offset) const;
        void SetReqEcuPacket(uint8_t *cmd, uint8_t cmdLen,
                             uint8_t *buffer, uint8_t bufLen);
        void SetReqEcuPacket(uint8_t *cmd, uint8_t cmdLen);
    };

    class HondaCommonObj : public EcuCommonObj {
    private:
        uint8_t CalcChecksum(const EcuPacket_t *packet) const;
        enum ReceiveStateType_t {
            RECEIVE_ECHO,
            RECEIVE_CMD_1,
            RECEIVE_CMD_2,
            RECEIVE_CMD_3,
            RECEIVE_LEN,
            RECEIVE_DATA,
            RECEIVE_CRC
        };

    public:
        HondaCommonObj() = default;
        void InitKlineBus();
        void SendRequestToEcu() override;
        bool ReceiveRequestFromEcu() override;
    };

    class PiaggioCommonObj : public EcuCommonObj {
    public:
        PiaggioCommonObj() = default;
        void InitKlineBus();
        void SendRequestToEcu() override;
        bool ReceiveRequestFromEcu() override;
    };

    class YamahaCommonObj : public EcuCommonObj {
    public:
        enum YamahaPingType_t {
            ECU_NORMAL_TYPE,
            ECU_KEY_CUJU
        };
      
    private:
        enum ReceiveState15800_t {
            RECEIVE_ECHO,
            RECEIVE_HEADER,
            RECEIVE_DATA,
            RECEIVE_CRC,
            RECEIVE_IGNORE
        };

        enum ReceiveStateSMK_t {
            RECEIVE_KEY_SMK,
            RECEIVE_ECHO_SMK,
            RECEIVE_DATA_SMK,
            RECEIVE_CRC_SMK
        };

        YamahaPingType_t ecuPingType;

    public:
        YamahaCommonObj() = default;
        YamahaPingType_t GetEcuPingType();
        void SetEcuPingType(YamahaPingType_t type);
        void InitKlineBus();
        void SendRequestToEcu() override;
        bool ReceiveRequestFromEcu() override;
        bool ProcessRequestEcu15800();
        bool ProcessRequestEcuSMK();
    };

    void GetCmdFromEcuRequest(EsyPro::CommunicationObj *commPtr,
                              EsyPro::CommunicationType_t commType);
}

#endif /* __ECU_COMMON_H */
