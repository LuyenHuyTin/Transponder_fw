#ifndef __COMMON_H
#define __COMMON_H

#include "main.h"

namespace EsyPro {
    enum CommunicationType_t {
        NOT_CONNECTED,
        PC_COMM_TYPE,
        BLE_COMM_TYPE
    };

    enum CommunicationCmd_t {
        CMD_BLE_OTA_DFU_MODE = 0x0A,

        CMD_ECU_IDLE_BASE = 0x00,
        CMD_PING_REQ = 0x00,
        CMD_PING_RES = 0x00,
        CMD_STATISTIC_REQ = 0x01,
        CMD_STATISTIC_RES = 0x01,
        CMD_ERROR_REQ = 0x02,
        CMD_ERROR_PAST_RES = 0x02,
        CMD_ERROR_CUR_RES = 0x03,
        CMD_END_ERROR_RES = 0x04,
        CMD_ERASE_REQ = 0x05,
        CMD_ERASE_RES = 0x05,
        CMD_RESET_REQ = 0x06,
        CMD_RESET_RES = 0x06,
        CMD_EN_ACTUATOR_REQ = 0x07,
        CMD_EN_ACTUATOR_RES = 0x07,
        CMD_END_STATISTIC_RES = 0x08,
        CMD_RESET_PARAMS_REQ = 0x09,
        CMD_RESET_PARAMS_RES = 0x09,
        CMD_BACKUP_PARAMS_REQ = 0x0B,
        CMD_BACKUP_PARAMS_RES = 0x0B,
        CMD_CHECK_SYNC_REQ = 0x0C,
        CMD_CHECK_SYNC_RES = 0x0C,
        CMD_DIAGNOSTIC_REQ = 0x0D,
        CMD_DIAGNOSTIC_RES = 0x0D,

        CMD_ECU_HONDA_BACKUP_BASE = 0x20,
        CMD_ECU_HONDA_TURNOFF_REQ = 0x00,
        CMD_ECU_HONDA_TURNOFF_RES = 0x20,
        CMD_ECU_HONDA_TURNON_REQ = 0x01,
        CMD_ECU_HONDA_TURNON_RES = 0x21,
        CMD_ECU_HONDA_START_READ_REQ = 0x02,
        CMD_ECU_HONDA_READ_ACK_REQ = 0x03,
        CMD_ECU_HONDA_READ_CONTINOUS_RES = 0x22,
        CMD_ECU_HONDA_READ_FINISH_RES = 0x23,

        CMD_ECU_HONDA_KEIHIN_UPDATE_BASE = 0x10,
        CMD_ECU_HONDA_KEIHIN_INIT_WRITE_REQ = 0x00,
        CMD_ECU_HONDA_KEIHIN_INIT_WRITE_RES = 0x10, 
        CMD_ECU_HONDA_KEIHIN_WRITE_REQ = 0x01,
        CMD_ECU_HONDA_KEIHIN_WRITE_RES = 0x11,
        CMD_ECU_HONDA_KEIHIN_WRITE_FINISH_REQ = 0x02,
        CMD_ECU_HONDA_KEIHIN_WRITE_FINISH_RES = 0x12,
        CMD_ECU_HONDA_KEIHIN_ERASE_REQ = 0x03,
        CMD_ECU_HONDA_KEIHIN_WRITE_64B_RES = 0x13,
        CMD_ECU_HONDA_KEIHIN_SETUP_EEPROM_REQ = 0x04,
        CMD_ECU_HONDA_KEIHIN_SETUP_EEPROM_RES = 0x14,
        CMD_ECU_HONDA_KEIHIN_WRITE_EEPROM_REQ = 0x05,
        CMD_ECU_HONDA_KEIHIN_WRITE_EEPROM_RES = 0x15,
        CMD_ECU_HONDA_KEIHIN_END_WRITE_EEPROM_REQ = 0x06,
        CMD_ECU_HONDA_KEIHIN_END_WRITE_EEPROM_RES = 0x16,
        CMD_ECU_HONDA_KEIHIN_EEPROM_OFF_REQ = 0x07,
        CMD_ECU_HONDA_KEIHIN_EEPROM_OFF_RES = 0x17,
        CMD_ECU_HONDA_KEIHIN_EEPROM_ON_REQ = 0x08,
        CMD_ECU_HONDA_KEIHIN_EEPROM_ON_RES = 0x18,
        CMD_ECU_HONDA_KEIHIN_READ_EEPROM_REQ = 0x09,
        CMD_ECU_HONDA_KEIHIN_READ_EEPROM_RES = 0x19,
        CMD_ECU_HONDA_KEIHIN_EEEPROM_FORMAT_REQ = 0x0A,
        CMD_ECU_HONDA_KEIHIN_EEEPROM_FORMAT_RES = 0x1A,

        CMD_QSPI_FLASH_BASE = 0x40,
        CMD_QSPI_FLASH_ERASE_REQ = 0x00,
        CMD_QSPI_FLASH_ERASE_RES = 0x40,
        CMD_QSPI_FLASH_WRITE_REQ = 0x01,
        CMD_QSPI_FLASH_WRITE_RES = 0x41,
        CMD_QSPI_FLASH_VERIFY_REQ = 0x02,
        CMD_QSPI_FLASH_VERIFY_RES = 0x42,
        CMD_QSPI_FLASH_READ_REQ = 0x03,
        CMD_QSPI_FLASH_READ_RES = 0x43,

        CMD_ECU_HONDA_SHINDENGEN_UPDATE_BASE = 0x30,
        CMD_ECU_HONDA_SHINDENGEN_INIT_WRITE_REQ = 0x00,
        CMD_ECU_HONDA_SHINDENGEN_INIT_WRITE_RES = 0x30,
        CMD_ECU_HONDA_SHINDENGEN_SEEDKEY_REQ = 0x01,
        CMD_ECU_HONDA_SHINDENGEN_SEEDKEY_RES = 0x31,
        CMD_ECU_HONDA_SHINDENGEN_ERASE_REQ = 0x02,
        CMD_ECU_HONDA_SHINDENGEN_ERASE_RES = 0x32,
        CMD_ECU_HONDA_SHINDENGEN_WRITE_REQ = 0x03,
        CMD_ECU_HONDA_SHINDENGEN_WRITE_RES = 0x33,
        CMD_ECU_HONDA_SHINDENGEN_WRITE_FINISH_REQ = 0x04,
        CMD_ECU_HONDA_SHINDENGEN_WRITE_FINISH_RES = 0x34,
        CMD_ECU_HONDA_SHINDENGEN_SETUP_EEPROM_REQ = 0x05,
        CMD_ECU_HONDA_SHINDENGEN_SETUP_EEPROM_RES = 0x35,
        CMD_ECU_HONDA_SHINDENGEN_WRITE_EEPROM_REQ = 0x06,
        CMD_ECU_HONDA_SHINDENGEN_WRITE_EEPROM_RES = 0x36,
        CMD_ECU_HONDA_SHINDENGEN_END_WRITE_EEPROM_REQ = 0x07,
        CMD_ECU_HONDA_SHINDENGEN_END_WRITE_EEPROM_RES = 0x37,
        CMD_ECU_HONDA_SHINDENGEN_EEPROM_OFF_REQ = 0x08,
        CMD_ECU_HONDA_SHINDENGEN_EEPROM_OFF_RES = 0x38,
        CMD_ECU_HONDA_SHINDENGEN_EEPROM_ON_REQ = 0x09,
        CMD_ECU_HONDA_SHINDENGEN_EEPROM_ON_RES = 0x39,
        CMD_ECU_HONDA_SHINDENGEN_READ_EEPROM_REQ = 0x0A,
        CMD_ECU_HONDA_SHINDENGEN_READ_EEPROM_RES = 0x3A,

        CMD_BASIC_MEM_COMM_BASE = 0x50,
        CMD_BASIC_MEM_SETUP_REQ = 0x00,
        CMD_BASIC_MEM_SETUP_RES = 0x50,
        CMD_BASIC_MEM_READ_DATA_REQ = 0x01,
        CMD_BASIC_MEM_READ_DATA_RES = 0x51,
        CMD_BASIC_MEM_WRITE_DATA_REQ = 0x02,
        CMD_BASIC_MEM_WRITE_DATA_RES = 0x52,
        CMD_BASIC_MEM_END_READ_DATA_RES = 0x53,
        CMD_BASIC_MEM_READ_KEY_REQ = 0x04,
        CMD_BASIC_MEM_READ_KEY_RES = 0x54,
        CMD_BASIC_MEM_END_WRITE_DATA_REQ = 0x05,
        CMD_BASIC_MEM_END_WRITE_DATA_RES = 0x55,
        CMD_BASIC_MEM_ERASE_REQ = 0x06,
        CMD_BASIC_MEM_ERASE_RES = 0x56,

        CMD_SCUTOOL_COMM_BASE = 0x60,
        CMD_SCUTOOL_PING_REQ = 0x00,
        CMD_SCUTOOL_PING_RES = 0x60,
        CMD_SCUTOOL_ENTER_PROGMODE_REQ = 0x01,
        CMD_SCUTOOL_ENTER_PROGMODE_RES = 0x61,
        CMD_SCUTOOL_WRITE_FIRMWARE_REQ = 0x02,
        CMD_SCUTOOL_WRITE_FIRMWARE_RES = 0x62,

        CMD_IGNORE_RES = 0xFF
    };

    struct CommPacket_t {
        uint16_t bleUUID;
        uint16_t bufLen;
        uint8_t cmd;
        uint8_t buffer[COMM_OBJ_BUF_SIZE];
    };

    class Command {
    private:
        bool isRepeat;

    public:
        void SetCommandRepeatState(bool state);
        bool GetCommandRepeatState() const;
        virtual ~Command() {}
        virtual void Execute(CommPacket_t *commResPacket,
                             const CommPacket_t *commReqPacket,
                             CommunicationType_t commType) = 0;
    };

    class CommunicationObj {
    protected:
        CommPacket_t sendPacket;
        CommPacket_t recvPacket;
        Command *commRequestCmd;

    public:
        virtual ~CommunicationObj() {}
        virtual void SendPacketToCommObj() = 0;
        virtual bool ReceivePacketFromCommObj() = 0;
        void GetPacketParams(CommPacket_t *packet) const;
        void SetCommand(Command *cmd);
        void ExecuteCommand(CommunicationType_t commType);
        bool IsRepetitiveRespondCommand() const;
    };

    class CommunicationModule {
    private:
        CommunicationObj *commObjPtr;
        CommunicationType_t commType;

    public:
        CommunicationModule(CommunicationObj *obj, CommunicationType_t type);
        void RunCommunicationModuleTask();
    };

    enum  UartMuxControl_t {
        MC33290_HONDA_MUX,
        MC33290_PIAGGIO_MUX,
        MC33290_YAMAHA_MUX,
        MC33290_YAMAHA_SMK_MUX,
        RL78_MUX,
        SCUTOOL_MUX
    };

    void MuxControl(UartMuxControl_t type);
    CommunicationType_t GetMainCommunication(void);
    void AddCommunication(CommunicationType_t type);
    void RemoveCommunication(CommunicationType_t type);
}

#endif /* __COMMON_H */
