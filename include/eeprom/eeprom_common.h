#ifndef __EEPROM_COMMON_H
#define __EEPROM_COMMON_H

#include "main.h"
#include "common.h"

#define MAX_EEPROM_BUF_SIZE 128
#define EEPROM_TIMEOUT      1000

#define EEPROM_E_OK			0

namespace EEPROM {
    enum EEPROMType_t {
        EEPROM_24CXX_1K,
        EEPROM_24CXX_2K,
        EEPROM_24CXX_4K,
        EEPROM_24CXX_8K,
        EEPROM_24CXX_16K,
        EEPROM_24CXX_32K,
        EEPROM_RL78,
        EEPROM_93CXX_46,
        EEPROM_93CXX_56,
        EEPROM_93CXX_66,
        EEPROM_93CXX_76,
        EEPROM_93CXX_86,
        EEPROM_9S12,
		EEPROM_9S12SEC,
        EEPROM_PIAGGIO_MIU,
        EEPROM_PIAGGIO_PG,
        EEPROM_PIAGGIO_AC19X,
        EEPROM_PIAGGIO_RISS,
        EEPROM_PIC16FXX,
        EEPROM_PIC18FXX
    };

    void GetCmdFromEEPROMRequest(EsyPro::CommunicationObj *commPtr);
}

#endif /* __EEPROM_COMMON_H */
