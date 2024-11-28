#ifndef __TRANSPONDER_COMMON_H
#define __TRANSPONDER_COMMON_H

#include "main.h"
#include "common.h"

// #define MAX_EEPROM_BUF_SIZE 128
// #define EEPROM_TIMEOUT      1000

// #define EEPROM_E_OK			0

namespace TRANSPONDER {
    enum TRANSPONDERType_t {
        TRANS_PCF7991,
        TRANS_TMS5030
    };

    void GetCmdFromTransponderRequest(EsyPro::CommunicationObj *commPtr);
}

#endif /* __TRANSPONDER_COMMON_H */
