#include "eeprom/eeprom_common.h"
#include "eeprom/at24cxx.h"
#include "eeprom/at93cxx.h"
#include "eeprom/rl78.h"
#include "eeprom/9s12.h"
#include "eeprom/9s12Sec.h"
#include "eeprom/pic16fxx.h"
#include "eeprom/pic18fxx.h"
#include "ecu/piaggio_flash.h"

using namespace EsyPro;
using namespace EEPROM;    

static bool isAT24CxxRequestCmd(EEPROMType_t type) {
    EEPROMType_t minIdx = EEPROM_24CXX_1K;
    EEPROMType_t maxIdx = EEPROM_24CXX_32K;

    if ((type >= minIdx) && (type <= maxIdx)) {
        return true;
    }

    return false;
}

static bool isAT93CxxRequestCmd(EEPROMType_t type) {
    EEPROMType_t minIdx = EEPROM_93CXX_46;
    EEPROMType_t maxIdx = EEPROM_93CXX_86;

    if ((type >= minIdx) && (type <= maxIdx)) {
        return true;
    }

    return false;
}

static bool isRL78RequestCmd(EEPROMType_t type) {
    if (type == EEPROM_RL78) {
        return true;
    }

    return false;
}

static bool isMC9S12RequestCmd(EEPROMType_t type) {
    if (type == EEPROM_9S12) {
        return true;
    }

    return false;
}

static bool isMC9S12SecRequestCmd(EEPROMType_t type) {
    if (type == EEPROM_9S12SEC) {
        return true;
    }

    return false;
}

static bool isPiaggioEEPRequestCmd(EEPROMType_t type) {
    EEPROMType_t minIdx = EEPROM_PIAGGIO_MIU;
    EEPROMType_t maxIdx = EEPROM_PIAGGIO_RISS;

    if ((type >= minIdx) && (type <= maxIdx)) {
        return true;
    }

    return false;
}

static bool isPic16fxxRequestCmd(EEPROMType_t type) {
    if (type == EEPROM_PIC16FXX) {
        return true;
    }

    return false;
}

static bool isPic18fxxRequestCmd(EEPROMType_t type) {
    if (type == EEPROM_PIC18FXX) {
        return true;
    }

    return false;
}

void EEPROM::GetCmdFromEEPROMRequest(EsyPro::CommunicationObj *commPtr) {
    Command *cmd = NULL;
    CommPacket_t reqPacket;
    static EEPROMType_t eepromType;

    commPtr->GetPacketParams(&reqPacket);
    if (reqPacket.cmd == CMD_BASIC_MEM_COMM_BASE) {
        eepromType = (EEPROMType_t)reqPacket.buffer[3];
    }

    if ((reqPacket.cmd & 0xF0) == CMD_BASIC_MEM_COMM_BASE) {
        if (isAT24CxxRequestCmd(eepromType)) {
            cmd = AT24CXX::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
        
        } else if (isAT93CxxRequestCmd(eepromType)) {
            cmd = AT93CXX::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

        } else if (isRL78RequestCmd(eepromType)) {
            cmd = RL78::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

        } 
         else if (isMC9S12RequestCmd(eepromType)) {
             cmd = MC9S12::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
         } 
        else if (isMC9S12SecRequestCmd(eepromType)) {
            cmd = MC9S12Sec::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
        }
        else if (isPiaggioEEPRequestCmd(eepromType)) {
            if (eepromType == EEPROM_PIAGGIO_MIU) {
                cmd = PiaggioMIUFlash::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

            } else if (eepromType == EEPROM_PIAGGIO_PG) {
                cmd = PiaggioPGFlash::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
            
            } else if (eepromType == EEPROM_PIAGGIO_AC19X) {
                cmd = PiaggioAC19xFlash::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

            } else if (eepromType == EEPROM_PIAGGIO_RISS) {
                cmd = PiaggioRISSFlash::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
            }
        
        } else if (isPic16fxxRequestCmd(eepromType)) {
            cmd = PIC16FXX::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

        } else if (isPic18fxxRequestCmd(eepromType)) {
            cmd = PIC18FXX::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
        }
    }

    if (cmd != NULL) {
        commPtr->SetCommand(cmd);
    }
}