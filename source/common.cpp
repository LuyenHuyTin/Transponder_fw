#include "common.h"
#include "peripheral/spi_flash.h"
#include "ecu/ecu_common.h"
#include "eeprom/eeprom_common.h"
#include "scutool/scutool.h"

using namespace EsyPro;
static CommunicationType_t connectedComm[2] = {NOT_CONNECTED, NOT_CONNECTED};
static const char LOG_DGB_NAME[] = "common";

CommunicationType_t EsyPro::GetMainCommunication(void) {
    return connectedComm[0];
}

void EsyPro::AddCommunication(CommunicationType_t type) {
    if (connectedComm[0] == NOT_CONNECTED) {
        connectedComm[0] = type;
        NRF_LOG_INFO("[%s]: INFO: mainComm: %d, subComm: %d",
                        LOG_DGB_NAME, connectedComm[0], connectedComm[1]);
    } else if ((connectedComm[1] == NOT_CONNECTED) 
        && (type != connectedComm[0])) {
        connectedComm[1] = type;
        NRF_LOG_INFO("[%s]: INFO: mainComm: %d, subComm: %d",
                        LOG_DGB_NAME, connectedComm[0], connectedComm[1]);
    }
}

void EsyPro::RemoveCommunication(CommunicationType_t type) {
    if (connectedComm[0] == type) {
        if (connectedComm[1] == NOT_CONNECTED) {
            connectedComm[0] = NOT_CONNECTED;
        } else {
            connectedComm[0] = connectedComm[1];
            connectedComm[1] = NOT_CONNECTED;
        }
        NRF_LOG_INFO("[%s]: INFO: mainComm: %d, subComm: %d",
                        LOG_DGB_NAME, connectedComm[0], connectedComm[1]);
    } else if (connectedComm[1] == type) {
        connectedComm[1] = NOT_CONNECTED;
        NRF_LOG_INFO("[%s]: INFO: mainComm: %d, subComm: %d",
                        LOG_DGB_NAME, connectedComm[0], connectedComm[1]);
    }
}

void EsyPro::MuxControl(UartMuxControl_t type) {
    switch (type) {
    case MC33290_HONDA_MUX:
    case MC33290_PIAGGIO_MUX:
    case MC33290_YAMAHA_MUX:
        nrf_gpio_pin_write(UART_MUX_PIN_S0, 1);
        nrf_gpio_pin_write(UART_MUX_PIN_S1, 1);
        break;

    case MC33290_YAMAHA_SMK_MUX:
    case RL78_MUX:
        nrf_gpio_pin_write(UART_MUX_PIN_S0, 0);
        nrf_gpio_pin_write(UART_MUX_PIN_S1, 0);
        break;
    
    case SCUTOOL_MUX:
        nrf_gpio_pin_write(UART_MUX_PIN_S0, 1);
        nrf_gpio_pin_write(UART_MUX_PIN_S1, 0);
        break;
    }
}

void Command::SetCommandRepeatState(bool state) {
    isRepeat = state;
}

bool Command::GetCommandRepeatState(void) const {
    return isRepeat;
}

void CommunicationObj::GetPacketParams(CommPacket_t *packet) const {
    memcpy(packet, &recvPacket, sizeof(recvPacket));
}

void CommunicationObj::SetCommand(Command *cmd) {
    commRequestCmd = cmd;
}

bool CommunicationObj::IsRepetitiveRespondCommand(void) const {
    return commRequestCmd->GetCommandRepeatState();
}

void CommunicationObj::ExecuteCommand(CommunicationType_t commType) {
    if (commRequestCmd != NULL) {
        commRequestCmd->SetCommandRepeatState(false);
        do {
            commRequestCmd->Execute(&sendPacket, &recvPacket, commType);
            this->SendPacketToCommObj();
        } while (this->IsRepetitiveRespondCommand()); 
    } else {
        NRF_LOG_INFO("[%s]: ERROR: Unknown COMMAND request", LOG_DGB_NAME);
    }
}

static void GetCmdFromCommonRequest(CommunicationObj *commPtr) {
    Command *cmd = NULL;
    CommPacket_t reqPacket;

    commPtr->GetPacketParams(&reqPacket);

    if (reqPacket.cmd == CMD_BLE_OTA_DFU_MODE) {
        sd_power_gpregret_clr(0, 0xFFFFFFFF);
        sd_power_gpregret_set(0, 0xB1);
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);
        return;
    }

    if ((CommunicationCmd_t)reqPacket.cmd == CMD_PING_REQ) {
        nrf_gpio_pin_clear(POW_OUT_5V);
        nrf_gpio_pin_set(POW_OUT_8V);
        nrf_gpio_pin_clear(POW_OUT_12V);
    }

    switch ((CommunicationCmd_t)reqPacket.cmd & 0xF0) {
    case CMD_QSPI_FLASH_BASE:
        cmd = SpiFlash::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
        break;
    }

    if (cmd != NULL) {
        commPtr->SetCommand(cmd);
    }
}

CommunicationModule::CommunicationModule(CommunicationObj *obj,
    CommunicationType_t type) : commObjPtr(obj), commType(type) {}

void CommunicationModule::RunCommunicationModuleTask(void) {
    commObjPtr->SetCommand(NULL);
    if (commObjPtr->ReceivePacketFromCommObj()) {
        GetCmdFromCommonRequest(commObjPtr);
        ECU::GetCmdFromEcuRequest(commObjPtr, commType);
        EEPROM::GetCmdFromEEPROMRequest(commObjPtr);
        SCUTool::GetCmdFromScutoolRequest(commObjPtr);

        commObjPtr->ExecuteCommand(commType);
    }
}