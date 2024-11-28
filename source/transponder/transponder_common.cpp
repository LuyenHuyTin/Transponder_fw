#include "transponder/transponder_common.h"
#include "transponder/pcf7991.h"

using namespace EsyPro;
using namespace TRANSPONDER;

static bool isPcf7991RequestCmd(TRANSPONDERType_t type)
{
    if (type == TRANS_PCF7991)
    {
        return true;
    }

    return false;
}

void TRANSPONDER::GetCmdFromTransponderRequest(EsyPro::CommunicationObj *commPtr)
{
    NRF_LOG_INFO("recevice transponder");
    Command *cmd = NULL;
    CommPacket_t reqPacket;
    static TRANSPONDERType_t transType;

    commPtr->GetPacketParams(&reqPacket);
    // if (reqPacket.cmd == CMD_BASIC_MEM_COMM_BASE) {
    //     eepromType = (TRANSPONDERType_t)reqPacket.buffer[3];
    // }
    NRF_LOG_INFO("reqPacket.cmd: %x", (CommunicationCmd_t)reqPacket.cmd & 0xF0);
    if (((CommunicationCmd_t)reqPacket.cmd & 0xF0) == CMD_BASIC_TRANS_COMM_BASE)
    {
        // if (isPcf7991RequestCmd(transType))
        // {
        NRF_LOG_INFO("recevice transponder 2");
        cmd = PCF7991::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
        // }
    }

    if (cmd != NULL)
    {
        commPtr->SetCommand(cmd);
    }
}