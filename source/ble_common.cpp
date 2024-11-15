#include "ble_common.h"

using namespace EsyPro;
using namespace Peripheral;

void BleCommObj::SendPacketToCommObj(void) {
    if (sendPacket.cmd == (uint8_t)CMD_IGNORE_RES) {
        return;
    }

    if (sendPacket.bufLen == 0) {
        sendPacket.bufLen = 1;
        sendPacket.buffer[0] = sendPacket.cmd;
    }

    ble.UpdateFromChar(&sendPacket);
}

bool BleCommObj::ReceivePacketFromCommObj(void) {
    static ReceiveStateType_t recvState = RECEIVE_FULL_LEN;
    static uint16_t recvByteCnt = 0;
    static uint16_t fullLen = 0;
    uint8_t recvByte;

    if (!ble.ReceiveFromChar(&recvByte)) {
        return false;
    }

    switch (recvState) {
    case RECEIVE_FULL_LEN:
        fullLen |= (recvByte << (8 * recvByteCnt));
        recvByteCnt++;
        if (recvByteCnt == 2) {
            recvState = RECEIVE_CMD;
            recvByteCnt = 0;
        }
        break;
    
    case RECEIVE_CMD:    
        recvPacket.cmd = recvByte;
        recvPacket.bufLen = fullLen - 1;
        if (recvPacket.bufLen == 0) {
            fullLen = 0;
            recvByteCnt = 0;
            recvState = RECEIVE_FULL_LEN;
            return true;
        } else {
            recvState = RECEIVE_DATA;
        }
        break;

    case RECEIVE_DATA:
        recvPacket.buffer[recvByteCnt++] = recvByte;
        if (recvPacket.bufLen == recvByteCnt) {
            fullLen = 0;
            recvByteCnt = 0;
            recvState = RECEIVE_FULL_LEN;
            return true;
        }
        break;

    default:
        break;
    }

    return false;
}
