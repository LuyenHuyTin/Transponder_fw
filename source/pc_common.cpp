#include "pc_common.h"
#include "peripheral/timer.h"

using namespace EsyPro;

void PcCommObj::CalculateCRC16(CommPacket_t *packet) {
    uint8_t tmpBuffer[PC_UART_COM_BUF_SIZE];
    crcVal = 0xFFFF;

    memcpy(&tmpBuffer[0], &packet->cmd, 1);
    memcpy(&tmpBuffer[1], &packet->bufLen, 2);
    memcpy(&tmpBuffer[3], &packet->buffer, packet->bufLen);

    for (int i = 0; i < (packet->bufLen + 3); i++) {
        crcVal ^= tmpBuffer[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crcVal & 0x8000) {
                crcVal = (crcVal << 1) ^ 0x1021;
            } else {
                crcVal <<= 1;
            }
        }
    }
}

void PcCommObj::SendPacketToCommObj(void) {
    Peripheral::Uart1Obj *serialComm = Peripheral::Uart1Obj::GetInstance();

    if (sendPacket.cmd == (uint8_t)CMD_IGNORE_RES) {
        return;
    }

    this->CalculateCRC16(&sendPacket);
    serialComm->SendBytes((uint8_t *)&header, 2);
    serialComm->SendBytes(&sendPacket.cmd, 1);
    serialComm->SendBytes((uint8_t *)&sendPacket.bufLen, 2);
    serialComm->SendBytes(sendPacket.buffer, sendPacket.bufLen);
    serialComm->SendBytes((uint8_t *)&crcVal, 2);
    serialComm->SendBytes(&end, 1);
}

void PcCommObj::CheckforDisconnected(void) {
    Peripheral::Timer3Obj *timer = Peripheral::Timer3Obj::GetInstance();
    if (isConnected) {
        isConnected = false;
        timer->Start(PC_COMM_DISCONNECT_TIMEOUT);
    } else {
        if (timer->IsTimeout()) {
            RemoveCommunication(CommunicationType_t::PC_COMM_TYPE);
        }
    }
}

bool PcCommObj::ReceivePacketFromCommObj(void) {
    Peripheral::Uart1Obj *serialComm = Peripheral::Uart1Obj::GetInstance();
    static ReceiveStateType_t recvState = RECEIVE_HEADER1;
    uint8_t recvByte;

    if (serialComm->ReceiveByte(&recvByte)) {
        static uint16_t recvByteCnt = 0;
        static uint16_t crcRecv = 0;

        switch (recvState) {
        case RECEIVE_HEADER1:
            if (recvByte == (header & 0xFF)) {
                recvState = RECEIVE_HEADER2;
                crcRecv = 0;
                recvByteCnt = 0;
            }
            break;

        case RECEIVE_HEADER2:
            if (recvByte == ((header >> 8) & 0xFF)) {
                recvState = RECEIVE_CMD;
                memset(&recvPacket, 0, sizeof(recvPacket));
            } else {
                recvState = RECEIVE_HEADER1;
            }
            break;

        case RECEIVE_CMD:
            recvPacket.cmd = recvByte;
            recvState = RECEIVE_DATALEN;
            break;

        case RECEIVE_DATALEN:
            recvPacket.bufLen |= (recvByte << (8 * recvByteCnt));
            recvByteCnt++;
            if (recvByteCnt == 2) {
                if (recvPacket.bufLen <= COMM_OBJ_BUF_SIZE) {
                    if (recvPacket.bufLen == 0) {
                        recvState = RECEIVE_CRC;
                    } else {
                        recvState = RECEIVE_DATA;
                    }
                } else {
                    recvState = RECEIVE_HEADER1;
                }
                recvByteCnt = 0;
            }
            break;

        case RECEIVE_DATA:
            recvPacket.buffer[recvByteCnt++] = recvByte;
            if (recvByteCnt == recvPacket.bufLen) {
                recvState = RECEIVE_CRC;
                recvByteCnt = 0;
            }
            break;

        case RECEIVE_CRC:
            crcRecv |= recvByte << (8 * recvByteCnt);
            recvByteCnt++;
            if (recvByteCnt == 2) {
                this->CalculateCRC16(&recvPacket);
                if (crcRecv == crcVal) {
                    recvState = RECEIVE_END;
                } else {
                    recvState = RECEIVE_HEADER1;
                }
            }
            break;

        case RECEIVE_END:
            recvState = RECEIVE_HEADER1;
            if (recvByte == end) {
                isConnected = true;
                AddCommunication(CommunicationType_t::PC_COMM_TYPE);
                return true;
            }
            break;

        default:
            break;
        }
    }
    CheckforDisconnected();

    return false;
}
