#ifndef __PC_COMMON_H
#define __PC_COMMON_H

#include "main.h"
#include "common.h"
#include "peripheral/uart.h"

#define PC_COMM_DISCONNECT_TIMEOUT  5000

class PcCommObj : public EsyPro::CommunicationObj {
private:
    bool isConnected;
    const uint16_t header = 0x0A0D;
    uint16_t crcVal;
    const uint8_t end = 0x55;
    enum ReceiveStateType_t {
        RECEIVE_HEADER1,
        RECEIVE_HEADER2,
        RECEIVE_CMD,
        RECEIVE_DATALEN,
        RECEIVE_DATA,
        RECEIVE_CRC,
        RECEIVE_END
    };
    void CalculateCRC16(EsyPro::CommPacket_t *packet);
    void CheckforDisconnected();

public:
    PcCommObj() = default;
    void SendPacketToCommObj() override;
    bool ReceivePacketFromCommObj() override;
};

#endif /* __PC_COMMON_H */
