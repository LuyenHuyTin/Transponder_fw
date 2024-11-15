#ifndef __BLE_COMMON_H
#define __BLE_COMMON_H

#include "peripheral/ble.h"

class BleCommObj : public EsyPro::CommunicationObj {
private:
    Peripheral::BleObj ble;
    enum ReceiveStateType_t {
        RECEIVE_FULL_LEN,
        RECEIVE_CMD,
        RECEIVE_DATA
    };

public:
    BleCommObj() = default;
    void SendPacketToCommObj() override;
    bool ReceivePacketFromCommObj() override;
};

#endif /* __BLE_COMMON_H */
