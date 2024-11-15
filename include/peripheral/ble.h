#ifndef __BLE_H
#define __BLE_H

#include "main.h"
#include "common.h"
#include "peripheral/ble_chars.h"

#define APP_BLE_CONN_CFG_TAG    1
#define APP_BLE_OBSERVER_PRIO   3
#define DEVICE_NAME             "ESYPROV2"

#define MIN_CONN_INTERVAL       MSEC_TO_UNITS(15, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL       MSEC_TO_UNITS(40, UNIT_1_25_MS)
#define CONN_SUP_TIMEOUT        MSEC_TO_UNITS(4000, UNIT_10_MS)

#define SLAVE_LATENCY           0
#define APP_ADV_INTERVAL        64
#define APP_ADV_DURATION        0

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)     /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)    /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define CUSTOM_SERVICE_UUID_BASE    {0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, \
                                     0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, 0xF3}
#define CUSTOM_SERVICE_UUID         0x1400

namespace Peripheral {
    class BleObj {
    private:
        ble_advertising_t mBleAdv;
        BleCus_t *pBleCus;
        void AdvertisingStart();
        void ConnParamsInit();
        void ServicesInit();
        void AdvertisingInit();
        void GattInit();
        void GapParamsInit();
        void StackInit();
        void Init();

    public:
        BleObj();
        bool ReceiveFromChar(uint8_t *pData);
        void UpdateFromChar(EsyPro::CommPacket_t *packet);
    };

    void CusOnBleEvt(ble_evt_t const *pBleEvt, void *pContext);
}

#endif /* __BLE_H */
