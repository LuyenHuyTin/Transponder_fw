#ifndef __BLE_CHARS_H
#define __BLE_CHARS_H

#include "main.h"

#define CUSTOM_VALUE_CTR_RES_CHAR_UUID          0x1406
#define CUSTOM_VALUE_FIRMWARE_CONFIG_CHAR_UUID  0x1403
#define CUSTOM_VALUE_CTR_REQ_CHAR_UUID          0x1405
#define CUSTOM_VALUE_WRITE_CHAR_UUID            0x1401
#define CUSTOM_VALUE_READ_CHAR_UUID             0x1402
#define CUSTOM_VALUE_STATISTICS_CHAR_UUID       0x1404
#define CUSTOM_VALUE_ERRORS_CHAR_UUID           0x1408
#define CUSTOM_VALUE_PING_RES_CHAR_UUID         0x1407

#define BLE_CUS_DEF(_name)                       \
static BleCus_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,              \
                     BLE_HRS_BLE_OBSERVER_PRIO,  \
                     CusOnBleEvt, &_name)

namespace Peripheral {
    typedef enum {
        BLE_CUS_EVT_NOTIFICATION_ENABLED,
        BLE_CUS_EVT_NOTIFICATION_DISABLED,
        BLE_CUS_EVT_DISCONNECTED,
        BLE_CUS_EVT_CONNECTED
    } BleCusEvtType_t;

    typedef struct {
        BleCusEvtType_t evtType; /**< Type of event. */
    } BleCusEvt_t;

    typedef struct BleCusService BleCus_t;
    typedef void (*BleCusEvtHandler_t) (BleCus_t *pBas, BleCusEvt_t *pEvt);

    typedef struct {
        ble_srv_cccd_security_mode_t  customValueCharAttrMd;
        BleCusEvtHandler_t evtHandler;
        uint8_t initialCustomValue;
    } BleCusInit_t;

    struct BleCusService {
        nrf_queue_t *bleRxMonitorQueue;
        nrf_queue_t *bleRxControlQueue;
        BleCusEvtHandler_t evtHandler;
        ble_gatts_char_handles_t customCtrlRespondCharHandles;
        ble_gatts_char_handles_t customCfgFirmwareCharHandles;
        ble_gatts_char_handles_t customCtrlRequestCharHandles;
        ble_gatts_char_handles_t customWriteHandles;
        ble_gatts_char_handles_t customReadHandles;
        ble_gatts_char_handles_t customStatisticsHandles;
        ble_gatts_char_handles_t customErrorsHandles;
        ble_gatts_char_handles_t customPingResHandles;
        uint16_t serviceHandle;
        uint16_t connHandle;
        uint8_t uuidType;
    };

    ret_code_t CtrlRespondCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit);
    ret_code_t FirmwareCfgCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit);
    ret_code_t CtrlRequestCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit);
    ret_code_t WriteCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit);
    ret_code_t ReadCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit);
    ret_code_t StatisticsCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit);
    ret_code_t ErrorsCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit);
    ret_code_t PingResCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit);
};

#endif /* __BLE_CHARS_H */