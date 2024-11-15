#include "peripheral/ble.h"
#include "app_timer.h"

using namespace EsyPro;
using namespace Peripheral;
static const char LOG_DGB_NAME[] = "ble";

NRF_BLE_QWR_DEF(mBleQwr);
NRF_BLE_GATT_DEF(mBleGatt);
BLE_ADVERTISING_DEF(mAdvertising);
BLE_CUS_DEF(bleCus);

NRF_QUEUE_DEF(uint8_t, bleRxMonitorQueue_t, 
              (3 * BLE_COM_MON_BUF_SIZE), NRF_QUEUE_MODE_OVERFLOW);

NRF_QUEUE_DEF(uint8_t, bleRxControlQueue_t, 
              (3 * BLE_COM_BUF_SIZE), NRF_QUEUE_MODE_OVERFLOW);

static uint16_t mConnHandle = BLE_CONN_HANDLE_INVALID; 
static ble_uuid_t mAdvUUIDs[] = { {BLE_UUID_DEVICE_INFORMATION_SERVICE,
                                   BLE_UUID_TYPE_BLE} };

static void BleEvtHandler(const ble_evt_t *pBleEvt, void *pContext) {
    ret_code_t errCode;
    const ble_gap_phys_t phys = {BLE_GAP_PHY_AUTO, BLE_GAP_PHY_AUTO};

    switch (pBleEvt->header.evt_id) {
    case BLE_GAP_EVT_DISCONNECTED:
        RemoveCommunication(CommunicationType_t::BLE_COMM_TYPE);
        NRF_LOG_INFO("[%s]: INFO: BLE Disconnected", LOG_DGB_NAME);
        break;

    case BLE_GAP_EVT_CONNECTED:
        mConnHandle = pBleEvt->evt.gap_evt.conn_handle;
        errCode = nrf_ble_qwr_conn_handle_assign(&mBleQwr, mConnHandle);
        APP_ERROR_CHECK(errCode);
        AddCommunication(CommunicationType_t::BLE_COMM_TYPE);
        NRF_LOG_INFO("[%s]: INFO: BLE Connected", LOG_DGB_NAME);
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        errCode = sd_ble_gap_phy_update(pBleEvt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(errCode);
        NRF_LOG_INFO("[%s]: INFO: BLE Update PHY", LOG_DGB_NAME);
        break;
    
    case BLE_GATTC_EVT_TIMEOUT:
        errCode = sd_ble_gap_disconnect(pBleEvt->evt.gattc_evt.conn_handle,
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(errCode);
        NRF_LOG_INFO("[%s]: INFO: BLE GATTC Timeout", LOG_DGB_NAME);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        errCode = sd_ble_gap_disconnect(pBleEvt->evt.gatts_evt.conn_handle,
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(errCode);
        NRF_LOG_INFO("[%s]: INFO: BLE GATTS Timeout", LOG_DGB_NAME);
        break;
    }
}

static void OnCusEvt(BleCus_t *pCusService, BleCusEvt_t *pEvt) {
    switch (pEvt->evtType) {
    case BLE_CUS_EVT_NOTIFICATION_ENABLED:
    case BLE_CUS_EVT_NOTIFICATION_DISABLED:
    case BLE_CUS_EVT_CONNECTED:
    case BLE_CUS_EVT_DISCONNECTED:
        break;
    }
}

static void QwrErrorHandler(uint32_t nrfError) {
    APP_ERROR_HANDLER(nrfError);
}

static ret_code_t BleCusInit(BleCus_t *pCus, const BleCusInit_t *pCusInit) {
    ret_code_t errCode;
    ble_uuid_t bleUUID;
    ble_uuid128_t baseUUID = {CUSTOM_SERVICE_UUID_BASE};

    if ((pCus == NULL) || (pCusInit == NULL)) {
        return NRF_ERROR_NULL;
    }

    pCus->evtHandler = pCusInit->evtHandler;
    pCus->connHandle = BLE_CONN_HANDLE_INVALID;
    errCode = sd_ble_uuid_vs_add(&baseUUID, &pCus->uuidType);
    APP_ERROR_CHECK(errCode);

    bleUUID.type = pCus->uuidType;
    bleUUID.uuid = CUSTOM_SERVICE_UUID;
    errCode = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                       &bleUUID, &pCus->serviceHandle);
    APP_ERROR_CHECK(errCode);

    errCode = CtrlRespondCharAdd(pCus, pCusInit);
    APP_ERROR_CHECK(errCode);
    errCode = FirmwareCfgCharAdd(pCus, pCusInit);
    APP_ERROR_CHECK(errCode);
    errCode = CtrlRequestCharAdd(pCus, pCusInit);
    APP_ERROR_CHECK(errCode);
    errCode = WriteCharAdd(pCus, pCusInit);
    APP_ERROR_CHECK(errCode);
    errCode = ReadCharAdd(pCus, pCusInit);
    APP_ERROR_CHECK(errCode);
    errCode = StatisticsCharAdd(pCus, pCusInit);
    APP_ERROR_CHECK(errCode);
    errCode = ErrorsCharAdd(pCus, pCusInit);
    APP_ERROR_CHECK(errCode);
    errCode = PingResCharAdd(pCus, pCusInit);
    APP_ERROR_CHECK(errCode);

    return errCode;
}

static void OnConnect(BleCus_t *pCus, const ble_evt_t *pBleEvt) {
    BleCusEvt_t cusEvt;

    pCus->connHandle = pBleEvt->evt.gap_evt.conn_handle;
    cusEvt.evtType = BLE_CUS_EVT_CONNECTED;
    pCus->evtHandler(pCus, &cusEvt);
}

static void OnDisconnect(BleCus_t *pCus, const ble_evt_t *pBleEvt) {
    pCus->connHandle = BLE_CONN_HANDLE_INVALID;
}

static bool IsMonitorCmd(uint8_t cmd) {
    bool isMonitor = false;

    switch (cmd) {
    case CMD_PING_REQ:
    case CMD_STATISTIC_REQ:
    case CMD_ERROR_REQ:
    case CMD_DIAGNOSTIC_REQ:
        isMonitor = true;
        break;
    
    default:
        isMonitor = false;
        break;
    }

    return isMonitor;
}

static void OnWrite(BleCus_t *pCus, const ble_evt_t *pBleEvt) {
    BleCusEvt_t bleCusEvt;
    const ble_gatts_evt_write_t *pEvtWrite = &pBleEvt->evt.gatts_evt.params.write;
    uint8_t hLenByte, lLenByte;
    uint8_t byteData;

    if ((pEvtWrite->handle == pCus->customWriteHandles.cccd_handle)
        && (pEvtWrite->len == 2)) {
        if (pCus->evtHandler != NULL) {
            if (ble_srv_is_notification_enabled(pEvtWrite->data)) {
                bleCusEvt.evtType = BLE_CUS_EVT_NOTIFICATION_ENABLED;
                NRF_LOG_INFO("[%s]: INFO: Notification Enabled", LOG_DGB_NAME);
            } else {
                bleCusEvt.evtType = BLE_CUS_EVT_NOTIFICATION_DISABLED;
                NRF_LOG_INFO("[%s]: INFO: Notification Disabled", LOG_DGB_NAME);
            }
            pCus->evtHandler(pCus, &bleCusEvt);
        }
    } else {
        if (pEvtWrite->data == NULL) {
            return;
        }

        nrf_queue_t *bleQueue = IsMonitorCmd(pEvtWrite->data[0])
                                ? pCus->bleRxMonitorQueue : pCus->bleRxControlQueue;
        lLenByte = (pEvtWrite->len & 0xFF);
        hLenByte = (pEvtWrite->len >> 8) & 0xFF;

        if (pEvtWrite->handle == pCus->customWriteHandles.value_handle) {
            nrf_queue_push(bleQueue, (void *)&lLenByte);
            nrf_queue_push(bleQueue, (void *)&hLenByte);
            for (int i = 0; i < pEvtWrite->len; i++) {
                byteData = pEvtWrite->data[i];
                nrf_queue_push(bleQueue, (void *)&byteData);
            }
            return;
        }
        if (pEvtWrite->handle == pCus->customCtrlRequestCharHandles.value_handle) {
            nrf_queue_push(bleQueue, (void *)&lLenByte);
            nrf_queue_push(bleQueue, (void *)&hLenByte);
            for (int i = 0; i < pEvtWrite->len; i++) {
                byteData = pEvtWrite->data[i];
                nrf_queue_push(bleQueue, (void *)&byteData);
            }
            return;
        } 
        if (pEvtWrite->handle == pCus->customCfgFirmwareCharHandles.value_handle) {
            nrf_queue_push(bleQueue, (void *)&lLenByte);
            nrf_queue_push(bleQueue, (void *)&hLenByte);
            for (int i = 0; i < pEvtWrite->len; i++) {
                byteData = pEvtWrite->data[i];
                nrf_queue_push(bleQueue, (void *)&byteData);
            }
            return;
        }
    }
}

void Peripheral::CusOnBleEvt(ble_evt_t const *pBleEvt, void *pContext) {
    BleCus_t *pCus = static_cast<BleCus_t *>(pContext);

    if (pCus == NULL || pBleEvt == NULL) {
        return;
    }

    switch (pBleEvt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        OnConnect(pCus, pBleEvt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        OnDisconnect(pCus, pBleEvt);
        break;
    
    case BLE_GATTS_EVT_WRITE:
        OnWrite(pCus, pBleEvt);
        break;
    }
}

static void OnConnParamsEvt(ble_conn_params_evt_t *pEvt) {
    ret_code_t errCode;

    if (pEvt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        errCode = sd_ble_gap_disconnect(mConnHandle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(errCode);
    }
}

static void ConnParamsErrorHandler(uint32_t nrfError) {
    APP_ERROR_HANDLER(nrfError);
}

BleObj::BleObj(void) {
    pBleCus = &bleCus;
    pBleCus->bleRxMonitorQueue = (nrf_queue_t *)&bleRxMonitorQueue_t;
    pBleCus->bleRxControlQueue = (nrf_queue_t *)&bleRxControlQueue_t;
    Init();
}

void BleObj::StackInit(void) {
    ret_code_t errCode;
    uint32_t ramStart = 0;

    errCode = nrf_sdh_enable_request();
    APP_ERROR_CHECK(errCode);
    
    errCode = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ramStart);
    APP_ERROR_CHECK(errCode);

    errCode = nrf_sdh_ble_enable(&ramStart);
    APP_ERROR_CHECK(errCode);

    NRF_SDH_BLE_OBSERVER(mBleObserver, APP_BLE_OBSERVER_PRIO, BleEvtHandler, NULL);
}

void BleObj::GapParamsInit(void) {
    ret_code_t errCode;
    ble_gap_conn_params_t gapConnParams;
    ble_gap_conn_sec_mode_t secMode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&secMode);
    errCode = sd_ble_gap_device_name_set(&secMode, 
                reinterpret_cast<const uint8_t *>(DEVICE_NAME), strlen(DEVICE_NAME));
    APP_ERROR_CHECK(errCode);

    memset(&gapConnParams, 0, sizeof(gapConnParams));
    gapConnParams.min_conn_interval = MIN_CONN_INTERVAL;
    gapConnParams.max_conn_interval = MAX_CONN_INTERVAL;
    gapConnParams.slave_latency = SLAVE_LATENCY;
    gapConnParams.conn_sup_timeout = CONN_SUP_TIMEOUT;
    errCode = sd_ble_gap_ppcp_set(&gapConnParams);
    APP_ERROR_CHECK(errCode);
}

void BleObj::GattInit(void) {
    ret_code_t errCode;

    errCode = nrf_ble_gatt_init(&mBleGatt, NULL);
    APP_ERROR_CHECK(errCode);
}

void BleObj::ServicesInit(void) {
    ret_code_t errCode;
    nrf_ble_qwr_init_t qwrInit = {0};
    BleCusInit_t cusInit;

    qwrInit.error_handler = QwrErrorHandler;
    errCode = nrf_ble_qwr_init(&mBleQwr, &qwrInit);
    APP_ERROR_CHECK(errCode);

    memset(&cusInit, 0, sizeof(cusInit));
    cusInit.evtHandler = OnCusEvt;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cusInit.customValueCharAttrMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cusInit.customValueCharAttrMd.write_perm);
    errCode = BleCusInit(pBleCus, &cusInit);
    APP_ERROR_CHECK(errCode);
}

void BleObj::AdvertisingInit(void) {
    ret_code_t errCode;
    ble_advertising_init_t advInit;

    memset(&advInit, 0, sizeof(advInit));
    advInit.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advInit.advdata.include_appearance = true;
    advInit.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advInit.advdata.uuids_complete.uuid_cnt = sizeof(mAdvUUIDs) / sizeof(mAdvUUIDs[0]);
    advInit.advdata.uuids_complete.p_uuids = mAdvUUIDs;
    advInit.config.ble_adv_fast_enabled = true;
    advInit.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    advInit.config.ble_adv_fast_timeout = APP_ADV_DURATION;
    errCode = ble_advertising_init(&mAdvertising, &advInit);
    APP_ERROR_CHECK(errCode);
    ble_advertising_conn_cfg_tag_set(&mAdvertising, APP_BLE_CONN_CFG_TAG);
}

void BleObj::ConnParamsInit(void) {
    ret_code_t errCode;
    ble_conn_params_init_t connParamsInit;

    memset(&connParamsInit, 0, sizeof(connParamsInit));
    connParamsInit.p_conn_params = NULL;
    connParamsInit.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    connParamsInit.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    connParamsInit.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    connParamsInit.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    connParamsInit.disconnect_on_fail = false;
    connParamsInit.evt_handler = OnConnParamsEvt;
    connParamsInit.error_handler = ConnParamsErrorHandler;
    errCode = ble_conn_params_init(&connParamsInit);
    APP_ERROR_CHECK(errCode);    
}

void BleObj::Init(void) {
    ret_code_t errCode;

    errCode = app_timer_init();
    APP_ERROR_CHECK(errCode);
    errCode = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(errCode);

    StackInit();
    GapParamsInit();
    GattInit();
    ServicesInit();
    AdvertisingInit();
    ConnParamsInit();

    errCode = ble_advertising_start(&mAdvertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(errCode);
}

bool BleObj::ReceiveFromChar(uint8_t *pData) {
    ret_code_t errCode;

    if (!nrf_queue_is_empty(pBleCus->bleRxControlQueue)) {
        errCode = nrf_queue_pop(pBleCus->bleRxControlQueue, pData);
        APP_ERROR_CHECK(errCode);
        return true;
    }    

    if (!nrf_queue_is_empty(pBleCus->bleRxMonitorQueue)) {
        errCode = nrf_queue_pop(pBleCus->bleRxMonitorQueue, pData);
        APP_ERROR_CHECK(errCode);
        return true;
    }

    return false;
}

void BleObj::UpdateFromChar(CommPacket_t *packet) {
    ret_code_t errCode;
    ble_gatts_value_t gattsVal;
    ble_gatts_hvx_params_t hvxParams;

    if (pBleCus == NULL) {
        return;
    }

    memset(&gattsVal, 0, sizeof(gattsVal));
    gattsVal.len = packet->bufLen;
    gattsVal.offset = 0;
    gattsVal.p_value = packet->buffer;

    memset(&hvxParams, 0, sizeof(hvxParams));
    if (packet->bleUUID == CUSTOM_VALUE_READ_CHAR_UUID) {
        errCode = sd_ble_gatts_value_set(pBleCus->connHandle,
                                         pBleCus->customReadHandles.value_handle,
                                         &gattsVal);
        if (errCode != NRF_SUCCESS) {
            return;
        }
        hvxParams.handle = pBleCus->customReadHandles.value_handle;

    } else if (packet->bleUUID == CUSTOM_VALUE_CTR_RES_CHAR_UUID) {
        errCode = sd_ble_gatts_value_set(pBleCus->connHandle,
                                         pBleCus->customCtrlRespondCharHandles.value_handle,
                                         &gattsVal);
        if (errCode != NRF_SUCCESS) {
            return;
        }
        hvxParams.handle = pBleCus->customCtrlRespondCharHandles.value_handle;

    } else if (packet->bleUUID == CUSTOM_VALUE_PING_RES_CHAR_UUID) {
        errCode = sd_ble_gatts_value_set(pBleCus->connHandle,
                                         pBleCus->customPingResHandles.value_handle,
                                         &gattsVal);
        if (errCode != NRF_SUCCESS) {
            return;
        }
        hvxParams.handle = pBleCus->customPingResHandles.value_handle;
    
    } else if (packet->bleUUID == CUSTOM_VALUE_STATISTICS_CHAR_UUID) {
        errCode = sd_ble_gatts_value_set(pBleCus->connHandle,
                                         pBleCus->customStatisticsHandles.value_handle,
                                         &gattsVal);
        if (errCode != NRF_SUCCESS) {
            return;
        }
        hvxParams.handle = pBleCus->customStatisticsHandles.value_handle;   
    
    } else if (packet->bleUUID == CUSTOM_VALUE_ERRORS_CHAR_UUID) {
        errCode = sd_ble_gatts_value_set(pBleCus->connHandle,
                                         pBleCus->customErrorsHandles.value_handle,
                                         &gattsVal);
        if (errCode != NRF_SUCCESS) {
            return;
        }
        hvxParams.handle = pBleCus->customErrorsHandles.value_handle;
        
    } else if (packet->bleUUID == CUSTOM_VALUE_ERRORS_CHAR_UUID) {
        errCode = sd_ble_gatts_value_set(pBleCus->connHandle,
                                         pBleCus->customErrorsHandles.value_handle,
                                         &gattsVal);
        if (errCode != NRF_SUCCESS) {
            return;
        }
        hvxParams.handle = pBleCus->customErrorsHandles.value_handle;
        
    }
    
    if (pBleCus->connHandle != BLE_CONN_HANDLE_INVALID) {
        hvxParams.type = BLE_GATT_HVX_NOTIFICATION;
        hvxParams.offset = gattsVal.offset;
        hvxParams.p_len = &gattsVal.len;
        hvxParams.p_data = gattsVal.p_value;

        sd_ble_gatts_hvx(pBleCus->connHandle, &hvxParams);
    }
}