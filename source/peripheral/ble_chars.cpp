#include "peripheral/ble_chars.h"

ret_code_t Peripheral::CtrlRespondCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit) {
    ret_code_t errCode;
    uint8_t buffer[50];
    ble_gatts_attr_md_t cccdMd;
    ble_gatts_char_md_t charMd;
    ble_gatts_attr_md_t attrMd;
    ble_uuid_t bleUUID;
    ble_gatts_attr_t attrCharVal; 

    memset(&cccdMd, 0, sizeof(cccdMd));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.write_perm);
    cccdMd.vloc = BLE_GATTS_VLOC_STACK;

    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.read = 1;
    charMd.char_props.write = 0;
    charMd.char_props.notify = 1;
    charMd.p_char_user_desc = NULL;
    charMd.p_char_pf = NULL;
    charMd.p_user_desc_md = NULL;
    charMd.p_cccd_md = &cccdMd;
    charMd.p_sccd_md = NULL;

    memset(&attrMd, 0, sizeof(attrMd));
    attrMd.read_perm = pCusInit->customValueCharAttrMd.read_perm;
    attrMd.write_perm = pCusInit->customValueCharAttrMd.write_perm;
    attrMd.vloc = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen = 1;

    bleUUID.type = pCus->uuidType;
    bleUUID.uuid = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
    memset(&attrCharVal, 0, sizeof(attrCharVal));
    attrCharVal.p_uuid = &bleUUID;
    attrCharVal.p_attr_md = &attrMd;
    attrCharVal.init_len = 30 * sizeof(uint8_t);
    attrCharVal.init_offs = 0;
    attrCharVal.max_len = 30 * sizeof(uint8_t);
    attrCharVal.p_value = buffer;
    
    errCode = sd_ble_gatts_characteristic_add(pCus->serviceHandle, &charMd, &attrCharVal,
                                              &pCus->customCtrlRespondCharHandles);
    return errCode;
}

ret_code_t Peripheral::FirmwareCfgCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit) {
    ret_code_t errCode;
    uint8_t buffer[30];
    ble_gatts_char_md_t charMd;
    ble_gatts_attr_md_t attrMd;
    ble_uuid_t bleUUID;
    ble_gatts_attr_t attrCharVal;

    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.read = 1;
    charMd.char_props.write = 1;
    charMd.char_props.notify = 0;
    charMd.p_char_user_desc = NULL;
    charMd.p_char_pf = NULL;
    charMd.p_user_desc_md = NULL;
    charMd.p_cccd_md = NULL;
    charMd.p_sccd_md = NULL;       

    memset(&attrMd, 0, sizeof(attrMd));
    attrMd.read_perm = pCusInit->customValueCharAttrMd.read_perm;
    attrMd.write_perm = pCusInit->customValueCharAttrMd.write_perm;
    attrMd.vloc = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen = 1;

    bleUUID.type = pCus->uuidType;
    bleUUID.uuid = CUSTOM_VALUE_FIRMWARE_CONFIG_CHAR_UUID;
    memset(&attrCharVal, 0, sizeof(attrCharVal));
    attrCharVal.p_uuid = &bleUUID;
    attrCharVal.p_attr_md = &attrMd;
    attrCharVal.init_len = 20 * sizeof(uint8_t);
    attrCharVal.init_offs = 0;
    attrCharVal.max_len = 20 * sizeof(uint8_t);
    attrCharVal.p_value = buffer;
    errCode = sd_ble_gatts_characteristic_add(pCus->serviceHandle, &charMd, &attrCharVal,
                                              &pCus->customCfgFirmwareCharHandles);
    return errCode;
}

ret_code_t Peripheral::CtrlRequestCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit) {
    ret_code_t errCode;
    uint8_t buffer[5];
    ble_gatts_char_md_t charMd;
    ble_uuid_t bleUUID;
    ble_gatts_attr_md_t attrMd;
    ble_gatts_attr_t attrCharVal;

    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.read = 1;
    charMd.char_props.write = 1;
    charMd.char_props.notify = 0;
    charMd.p_char_user_desc = NULL;
    charMd.p_char_pf = NULL;
    charMd.p_user_desc_md = NULL;
    charMd.p_cccd_md = NULL;
    charMd.p_sccd_md = NULL; 

    memset(&attrMd, 0, sizeof(attrMd));
    attrMd.read_perm = pCusInit->customValueCharAttrMd.read_perm;
    attrMd.write_perm = pCusInit->customValueCharAttrMd.write_perm;
    attrMd.vloc = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen = 1;

    bleUUID.type = pCus->uuidType;
    bleUUID.uuid = CUSTOM_VALUE_CTR_REQ_CHAR_UUID;
    memset(&attrCharVal, 0, sizeof(attrCharVal));
    attrCharVal.p_uuid = &bleUUID;
    attrCharVal.p_attr_md = &attrMd;
    attrCharVal.init_len = 5 * sizeof(uint8_t);
    attrCharVal.init_offs = 0;
    attrCharVal.max_len = 5 * sizeof(uint8_t);
    attrCharVal.p_value = buffer;

    errCode = sd_ble_gatts_characteristic_add(pCus->serviceHandle, &charMd, &attrCharVal,
                                              &pCus->customCtrlRequestCharHandles);
    return errCode;
}

ret_code_t Peripheral::WriteCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit) {
    ret_code_t errCode;
    uint8_t buffer[BLE_COM_BUF_SIZE];
    ble_gatts_char_md_t charMd;
    ble_uuid_t bleUUID;
    ble_gatts_attr_md_t attrMd;
    ble_gatts_attr_t attrCharVal;

    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.read = 1;
    charMd.char_props.write = 1;
    charMd.char_props.notify = 0;
    charMd.p_char_user_desc = NULL;
    charMd.p_char_pf = NULL;
    charMd.p_user_desc_md = NULL;
    charMd.p_cccd_md = NULL;
    charMd.p_sccd_md = NULL;

    memset(&attrMd, 0, sizeof(attrMd));
    attrMd.read_perm = pCusInit->customValueCharAttrMd.read_perm;
    attrMd.write_perm = pCusInit->customValueCharAttrMd.write_perm;
    attrMd.vloc = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen = 1;    

    bleUUID.type = pCus->uuidType;
    bleUUID.uuid = CUSTOM_VALUE_WRITE_CHAR_UUID;
    memset(&attrCharVal, 0, sizeof(attrCharVal));
    attrCharVal.p_uuid = &bleUUID;
    attrCharVal.p_attr_md = &attrMd;
    attrCharVal.init_len = BLE_COM_BUF_SIZE * sizeof(uint8_t);
    attrCharVal.init_offs = 0;
    attrCharVal.max_len = BLE_COM_BUF_SIZE * sizeof(uint8_t);
    attrCharVal.p_value = buffer;

    errCode = sd_ble_gatts_characteristic_add(pCus->serviceHandle, &charMd, &attrCharVal,
                                              &pCus->customWriteHandles);
    return errCode;
}

ret_code_t Peripheral::ReadCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit) {
    ret_code_t errCode;
    uint8_t buffer[BLE_COM_BUF_SIZE];
    ble_gatts_char_md_t charMd;
    ble_gatts_attr_md_t cccdMd;
    ble_uuid_t bleUUID;
    ble_gatts_attr_md_t attrMd;
    ble_gatts_attr_t attrCharVal;

    memset(&cccdMd, 0, sizeof(cccdMd));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.write_perm);
    cccdMd.vloc = BLE_GATTS_VLOC_STACK;

    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.read = 1;
    charMd.char_props.write = 0;
    charMd.char_props.notify = 1;
    charMd.p_char_user_desc = NULL;
    charMd.p_char_pf = NULL;
    charMd.p_user_desc_md = NULL;
    charMd.p_cccd_md = &cccdMd;
    charMd.p_sccd_md = NULL;

    memset(&attrMd, 0, sizeof(attrMd));
    attrMd.read_perm = pCusInit->customValueCharAttrMd.read_perm;
    attrMd.write_perm = pCusInit->customValueCharAttrMd.write_perm;
    attrMd.vloc = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen = 1;

    bleUUID.type = pCus->uuidType;
    bleUUID.uuid = CUSTOM_VALUE_READ_CHAR_UUID;
    memset(&attrCharVal, 0, sizeof(attrCharVal));
    attrCharVal.p_uuid = &bleUUID;
    attrCharVal.p_attr_md = &attrMd;
    attrCharVal.init_len = BLE_COM_BUF_SIZE * sizeof(uint8_t);
    attrCharVal.init_offs = 0;
    attrCharVal.max_len = BLE_COM_BUF_SIZE * sizeof(uint8_t);
    attrCharVal.p_value = buffer;

    errCode = sd_ble_gatts_characteristic_add(pCus->serviceHandle, &charMd, &attrCharVal,
                                              &pCus->customReadHandles);    
    return errCode;
}

ret_code_t Peripheral::StatisticsCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit) {
    ret_code_t errCode;
    uint8_t buffer[BLE_COM_BUF_SIZE];
    ble_gatts_char_md_t charMd;
    ble_gatts_attr_md_t cccdMd;
    ble_uuid_t bleUUID;
    ble_gatts_attr_md_t attrMd;
    ble_gatts_attr_t attrCharVal;

    memset(&cccdMd, 0, sizeof(cccdMd));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.write_perm);
    cccdMd.vloc = BLE_GATTS_VLOC_STACK;

    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.read = 1;
    charMd.char_props.write = 0;
    charMd.char_props.notify = 1;
    charMd.p_char_user_desc = NULL;
    charMd.p_char_pf = NULL;
    charMd.p_user_desc_md = NULL;
    charMd.p_cccd_md = &cccdMd;
    charMd.p_sccd_md = NULL;

    memset(&attrMd, 0, sizeof(attrMd));
    attrMd.read_perm = pCusInit->customValueCharAttrMd.read_perm;
    attrMd.write_perm = pCusInit->customValueCharAttrMd.write_perm;
    attrMd.vloc = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen = 1;

    bleUUID.type = pCus->uuidType;
    bleUUID.uuid = CUSTOM_VALUE_STATISTICS_CHAR_UUID;
    memset(&attrCharVal, 0, sizeof(attrCharVal));
    attrCharVal.p_uuid = &bleUUID;
    attrCharVal.p_attr_md = &attrMd;
    attrCharVal.init_len = BLE_COM_BUF_SIZE * sizeof(uint8_t);
    attrCharVal.init_offs = 0;
    attrCharVal.max_len = BLE_COM_BUF_SIZE * sizeof(uint8_t);
    attrCharVal.p_value = buffer;

    errCode = sd_ble_gatts_characteristic_add(pCus->serviceHandle, &charMd, &attrCharVal,
                                              &pCus->customStatisticsHandles);    
    return errCode;
}

ret_code_t Peripheral::ErrorsCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit) {
    ret_code_t errCode;
    uint8_t buffer[BLE_COM_BUF_SIZE];
    ble_gatts_char_md_t charMd;
    ble_gatts_attr_md_t cccdMd;
    ble_uuid_t bleUUID;
    ble_gatts_attr_md_t attrMd;
    ble_gatts_attr_t attrCharVal;

    memset(&cccdMd, 0, sizeof(cccdMd));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.write_perm);
    cccdMd.vloc = BLE_GATTS_VLOC_STACK;

    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.read = 1;
    charMd.char_props.write = 0;
    charMd.char_props.notify = 1;
    charMd.p_char_user_desc = NULL;
    charMd.p_char_pf = NULL;
    charMd.p_user_desc_md = NULL;
    charMd.p_cccd_md = &cccdMd;
    charMd.p_sccd_md = NULL;

    memset(&attrMd, 0, sizeof(attrMd));
    attrMd.read_perm = pCusInit->customValueCharAttrMd.read_perm;
    attrMd.write_perm = pCusInit->customValueCharAttrMd.write_perm;
    attrMd.vloc = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen = 1;

    bleUUID.type = pCus->uuidType;
    bleUUID.uuid = CUSTOM_VALUE_ERRORS_CHAR_UUID;
    memset(&attrCharVal, 0, sizeof(attrCharVal));
    attrCharVal.p_uuid = &bleUUID;
    attrCharVal.p_attr_md = &attrMd;
    attrCharVal.init_len = BLE_COM_BUF_SIZE * sizeof(uint8_t);
    attrCharVal.init_offs = 0;
    attrCharVal.max_len = BLE_COM_BUF_SIZE * sizeof(uint8_t);
    attrCharVal.p_value = buffer;

    errCode = sd_ble_gatts_characteristic_add(pCus->serviceHandle, &charMd, &attrCharVal,
                                              &pCus->customErrorsHandles);    
    return errCode;
}

ret_code_t Peripheral::PingResCharAdd(BleCus_t *pCus, const BleCusInit_t *pCusInit) {
    ret_code_t errCode;
    uint8_t buffer[BLE_COM_BUF_SIZE];
    ble_gatts_char_md_t charMd;
    ble_gatts_attr_md_t cccdMd;
    ble_uuid_t bleUUID;
    ble_gatts_attr_md_t attrMd;
    ble_gatts_attr_t attrCharVal;

    memset(&cccdMd, 0, sizeof(cccdMd));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccdMd.write_perm);
    cccdMd.vloc = BLE_GATTS_VLOC_STACK;

    memset(&charMd, 0, sizeof(charMd));
    charMd.char_props.read = 1;
    charMd.char_props.write = 0;
    charMd.char_props.notify = 1;
    charMd.p_char_user_desc = NULL;
    charMd.p_char_pf = NULL;
    charMd.p_user_desc_md = NULL;
    charMd.p_cccd_md = &cccdMd;
    charMd.p_sccd_md = NULL;

    memset(&attrMd, 0, sizeof(attrMd));
    attrMd.read_perm = pCusInit->customValueCharAttrMd.read_perm;
    attrMd.write_perm = pCusInit->customValueCharAttrMd.write_perm;
    attrMd.vloc = BLE_GATTS_VLOC_STACK;
    attrMd.rd_auth = 0;
    attrMd.wr_auth = 0;
    attrMd.vlen = 1;

    bleUUID.type = pCus->uuidType;
    bleUUID.uuid = CUSTOM_VALUE_PING_RES_CHAR_UUID;
    memset(&attrCharVal, 0, sizeof(attrCharVal));
    attrCharVal.p_uuid = &bleUUID;
    attrCharVal.p_attr_md = &attrMd;
    attrCharVal.init_len = BLE_COM_BUF_SIZE * sizeof(uint8_t);
    attrCharVal.init_offs = 0;
    attrCharVal.max_len = BLE_COM_BUF_SIZE * sizeof(uint8_t);
    attrCharVal.p_value = buffer;

    errCode = sd_ble_gatts_characteristic_add(pCus->serviceHandle, &charMd, &attrCharVal,
                                              &pCus->customPingResHandles);    
    return errCode;
}