#include "peripheral/i2c.h"
#include "peripheral/timer.h"

using namespace Peripheral;

static const nrf_drv_twi_t i2c1 = NRF_DRV_TWI_INSTANCE(EEPROM_24CXX_I2C_IDX);
I2C1Obj *I2C1Obj::pInstance = NULL;

I2C1Obj::I2C1Obj(void) {
    i2cDrv = i2c1;
    transferDone = false;
    isInit = false;
}

I2C1Obj *I2C1Obj::GetInstance(void) {
    if (pInstance == NULL) {
        pInstance = new I2C1Obj();
    }

    return pInstance;
}

void I2C1Obj::SetTransferDone(bool state) {
    transferDone = state;
}

static void I2C1EventHandler(const nrf_drv_twi_evt_t *pEvt, void *pContext) {
    I2C1Obj *i2c = I2C1Obj::GetInstance();

    switch (pEvt->type) {
    case NRF_DRV_TWI_EVT_DONE:
        if (pEvt->xfer_desc.type == NRF_DRV_TWI_XFER_RX
        || pEvt->xfer_desc.type == NRF_DRV_TWI_XFER_TX) {
            i2c->SetTransferDone(true);
        }
        break;

    default:
        break;
    }
}

void I2C1Obj::Init(void) {
    ret_code_t errCode;
    const nrf_drv_twi_config_t i2cCfg = {
        .scl = EEPROM_24CXX_I2C_SCL,
        .sda = EEPROM_24CXX_I2C_SDA,
        .frequency = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init = false
    };

    if (!isInit) {
        errCode = nrf_drv_twi_init(&i2cDrv, &i2cCfg, I2C1EventHandler, NULL);
        APP_ERROR_CHECK(errCode);
        nrf_drv_twi_enable(&i2cDrv);
        isInit = true;
    }
}

void I2C1Obj::DeInit(void) {
    if (isInit) {
        nrf_drv_twi_disable(&i2cDrv);
        nrf_drv_twi_uninit(&i2cDrv);
        isInit = false;
    }
}

bool I2C1Obj::SendBytes(uint8_t addr, const uint8_t *pData,
                        uint8_t dataLen, bool noStop, int ms) {
    bool ret;
    Timer2Obj *timer = Timer2Obj::GetInstance();

    transferDone = false;
    timer->Start(ms);
    ret = nrf_drv_twi_tx(&i2cDrv, addr, pData, dataLen, noStop);
    if (ret != NRF_SUCCESS) {
        return false;
    }

    while (!timer->IsTimeout()) {
        if (transferDone) {
            return true;
        }
    }

    return false;
}

bool I2C1Obj::ReceiveByte(uint8_t addr, uint8_t *pData,
                          uint8_t dataLen, int ms) {
    bool ret;
    Timer2Obj *timer = Timer2Obj::GetInstance();

    transferDone = false;
    timer->Start(ms);
    ret = nrf_drv_twi_rx(&i2cDrv, addr, pData, dataLen);
    if (ret != NRF_SUCCESS) {
        return false;
    }

    while (!timer->IsTimeout()) {
        if (transferDone) {
            return true;
        }
    }

    return false;
}