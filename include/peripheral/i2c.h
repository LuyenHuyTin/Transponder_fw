#ifndef __I2C_H
#define __I2C_H

#include "main.h"
#include "common.h"

namespace Peripheral {
    class I2C1Obj {
    private:
        bool isInit;
        volatile bool transferDone;
        nrf_drv_twi_t i2cDrv;
        static I2C1Obj *pInstance;
        I2C1Obj();

    public:
        I2C1Obj(I2C1Obj *obj) = delete;
        void operator=(const I2C1Obj *) = delete;
        static I2C1Obj *GetInstance();

        void DeInit();
        void Init();
        void SetTransferDone(bool state);
        bool SendBytes(uint8_t addr, const uint8_t *pData, uint8_t dataLen,
                       bool noStop, int ms);
        bool ReceiveByte(uint8_t addr, uint8_t *pData,
                         uint8_t dataLen, int ms);
    };
}

#endif /* __I2C_H */
