#ifndef __TIMER_H
#define __TIMER_H

#include "main.h"

#define TIMER_BIT_WIDTH     2 /* 24-bit bit_width */
#define TIMER_FREQUENCY     4 /* 1-MHz freq */ 

namespace Peripheral {
    class Timer2Obj {
    private:
        volatile bool isTimeout;
        nrf_drv_timer_t timerDrv;
        static Timer2Obj *pInstance;
        Timer2Obj();
        void Stop();
        void Init();

    public:
        Timer2Obj(Timer2Obj *obj) = delete;
        void operator=(const Timer2Obj *) = delete;
        static Timer2Obj *GetInstance();

        void Start(uint16_t ms);
        bool IsTimeout();
        void SetTimeout(bool state);
    };

    class Timer3Obj {
    private:
        volatile bool isTimeout;
        nrf_drv_timer_t timerDrv;
        static Timer3Obj *pInstance;
        Timer3Obj();
        void Stop();
        void Init();

    public:
        Timer3Obj(Timer3Obj *obj) = delete;
        void operator=(const Timer3Obj *) = delete;
        static Timer3Obj *GetInstance();

        void Start(uint16_t ms);
        bool IsTimeout();
        void SetTimeout(bool state);
    };
}

#endif /* __TIMER_H */
