#include "peripheral/timer.h"

using namespace Peripheral;

const nrf_drv_timer_t timer3 = NRF_DRV_TIMER_INSTANCE(PC_COMM_TIMER_IDX);
const nrf_drv_timer_t timer2 = NRF_DRV_TIMER_INSTANCE(PERIPHERAL_TIMER_IDX);
Timer2Obj *Timer2Obj::pInstance = NULL;
Timer3Obj *Timer3Obj::pInstance = NULL;

void Timer2EventHandler(nrf_timer_event_t evtType, void *pContext) {
    Timer2Obj *timer = Timer2Obj::GetInstance();

    switch (evtType) {
    case NRF_TIMER_EVENT_COMPARE0:
        timer->SetTimeout(true);
        break;
        
    default:
        break;
    }
}

Timer2Obj *Timer2Obj::GetInstance(void) {
    if (pInstance == NULL) {
        pInstance = new Timer2Obj();
    }

    return pInstance;
}

Timer2Obj::Timer2Obj(void) {
    timerDrv = timer2;
    isTimeout = false;
    Init();
}

void Timer2Obj::Init(void) {
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&timerDrv, 0);
    ret_code_t errCode;
    nrf_drv_timer_config_t timerCfg;

    timerCfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timerCfg.mode = NRF_TIMER_MODE_TIMER;
    timerCfg.bit_width = (nrf_timer_bit_width_t)TIMER_BIT_WIDTH;
    timerCfg.frequency = (nrf_timer_frequency_t)TIMER_FREQUENCY;
    errCode = nrf_drv_timer_init(&timerDrv, &timerCfg, Timer2EventHandler);
    APP_ERROR_CHECK(errCode);
    nrf_drv_timer_extended_compare(&timerDrv, NRF_TIMER_CC_CHANNEL0, ticks,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);
    nrf_drv_timer_enable(&timerDrv);
}

void Timer2Obj::Stop(void) {
    if (!isTimeout) {
        nrf_drv_timer_pause(&timerDrv);
    }
}

void Timer2Obj::Start(uint16_t ms) {
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&timerDrv, ms);

    Stop();
    nrf_drv_timer_clear(&timerDrv);
    nrf_drv_timer_extended_compare(&timerDrv, NRF_TIMER_CC_CHANNEL0, ticks,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);
    nrf_drv_timer_resume(&timerDrv);
    isTimeout = false;  
    nrf_delay_us(100);
}

bool Timer2Obj::IsTimeout(void) {
    return isTimeout;
}

void Timer2Obj::SetTimeout(bool state) {
    isTimeout = state;
}

void Timer3EventHandler(nrf_timer_event_t evtType, void *pContext) {
    Timer3Obj *timer = Timer3Obj::GetInstance();

    switch (evtType) {
    case NRF_TIMER_EVENT_COMPARE0:
        timer->SetTimeout(true);
        break;
        
    default:
        break;
    }
}

Timer3Obj *Timer3Obj::GetInstance(void) {
    if (pInstance == NULL) {
        pInstance = new Timer3Obj();
    }

    return pInstance;
}

Timer3Obj::Timer3Obj(void) {
    timerDrv = timer3;
    isTimeout = false;
    Init();
}

void Timer3Obj::Init(void) {
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&timerDrv, 0);
    ret_code_t errCode;
    nrf_drv_timer_config_t timerCfg;

    timerCfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timerCfg.mode = NRF_TIMER_MODE_TIMER;
    timerCfg.bit_width = (nrf_timer_bit_width_t)TIMER_BIT_WIDTH;
    timerCfg.frequency = (nrf_timer_frequency_t)TIMER_FREQUENCY;
    errCode = nrf_drv_timer_init(&timerDrv, &timerCfg, Timer3EventHandler);
    APP_ERROR_CHECK(errCode);
    nrf_drv_timer_extended_compare(&timerDrv, NRF_TIMER_CC_CHANNEL0, ticks,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);
    nrf_drv_timer_enable(&timerDrv);
}

void Timer3Obj::Stop(void) {
    if (!isTimeout) {
        nrf_drv_timer_pause(&timerDrv);
    }
}

void Timer3Obj::Start(uint16_t ms) {
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&timerDrv, ms);

    Stop();
    nrf_drv_timer_clear(&timerDrv);
    nrf_drv_timer_extended_compare(&timerDrv, NRF_TIMER_CC_CHANNEL0, ticks,
                                   NRF_TIMER_SHORT_COMPARE0_STOP_MASK, true);
    nrf_drv_timer_resume(&timerDrv);
    isTimeout = false;
    nrf_delay_us(100);
}

bool Timer3Obj::IsTimeout(void) {
    return isTimeout;
}

void Timer3Obj::SetTimeout(bool state) {
    isTimeout = state;
}
