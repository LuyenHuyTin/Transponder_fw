#include "peripheral/uart.h"

using namespace Peripheral;

uint8_t tmpRxByte[1];
const nrf_drv_uart_t uart0 = NRF_DRV_UART_INSTANCE(MC33290_UART_COMM_IDX);
const nrf_drv_uart_t uart1 = NRF_DRV_UART_INSTANCE(PC_UART_COMM_IDX);
NRF_QUEUE_DEF(uint8_t, pcRxQueue_t, 
              PC_UART_COM_BUF_SIZE, NRF_QUEUE_MODE_OVERFLOW);
NRF_QUEUE_DEF(uint8_t, mc33290RxQueue_t, 
              MC33290_OBJ_BUF_SIZE, NRF_QUEUE_MODE_OVERFLOW);

Uart0Obj *Uart0Obj::pInstance = NULL;
Uart1Obj *Uart1Obj::pInstance = NULL;

Uart0Obj::Uart0Obj(void) {
    sUartRxQueue = (nrf_queue_t *)&mc33290RxQueue_t;
    isInit = false;
    uartDrv = uart0;
}

Uart0Obj *Uart0Obj::GetInstance(void) {
    if (pInstance == NULL) {
        pInstance = new Uart0Obj();
    }

    return pInstance;
}

nrf_drv_uart_t Uart0Obj::GetSerialCommDrv(void) {
    return uartDrv;
}

nrf_queue_t *Uart0Obj::GetSerialCommQueuePtr(void) {
    return sUartRxQueue;
}

void Uart0EventHandler(nrf_drv_uart_event_t *pEvt, void *pContext) {
    ret_code_t errCode;
    Uart0Obj *uart = Uart0Obj::GetInstance();
    nrf_drv_uart_t uartDrv = uart->GetSerialCommDrv();
    nrf_queue_t *queue = uart->GetSerialCommQueuePtr();

    switch (pEvt->type) {
    case NRF_DRV_UART_EVT_RX_DONE:
        if (pEvt->data.rxtx.bytes) {
            nrf_queue_push(queue, (void *)&pEvt->data.rxtx.p_data[0]);
        }
        errCode = nrf_drv_uart_rx(&uartDrv, tmpRxByte, 1);
        APP_ERROR_CHECK(errCode);
        break;

    case NRF_DRV_UART_EVT_ERROR:
        break;
            
    default:
        break;
    }
}

void Uart0Obj::ScutoolInit(void) {
    ret_code_t errCode;
    nrf_drv_uart_config_t config;

    if (isInit) {
        return;
    }

    config.hwfc = NRF_UART_HWFC_DISABLED;
    config.parity = NRF_UART_PARITY_EXCLUDED;
    config.interrupt_priority = APP_IRQ_PRIORITY_LOWEST;
    config.baudrate = NRF_UART_BAUDRATE_115200;
    config.p_context = NULL;
    config.pseltxd = MC33290_COMM_RX_PIN;
    config.pselrxd = MC33290_COMM_TX_PIN;

    errCode = nrf_drv_uart_init(&uartDrv, &config, Uart0EventHandler);
    APP_ERROR_CHECK(errCode);
    errCode = nrf_drv_uart_rx(&uartDrv, tmpRxByte, 1);
    APP_ERROR_CHECK(errCode);

    isInit = true;
}

void Uart0Obj::Init(nrf_uart_baudrate_t baud) {
    ret_code_t errCode;
    nrf_drv_uart_config_t config;

    if (isInit) {
        return;
    }
    config.hwfc = NRF_UART_HWFC_DISABLED;
    config.parity = NRF_UART_PARITY_EXCLUDED;
    config.interrupt_priority = APP_IRQ_PRIORITY_LOWEST;
    config.baudrate = baud;
    config.p_context = NULL;
    config.pseltxd = MC33290_COMM_TX_PIN;
    config.pselrxd = MC33290_COMM_RX_PIN;

    errCode = nrf_drv_uart_init(&uartDrv, &config, Uart0EventHandler);
    APP_ERROR_CHECK(errCode);
    errCode = nrf_drv_uart_rx(&uartDrv, tmpRxByte, 1);
    APP_ERROR_CHECK(errCode);

    isInit = true;
}

void Uart0Obj::DeInit(void) {
    if (isInit) {
        nrf_drv_uart_uninit(&uartDrv);
        isInit = false;
    }
}

void Uart0Obj::SendBytes(uint8_t const *pData, uint16_t dataLen) {
    for (int i = 0; i < dataLen; i++) {
        while (nrf_drv_uart_tx(&uartDrv, &pData[i], 1) != NRF_SUCCESS);
    }
}

bool Uart0Obj::ReceiveByte(uint8_t *pData) {
    ret_code_t errCode;

    if (nrf_queue_is_empty(sUartRxQueue)) {
        return false;
    }

    errCode = nrf_queue_pop(sUartRxQueue, pData);
    APP_ERROR_CHECK(errCode);
    return true;
}

void Uart0Obj::FlushBytes(void) {
    ret_code_t errCode;
    uint8_t tmp;

    while (!nrf_queue_is_empty(sUartRxQueue)) {
        errCode = nrf_queue_pop(sUartRxQueue, &tmp);
        APP_ERROR_CHECK(errCode);
    }
}

Uart1Obj::Uart1Obj(void) {
    sUartRxQueue = (nrf_queue_t *)&pcRxQueue_t;
    isInit = false;
    uartDrv = uart1;
}

Uart1Obj *Uart1Obj::GetInstance(void) {
    if (pInstance == NULL) {
        pInstance = new Uart1Obj();
    }

    return pInstance;
}

nrf_drv_uart_t Uart1Obj::GetSerialCommDrv(void) {
    return uartDrv;
}

nrf_queue_t *Uart1Obj::GetSerialCommQueuePtr(void) {
    return sUartRxQueue;
}

void Uart1EventHandler(nrf_drv_uart_event_t *pEvt, void *pContext) {
    ret_code_t errCode;
    Uart1Obj *uart = Uart1Obj::GetInstance();
    nrf_drv_uart_t uartDrv = uart->GetSerialCommDrv();
    nrf_queue_t *queue = uart->GetSerialCommQueuePtr();

    switch (pEvt->type) {
    case NRF_DRV_UART_EVT_RX_DONE:
        if (pEvt->data.rxtx.bytes) {
            nrf_queue_push(queue, (void *)&pEvt->data.rxtx.p_data[0]);
        }
        errCode = nrf_drv_uart_rx(&uartDrv, tmpRxByte, 1);
        APP_ERROR_CHECK(errCode);
        break;

    case NRF_DRV_UART_EVT_ERROR:
        break;
            
    default:
        break;
    }
}    

void Uart1Obj::Init(nrf_uart_baudrate_t baud) {
    ret_code_t errCode;
    nrf_drv_uart_config_t config;

    if (isInit) {
        return;
    }
    config.hwfc = NRF_UART_HWFC_DISABLED;
    config.parity = NRF_UART_PARITY_EXCLUDED;
    config.interrupt_priority = APP_IRQ_PRIORITY_LOWEST;
    config.baudrate = baud;
    config.p_context = NULL;
    config.pseltxd = PC_UART_COMM_TX_PIN;
    config.pselrxd = PC_UART_COMM_RX_PIN;

    errCode = nrf_drv_uart_init(&uartDrv, &config, Uart1EventHandler);
    APP_ERROR_CHECK(errCode);
    errCode = nrf_drv_uart_rx(&uartDrv, tmpRxByte, 1);
    APP_ERROR_CHECK(errCode);

    isInit = true;
}

void Uart1Obj::DeInit(void) {
    if (isInit) {
        nrf_drv_uart_uninit(&uartDrv);
        isInit = false;
    }
}

void Uart1Obj::SendBytes(uint8_t const *pData, uint16_t dataLen) {
    for (int i = 0; i < dataLen; i++) {
        while (nrf_drv_uart_tx(&uartDrv, &pData[i], 1) != NRF_SUCCESS);
    }
}

bool Uart1Obj::ReceiveByte(uint8_t *pData) {
    ret_code_t errCode;

    if (nrf_queue_is_empty(sUartRxQueue)) {
        return false;
    }

    nrf_delay_us(1);

    errCode = nrf_queue_pop(sUartRxQueue, pData);
    APP_ERROR_CHECK(errCode);
    return true;
}
