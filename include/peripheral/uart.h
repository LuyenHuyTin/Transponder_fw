#ifndef __UART_H
#define __UART_H

#include "main.h"

namespace Peripheral {
    class Uart0Obj {
    private:
        bool isInit;
        nrf_queue_t *sUartRxQueue;
        nrf_drv_uart_t uartDrv;
        static Uart0Obj *pInstance;
        Uart0Obj();

    public:
        Uart0Obj(Uart0Obj *obj) = delete;
        void operator=(const Uart0Obj *) = delete;
        static Uart0Obj *GetInstance();

        void DeInit();
        void Init(nrf_uart_baudrate_t baud);
        void ScutoolInit(void);
        nrf_queue_t *GetSerialCommQueuePtr();
        nrf_drv_uart_t GetSerialCommDrv();
        void SendBytes(const uint8_t *pData, uint16_t dataLen);
        bool ReceiveByte(uint8_t *pData);
        void FlushBytes();
    };

    class Uart1Obj {
    private:
        bool isInit;
        Uart1Obj();
        nrf_queue_t *sUartRxQueue;
        nrf_drv_uart_t uartDrv;
        static Uart1Obj *pInstance;

    public:
        Uart1Obj(Uart1Obj *obj) = delete;
        void operator=(const Uart1Obj *) = delete;
        static Uart1Obj *GetInstance();

        void DeInit();
        void Init(nrf_uart_baudrate_t baud);
        nrf_queue_t *GetSerialCommQueuePtr();
        nrf_drv_uart_t GetSerialCommDrv();
        void SendBytes(const uint8_t *pData, uint16_t dataLen);
        bool ReceiveByte(uint8_t *pData);
    };
}

#endif /* __UART_H */
