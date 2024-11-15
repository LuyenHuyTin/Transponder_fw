#include "ecu/ecu_common.h"
#include "ecu/honda_idle.h"
#include "ecu/honda_backup.h"
#include "ecu/honda_update.h"
#include "ecu/piaggio_idle.h"
#include "ecu/piaggio_flash.h"
#include "ecu/yamaha_idle.h"
#include "peripheral/uart.h"
#include "peripheral/timer.h"

using namespace EsyPro;
using namespace ECU;

void ECU::GetCmdFromEcuRequest(CommunicationObj *commPtr,
                               CommunicationType_t commType) {
    EcuType_t ecuType;
    Command *cmd = NULL;
    CommPacket_t reqPacket;

    commPtr->GetPacketParams(&reqPacket);
    switch ((CommunicationCmd_t)reqPacket.cmd & 0xF0) {
    case CMD_ECU_IDLE_BASE:
        ecuType = (EcuType_t)reqPacket.buffer[0];

        if (ecuType == ECU_HONDA_PGMFI) {
            cmd = HondaPgmfi::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

        } else if (ecuType == ECU_HONDA_ABS) {
            cmd = HondaAbs::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

        } else if (ecuType == ECU_HONDA_DCT) {
            cmd = HondaDCT::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

        } else if (ecuType == ECU_PIAGGIO_MIU) {
            cmd = PiaggioMIU::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

        } else if (ecuType == ECU_PIAGGIO_PG) {
            cmd = PiaggioPG::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

        } else if (ecuType == ECU_YAMAHA) {
            cmd = Yamaha::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

        } else if (ecuType == ECU_YAMAHA_SMK) {
            cmd = YamahaSMK::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
        
        } else if (ecuType == ECU_YAMAHA_OBD2) {
            cmd = YamahaObd2::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);

        }
        break;
        
    case CMD_ECU_HONDA_BACKUP_BASE:
        cmd = HondaKeihinBackup::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
        break;

    case CMD_ECU_HONDA_KEIHIN_UPDATE_BASE:
        cmd = HondaKeihinUpdate::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
        break;

    case CMD_ECU_HONDA_SHINDENGEN_UPDATE_BASE:
        cmd = HondaShindengenUpdate::GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
        break;
    }

    if (cmd != NULL) {
        commPtr->SetCommand(cmd);
    }
}

void EcuCommonObj::GetEcuPacketResData(uint8_t *data, uint8_t *dataLen,
                                       uint8_t offset) const {
    *dataLen = resPacket.bufLen;
    memcpy(data, &resPacket.buffer[offset], *dataLen);
}

void EcuCommonObj::GetEcuCmdResData(uint8_t *cmd) const {
    memcpy(cmd, &resPacket.cmd[0], resPacket.cmdLen);
}

void EcuCommonObj::SetReqEcuPacket(uint8_t *cmd, uint8_t cmdLen,
                                   uint8_t *buffer, uint8_t bufLen) {
    reqPacket.cmdLen = cmdLen;
    memcpy(reqPacket.cmd, cmd, reqPacket.cmdLen);
    reqPacket.bufLen = bufLen;
    memcpy(reqPacket.buffer, buffer, reqPacket.bufLen);
}

void EcuCommonObj::SetReqEcuPacket(uint8_t *cmd, uint8_t cmdLen) {
    reqPacket.cmdLen = cmdLen;
    memcpy(reqPacket.cmd, cmd, reqPacket.cmdLen);
}

void HondaCommonObj::InitKlineBus(void) {
    Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();

    mc33290->DeInit();
    nrf_gpio_cfg_output(MC33290_COMM_TX_PIN);
    nrf_gpio_pin_write(MC33290_COMM_TX_PIN, 0);
    nrf_delay_ms(70);
    nrf_gpio_pin_write(MC33290_COMM_TX_PIN, 1);
    nrf_delay_ms(130);
    mc33290->Init((nrf_uart_baudrate_t)MC33290_HONDA_UART_BAUDRATE);
}

uint8_t HondaCommonObj::CalcChecksum(const EcuPacket_t *packet) const {
    uint8_t checksum;
    uint32_t sum = 0;
    uint8_t messageLen = packet->cmdLen + packet->bufLen + 2;

    for (int i = 0; i < packet->cmdLen; i++) {
        sum += packet->cmd[i];
    }
    sum += messageLen;

    for (int i = 0; i < packet->bufLen; i++) {
        sum += packet->buffer[i];
    }
    checksum = ((sum ^ 0xFF) + 1) & 0xFF;

    return checksum;
}

void HondaCommonObj::SendRequestToEcu(void) {
    uint8_t checksum;
    Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();
    uint8_t messageLen = reqPacket.cmdLen + reqPacket.bufLen + 2;

    if (reqPacket.cmdLen == 2) {
        uint8_t cmdBackup[2] = {0x00};

        cmdBackup[0] = reqPacket.cmd[0];
        cmdBackup[1] = reqPacket.cmd[1];
        reqPacket.cmd[0] = 0x00;
        reqPacket.cmd[1] = 0x00;
        checksum = CalcChecksum(&reqPacket);
        reqPacket.cmd[0] = cmdBackup[0];
        reqPacket.cmd[1] = cmdBackup[1];
    
    } else {
        checksum = CalcChecksum(&reqPacket);
    }

    mc33290->SendBytes(reqPacket.cmd, reqPacket.cmdLen);
    mc33290->SendBytes(&messageLen, 1);
    mc33290->SendBytes(reqPacket.buffer, reqPacket.bufLen);
    mc33290->SendBytes(&checksum, 1);
}

bool HondaCommonObj::ReceiveRequestFromEcu(void) {
    uint8_t recvByte, messageLen, checksum;
    uint8_t recvByteCnt = 0;
    Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();
    Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();
    ReceiveStateType_t recvState = RECEIVE_ECHO;

    timer->Start(ECU_HONDA_TIMEOUT);
    while (!timer->IsTimeout()) {
        if (!mc33290->ReceiveByte(&recvByte)) {
            continue;
        }

        switch (recvState) {
        case RECEIVE_ECHO:
            recvByteCnt++;
            if (recvByteCnt == (reqPacket.cmdLen + reqPacket.bufLen + 2)) {
                recvByteCnt = 0;
                resPacket.cmdLen = reqPacket.cmdLen;
                switch (reqPacket.cmdLen) {
                case 1:
                    recvState = RECEIVE_CMD_1;
                    break;

                case 2:
                    recvState = RECEIVE_CMD_2;
                    break;

                case 3:
                    recvState = RECEIVE_CMD_3;
                    break;
                
                default:
                    break;
                }
            }
            break;

        case RECEIVE_CMD_1:
            reqPacket.cmd[recvByteCnt] &= 0x0F;
            resPacket.cmd[recvByteCnt++] = recvByte;
            if (recvByteCnt == resPacket.cmdLen) {
                if (!memcmp(reqPacket.cmd, resPacket.cmd, resPacket.cmdLen)) {
                    recvByteCnt = 0;
                    recvState = RECEIVE_LEN;
                } else {
                    recvByteCnt = 0;
                }
            }
            break;

        case RECEIVE_CMD_2:
            resPacket.cmd[recvByteCnt++] = recvByte;
            if (recvByteCnt == resPacket.cmdLen) {
                memset(resPacket.cmd, 0, sizeof(resPacket.cmd));
                recvState = RECEIVE_LEN;
                recvByteCnt = 0;
            }

            break;

        case RECEIVE_CMD_3:
            resPacket.cmd[recvByteCnt++] = recvByte;
            if (recvByteCnt == resPacket.cmdLen) {
                reqPacket.cmd[0] |= 0x10;
                reqPacket.cmd[1] |= 0x10;
                if (!memcmp(reqPacket.cmd, resPacket.cmd, resPacket.cmdLen)) {
                    recvByteCnt = 0;
                    recvState = RECEIVE_LEN;
                } else {
                    recvByteCnt = 0;
                }
            }
            break;

        case RECEIVE_LEN:
            messageLen = recvByte;
            resPacket.bufLen = messageLen - resPacket.cmdLen - 2;
            recvState = (resPacket.bufLen == 0) ? RECEIVE_CRC : RECEIVE_DATA;
            break;

        case RECEIVE_DATA:
            resPacket.buffer[recvByteCnt++] = recvByte;
            if ((reqPacket.cmdLen == 2) && (resPacket.buffer[0] != reqPacket.buffer[0])) {
                recvByteCnt = 0;
                return false;
            }

            if (recvByteCnt == resPacket.bufLen) {
                recvState = RECEIVE_CRC;
            }
            break;

        case RECEIVE_CRC:
            checksum = recvByte;
            if (checksum == CalcChecksum(&resPacket)) {
                return true;
            }
            break;

        default:
            break;
        }
    }

    return false;
}

static uint8_t Iso14230CalcChecksum(const EcuPacket_t *packet) {
    uint8_t checksum;
    uint32_t sum = 0;

    for (int i = 0; i < packet->cmdLen; i++) {
        sum += packet->cmd[i];
    }

    for (int i = 0; i < packet->bufLen; i++) {
        sum += packet->buffer[i];
    }
    checksum = sum & 0xFF;
    return checksum;
}

static void Iso14230InitKlineBus(void) {
    Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();

    mc33290->DeInit();
    nrf_gpio_cfg_output(MC33290_COMM_TX_PIN);
    nrf_gpio_pin_write(MC33290_COMM_TX_PIN, 0);
    nrf_delay_ms(25);
    nrf_gpio_pin_write(MC33290_COMM_TX_PIN, 1);
    nrf_delay_ms(25);
    mc33290->Init((nrf_uart_baudrate_t)MC33290_ISO14230_UART_BAUDRATE);
}

static bool Iso14230AllowSecondRespondMsg(const EcuPacket_t *reqPacket) {
    uint8_t header[] = {0x80, 0x10, 0x01, 0x26};
    uint8_t cmd = 0x36;

    return (!memcmp(reqPacket->cmd, header, sizeof(header))
            && (cmd == reqPacket->buffer[0]));
}

static bool Iso14230ReceiveRequestFromEcu(unsigned int timeout,
                                          EcuPacket_t *reqPacket,
                                          EcuPacket_t *resPacket) {
    uint8_t recvByte;
    uint8_t recvByteCnt = 0;
    Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();
    Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();
    Iso14230ReceiveStateType_t recvState = RECEIVE_ECHO;
    Iso14230ReceiveStateType_t prevCmdState;

    timer->Start(timeout);
    while (!timer->IsTimeout()) {
        if (!mc33290->ReceiveByte(&recvByte)) {
            continue;
        }

        switch (recvState) {
        case RECEIVE_ECHO:
            recvByteCnt++;
            if (recvByteCnt == (reqPacket->cmdLen + reqPacket->bufLen + 1)) {
                recvByteCnt = 0;
                resPacket->cmdLen = reqPacket->cmdLen;
                if (resPacket->cmdLen == 1) {
                    recvState = RECEIVE_CMD_1;
                    prevCmdState = RECEIVE_CMD_1;

                } else {
                    recvState = RECEIVE_CMD_CHECK;
                } 
            }
            break;

        case RECEIVE_CMD_1:
            resPacket->cmd[0] = recvByte;
            resPacket->bufLen = resPacket->cmd[0] & 0x3F;
            recvState = RECEIVE_DATA;
            break;

        case RECEIVE_CMD_CHECK:
            resPacket->cmd[recvByteCnt++] = recvByte;
            if (resPacket->cmd[0] == 0x80) {
                recvState = RECEIVE_CMD_4;
                resPacket->cmdLen = 4;
                prevCmdState = RECEIVE_CMD_4;

            } else {
                recvState = RECEIVE_CMD_3;
                resPacket->cmdLen = 3;
                prevCmdState = RECEIVE_CMD_3;
            }
            break;

        case RECEIVE_CMD_3:
            resPacket->cmd[recvByteCnt++] = recvByte;
            if (recvByteCnt == 3) {
                if ((reqPacket->cmd[2] == resPacket->cmd[1])
                    && (reqPacket->cmd[1] == resPacket->cmd[2])) {
                    resPacket->bufLen = resPacket->cmd[0] & 0x3F;
                    recvState = RECEIVE_DATA;
                    recvByteCnt = 0;                    
                } else {
                    recvByteCnt = 0;
                }
            }
            break;

        case RECEIVE_CMD_4:
            resPacket->cmd[recvByteCnt++] = recvByte;
            if (recvByteCnt == 4) {
                if ((reqPacket->cmd[2] == resPacket->cmd[1])
                    && (reqPacket->cmd[1] == resPacket->cmd[2])) {
                    resPacket->bufLen = resPacket->cmd[3];
                    recvState = RECEIVE_DATA;
                    recvByteCnt = 0;
                    recvByteCnt = 0;
                }
            }
            break;

        case RECEIVE_DATA:
            resPacket->buffer[recvByteCnt++] = recvByte;
            if (recvByteCnt == resPacket->bufLen) {
                recvState = RECEIVE_CRC;
            }
            break;

        case RECEIVE_CRC:
            if (Iso14230CalcChecksum(resPacket) == recvByte) {
                if ((resPacket->buffer[0] == 0x7F)
                    && Iso14230AllowSecondRespondMsg(reqPacket)) {
                    recvState = prevCmdState;
                    recvByteCnt = 0;
                } else {
                    return true;
                }
            }
            break;
        
        default:
            break;
        }
    }

    return false;
}

static bool AC19xReceiveRequestFromEcu(unsigned int timeout,
                                       EcuPacket_t *reqPacket,
                                       EcuPacket_t *resPacket) {
    uint8_t recvByte;
    uint8_t recvByteCnt = 0;

    Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();
    Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();

    AC19xReceiveStateType_t recvState = AC19X_RECEIVE_ECHO;
    
    timer->Start(timeout);
    while (!timer->IsTimeout()) {
        if (!mc33290->ReceiveByte(&recvByte)) {
            continue;
        }

        switch (recvState) {
        case AC19X_RECEIVE_ECHO:
            recvByteCnt++;
            if (recvByteCnt == (reqPacket->cmdLen + reqPacket->bufLen + 1)) {
                recvByteCnt = 0;
                resPacket->cmdLen = reqPacket->cmdLen;
                resPacket->bufLen = reqPacket->cmd[reqPacket->cmdLen];
                recvState = AC19X_RECEIVE_CMD;
            }
            break;

        case AC19X_RECEIVE_CMD:
            resPacket->cmd[recvByteCnt++] = recvByte;
            if (recvByteCnt == resPacket->cmdLen) {
                recvState = (resPacket->bufLen) ? AC19X_RECEIVE_DATA : AC19X_RECEIVE_CRC;
                recvByteCnt = 0;
            }
            break;

        case AC19X_RECEIVE_DATA:
            resPacket->buffer[recvByteCnt++] = recvByte;
            if (recvByteCnt == resPacket->bufLen) {
                recvState = AC19X_RECEIVE_CRC;
            }
            break;

        case AC19X_RECEIVE_CRC:
            if (Iso14230CalcChecksum(resPacket) == recvByte) {
                return true;
            }
            break;
        
        default:
            break;
        }
    }

    return false;
}

static void Iso14230SendRequestToEcu(EcuPacket_t *reqPacket) {
    Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();
    uint8_t checksum = Iso14230CalcChecksum(reqPacket);

    mc33290->SendBytes(reqPacket->cmd, reqPacket->cmdLen);
    mc33290->SendBytes(reqPacket->buffer, reqPacket->bufLen);
    mc33290->SendBytes(&checksum, 1);
}

static void AC19xSendRequestToEcu(EcuPacket_t *reqPacket) {
    Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();
    
    reqPacket->cmdLen -= 1;
    uint8_t checksum = Iso14230CalcChecksum(reqPacket);

    mc33290->SendBytes(reqPacket->cmd, reqPacket->cmdLen);
    mc33290->SendBytes(reqPacket->buffer, reqPacket->bufLen);
    mc33290->SendBytes(&checksum, 1);    
}

void PiaggioCommonObj::InitKlineBus(void) {
    Iso14230InitKlineBus();
}

void PiaggioCommonObj::SendRequestToEcu(void) {
    if (reqPacket.cmd[0] == PIAGGIO_AC19X_HEADER) {
        AC19xSendRequestToEcu(&reqPacket);

    } else {
        Iso14230SendRequestToEcu(&reqPacket);

    }
    
}

bool PiaggioCommonObj::ReceiveRequestFromEcu(void) {
    bool ret;

    if (reqPacket.cmd[0] == PIAGGIO_AC19X_HEADER) {
        ret = AC19xReceiveRequestFromEcu(ECU_PIAGGIO_TIMEOUT,
                                         &reqPacket, &resPacket);
    } else {
        ret = Iso14230ReceiveRequestFromEcu(ECU_PIAGGIO_TIMEOUT, 
                                            &reqPacket, &resPacket);
    }

    return ret;
}

void YamahaCommonObj::SetEcuPingType(YamahaCommonObj::YamahaPingType_t type) {
    ecuPingType = type;
}

YamahaCommonObj::YamahaPingType_t YamahaCommonObj::GetEcuPingType(void) {
    return ecuPingType;
}

void YamahaCommonObj::InitKlineBus(void) {
    Iso14230InitKlineBus();
}

void YamahaCommonObj::SendRequestToEcu(void) { 
    Iso14230SendRequestToEcu(&reqPacket);
}

bool YamahaCommonObj::ReceiveRequestFromEcu(void) {
    bool ret = Iso14230ReceiveRequestFromEcu(ECU_YAMAHA_TIMEOUT, 
                                             &reqPacket, &resPacket);
    return ret;
}

bool YamahaCommonObj::ProcessRequestEcu15800(void) {
    uint8_t recvByte[10];
    uint8_t recvByteCnt = 0;

    Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();
    ReceiveState15800_t recvState = RECEIVE_ECHO;

    mc33290->SendBytes(reqPacket.cmd, reqPacket.cmdLen);
    nrf_delay_ms(ECU_YAMAHA_SYNC_DELAY);

    while (mc33290->ReceiveByte(&recvByte[recvByteCnt])) {
        switch (recvState) {
        case RECEIVE_ECHO:
            recvState = RECEIVE_HEADER;
            break;

        case RECEIVE_HEADER:
            recvByteCnt++;
            if (recvByteCnt == 2) {
                resPacket.cmdLen = 2;
                resPacket.cmd[0] = recvByte[0];
                resPacket.cmd[1] = recvByte[1];
                recvByteCnt = 0;
                recvState = RECEIVE_DATA;
            }
            break;

        case RECEIVE_DATA:
            recvByteCnt++;
            if (recvByteCnt == 2) {
                resPacket.bufLen = 2;
                resPacket.buffer[0] = recvByte[0];
                resPacket.buffer[1] = recvByte[1];
                recvByteCnt = 0;
                recvState = RECEIVE_CRC;
            }
            break;

        case RECEIVE_CRC:
            if (Iso14230CalcChecksum(&resPacket) == recvByte[0]) {
                return true;
            } else {
                recvByteCnt = 0;
                recvState = RECEIVE_IGNORE;
            }
            break;

        case RECEIVE_IGNORE:
            break;
        }
    }

    return false;
}

bool YamahaCommonObj::ProcessRequestEcuSMK(void) {
    uint8_t recvByte, crcRecv, smkByte;
    uint8_t recvByteCnt = 0;
    ReceiveStateSMK_t recvState = RECEIVE_KEY_SMK;
    Peripheral::Uart0Obj *mc33290 = Peripheral::Uart0Obj::GetInstance();
    Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();

    MuxControl(MC33290_YAMAHA_SMK_MUX);
    timer->Start(ECU_YAMAHA_TIMEOUT);
    while (!timer->IsTimeout()) {
        if (!mc33290->ReceiveByte(&recvByte)) {
            continue;
        }
        switch (recvState) {
        case RECEIVE_KEY_SMK:
            MuxControl(MC33290_YAMAHA_MUX);
            mc33290->SendBytes(&recvByte, 1);
            smkByte = recvByte;
            recvState = RECEIVE_ECHO_SMK;
            break;
        
        case RECEIVE_ECHO_SMK:
            if (recvByteCnt == 0) {
                recvState = RECEIVE_DATA_SMK;
            } else {
                recvByteCnt++;
                if (recvByteCnt == 9) {
                    resPacket.bufLen = 5;
                    resPacket.buffer[4] = smkByte;
                    MuxControl(MC33290_YAMAHA_MUX);
                    return true;
                }
            }
            break;

        case RECEIVE_DATA_SMK:
            resPacket.buffer[recvByteCnt] = recvByte;
            recvByteCnt++;
            if (recvByteCnt == 4) {
                recvState = RECEIVE_CRC_SMK;
            }
            break;

        case RECEIVE_CRC_SMK:
            crcRecv = recvByte;
            resPacket.cmdLen = 0;
            resPacket.bufLen = 4;
            if (crcRecv == Iso14230CalcChecksum(&resPacket)) {
                recvState = RECEIVE_ECHO_SMK;
                MuxControl(MC33290_YAMAHA_SMK_MUX);
                mc33290->SendBytes(resPacket.buffer, 4);
                mc33290->SendBytes(&crcRecv, 1);
            }
            break;

        default:
            break;
        }
    }

    return false;
}
