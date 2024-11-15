#include "scutool/scutool.h"
#include "peripheral/uart.h"
#include "peripheral/timer.h"
#include "peripheral/spi_flash.h"

using namespace EsyPro;
using namespace SCUTool;

static const char LOG_DGB_NAME[] = "scutool";

static PingCommand scuToolPingCmd;
static EnterProgmodeCommand scuToolEnterProgmodeCmd;
static WriteFirmwareCommand scuToolWriteFirmwareCmd;

static ScutoolObj scuToolObj;
static uint32_t flashSize = 0;
static uint32_t spiFlashAddr = 0;
static uint8_t packetID1 = 0;

namespace SCUTool {

    void GetCmdFromScutoolRequest(EsyPro::CommunicationObj *commPtr) {
        Command *cmd = NULL;
        CommPacket_t reqPacket;

        commPtr->GetPacketParams(&reqPacket);

        if ((reqPacket.cmd & 0xF0) == CMD_SCUTOOL_COMM_BASE) {
            cmd = GetSpecificCmd((CommunicationCmd_t)reqPacket.cmd);
        }

        if (cmd != NULL) {
            commPtr->SetCommand(cmd);
        }
    }

    void ScutoolObj::GetScutoolPacketResData(uint8_t *data, uint8_t *dataLen) const {
        *dataLen = resPacket.bufLen;
        memcpy(data, &resPacket.buffer, *dataLen);
    }

    void ScutoolObj::SetReqScutoolPacket(uint8_t *buffer, uint8_t bufLen) {
        reqPacket.bufLen = bufLen;
        if (bufLen) {
            memcpy(reqPacket.buffer, buffer, reqPacket.bufLen);
        } 
    }

    void ScutoolObj::SendRequestToScutool(uint8_t cmd, bool isAppendZeros) {
        Peripheral::Uart0Obj *uart0 = Peripheral::Uart0Obj::GetInstance();
        uint8_t sof = SCUTOOL_SOF;
        uint8_t id = ESYTOOL_ID;
        uint8_t bufLen1, bufLen2, bufLen3, bufLen4;
        uint8_t tmpByte = 0x00;
        uint8_t recvByte;

        bufLen1 = reqPacket.bufLen >> 24;
        bufLen2 = reqPacket.bufLen >> 16;
        bufLen3 = reqPacket.bufLen >> 8;
        bufLen4 = reqPacket.bufLen & 0xFF;

        while (uart0->ReceiveByte(&recvByte));

        uart0->SendBytes(&sof, 1);
        uart0->SendBytes(&id, 1);
        uart0->SendBytes(&bufLen1, 1);
        uart0->SendBytes(&bufLen2, 1);
        uart0->SendBytes(&bufLen3, 1);
        uart0->SendBytes(&bufLen4, 1);
        uart0->SendBytes(&cmd, 1);

        if (isAppendZeros) {
            uart0->SendBytes(&tmpByte, 1);
            uart0->SendBytes(&tmpByte, 1);
        }
    }

    bool ScutoolObj::ReceiveRequestFromScutool(const uint8_t expectCmd) {
        Peripheral::Uart0Obj *uart0 = Peripheral::Uart0Obj::GetInstance();
        Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();
        ReceiveStateType_t recvState = RECEIVE_SOF;
        uint8_t recvByte;
        uint8_t recvByteCnt = 0;

        timer->Start(SCUTOOL_TIMEOUT);
        while (!timer->IsTimeout()) {
            if (!uart0->ReceiveByte(&recvByte)) {
                continue;
            }

            switch (recvState) {
            case RECEIVE_SOF:
                if (recvByte == SCUTOOL_SOF) {
                    recvState = RECEIVE_ID;
                }
                break;

            case RECEIVE_ID:
                if (recvByte == SCUTOOL_ID) {
                    recvState = RECEIVE_DATALEN;
                    resPacket.bufLen = 0;

                } else {
                    recvState = RECEIVE_SOF;
                }
                break;

            case RECEIVE_DATALEN:
                resPacket.bufLen = (resPacket.bufLen << 8) | recvByte;
                recvByteCnt++;
                if (recvByteCnt == 4) {
                    recvState = RECEIVE_CMD;
                    if (resPacket.bufLen > MAX_SCUTOOL_RECV_BUF_SIZE) {
                        recvState = RECEIVE_SOF;
                    }
                    recvByteCnt = 0;
                }
                break;

            case RECEIVE_CMD:
                if (expectCmd == recvByte) {
                    if (resPacket.bufLen) {
                        recvState = RECEIVE_DATA;
                    
                    } else {
                        return true;
                    }
                
                } else {
                    recvState = RECEIVE_SOF;
                }
                break;

            case RECEIVE_DATA:
                resPacket.buffer[recvByteCnt++] = recvByte;
                if (resPacket.bufLen == recvByteCnt) {
                    return true;
                }
                break;

            default:
                break;
            }
        }

        return false;
    }

    Command *GetSpecificCmd(CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;
    
        MuxControl(SCUTOOL_MUX);
    
        switch (commCmdType & 0x0F) {
        case CMD_SCUTOOL_PING_REQ:
            NRF_LOG_INFO("[%s]: INFO: SCUTool Ping Request", LOG_DGB_NAME);
            cmd = &scuToolPingCmd;
            break;

        case CMD_SCUTOOL_ENTER_PROGMODE_REQ:
            NRF_LOG_INFO("[%s]: INFO: SCUTool Reboot Request", LOG_DGB_NAME);
            cmd = &scuToolEnterProgmodeCmd;
            break;

        case CMD_SCUTOOL_WRITE_FIRMWARE_REQ:
            NRF_LOG_INFO("[%s]: INFO: SCUTool Write firmware Request", LOG_DGB_NAME);
            cmd = &scuToolWriteFirmwareCmd;
            break;
        }
    
        return cmd;
    }

    static void ScutoolDetectState(uint8_t *data) {
        bool ret;
        uint8_t dataLen;

        scuToolObj.SetReqScutoolPacket(nullptr, 0);
        scuToolObj.SendRequestToScutool(0x62, true);
        ret = scuToolObj.ReceiveRequestFromScutool(0x63);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: SCUTool detect state", LOG_DGB_NAME);
            return;
        }
        scuToolObj.GetScutoolPacketResData(data, &dataLen);
        data[5] = 0x01;
    }

    static uint16_t CalculateCRC16(uint8_t *data, uint16_t dataLen) {
        uint16_t crcVal = 0x0000;

        for (int i = 0; i < dataLen; i++) {
            crcVal ^= data[i] << 8;
            for (int j = 0; j < 8; j++) {
                if (crcVal & 0x8000) {
                    crcVal = (crcVal << 1) ^ 0x1021;
                } else {
                    crcVal <<= 1;
                }
            }
        }

        return crcVal;
    }

    static bool ScutoolCheckEnterProgmode(void) {
        uint8_t recvByte;
        Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();
        Peripheral::Uart0Obj *uart0 = Peripheral::Uart0Obj::GetInstance();

        timer->Start(SCUTOOL_PROGMODE_TIMEOUT);
        while (!timer->IsTimeout()) {
            if (!uart0->ReceiveByte(&recvByte)) {
                continue;
            }

            if (recvByte == SCUTOOL_ENTERPROGMODE_OK) {
                return true;
            }
        }

        return false;
    }

    static bool ScutoolEnterProgmode(void) {
        bool ret;
        
        scuToolObj.SetReqScutoolPacket(nullptr, 0);
        scuToolObj.SendRequestToScutool(0x44, true);
        ret = scuToolObj.ReceiveRequestFromScutool(0xFE);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: SCUTool enter progmode-1", LOG_DGB_NAME);
            return false;
        }

        ret = ScutoolCheckEnterProgmode();
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: SCUTool enter progmode-2", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool ScutoolCheckWriteFirmware(uint8_t packetID, uint8_t *needRetry) {
        Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();
        Peripheral::Uart0Obj *uart0 = Peripheral::Uart0Obj::GetInstance();
        *needRetry = 0x01;
        uint8_t recvByte;

        timer->Start(SCUTOOL_PROGMODE_TIMEOUT);
        while (!timer->IsTimeout()) {
            if (!uart0->ReceiveByte(&recvByte)) {
                continue;
            }

            if (recvByte == SCUTOOL_BOOTLOADER_RSP_OK) {
                NRF_LOG_INFO("[%s]: OK: SCUTool write packetID-%d", LOG_DGB_NAME, packetID);
                *needRetry = 0x00;
                return true;
            
            } else if (recvByte == SCUTOOL_BOOTLOADER_RSP_RETRY) {
                NRF_LOG_INFO("[%s]: RETRY: SCUTool write packetID-%d", LOG_DGB_NAME, packetID);
                *needRetry = 0x01;
                return true;
            
            } else if (recvByte == SCUTOOL_ENTERPROGMODE_OK) {
                NRF_LOG_INFO("[%s]: CONTINUE: SCUTOOL_ENTERPROGMODE_OK", LOG_DGB_NAME);
                continue;
            
            } else {
                NRF_LOG_INFO("[%s]: ERROR-1: SCUTool write packetID-%d: %x", LOG_DGB_NAME, packetID, recvByte);
                return false;
            }
        }

        NRF_LOG_INFO("[%s]: ERROR-2: SCUTool write packetID-%d", LOG_DGB_NAME, packetID);
        return false;
    }

    static bool ScutoolWriteFirmware(uint32_t addr, uint8_t packetID1, uint32_t size) {
        bool ret;
        uint8_t spiFlashData[SCUTOOL_FIRMWARE_PACKET_SIZE];
        SpiFlash::Spi0Obj *spiFlash = SpiFlash::Spi0Obj::GetInstance();
        uint16_t bytesToRead = 0;

        for (int i = 0; i < SCUTOOL_FIRMWARE_PACKET_SIZE; i++) {
            spiFlashData[i] = 0xFF;
        }

        bytesToRead = (size < SCUTOOL_FIRMWARE_PACKET_SIZE) ? size : SCUTOOL_FIRMWARE_PACKET_SIZE;

        for (int i = 0; i < bytesToRead / 128; i++) {
            ret = spiFlash->ReadFromFlash(addr, &spiFlashData[i * 128], 128);
            if (!ret) {
                NRF_LOG_INFO("[%s]: ERROR: SCUTool write firmware-1", LOG_DGB_NAME);
                return false;
            }
            addr += 128;
        }

        Peripheral::Uart0Obj *uart0 = Peripheral::Uart0Obj::GetInstance();
        uint8_t recvByte;
        uint8_t packetID2 = 255 - packetID1;
        uint8_t sof = SCUTOOL_BOOTLOADER_SOF;
        uint16_t crc = CalculateCRC16(spiFlashData, SCUTOOL_FIRMWARE_PACKET_SIZE);
        uint8_t lCrc = crc & 0xFF;
        uint8_t hCrc = (crc >> 8) & 0xFF;

        while (uart0->ReceiveByte(&recvByte));

        uart0->SendBytes(&sof, 1);
        uart0->SendBytes(&packetID1, 1);
        uart0->SendBytes(&packetID2, 1);
        uart0->SendBytes(spiFlashData, SCUTOOL_FIRMWARE_PACKET_SIZE);
        uart0->SendBytes(&hCrc, 1);
        uart0->SendBytes(&lCrc, 1);

        return true;
    }

    void PingCommand::Execute(CommPacket_t *commResPacket,
                              const CommPacket_t *commReqPacket,
                              CommunicationType_t commType) {
        Peripheral::Uart0Obj *uart0 = Peripheral::Uart0Obj::GetInstance();

        uart0->DeInit();
        uart0->ScutoolInit();

        for (int i = 0; i < 4; i++) {
            commResPacket->buffer[i] = (NRF_FICR->DEVICEID[0] >> (8 * i)) & 0xFF;
            commResPacket->buffer[i + 4] = (NRF_FICR->DEVICEID[1] >> (8 * i)) & 0xFF;
        }

        memset(&commResPacket->buffer[8], 0, 12);
        ScutoolDetectState(&commResPacket->buffer[8]);
        memcpy(&commResPacket->buffer[20], DEVICE_VERSION, 3);
        commResPacket->cmd = CMD_SCUTOOL_PING_RES;
        commResPacket->bufLen = 23;
        this->SetCommandRepeatState(false);
    }

    void EnterProgmodeCommand::Execute(CommPacket_t *commResPacket,
                                const CommPacket_t *commReqPacket,
                                CommunicationType_t commType) {
        bool ret;

        memcpy(&flashSize, commReqPacket->buffer, 4);
        packetID1 = 1;
        spiFlashAddr = 0;

        ret = ScutoolEnterProgmode();
        if (ret) {
            commResPacket->cmd = CMD_SCUTOOL_ENTER_PROGMODE_RES;
            commResPacket->bufLen = 0;
        } else {
            commResPacket->cmd = CMD_IGNORE_RES;
        }

        this->SetCommandRepeatState(false);
    }

    void WriteFirmwareCommand::Execute(CommPacket_t *commResPacket,
                                       const CommPacket_t *commReqPacket,
                                       CommunicationType_t commType) {
        bool ret;

        if (flashSize) {
            
            uint8_t needRetry = 0;

            ret = ScutoolWriteFirmware(spiFlashAddr, packetID1, flashSize);
            if (!ret) {
                commResPacket->cmd = CMD_IGNORE_RES;
                this->SetCommandRepeatState(false);
                return;
            }

            ret = ScutoolCheckWriteFirmware(packetID1, &needRetry);
            if (!ret) {
                this->SetCommandRepeatState(false);
                return;
            }
            
            if (needRetry) {
                commResPacket->cmd = CMD_IGNORE_RES;
                
            } else {
                packetID1++;
                flashSize = (flashSize > SCUTOOL_FIRMWARE_PACKET_SIZE) 
                            ? (flashSize - SCUTOOL_FIRMWARE_PACKET_SIZE) : 0;
                spiFlashAddr += SCUTOOL_FIRMWARE_PACKET_SIZE;
                commResPacket->cmd = CMD_SCUTOOL_WRITE_FIRMWARE_RES;
                commResPacket->bufLen = 0;
            }

            this->SetCommandRepeatState(true);
        
        } else {
            this->SetCommandRepeatState(false);
        }
    }
}
