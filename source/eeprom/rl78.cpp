#include "ble_common.h"
#include "peripheral/uart.h"
#include "peripheral/timer.h"
#include "eeprom/rl78.h"

using namespace EsyPro;
using namespace EEPROM;
using namespace RL78;

static const char LOG_DGB_NAME[] = "rl78";

static RL78::SetupCommand rl78SetupCmd;
static RL78::WriteCommand rl78WriteCmd;
static RL78::ReadCommand rl78ReadCmd;

static RL78Obj rl78Obj;
static uint8_t tmpBuffer[RL78_PAGE_SIZE] = {0};
static RL78::WriteStateType_t writeState = RL78::WriteStateType_t::RECV_FIRST_BLOCK;
static RL78::ReadStateType_t readState = RL78::ReadStateType_t::READ_RL78;

static uint32_t blockOffset;
static uint32_t curFlashAddress = 0x00000000;
static uint32_t endFlashAddress = 0x00000000;

namespace RL78Cmd {
    void InitRL78(void) {
        Peripheral::Uart0Obj *rl78 = Peripheral::Uart0Obj::GetInstance();

        nrf_gpio_cfg_output(RL78_RES_PIN);

        rl78->DeInit();
        nrf_gpio_cfg_output(MC33290_COMM_TX_PIN);
        nrf_gpio_pin_write(MC33290_COMM_TX_PIN, 0);
        nrf_delay_ms(400);
        nrf_gpio_pin_write(MC33290_COMM_TX_PIN, 1);
        nrf_gpio_pin_write(RL78_RES_PIN, 1);
        nrf_delay_ms(550);
        nrf_gpio_pin_write(MC33290_COMM_TX_PIN, 0);
        nrf_gpio_pin_write(RL78_RES_PIN, 0);
        nrf_delay_ms(6);
        nrf_gpio_pin_write(RL78_RES_PIN, 1);
        nrf_delay_ms(6);
        nrf_gpio_pin_write(MC33290_COMM_TX_PIN, 1);
        rl78->Init(MC33290_RL78_UART_BAUDRATE);
    }

    static bool SettingOneWire(void) {
        uint8_t recvByte;
        uint8_t srcCmd[] = {0x3A};
        Peripheral::Uart0Obj *rl78 = Peripheral::Uart0Obj::GetInstance();
        Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();

        rl78->SendBytes(srcCmd, sizeof(srcCmd));
        timer->Start(EEPROM_TIMEOUT);
        while (!timer->IsTimeout()) {
            if (rl78->ReceiveByte(&recvByte)) {
                if (recvByte == srcCmd[0]) {
                    return true;
                }
            }
        }

        NRF_LOG_INFO("[%s]: ERROR: Check OneWire", LOG_DGB_NAME);
        return false;
    }

    static bool SetupBaudrate(void) {
        bool ret;
        uint8_t cmd = 0x9A;
        uint8_t srcBuf[] = {0x00, 0x31};
        uint16_t dataLen;
        uint8_t statusByte[MAX_RL78_COMM_BUF_SIZE] = {0};

        rl78Obj.SetPacketReqData(cmd, 1, srcBuf, sizeof(srcBuf), COMMAND_TYPE);
        rl78Obj.SendRequestToRL78();
        ret = rl78Obj.ReceiveRequestFromRL78(ONE_MSGS);
        rl78Obj.GetPacketResData(statusByte, &dataLen, 0);
        if (!ret || statusByte[0] != 0x06) {
            NRF_LOG_INFO("[%s]: ERROR: Check Baudrate", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool CheckResetSync(void) {
        bool ret;
        uint8_t cmd = 0x00;
        uint8_t tmpByte;
        uint16_t dataLen;
        uint8_t statusByte[MAX_RL78_COMM_BUF_SIZE] = {0};

        rl78Obj.SetPacketReqData(cmd, 1, &tmpByte, 0, COMMAND_TYPE);
        rl78Obj.SendRequestToRL78();
        ret = rl78Obj.ReceiveRequestFromRL78(ONE_MSGS);
        rl78Obj.GetPacketResData(statusByte, &dataLen, 0);
        if (!ret || statusByte[0] != 0x06) {
            NRF_LOG_INFO("[%s]: ERROR: Check ResetSync", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool GetSignature(uint8_t *data) {
        bool ret;
        uint8_t cmd = 0xC0;
        uint8_t tmpByte;
        uint16_t dataLen;
        uint8_t signature[MAX_RL78_COMM_BUF_SIZE] = {0};

        rl78Obj.SetPacketReqData(cmd, 1, &tmpByte, 0, COMMAND_TYPE);
        rl78Obj.SendRequestToRL78();
        ret = rl78Obj.ReceiveRequestFromRL78(TWO_MSGS);
        rl78Obj.GetPacketResData(signature, &dataLen, 0);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Get Signature", LOG_DGB_NAME);
            return false;
        }
        uint32_t flashCodeSize = (1 + signature[13]) + (256 * signature[14]) 
                                    + (65536 * signature[15]);
        uint32_t flashDataSize = (1 + signature[16]) + (256 * (signature[17] - 0x10)) 
                                    + (65536 * (signature[18] - 0x0F));

        memcpy(&data[0], &signature[3], 10);
        memcpy(&data[10], (uint8_t *)&flashCodeSize, 3);
        memcpy(&data[13], (uint8_t *)&flashDataSize, 3);

        return true;
    }

    static bool BlockBlankCheck(const uint32_t startAddr, const uint32_t endAddr,
                                uint8_t *isBlank) {
        bool ret;
        uint8_t cmd = 0x32;
        uint16_t dataLen;
        uint8_t srcBuf[7] = {0};
        uint8_t statusByte[MAX_RL78_COMM_BUF_SIZE] = {0};

        srcBuf[0] = startAddr & 0xFF;
        srcBuf[1] = (startAddr >> 8) & 0xFF;
        srcBuf[2] = (startAddr >> 16) & 0xFF;
        srcBuf[3] = endAddr & 0xFF;
        srcBuf[4] = (endAddr >> 8) & 0xFF;
        srcBuf[5] = (endAddr >> 16) & 0xFF;

        rl78Obj.SetPacketReqData(cmd, 1, srcBuf, sizeof(srcBuf), COMMAND_TYPE);
        rl78Obj.SendRequestToRL78();
        ret = rl78Obj.ReceiveRequestFromRL78(ONE_MSGS);
        rl78Obj.GetPacketResData(statusByte, &dataLen, 0);
        if (!ret || ((statusByte[0] != 0x06) && (statusByte[0] != 0x1B))) {
            NRF_LOG_INFO("[%s]: ERROR: Check BlockBlank", LOG_DGB_NAME);
            return false;
        }

        if (statusByte[0] == 0x1B) {
            *isBlank = 0x00;
        } else {
            *isBlank = 0x01;
        }

        return true;
    }

    static bool CheckEraseBlock(const uint32_t addr) {
        bool ret;
        uint8_t cmd = 0x22;
        uint8_t srcBuf[3] = {0};
        uint16_t dataLen;
        uint8_t statusByte[MAX_RL78_COMM_BUF_SIZE] = {0};

        srcBuf[0] = addr & 0xFF;
        srcBuf[1] = (addr >> 8) & 0xFF;
        srcBuf[2] = (addr >> 16) & 0xFF;

        rl78Obj.SetPacketReqData(cmd, 1, srcBuf, sizeof(srcBuf), COMMAND_TYPE);
        rl78Obj.SendRequestToRL78();
        ret = rl78Obj.ReceiveRequestFromRL78(ONE_MSGS);
        rl78Obj.GetPacketResData(statusByte, &dataLen, 0);
        if (!ret || (statusByte[0] != 0x06)) {
            NRF_LOG_INFO("[%s]: ERROR: Check EraseBlock", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool StartFlashing(const uint32_t startAddr, const uint32_t endAddr) {
        bool ret;
        uint8_t cmd = 0x40;
        uint16_t dataLen;
        uint8_t srcBuf[6] = {0};
        uint8_t statusByte[MAX_RL78_COMM_BUF_SIZE] = {0};

        srcBuf[0] = startAddr & 0xFF;
        srcBuf[1] = (startAddr >> 8) & 0xFF;
        srcBuf[2] = (startAddr >> 16) & 0xFF;
        srcBuf[3] = endAddr & 0xFF;
        srcBuf[4] = (endAddr >> 8) & 0xFF;
        srcBuf[5] = (endAddr >> 16) & 0xFF;

        rl78Obj.SetPacketReqData(cmd, 1, srcBuf, sizeof(srcBuf), COMMAND_TYPE);
        rl78Obj.SendRequestToRL78();
        ret = rl78Obj.ReceiveRequestFromRL78(ONE_MSGS);
        rl78Obj.GetPacketResData(statusByte, &dataLen, 0);
        if (!ret || (statusByte[0] != 0x06)) {
            NRF_LOG_INFO("[%s]: ERROR: Start Flashing", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool FlashProgram(uint8_t *data) {
        bool ret;
        uint8_t cmd = 0x00;
        uint16_t dataLen;
        uint8_t statusByte[MAX_RL78_COMM_BUF_SIZE] = {0};

        rl78Obj.SetPacketReqData(cmd, 0, data, RL78_PAGE_SIZE, DATA_TYPE);
        rl78Obj.SendRequestToRL78();
        ret = rl78Obj.ReceiveRequestFromRL78(TWO_MSGS);
        rl78Obj.GetPacketResData(statusByte, &dataLen, 0);
        if (!ret || (statusByte[0] != 0x06)) {
            NRF_LOG_INFO("[%s]: ERROR: Flash Program", LOG_DGB_NAME);
            return false;
        }

        return true;
    }

    static bool GetChecksum(uint8_t *cksum, const uint32_t startAddr,
                            const uint32_t endAddr) {
        bool ret;
        uint8_t cmd = 0xB0;
        uint8_t dummyByte;
        uint16_t dataLen;
        uint8_t srcBuf[6] = {0};
        uint8_t statusByte[MAX_RL78_COMM_BUF_SIZE] = {0};
        Peripheral::Uart0Obj *rl78 = Peripheral::Uart0Obj::GetInstance();

        srcBuf[0] = startAddr & 0xFF;
        srcBuf[1] = (startAddr >> 8) & 0xFF;
        srcBuf[2] = (startAddr >> 16) & 0xFF;
        srcBuf[3] = endAddr & 0xFF;
        srcBuf[4] = (endAddr >> 8) & 0xFF;
        srcBuf[5] = (endAddr >> 16) & 0xFF;

        while (rl78->ReceiveByte(&dummyByte));

        rl78Obj.SetPacketReqData(cmd, 1, srcBuf, sizeof(srcBuf), COMMAND_TYPE);
        rl78Obj.SendRequestToRL78();
        ret = rl78Obj.ReceiveRequestFromRL78(TWO_MSGS);
        rl78Obj.GetPacketResData(statusByte, &dataLen, 0);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Get Checksum", LOG_DGB_NAME);
            return false;
        }
        *cksum = statusByte[0];

        return true;
    }

    static bool CheckDataRead(uint16_t idx) {
        bool ret;
        uint8_t cmd = 0x00;
        uint8_t data[RL78_PAGE_SIZE] = {0};
        NumResMessages_t numMsgs = ((idx == (RL78_PAGE_SIZE - 1)) ? TWO_MSGS : ONE_MSGS);

        for (int i = 0; i < RL78_PAGE_SIZE; i++) {
            data[i] = (i > idx) ? 0xFF : 0x00;
        }

        rl78Obj.SetPacketReqData(cmd, 0, data, RL78_PAGE_SIZE, DATA_TYPE);
        rl78Obj.SendRequestToRL78();
        ret = rl78Obj.ReceiveRequestFromRL78(numMsgs);
        if (!ret) {
            NRF_LOG_INFO("[%s]: ERROR: Check DataRead", LOG_DGB_NAME);
            return false;
        }

        return true;
    }
}

namespace RL78 {
    Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType) {
        Command *cmd = NULL;

        MuxControl(RL78_MUX);

        switch (commCmdType & 0x0F) {
        case CMD_BASIC_MEM_SETUP_REQ:
            cmd = &rl78SetupCmd;
            NRF_LOG_INFO("[%s]: INFO: Setup Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_WRITE_DATA_REQ:
            cmd = &rl78WriteCmd;
            NRF_LOG_INFO("[%s]: INFO: Write Request", LOG_DGB_NAME);
            break;

        case CMD_BASIC_MEM_READ_DATA_REQ:
            cmd = &rl78ReadCmd;
            NRF_LOG_INFO("[%s]: INFO: Read Request", LOG_DGB_NAME);
            break;
        }

        return cmd;    
    }

    uint8_t RL78Obj::CalcChecksum(const RL78Packet_t *packet) const {
        uint32_t sum = 0;
        uint8_t cksum = 0;
        uint16_t bufLen = packet->cmdLen + packet->bufLen;

        if (bufLen == 256) {
            bufLen = 0;
        }
        sum += bufLen;
        sum += (packet->cmdLen) ? packet->cmd : 0;
        for (int i = 0; i < packet->bufLen; i++) {
            sum += packet->buffer[i];
        }
        cksum = ~(sum - 1) & 0xFF;

        return cksum;
    }

    void RL78Obj::SendRequestToRL78(void) {
        uint8_t bufLen;
        uint8_t cksum, header;
        uint8_t end = 0x03;
        Peripheral::Uart0Obj *rl78 = Peripheral::Uart0Obj::GetInstance();

        header = (reqPacket.packetType == COMMAND_TYPE) ? 0x01 : 0x02;
        bufLen = reqPacket.cmdLen + reqPacket.bufLen;
        if (bufLen == 256) {
            bufLen = 0;
        }

        rl78->SendBytes(&header, 1);
        rl78->SendBytes(&bufLen, 1);
        rl78->SendBytes(&reqPacket.cmd, reqPacket.cmdLen);
        rl78->SendBytes(reqPacket.buffer, reqPacket.bufLen);
        cksum = CalcChecksum(&reqPacket);
        rl78->SendBytes(&cksum, 1);
        rl78->SendBytes(&end, 1);
    }

    bool RL78Obj::ReceiveRequestFromRL78(NumResMessages_t numResMsgs) {
        uint8_t recvByte, cksum;
        uint16_t recvByteCnt = 0;
        Peripheral::Uart0Obj *rl78 = Peripheral::Uart0Obj::GetInstance();
        Peripheral::Timer2Obj *timer = Peripheral::Timer2Obj::GetInstance();
        RL78::ReceiveStateType_t recvState = RL78::RECEIVE_ECHO;

        timer->Start(EEPROM_TIMEOUT);
        while (!timer->IsTimeout()) {
            if (!rl78->ReceiveByte(&recvByte)) {
                continue;
            }
 
            switch (recvState) {
            case RECEIVE_ECHO:
                recvByteCnt++;
                if (recvByteCnt == (reqPacket.cmdLen + reqPacket.bufLen + 4)) {
                    recvByteCnt = 0;
                    recvState = RECEIVE_HEADER;
                }
                break;

            case RECEIVE_HEADER:
                if (recvByte == 0x02) {
                    recvState = RECEIVE_LEN;
                    resPacket.cmdLen = 0;
                } else {
                    return false;
                }
                break;

            case RECEIVE_LEN:
                resPacket.bufLen = recvByte;
                if (recvByte == 0x00) {
                    resPacket.bufLen = 256;
                }
                recvState = RECEIVE_DATA;
                break;

            case RECEIVE_DATA:
                resPacket.buffer[recvByteCnt++] = recvByte;
                if (recvByteCnt == resPacket.bufLen) {
                    recvState = RECEIVE_CKSUM;
                    recvByteCnt = 0;
                }
                break;

            case RECEIVE_CKSUM:
                cksum = recvByte;
                if (cksum == CalcChecksum(&resPacket)) {
                    recvState = RECEIVE_END;
                } else {
                    return false;
                }
                break;

            case RECEIVE_END:
                if (recvByte == 0x03) {
                    if (numResMsgs == ONE_MSGS) {
                        return true;
                    } else {
                        if (resPacket.buffer[0] != 0x06) {
                            return false;
                        }
                        numResMsgs = ONE_MSGS;
                        recvState = RECEIVE_HEADER;
                    }
                } else {
                    return false;
                }
                break;
            
            default:
                break;
            }
        }

        return false;
    }

    void RL78Obj::GetPacketResData(uint8_t *data, uint16_t *dataLen,
                                   uint8_t offset) const {
        *dataLen = resPacket.bufLen;
        memcpy(data, &resPacket.buffer[offset], *dataLen);
    }

    void RL78Obj::SetPacketReqData(uint8_t cmd, uint8_t cmdLen, 
                                   uint8_t *buffer, uint16_t bufLen,
                                   RL78PacketType_t type) {
        reqPacket.cmdLen = cmdLen;
        reqPacket.cmd = cmd;
        reqPacket.bufLen = bufLen;
        memcpy(reqPacket.buffer, buffer, reqPacket.bufLen);
        reqPacket.packetType = type;
    }

    static bool SetupRL78(uint8_t *data) {
        RL78Cmd::InitRL78();

        if (!RL78Cmd::SettingOneWire()) {
            return false;
        }

        if (!RL78Cmd::SetupBaudrate()) {
            return false;
        }

        if (!RL78Cmd::GetSignature(data)) {
            return false;
        }

        blockOffset = 0;
        writeState = RECV_FIRST_BLOCK;
        readState = READ_RL78;

        return true;
    }

    static bool ReadRL78(ReadCommand *cmd, uint32_t *startAddr,
                         const uint32_t endAddr, EsyPro::CommPacket_t *packet) {
        uint8_t readByte;
        uint8_t compareByte = 0;

        switch (readState) {
        case READ_RL78:
            for (int i = 0; i < (RL78_PAGE_SIZE + 1); i++) {
                if (!RL78Cmd::GetChecksum(&readByte, *startAddr,
                                          (*startAddr + RL78_PAGE_SIZE - 1))) {
                    return false;
                }

                if (i) {
                    tmpBuffer[i - 1] = readByte - compareByte;
                    if (i == RL78_PAGE_SIZE) {
                        break;
                    }
                }
                    
                compareByte = readByte;
                if (!RL78Cmd::StartFlashing(*startAddr,
                            (*startAddr + RL78_PAGE_SIZE - 1))) {
                    readState = READ_RL78;
                    return false;
                }
                if (!RL78Cmd::CheckDataRead(i)) {
                    readState = READ_RL78;
                    return false;
                }
                nrf_delay_ms(5);
            }
            *startAddr += 256;
            readState = SEND_FIRST_BLOCK;
            packet->cmd = CMD_IGNORE_RES;
            cmd->SetCommandRepeatState(true);
            break;

        case SEND_FIRST_BLOCK:
            readState = SEND_SECOND_BLOCK;
            packet->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
            packet->cmd = CMD_BASIC_MEM_READ_DATA_RES;
            packet->bufLen = RL78_BUF_SIZE;
            memcpy(&packet->buffer[0], &tmpBuffer[0], RL78_BUF_SIZE);
            cmd->SetCommandRepeatState(false);
            break;

        case SEND_SECOND_BLOCK:
            readState = (*startAddr >= endAddr) ? END_READ_RL78 : READ_RL78;
            packet->bleUUID = CUSTOM_VALUE_READ_CHAR_UUID;
            packet->cmd = CMD_BASIC_MEM_READ_DATA_RES;
            packet->bufLen = RL78_BUF_SIZE;
            memcpy(&packet->buffer[0], &tmpBuffer[RL78_BUF_SIZE], RL78_BUF_SIZE);
            cmd->SetCommandRepeatState(false);
            break;

        case END_READ_RL78:
            packet->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            packet->cmd = CMD_BASIC_MEM_END_READ_DATA_RES;
            packet->bufLen = 0;
            cmd->SetCommandRepeatState(false);
            readState = READ_RL78;
            break;
        
        default:
            break;
        }

        return true;
    }

    static bool WriteRL78(WriteCommand *cmd, const uint8_t *data, 
                          uint32_t *addr, EsyPro::CommPacket_t *packet) {
        uint8_t isBlank = 0x00;

        switch (writeState) {
        case RECV_FIRST_BLOCK:
            memcpy(&tmpBuffer[0], data, RL78_BUF_SIZE);
            packet->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            packet->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
            packet->bufLen = 0;
            cmd->SetCommandRepeatState(false);
            writeState = RECV_SECOND_BLOCK;
            break;

        case RECV_SECOND_BLOCK:
            memcpy(&tmpBuffer[RL78_BUF_SIZE], data, RL78_BUF_SIZE);
            writeState = FLASHING_DATA;
            packet->cmd = CMD_IGNORE_RES;
            cmd->SetCommandRepeatState(true);
            break;

        case FLASHING_DATA:
            if (blockOffset == 0) {
                if (!RL78Cmd::BlockBlankCheck(*addr, 
                                (*addr + RL78_BLOCK_SIZE - 1), &isBlank)) {
                    writeState = RECV_FIRST_BLOCK;
                    return false;
                }
                if (isBlank == 0x00) {
                    if (!RL78Cmd::CheckEraseBlock(*addr)) {
                        writeState = RECV_FIRST_BLOCK;
                        return false;
                    }
                }
            }
            if (!RL78Cmd::StartFlashing(*addr, (*addr + RL78_PAGE_SIZE - 1))) {
                writeState = RECV_FIRST_BLOCK;
                return false;
            }
            if (!RL78Cmd::FlashProgram(tmpBuffer)) {
                writeState = RECV_FIRST_BLOCK;
                return false;
            }

            blockOffset++;
            *addr += 256;
            if (blockOffset == 4) {
                blockOffset = 0;
            }
            writeState = RECV_FIRST_BLOCK;
            packet->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            packet->cmd = CMD_BASIC_MEM_WRITE_DATA_RES;
            packet->bufLen = 0;
            cmd->SetCommandRepeatState(false);
            break;

        default:
            break;
        }

        return true;
    }

    void SetupCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        bool ret;

        memcpy(&curFlashAddress, &commReqPacket->buffer[0], 3);
        memcpy(&endFlashAddress, &commReqPacket->buffer[4], 3);

        ret = SetupRL78(&commResPacket->buffer[1]);
        if (ret) {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
            commResPacket->buffer[0] = 0x01;
            commResPacket->bufLen = 17;
            
        } else {
            commResPacket->bleUUID = CUSTOM_VALUE_CTR_RES_CHAR_UUID;
            commResPacket->cmd = CMD_BASIC_MEM_SETUP_RES;
            commResPacket->buffer[0] = 0x00;
            commResPacket->bufLen = 1;
        }

        this->SetCommandRepeatState(false);
    }

    void ReadCommand::Execute(CommPacket_t *commResPacket,
                              const CommPacket_t *commReqPacket,
                              CommunicationType_t commType) {
        bool ret;

        ret = ReadRL78(this, &curFlashAddress,
                       endFlashAddress, commResPacket);
        if (!ret) {
            commResPacket->cmd = CMD_IGNORE_RES;
            this->SetCommandRepeatState(false);
        }
    }

    void WriteCommand::Execute(CommPacket_t *commResPacket,
                               const CommPacket_t *commReqPacket,
                               CommunicationType_t commType) {
        bool ret;

        ret = WriteRL78(this, commReqPacket->buffer,
                        &curFlashAddress, commResPacket);
        if (!ret) {
            commResPacket->cmd = CMD_IGNORE_RES;
            this->SetCommandRepeatState(false);
        }
    }
}
