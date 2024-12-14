#ifndef __PCF7991_H
#define __PCF7991_H

#include "main.h"
#include "transponder/transponder_common.h"
#include <string>
#include <vector>
#include <iostream>

// Public methods
#define VALID_READ_RESPONSE_SIZE_TRANS  36
#define VALID_WRITE_RESPONSE_SIZE_TRANS  14

namespace PCF7991
{
    static union
    {
        struct
        {
            unsigned char filter_l : 1;
            unsigned char filter_h : 1;
            unsigned char gain : 2;
            unsigned char page_nr : 2;
            unsigned char SetPageCmd : 2;
        };
        unsigned char byteval;
    } AbicConf_Page0;

    static union
    {
        struct
        {
            unsigned char txdis : 1;
            unsigned char hysteresis : 1;
            unsigned char pd : 1;
            unsigned char pd_mode : 1;
            unsigned char page_nr : 2;
            unsigned char SetPageCmd : 2;
        };
        unsigned char byteval;
    } AbicConf_Page1;

    static union
    {
        struct
        {
            unsigned char freeze : 2;
            unsigned char acqamp : 1;
            unsigned char threset : 1;
            unsigned char page_nr : 2;
            unsigned char SetPageCmd : 2;
        };
        unsigned char byteval;
    } AbicConf_Page2;
    class Pcf7991
    {
    private:
        static Pcf7991 *instance;
    public:
        static Pcf7991* getInstance() {
            if(nullptr == instance) {
                instance = new Pcf7991();
            }
            return instance;
        }
    };
    EsyPro::Command *GetSpecificCmd(EsyPro::CommunicationCmd_t commCmdType);

    class SetupCommand : public EsyPro::Command {
    public:
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class ReadCommand : public EsyPro::Command {
    public:
        std::vector<std::string> mesReqRead = {"05C0", "204D494B52", "0aD900", "0aC980",
                                                "0aD140", "0aE0C0", "0aE880", "0aF040", "0aF800"};
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };

    class WriteCommand : public EsyPro::Command {
    public:
        std::vector<std::string> mesReqWrite = {"05C0", "204D494B52"};
        std::vector<std::string> mesEachPage = {"0a83C0", "0a8B80", "0a9340", "0a9B00", "0aA2C0", "0aAA80", "0aB240", "0aBA00"};
        void Execute(EsyPro::CommPacket_t *commResPacket,
                     const EsyPro::CommPacket_t *commReqPacket,
                     EsyPro::CommunicationType_t commType) override;
    };
    void ReadAllthing();
    void WritePage();
    void Setup();
}

#endif /* __PCF7991_H */