#pragma once

#include <windows.h>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <stdexcept>
#include <fstream>      // Add this
#include <chrono>       // Add this  
#include <thread>       // Add this

// PEAK PCAN includes
#include "PCANBasic.h"

// OpenSSL includes  
#include <openssl/aes.h>
#include <openssl/evp.h>

namespace RC40Flasher {

    struct MemoryBlock {
        std::string name;
        uint8_t areaId;
        uint32_t startAddress;
        uint32_t size;

        MemoryBlock(const std::string& n, uint8_t id, uint32_t addr, uint32_t sz);
    };

    struct ControllerConfig {
        std::string variant;
        std::string controllerId;
        TPCANHandle canChannel;
        uint32_t canIdRequest;
        uint32_t canIdResponse;
        std::vector<MemoryBlock> blocks;

        ControllerConfig(const std::string& var, const std::string& id, TPCANHandle ch);
    };

    class AES128Security {
    public:
        static std::vector<uint8_t> calculateKeyFromSeed(
            const std::vector<uint8_t>& seed,
            const std::vector<uint8_t>& password);
        static bool testAES();
    };

    class RC40FlasherDevice {
    private:
        ControllerConfig config;
        bool canInitialized;

    public:
        RC40FlasherDevice(const ControllerConfig& cfg);
        ~RC40FlasherDevice();

        bool initialize();
        void cleanup();
        bool flashController(const std::map<std::string, std::string>& firmwareFiles,
            const std::vector<uint8_t>& password);
        const std::string& getControllerId() const;

        // UDS service methods
        bool sendFunctionalCommand(const std::vector<uint8_t>& command);
        std::vector<uint8_t> sendUDSRequestWithMultiFrame(const std::vector<uint8_t>& request, int timeoutMs);
        std::vector<uint8_t> sendUDSRequest(const std::vector<uint8_t>& request, int timeoutMs = 1000);
        bool diagnosticSessionControl(uint8_t sessionType);
        std::vector<uint8_t> securityAccessRequestSeed();
        bool securityAccessSendKey(const std::vector<uint8_t>& key);
        bool eraseMemory(uint8_t areaId);
        uint16_t requestDownload(uint32_t address, uint32_t size);
        bool transferData(const std::vector<uint8_t>& data, uint16_t maxBlockSize);
        bool requestTransferExit();
        bool checkMemory(uint8_t areaId);
        bool ecuReset();
        bool transferDataSparse(uint8_t areaId, const std::vector<uint8_t>& firmwareData, uint32_t baseAddress);
    };

    class MultiControllerFlasher {
    public:
        static std::map<std::string, bool> flashMultipleControllers(
            const std::vector<std::pair<ControllerConfig, std::map<std::string, std::string>>>& controllers,
            const std::vector<uint8_t>& password);
    };

    // Helper functions
    std::vector<ControllerConfig> createControllerConfigs(
        const std::string& variant,
        const std::vector<std::string>& controllerIds);

    bool pingECU(TPCANHandle channel);
    std::vector<uint8_t> parseIntelHexFile(const std::string& filename);

} // namespace RC40Flasher
