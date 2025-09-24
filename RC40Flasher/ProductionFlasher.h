#pragma once

#include "RC40Flasher.h"
#include <thread>
#include <future>
#include <mutex>
#include <atomic>
#include <chrono>
#include <csignal>

namespace RC40Flasher {

    class MultiChannelFlasher {
    protected:
        std::mutex logMutex;

        void threadSafeLog(const std::string& message);

    public:
        struct FlashJob {
            std::string controllerId;
            TPCANHandle canChannel;
            std::string hexFilePath;
            std::string password;
            uint32_t requestId;
            uint32_t responseId;
            //std::vector<uint8_t> firmwareData;
        };

        bool flashSingleECU(const FlashJob& job, const std::vector<uint8_t>& firmwareData);
        std::map<std::string, bool> flashMultipleECUs(const std::vector<FlashJob>& jobs);
    };

    class AutoDetectMultiChannelFlasher : public MultiChannelFlasher {
    public:
        std::vector<TPCANHandle> detectChannels();
        std::vector<FlashJob> generateFlashJobs(const std::string& hexFilePath,
            const std::string& password,
            int maxECUs = 0);
    };

    class ProductionLineFlasher : public AutoDetectMultiChannelFlasher {
    private:
        std::atomic<bool> stopScanning{ false };

    public:
        enum ECUState {
            DISCONNECTED,
            DETECTED,
            FLASHING,
            FLASH_SUCCESS,
            FLASH_FAILED,
            READY_FOR_REMOVAL
        };

        struct ECUStation {
            int stationId;
            TPCANHandle canChannel;
            ECUState state;
            std::string controllerId;
            std::chrono::steady_clock::time_point lastStateChange;
            int flashAttempts;

            ECUStation(int id, TPCANHandle ch);
        };

        void scanStations(std::vector<ECUStation>& stations);
        void runProductionLine(const std::string& hexFilePath, const std::string& password);
        void displayStationStatus(const std::vector<ECUStation>& stations);
        void stopProductionLine();
    };

    // Test functions
    void testAutoDetectMultiFlash();
    void testChannelDetection();
    void testProductionLineFlashing();

} // namespace RC40Flasher
