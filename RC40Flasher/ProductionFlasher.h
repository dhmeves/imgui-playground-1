#pragma once

#include "RC40Flasher.h"
#include <thread>
#include <future>
#include <mutex>
#include <atomic>
#include <chrono>
#include <csignal>
#include <deque>



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
    std::string readCBVersion(TPCANHandle channel);
    void checkAllCBVersions();
    void testCBVersionCheck();

    // Thread-safe progress tracking structure
    struct FlashProgress {
        std::atomic<float> progress{ 0.0f };  // 0.0 to 1.0
        std::atomic<bool> is_complete{ false };
        std::atomic<bool> is_success{ false };
        std::string controller_id;

        // Thread-safe log buffer
        std::mutex log_mutex;
        std::deque<std::string> logs;
        static constexpr size_t MAX_LOGS = 50;

        void addLog(const std::string& message) {
            std::lock_guard<std::mutex> lock(log_mutex);
            logs.push_back(message);
            if (logs.size() > MAX_LOGS) {
                logs.pop_front();
            }
        }

        std::vector<std::string> getLogs() {
            std::lock_guard<std::mutex> lock(log_mutex);
            return std::vector<std::string>(logs.begin(), logs.end());
        }
    };

    class ProductionFlasherGUI {
    private:
        std::vector<std::shared_ptr<FlashProgress>> progress_trackers;
        std::vector<std::future<void>> flash_futures;
        std::atomic<bool> is_flashing{ false };
        AutoDetectMultiChannelFlasher detector;

        void flashWithProgress(const MultiChannelFlasher::FlashJob& job,
            const std::vector<uint8_t>& firmwareData,
            std::shared_ptr<FlashProgress> tracker);

    public:
        void startFlashing(const std::string& hexFilePath, const std::string& password);
        const std::vector<std::shared_ptr<FlashProgress>>& getProgress() const { return progress_trackers; }
        bool isFlashing() const { return is_flashing; }
        void checkCompletion();
    };
} // namespace RC40Flasher
