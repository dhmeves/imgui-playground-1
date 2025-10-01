/**
 * @file ProductionFlasher.h
 * @brief Multi-ECU production line flashing functionality
 *
 * Provides high-level interfaces for flashing multiple ECUs simultaneously
 * with progress tracking and GUI integration support.
 */

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

    /**
     * @class MultiChannelFlasher
     * @brief Base class for multi-ECU flashing operations
     */
    class MultiChannelFlasher {
    protected:
        std::mutex logMutex;  ///< Mutex for thread-safe logging

        /**
         * @brief Thread-safe console logging
         * @param message Message to log
         */
        void threadSafeLog(const std::string& message);

    public:
        /**
         * @struct FlashJob
         * @brief Configuration for a single ECU flash operation
         */
        struct FlashJob {
            std::string controllerId;   ///< Unique controller identifier
            TPCANHandle canChannel;     ///< PCAN channel for this ECU
            std::string hexFilePath;    ///< Path to firmware HEX file
            std::string password;       ///< Security access password
            uint32_t requestId;         ///< CAN request ID
            uint32_t responseId;        ///< CAN response ID
        };

        /**
         * @brief Flash a single ECU (called by worker threads)
         * @param job Flash job configuration
         * @param firmwareData Pre-loaded firmware binary data
         * @return true if flashing successful
         */
        bool flashSingleECU(const FlashJob& job, const std::vector<uint8_t>& firmwareData);

        /**
         * @brief Flash multiple ECUs simultaneously
         *
         * Launches parallel threads for each ECU and waits for completion.
         * Firmware is loaded once and shared across all threads.
         *
         * @param jobs Vector of flash job configurations
         * @return Map of controller IDs to success status
         */
        std::map<std::string, bool> flashMultipleECUs(const std::vector<FlashJob>& jobs);
    };

    /**
     * @class AutoDetectMultiChannelFlasher
     * @brief Auto-detects PCAN channels and generates flash jobs
     */
    class AutoDetectMultiChannelFlasher : public MultiChannelFlasher {
    public:
        /**
         * @brief Detect all available PCAN USB channels
         * @return Vector of available PCAN channel handles
         */
        std::vector<TPCANHandle> detectChannels();

        /**
         * @brief Generate flash jobs for detected ECUs
         * @param hexFilePath Path to firmware HEX file
         * @param password Security access password
         * @param maxECUs Maximum number of ECUs to flash (0 = all detected)
         * @return Vector of configured flash jobs
         */
        std::vector<FlashJob> generateFlashJobs(
            const std::string& hexFilePath,
            const std::string& password,
            int maxECUs = 0);
    };

    /**
     * @class ProductionLineFlasher
     * @brief Continuous production line flashing with auto-detection
     *
     * UNUSED IN CURRENT GUI - Kept for future command-line interface
     */
    class ProductionLineFlasher : public AutoDetectMultiChannelFlasher {
    private:
        std::atomic<bool> stopScanning{ false };  ///< Signal to stop scanning

    public:
        /**
         * @enum ECUState
         * @brief Current state of an ECU station
         */
        enum ECUState {
            DISCONNECTED,        ///< No ECU present
            DETECTED,            ///< ECU detected, waiting to flash
            FLASHING,            ///< Currently flashing
            FLASH_SUCCESS,       ///< Flash completed successfully
            FLASH_FAILED,        ///< Flash failed
            READY_FOR_REMOVAL    ///< Flash complete, prompting removal
        };

        /**
         * @struct ECUStation
         * @brief Represents a single flashing station
         */
        struct ECUStation {
            int stationId;                                      ///< Station number
            TPCANHandle canChannel;                             ///< Associated CAN channel
            ECUState state;                                     ///< Current state
            std::string controllerId;                           ///< ECU identifier
            std::chrono::steady_clock::time_point lastStateChange; ///< Last state change time
            int flashAttempts;                                  ///< Number of flash attempts

            ECUStation(int id, TPCANHandle ch);
        };

        /**
         * @brief Scan all stations for ECU presence/removal
         * @param stations Vector of station configurations
         */
        void scanStations(std::vector<ECUStation>& stations);

        /**
         * @brief Run continuous production line (blocking)
         * @param hexFilePath Path to firmware HEX file
         * @param password Security access password
         */
        void runProductionLine(const std::string& hexFilePath, const std::string& password);

        /**
         * @brief Display current status of all stations
         * @param stations Vector of station configurations
         */
        void displayStationStatus(const std::vector<ECUStation>& stations);

        /**
         * @brief Stop production line scanning
         */
        void stopProductionLine();
    };

    // ===== Test Functions (UNUSED in GUI) =====

    /**
     * @brief Test auto-detect and multi-flash functionality
     */
    void testAutoDetectMultiFlash();

    /**
     * @brief Test PCAN channel detection
     */
    void testChannelDetection();

    /**
     * @brief Test production line continuous flashing
     */
    void testProductionLineFlashing();

    /**
     * @brief Read CB (Calibration Block) version from ECU
     * @param channel PCAN channel to query
     * @return CB version string or error message
     */
    std::string readCBVersion(TPCANHandle channel);

    /**
     * @brief Check CB versions on all detected ECUs
     */
    void checkAllCBVersions();

    /**
     * @brief Test CB version reading functionality
     */
    void testCBVersionCheck();

    // ===== GUI Integration =====

    /**
     * @struct FlashProgress
     * @brief Thread-safe progress tracking for GUI display
     */
    struct FlashProgress {
        std::atomic<float> progress{ 0.0f };        ///< Progress (0.0 to 1.0)
        std::atomic<bool> is_complete{ false };     ///< Operation complete flag
        std::atomic<bool> is_success{ false };      ///< Success/failure flag
        std::string controller_id;                   ///< Controller identifier

        // Thread-safe log buffer
        std::mutex log_mutex;                        ///< Mutex for log access
        std::deque<std::string> logs;                ///< Rolling log buffer
        static constexpr size_t MAX_LOGS = 50;       ///< Maximum log entries

        /**
         * @brief Add log message (thread-safe)
         * @param message Log message
         */
        void addLog(const std::string& message) {
            std::lock_guard<std::mutex> lock(log_mutex);
            logs.push_back(message);
            if (logs.size() > MAX_LOGS) {
                logs.pop_front();
            }
        }

        /**
         * @brief Get copy of all log messages (thread-safe)
         * @return Vector of log messages
         */
        std::vector<std::string> getLogs() {
            std::lock_guard<std::mutex> lock(log_mutex);
            return std::vector<std::string>(logs.begin(), logs.end());
        }
    };

    /**
     * @class ProductionFlasherGUI
     * @brief GUI-friendly multi-ECU flasher with progress tracking
     *
     * Main class used by Dear ImGui interface for production flashing.
     * Provides asynchronous flashing with real-time progress updates.
     */
    class ProductionFlasherGUI {
    private:
        std::vector<std::shared_ptr<FlashProgress>> progress_trackers; ///< Progress for each ECU
        std::vector<std::future<void>> flash_futures;                  ///< Async operation handles
        std::atomic<bool> is_flashing{ false };                        ///< Currently flashing flag
        std::atomic<bool> is_initializing{ false };                    ///< Channel detection in progress
        AutoDetectMultiChannelFlasher detector;                        ///< Channel detector

        /**
         * @brief Flash single ECU with progress updates
         * @param job Flash job configuration
         * @param firmwareData Pre-loaded firmware data
         * @param tracker Progress tracker for this ECU
         */
        void flashWithProgress(
            const MultiChannelFlasher::FlashJob& job,
            const std::vector<uint8_t>& firmwareData,
            std::shared_ptr<FlashProgress> tracker);

    public:
        /**
         * @brief Start flashing all detected ECUs
         *
         * Detects channels, loads firmware, and launches parallel flash operations.
         * Non-blocking - returns immediately.
         *
         * @param hexFilePath Path to firmware HEX file
         * @param password Security access password
         */
        void startFlashing(const std::string& hexFilePath, const std::string& password);

        /**
         * @brief Get progress trackers for all ECUs
         * @return Const reference to progress tracker vector
         */
        const std::vector<std::shared_ptr<FlashProgress>>& getProgress() const {
            return progress_trackers;
        }

        /**
         * @brief Check if flashing operation is in progress
         * @return true if any ECU is still flashing
         */
        bool isFlashing() const {
            return is_flashing;
        }

        /**
         * @brief Check if channel detection/initialization is in progress
         * @return true if detecting channels
         */
        bool isInitializing() const {
            return is_initializing;
        }

        /**
         * @brief Check for completion and clean up finished operations
         *
         * Call this periodically from GUI update loop.
         */
        void checkCompletion();
    };

} // namespace RC40Flasher
