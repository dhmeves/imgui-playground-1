/**
 * @file ProductionFlasher.cpp
 * @brief Implementation of multi-ECU production flashing
 */

#include "ProductionFlasher.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace RC40Flasher {

    // Global mutex for serializing programming mode entry
    static std::mutex programmingMutex;

    // ===== MultiChannelFlasher Implementation =====

    void MultiChannelFlasher::threadSafeLog(const std::string& message) {
        std::lock_guard<std::mutex> lock(logMutex);
        std::cout << message << std::endl;
    }

    bool MultiChannelFlasher::flashSingleECU(const FlashJob& job, const std::vector<uint8_t>& firmwareData) {
        ControllerConfig config("RC5-6/40", job.controllerId, job.canChannel);
        config.canIdRequest = job.requestId;
        config.canIdResponse = job.responseId;

        RC40FlasherDevice flasher(config);

        // Reset ECU to clean state
        threadSafeLog("[" + job.controllerId + "] Resetting ECU to clean state...");
        try {
            std::vector<uint8_t> resetCmd = { 0x11, 0x01 }; // Hard reset
            flasher.sendUDSRequest(resetCmd, 2000);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // Wait for reboot
        }
        catch (...) {
            // Reset may not respond - that's normal
        }

        threadSafeLog("[" + job.controllerId + "] Starting flash process...");

        if (!flasher.initialize()) {
            threadSafeLog("[" + job.controllerId + "] CAN initialization failed");
            return false;
        }

        try {
            // Tester present
            std::vector<uint8_t> testerPresent = { 0x3E, 0x00 };
            flasher.sendUDSRequest(testerPresent, 1000);

            // Enter programming mode (serialized to avoid conflicts)
            {
                std::lock_guard<std::mutex> lock(programmingMutex);
                if (!flasher.diagnosticSessionControl(0x02)) {
                    threadSafeLog("[" + job.controllerId + "] Programming mode failed");
                    return false;
                }
            }

            // Security access
            auto seed = flasher.securityAccessRequestSeed();
            std::vector<uint8_t> password(job.password.begin(), job.password.end());
            auto key = AES128Security::calculateKeyFromSeed(seed, password);

            if (!flasher.securityAccessSendKey(key)) {
                threadSafeLog("[" + job.controllerId + "] Security access failed");
                return false;
            }

            // Write fingerprint data
            std::vector<uint8_t> dateCmd = { 0x2E, 0xF1, 0x99 };
            std::string progDate = "30092025"; // Current date
            dateCmd.insert(dateCmd.end(), progDate.begin(), progDate.end());

            auto dateResp = flasher.sendUDSRequestWithMultiFrame(dateCmd, 2000);
            if (!(dateResp.size() >= 2 && dateResp[0] == 0x6E)) {
                threadSafeLog("[" + job.controllerId + "] Fingerprint write failed");
                return false;
            }

            // Split firmware data into ASW0 and DS0
            uint32_t asw0Start = 0x09020000;
            uint32_t asw0End = 0x093BFFFF;
            uint32_t ds0Start = 0x093C0000;
            uint32_t asw0Size = asw0End - asw0Start + 1;
            uint32_t ds0Offset = ds0Start - asw0Start;

            std::vector<uint8_t> asw0Data(firmwareData.begin(),
                firmwareData.size() >= asw0Size ? firmwareData.begin() + asw0Size : firmwareData.end());

            if (asw0Data.size() < asw0Size) {
                asw0Data.resize(asw0Size, 0xFF); // Pad with 0xFF
            }

            std::vector<uint8_t> ds0Data;
            if (firmwareData.size() > ds0Offset) {
                ds0Data = std::vector<uint8_t>(firmwareData.begin() + ds0Offset, firmwareData.end());
            }

            // Flash ASW0 (Application Software)
            threadSafeLog("[" + job.controllerId + "] Flashing ASW0...");
            if (!flasher.eraseMemory(0x01)) {
                threadSafeLog("[" + job.controllerId + "] ASW0 erase failed");
                return false;
            }

            if (!flasher.transferDataSparse(0x01, asw0Data, asw0Start)) {
                threadSafeLog("[" + job.controllerId + "] ASW0 transfer failed");
                return false;
            }

            if (!flasher.checkMemory(0x01)) {
                threadSafeLog("[" + job.controllerId + "] ASW0 verification failed");
                return false;
            }

            // Flash DS0 (Data Set) if present
            if (!ds0Data.empty()) {
                threadSafeLog("[" + job.controllerId + "] Flashing DS0...");
                if (!flasher.eraseMemory(0x02)) {
                    threadSafeLog("[" + job.controllerId + "] DS0 erase failed");
                    return false;
                }

                if (!flasher.transferDataSparse(0x02, ds0Data, ds0Start)) {
                    threadSafeLog("[" + job.controllerId + "] DS0 transfer failed");
                    return false;
                }

                if (!flasher.checkMemory(0x02)) {
                    threadSafeLog("[" + job.controllerId + "] DS0 verification failed");
                    return false;
                }
            }

            // Reset ECU to run new firmware
            flasher.ecuReset();
            threadSafeLog("[" + job.controllerId + "] Flash completed successfully!");
            return true;

        }
        catch (const std::exception& e) {
            threadSafeLog("[" + job.controllerId + "] Exception: " + std::string(e.what()));
            return false;
        }

        flasher.cleanup();
        return false;
    }

    std::map<std::string, bool> MultiChannelFlasher::flashMultipleECUs(const std::vector<FlashJob>& jobs) {
        if (jobs.empty()) return {};

        // Parse HEX file once and share across all threads
        threadSafeLog("Loading firmware file: " + jobs[0].hexFilePath);
        std::vector<uint8_t> firmwareData;
        try {
            firmwareData = parseIntelHexFile(jobs[0].hexFilePath);
            threadSafeLog("Loaded " + std::to_string(firmwareData.size()) + " bytes for all ECUs");
        }
        catch (const std::exception& e) {
            threadSafeLog("HEX file error: " + std::string(e.what()));
            return {};
        }

        std::vector<std::future<std::pair<std::string, bool>>> futures;
        threadSafeLog("Starting simultaneous flash of " + std::to_string(jobs.size()) + " ECUs...");

        // Launch async threads for each ECU
        for (const auto& job : jobs) {
            auto future = std::async(std::launch::async, [this, job, firmwareData]() {
                bool result = flashSingleECU(job, firmwareData);
                return std::make_pair(job.controllerId, result);
                });
            futures.push_back(std::move(future));
        }

        // Collect results
        std::map<std::string, bool> results;
        for (auto& future : futures) {
            auto result = future.get();
            results[result.first] = result.second;
        }

        // Print summary
        int successCount = 0;
        int failCount = 0;
        for (const auto& result : results) {
            if (result.second) {
                successCount++;
                threadSafeLog("[SUMMARY] " + result.first + ": SUCCESS");
            }
            else {
                failCount++;
                threadSafeLog("[SUMMARY] " + result.first + ": FAILED");
            }
        }

        threadSafeLog("=== FLASH SUMMARY ===");
        threadSafeLog("Success: " + std::to_string(successCount));
        threadSafeLog("Failed:  " + std::to_string(failCount));

        return results;
    }

    // ===== AutoDetectMultiChannelFlasher Implementation =====

    std::vector<TPCANHandle> AutoDetectMultiChannelFlasher::detectChannels() {
        std::vector<TPCANHandle> availableChannels;

        std::cout << "Auto-detecting PCAN channels..." << std::endl;

        // Test channels 1-16 (covers most PCAN configurations)
        std::vector<TPCANHandle> testChannels = {
            PCAN_USBBUS1, PCAN_USBBUS2, PCAN_USBBUS3, PCAN_USBBUS4,
            PCAN_USBBUS5, PCAN_USBBUS6, PCAN_USBBUS7, PCAN_USBBUS8,
            PCAN_USBBUS9, PCAN_USBBUS10, PCAN_USBBUS11, PCAN_USBBUS12,
            PCAN_USBBUS13, PCAN_USBBUS14, PCAN_USBBUS15, PCAN_USBBUS16
        };

        for (size_t i = 0; i < testChannels.size(); ++i) {
            TPCANStatus result = CAN_Initialize(testChannels[i], PCAN_BAUD_250K, 0, 0, 0);

            if (result == PCAN_ERROR_OK || result == PCAN_ERROR_CAUTION) {
                availableChannels.push_back(testChannels[i]);
                std::cout << "Found channel: PCAN_USBBUS" << (i + 1) << std::endl;
                CAN_Uninitialize(testChannels[i]);
            }
        }

        std::cout << "Detected " << availableChannels.size() << " available channels" << std::endl;
        return availableChannels;
    }

    std::vector<MultiChannelFlasher::FlashJob> AutoDetectMultiChannelFlasher::generateFlashJobs(
        const std::string& hexFilePath, const std::string& password, int maxECUs) {

        auto channels = detectChannels();

        if (channels.empty()) {
            std::cout << "No PCAN channels detected!" << std::endl;
            return {};
        }

        // Limit to requested number of ECUs or all available channels
        int numECUs = (maxECUs > 0 && maxECUs < (int)channels.size()) ? maxECUs : (int)channels.size();

        std::vector<FlashJob> jobs;

        for (int i = 0; i < numECUs; ++i) {
            if (pingECU(channels[i])) {
                FlashJob job;
                job.controllerId = "RC40_" + std::to_string(i + 1);
                job.canChannel = channels[i];
                job.hexFilePath = hexFilePath;
                job.password = password;
                job.requestId = 0x18DA01FA;
                job.responseId = 0x18DAFA01;

                jobs.push_back(job);
            }
        }

        std::cout << "Generated " << jobs.size() << " flash jobs" << std::endl;
        return jobs;
    }

    // ===== ProductionLineFlasher Implementation (UNUSED in GUI) =====

    ProductionLineFlasher::ECUStation::ECUStation(int id, TPCANHandle ch) :
        stationId(id), canChannel(ch), state(DISCONNECTED),
        controllerId(""), lastStateChange(std::chrono::steady_clock::now()),
        flashAttempts(0) {}

    void ProductionLineFlasher::scanStations(std::vector<ECUStation>& stations) {
        for (auto& station : stations) {
            bool ecuPresent = pingECU(station.canChannel);
            auto now = std::chrono::steady_clock::now();

            switch (station.state) {
            case DISCONNECTED:
                if (ecuPresent) {
                    station.state = DETECTED;
                    station.controllerId = "RC40_ST" + std::to_string(station.stationId);
                    station.lastStateChange = now;
                    station.flashAttempts = 0;
                    threadSafeLog("[Station " + std::to_string(station.stationId) +
                        "] ECU detected: " + station.controllerId);
                }
                break;

            case DETECTED:
                if (!ecuPresent) {
                    station.state = DISCONNECTED;
                    station.lastStateChange = now;
                    threadSafeLog("[Station " + std::to_string(station.stationId) + "] ECU disconnected");
                }
                break;

            case FLASH_SUCCESS:
            case FLASH_FAILED:
                if (!ecuPresent) {
                    station.state = DISCONNECTED;
                    station.lastStateChange = now;
                    threadSafeLog("[Station " + std::to_string(station.stationId) +
                        "] ECU removed - ready for next");
                }
                else {
                    // Show removal prompt after 10 seconds
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        now - station.lastStateChange).count();
                    if (elapsed > 10 && station.state != READY_FOR_REMOVAL) {
                        station.state = READY_FOR_REMOVAL;
                        std::string status = (station.state == FLASH_SUCCESS) ? "SUCCESS" : "FAILED";
                        threadSafeLog("[Station " + std::to_string(station.stationId) +
                            "] Flash " + status + " - Please remove ECU");
                    }
                }
                break;

            default:
                break;
            }
        }
    }

    void ProductionLineFlasher::runProductionLine(const std::string& hexFilePath, const std::string& password) {
        auto channels = detectChannels();
        if (channels.empty()) {
            std::cout << "No PCAN channels detected!" << std::endl;
            return;
        }

        // Parse HEX file once at startup
        std::vector<uint8_t> firmwareData;
        try {
            firmwareData = parseIntelHexFile(hexFilePath);
            threadSafeLog("Loaded firmware: " + std::to_string(firmwareData.size()) + " bytes");
        }
        catch (const std::exception& e) {
            threadSafeLog("HEX file error: " + std::string(e.what()));
            return;
        }

        // Create stations for each channel
        std::vector<ECUStation> stations;
        for (size_t i = 0; i < channels.size(); ++i) {
            stations.emplace_back(i + 1, channels[i]);
        }

        threadSafeLog("Production line started with " + std::to_string(stations.size()) + " stations");
        threadSafeLog("Insert ECUs to begin flashing...");

        while (!stopScanning) {
            scanStations(stations);

            // Start flashing for detected ECUs (after 2 second settle time)
            for (auto& station : stations) {
                if (station.state == DETECTED) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::steady_clock::now() - station.lastStateChange).count();

                    if (elapsed >= 2) {
                        station.state = FLASHING;
                        station.lastStateChange = std::chrono::steady_clock::now();

                        // Launch flash thread with pre-loaded firmware data
                        std::thread flashThread([this, &station, password, firmwareData]() {
                            FlashJob job;
                            job.controllerId = station.controllerId;
                            job.canChannel = station.canChannel;
                            job.hexFilePath = "";
                            job.password = password;
                            job.requestId = 0x18DA01FA;
                            job.responseId = 0x18DAFA01;

                            threadSafeLog("[Station " + std::to_string(station.stationId) +
                                "] Starting flash...");

                            bool result = flashSingleECU(job, firmwareData);

                            station.flashAttempts++;
                            station.state = result ? FLASH_SUCCESS : FLASH_FAILED;
                            station.lastStateChange = std::chrono::steady_clock::now();

                            if (result) {
                                threadSafeLog("[Station " + std::to_string(station.stationId) +
                                    "] *** FLASH SUCCESS *** (attempt " +
                                    std::to_string(station.flashAttempts) + ")");
                            }
                            else {
                                threadSafeLog("[Station " + std::to_string(station.stationId) +
                                    "] *** FLASH FAILED *** (attempt " +
                                    std::to_string(station.flashAttempts) + ")");
                            }
                            });
                        flashThread.detach();
                    }
                }
            }

            // Status display every 30 seconds
            static auto lastStatus = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - lastStatus).count() >= 30) {

                displayStationStatus(stations);
                lastStatus = std::chrono::steady_clock::now();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Scan every 500ms
        }
    }

    void ProductionLineFlasher::displayStationStatus(const std::vector<ECUStation>& stations) {
        threadSafeLog("=== Station Status ===");
        for (const auto& station : stations) {
            std::string status;
            switch (station.state) {
            case DISCONNECTED: status = "Empty"; break;
            case DETECTED: status = "ECU Detected"; break;
            case FLASHING: status = "Flashing..."; break;
            case FLASH_SUCCESS: status = "SUCCESS - Remove ECU"; break;
            case FLASH_FAILED: status = "FAILED - Remove ECU"; break;
            case READY_FOR_REMOVAL: status = "Please Remove ECU"; break;
            }
            threadSafeLog("Station " + std::to_string(station.stationId) + ": " + status +
                (station.flashAttempts > 0 ? " (attempts: " +
                    std::to_string(station.flashAttempts) + ")" : ""));
        }
    }

    void ProductionLineFlasher::stopProductionLine() {
        stopScanning = true;
    }

    // ===== ProductionFlasherGUI Implementation (USED BY IMGUI) =====

    void ProductionFlasherGUI::startFlashing(const std::string& hexFilePath, const std::string& password) {
        if (is_flashing || is_initializing) return;

        is_initializing = true;

        // Run channel detection and job generation in separate thread to keep UI responsive
        std::thread init_thread([this, hexFilePath, password]() {
            auto jobs = detector.generateFlashJobs(hexFilePath, password, 16);

            if (jobs.empty()) {
                is_initializing = false;
                return;
            }

            // Parse firmware once
            std::vector<uint8_t> firmwareData;
            try {
                firmwareData = parseIntelHexFile(hexFilePath);
            }
            catch (const std::exception& e) {
                is_initializing = false;
                return;
            }

            is_initializing = false;
            is_flashing = true;
            progress_trackers.clear();
            flash_futures.clear();

            // Create progress tracker for each job
            for (const auto& job : jobs) {
                auto tracker = std::make_shared<FlashProgress>();
                tracker->controller_id = job.controllerId;
                progress_trackers.push_back(tracker);

                // Launch async flash operation
                flash_futures.push_back(std::async(std::launch::async,
                    [this, job, firmwareData, tracker]() {
                        flashWithProgress(job, firmwareData, tracker);
                    }));
            }
            });

        init_thread.detach();
    }

    void ProductionFlasherGUI::checkCompletion() {
        if (!is_flashing) return;

        bool all_complete = true;
        for (const auto& tracker : progress_trackers) {
            if (!tracker->is_complete) {
                all_complete = false;
                break;
            }
        }

        if (all_complete) {
            for (auto& future : flash_futures) {
                if (future.valid()) {
                    future.wait();
                }
            }
            is_flashing = false;
        }
    }

    void ProductionFlasherGUI::flashWithProgress(
        const MultiChannelFlasher::FlashJob& job,
        const std::vector<uint8_t>& firmwareData,
        std::shared_ptr<FlashProgress> tracker) {

        try {
            ControllerConfig config("RC5-6/40", job.controllerId, job.canChannel);
            config.canIdRequest = job.requestId;
            config.canIdResponse = job.responseId;
            RC40FlasherDevice flasher(config);

            // Progress weights for each operation
            const float INIT_WEIGHT = 0.05f;
            const float SECURITY_WEIGHT = 0.10f;
            const float ASW0_ERASE_WEIGHT = 0.05f;
            const float ASW0_PROGRAM_WEIGHT = 0.35f;
            const float ASW0_VERIFY_WEIGHT = 0.05f;
            const float DS0_ERASE_WEIGHT = 0.05f;
            const float DS0_PROGRAM_WEIGHT = 0.25f;
            const float DS0_VERIFY_WEIGHT = 0.05f;
            const float RESET_WEIGHT = 0.05f;

            float base_progress = 0.0f;

            tracker->addLog("Initializing CAN...");
            if (!flasher.initialize()) {
                tracker->addLog("CAN init failed");
                tracker->is_complete = true;
                return;
            }
            base_progress += INIT_WEIGHT;
            tracker->progress = base_progress;

            // Reset ECU
            try {
                std::vector<uint8_t> resetCmd = { 0x11, 0x01 };
                flasher.sendUDSRequest(resetCmd, 2000);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }
            catch (...) {}

            tracker->addLog("Programming mode...");
            std::vector<uint8_t> testerPresent = { 0x3E, 0x00 };
            flasher.sendUDSRequest(testerPresent, 1000);

            if (!flasher.diagnosticSessionControl(0x02)) {
                tracker->addLog("Programming mode failed");
                tracker->is_complete = true;
                return;
            }

            tracker->addLog("Security access...");
            auto seed = flasher.securityAccessRequestSeed();
            std::vector<uint8_t> password(job.password.begin(), job.password.end());
            auto key = AES128Security::calculateKeyFromSeed(seed, password);

            if (!flasher.securityAccessSendKey(key)) {
                tracker->addLog("Security failed");
                tracker->is_complete = true;
                return;
            }
            base_progress += SECURITY_WEIGHT;
            tracker->progress = base_progress;

            // Fingerprint with current date
            std::vector<uint8_t> dateCmd = { 0x2E, 0xF1, 0x99 };

            // Get current date from system
            auto now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            std::tm local_time;
            localtime_s(&local_time, &now_time);

            // Format as DDMMYYYY
            char dateBuffer[9];
            std::strftime(dateBuffer, sizeof(dateBuffer), "%d%m%Y", &local_time);
            std::string progDate(dateBuffer);

            dateCmd.insert(dateCmd.end(), progDate.begin(), progDate.end());
            auto dateResp = flasher.sendUDSRequestWithMultiFrame(dateCmd, 2000);

            // Split firmware
            uint32_t asw0Start = 0x09020000;
            uint32_t asw0End = 0x093BFFFF;
            uint32_t ds0Start = 0x093C0000;
            uint32_t asw0Size = asw0End - asw0Start + 1;
            uint32_t ds0Offset = ds0Start - asw0Start;

            std::vector<uint8_t> asw0Data(firmwareData.begin(),
                firmwareData.size() >= asw0Size ? firmwareData.begin() + asw0Size : firmwareData.end());
            if (asw0Data.size() < asw0Size) {
                asw0Data.resize(asw0Size, 0xFF);
            }

            std::vector<uint8_t> ds0Data;
            if (firmwareData.size() > ds0Offset) {
                ds0Data = std::vector<uint8_t>(firmwareData.begin() + ds0Offset, firmwareData.end());
            }

            // === ASW0 Flash ===
            tracker->addLog("Erasing ASW0...");
            if (!flasher.eraseMemory(0x01)) {
                tracker->addLog("ASW0 erase failed");
                tracker->is_complete = true;
                return;
            }
            base_progress += ASW0_ERASE_WEIGHT;
            tracker->progress = base_progress;

            tracker->addLog("Programming ASW0...");
            float asw0_prog_start = base_progress;
            if (!flasher.transferDataSparse(0x01, asw0Data, asw0Start,
                [tracker, asw0_prog_start, ASW0_PROGRAM_WEIGHT](float sub_prog) {
                    tracker->progress = asw0_prog_start + (sub_prog * ASW0_PROGRAM_WEIGHT);
                })) {
                tracker->addLog("ASW0 program failed");
                tracker->is_complete = true;
                return;
            }
            base_progress += ASW0_PROGRAM_WEIGHT;
            tracker->progress = base_progress;

            tracker->addLog("Verifying ASW0...");
            if (!flasher.checkMemory(0x01)) {
                tracker->addLog("ASW0 verify failed");
                tracker->is_complete = true;
                return;
            }
            base_progress += ASW0_VERIFY_WEIGHT;
            tracker->progress = base_progress;

            // === DS0 Flash ===
            if (!ds0Data.empty()) {
                tracker->addLog("Erasing DS0...");
                if (!flasher.eraseMemory(0x02)) {
                    tracker->addLog("DS0 erase failed");
                    tracker->is_complete = true;
                    return;
                }
                base_progress += DS0_ERASE_WEIGHT;
                tracker->progress = base_progress;

                tracker->addLog("Programming DS0...");
                float ds0_prog_start = base_progress;
                if (!flasher.transferDataSparse(0x02, ds0Data, ds0Start,
                    [tracker, ds0_prog_start, DS0_PROGRAM_WEIGHT](float sub_prog) {
                        tracker->progress = ds0_prog_start + (sub_prog * DS0_PROGRAM_WEIGHT);
                    })) {
                    tracker->addLog("DS0 program failed");
                    tracker->is_complete = true;
                    return;
                }
                base_progress += DS0_PROGRAM_WEIGHT;
                tracker->progress = base_progress;

                tracker->addLog("Verifying DS0...");
                if (!flasher.checkMemory(0x02)) {
                    tracker->addLog("DS0 verify failed");
                    tracker->is_complete = true;
                    return;
                }
                base_progress += DS0_VERIFY_WEIGHT;
                tracker->progress = base_progress;
            }

            tracker->addLog("Resetting ECU...");
            flasher.ecuReset();
            tracker->progress = 1.0f;
            tracker->addLog("Success!");
            tracker->is_success = true;
            tracker->is_complete = true;
            flasher.cleanup();

        }
        catch (const std::exception& e) {
            tracker->addLog(std::string("Error: ") + e.what());
            tracker->is_complete = true;
        }
    }

    // ===== Test Functions (UNUSED in GUI - kept for command-line testing) =====

    void testAutoDetectMultiFlash() {
        std::cout << "=== Auto-Detect Multi-Channel Flash Test ===" << std::endl;

        AutoDetectMultiChannelFlasher flasher;

        auto jobs = flasher.generateFlashJobs(
            "firmware/rc5_6_40_asw0.hex",
            "DEF_PASSWORD_021",
            16
        );

        if (jobs.empty()) {
            std::cout << "No flash jobs generated - check PCAN hardware" << std::endl;
            return;
        }

        std::cout << "\n=== Flash Plan ===" << std::endl;
        for (const auto& job : jobs) {
            // Handle PCAN enum discontinuity after USBBUS8
            if (job.canChannel > PCAN_USBBUS8)
                std::cout << job.controllerId << ": Channel "
                << (job.canChannel - PCAN_USBBUS1 - 0x4AF) << std::endl;
            else
                std::cout << job.controllerId << ": Channel "
                << (job.canChannel - PCAN_USBBUS1 + 1) << std::endl;
        }

        std::cout << "\nPress Enter to start simultaneous flashing (or Ctrl+C to cancel)...";
        std::cin.get();

        auto results = flasher.flashMultipleECUs(jobs);

        int success = 0;
        int failed = 0;
        for (const auto& result : results) {
            if (result.second) success++;
            else failed++;
        }

        std::cout << "\n=== FINAL RESULTS ===" << std::endl;
        std::cout << "Successfully flashed: " << success << " ECUs" << std::endl;
        std::cout << "Failed: " << failed << " ECUs" << std::endl;

        if (failed > 0) {
            std::cout << "\nFailed ECUs:" << std::endl;
            for (const auto& result : results) {
                if (!result.second) {
                    std::cout << "- " << result.first << std::endl;
                }
            }
        }
    }

    void testChannelDetection() {
        std::cout << "=== Quick Channel Detection Test ===" << std::endl;

        AutoDetectMultiChannelFlasher flasher;
        auto channels = flasher.detectChannels();

        if (channels.size() >= 6) {
            std::cout << "Perfect! Found " << channels.size()
                << " channels - ready for 6-ECU flashing" << std::endl;
        }
        else if (channels.size() > 0) {
            std::cout << "Found " << channels.size() << " channels - can flash "
                << channels.size() << " ECUs simultaneously" << std::endl;
        }
        else {
            std::cout << "No channels found - check PCAN-USB X6 connection" << std::endl;
        }

        std::cout << "\nDetected channels:" << std::endl;
        for (size_t i = 0; i < channels.size(); ++i) {
            // Handle PCAN enum discontinuity after USBBUS8
            if (channels[i] > PCAN_USBBUS8)
                std::cout << "PCAN_USBBUS" << (channels[i] - PCAN_USBBUS1 - 0x4AF) << std::endl;
            else
                std::cout << "PCAN_USBBUS" << (channels[i] - PCAN_USBBUS1 + 1) << std::endl;
        }
    }

    void testProductionLineFlashing() {
        std::cout << "=== Production Line ECU Flashing ===" << std::endl;
        std::cout << "This will continuously monitor for ECU insertion and flash automatically." << std::endl;
        std::cout << "Press Ctrl+C to stop the production line." << std::endl;
        std::cout << "Press Enter to start...";
        std::cin.get();

        ProductionLineFlasher flasher;

        flasher.runProductionLine(
            "firmware/rc5_6_40_asw0.hex",
            "DEF_PASSWORD_021"
        );
    }

    std::string readCBVersion(TPCANHandle channel) {
        RC40Flasher::ControllerConfig config("RC5-6/40", "CB_VERSION_CHECK", channel);
        config.canIdRequest = 0x18DA01FA;
        config.canIdResponse = 0x18DAFA01;

        RC40Flasher::RC40FlasherDevice flasher(config);

        if (!flasher.initialize()) {
            return "CAN_INIT_FAILED";
        }

        try {
            // Send tester present first
            std::vector<uint8_t> testerPresent = { 0x3E, 0x00 };
            flasher.sendUDSRequest(testerPresent, 1000);

            // Read CB version using DID 0xF180
            std::vector<uint8_t> readCmd = { 0x22, 0xF1, 0x80 };
            auto response = flasher.sendUDSRequest(readCmd, 2000);

            if (response.size() >= 3 && response[1] == 0x62 &&
                response[2] == 0xF1 && response[3] == 0x80) {
                // Extract version data (skip header: 62 F1 80)
                std::string version = "";
                for (size_t i = 4; i < response.size(); ++i) {
                    if (response[i] >= 0x20 && response[i] <= 0x7E) { // Printable ASCII
                        version += static_cast<char>(response[i]);
                    }
                    else {
                        version += "\\x" + std::to_string(response[i]);
                    }
                }
                flasher.cleanup();
                return version;
            }
            else {
                flasher.cleanup();
                return "READ_FAILED";
            }

        }
        catch (const std::exception& e) {
            flasher.cleanup();
            return "EXCEPTION: " + std::string(e.what());
        }
    }

    void checkAllCBVersions() {
        std::cout << "=== CB Version Check for All ECUs ===" << std::endl;

        RC40Flasher::AutoDetectMultiChannelFlasher detector;
        auto channels = detector.detectChannels();

        if (channels.empty()) {
            std::cout << "No channels detected" << std::endl;
            return;
        }

        for (size_t i = 0; i < channels.size(); ++i) {
            std::cout << "Channel " << (channels[i] - PCAN_USBBUS1 + 1) << ": ";

            // Check if ECU responds first
            if (RC40Flasher::pingECU(channels[i])) {
                std::string version = readCBVersion(channels[i]);
                std::cout << "CB Version: " << version << std::endl;
            }
            else {
                std::cout << "No ECU detected" << std::endl;
            }
        }

        std::cout << "\nDone checking CB versions." << std::endl;
    }

    void testCBVersionCheck() {
        std::cout << "=== Testing CB Version Reading ===" << std::endl;
        std::cout << "This will read CB version (DID 0xF180) from all connected ECUs" << std::endl;
        std::cout << "Press Enter to start...";
        std::cin.get();

        checkAllCBVersions();
    }

} // namespace RC40Flasher
