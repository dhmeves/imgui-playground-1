#include "ProductionFlasher.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace RC40Flasher {

    // MultiChannelFlasher implementation
    void MultiChannelFlasher::threadSafeLog(const std::string& message) {
        std::lock_guard<std::mutex> lock(logMutex);
        std::cout << message << std::endl;
    }

    bool MultiChannelFlasher::flashSingleECU(const FlashJob& job, const std::vector<uint8_t>& firmwareData) {
        // Add startup delay based on channel number to stagger initialization
        int channelDelay = (job.canChannel - PCAN_USBBUS1) * 3000; // 1 second per channel
        if (channelDelay > 0) {
            threadSafeLog("[" + job.controllerId + "] Waiting " + std::to_string(channelDelay / 1000) + "s before start...");
            std::this_thread::sleep_for(std::chrono::milliseconds(channelDelay));
        }

        ControllerConfig config("RC5-6/40", job.controllerId, job.canChannel);
        config.canIdRequest = job.requestId;
        config.canIdResponse = job.responseId;

        RC40FlasherDevice flasher(config);

        threadSafeLog("[" + job.controllerId + "] Starting flash process...");

        if (!flasher.initialize()) {
            threadSafeLog("[" + job.controllerId + "] CAN initialization failed");
            return false;
        }

        // Load firmware data first (before any CAN operations)
        //std::vector<uint8_t> firmwareData;
        //try {
        //    firmwareData = flasher.parseIntelHexFile(job.hexFilePath);
        //    threadSafeLog("[" + job.controllerId + "] Loaded " + std::to_string(firmwareData.size()) + " bytes");
        //}
        //catch (const std::exception& e) {
        //    threadSafeLog("[" + job.controllerId + "] HEX file error: " + std::string(e.what()));
        //    return false;
        //}

        try {
            // Tester present
            std::vector<uint8_t> testerPresent = { 0x3E, 0x00 };
            flasher.sendUDSRequest(testerPresent, 1000);

            // Programming mode
            if (!flasher.diagnosticSessionControl(0x02)) {
                threadSafeLog("[" + job.controllerId + "] Programming mode failed");
                return false;
            }

            // Security access
            auto seed = flasher.securityAccessRequestSeed();
            std::vector<uint8_t> password(job.password.begin(), job.password.end());
            auto key = AES128Security::calculateKeyFromSeed(seed, password);

            if (!flasher.securityAccessSendKey(key)) {
                threadSafeLog("[" + job.controllerId + "] Security access failed");
                return false;
            }

            // Fingerprint data
            std::vector<uint8_t> dateCmd = { 0x2E, 0xF1, 0x99 };
            std::string progDate = "22092025";
            dateCmd.insert(dateCmd.end(), progDate.begin(), progDate.end());

            auto dateResp = flasher.sendUDSRequestWithMultiFrame(dateCmd, 2000);
            if (!(dateResp.size() >= 2 && dateResp[0] == 0x6E)) {
                threadSafeLog("[" + job.controllerId + "] Fingerprint failed");
                return false;
            }

            // Split firmware data
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

            // Flash ASW0
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
                threadSafeLog("[" + job.controllerId + "] ASW0 check failed");
                return false;
            }

            // Flash DS0 if exists
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
                    threadSafeLog("[" + job.controllerId + "] DS0 check failed");
                    return false;
                }
            }

            // Reset ECU
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

        // Parse HEX file once
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

        // Launch threads with shared firmware data
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

        // Summary
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

    // AutoDetectMultiChannelFlasher implementation
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

    // ProductionLineFlasher implementation
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
                    threadSafeLog("[Station " + std::to_string(station.stationId) + "] ECU detected: " + station.controllerId);
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
                    threadSafeLog("[Station " + std::to_string(station.stationId) + "] ECU removed - ready for next");
                }
                else {
                    // Show removal prompt after 10 seconds
                    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - station.lastStateChange).count();
                    if (elapsed > 10 && station.state != READY_FOR_REMOVAL) {
                        station.state = READY_FOR_REMOVAL;
                        std::string status = (station.state == FLASH_SUCCESS) ? "SUCCESS" : "FAILED";
                        threadSafeLog("[Station " + std::to_string(station.stationId) + "] Flash " + status + " - Please remove ECU");
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

            // Start flashing for detected ECUs
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
                            job.hexFilePath = ""; // Not needed anymore
                            job.password = password;
                            job.requestId = 0x18DA01FA;
                            job.responseId = 0x18DAFA01;

                            threadSafeLog("[Station " + std::to_string(station.stationId) + "] Starting flash...");

                            bool result = flashSingleECU(job, firmwareData); // Pass firmware data

                            station.flashAttempts++;
                            station.state = result ? FLASH_SUCCESS : FLASH_FAILED;
                            station.lastStateChange = std::chrono::steady_clock::now();

                            if (result) {
                                threadSafeLog("[Station " + std::to_string(station.stationId) + "] *** FLASH SUCCESS *** (attempt " + std::to_string(station.flashAttempts) + ")");
                            }
                            else {
                                threadSafeLog("[Station " + std::to_string(station.stationId) + "] *** FLASH FAILED *** (attempt " + std::to_string(station.flashAttempts) + ")");
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
                (station.flashAttempts > 0 ? " (attempts: " + std::to_string(station.flashAttempts) + ")" : ""));
        }
    }

    void ProductionLineFlasher::stopProductionLine() {
        stopScanning = true;
    }

    // Test functions
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

            if (job.canChannel > PCAN_USBBUS8) // for some reason the PCAN_USB enum changes its increment after 8
                std::cout << job.controllerId << ": Channel " << (job.canChannel - PCAN_USBBUS1 - 0x4AF) << std::endl;
            else
                std::cout << job.controllerId << ": Channel " << (job.canChannel - PCAN_USBBUS1 + 1) << std::endl;
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
            std::cout << "Perfect! Found " << channels.size() << " channels - ready for 6-ECU flashing" << std::endl;
        }
        else if (channels.size() > 0) {
            std::cout << "Found " << channels.size() << " channels - can flash " << channels.size() << " ECUs simultaneously" << std::endl;
        }
        else {
            std::cout << "No channels found - check PCAN-USB X6 connection" << std::endl;
        }

        std::cout << "\nDetected channels:" << std::endl;
        for (size_t i = 0; i < channels.size(); ++i) {
            if (channels[i] > PCAN_USBBUS8) // for some reason the PCAN_USB enum changes its increment after 8
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

} // namespace RC40Flasher
