#include "RC40Flasher.h"
#include <iostream>
#include <iomanip>
#include <algorithm>


namespace RC40Flasher {

    // Constructor implementations
    MemoryBlock::MemoryBlock(const std::string& n, uint8_t id, uint32_t addr, uint32_t sz)
        : name(n), areaId(id), startAddress(addr), size(sz) {}

    ControllerConfig::ControllerConfig(const std::string& var, const std::string& id, TPCANHandle ch)
        : variant(var), controllerId(id), canChannel(ch), canIdRequest(0x18DA01FA), canIdResponse(0x18DAFA01) {}

    // RC40FlasherDevice implementation
    RC40FlasherDevice::RC40FlasherDevice(const ControllerConfig& cfg)
        : config(cfg), canInitialized(false) {}

    RC40FlasherDevice::~RC40FlasherDevice() {
        cleanup();
    }

    // Add this HEX file parser function
    std::vector<uint8_t> parseIntelHexFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open HEX file: " + filename);
        }

        std::map<uint32_t, uint8_t> addressData; // Address -> Data mapping
        uint32_t extendedAddress = 0;
        uint32_t minAddress = UINT32_MAX;
        uint32_t maxAddress = 0;

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] != ':') continue;

            if (line.length() < 11) {
                throw std::runtime_error("Invalid HEX line: " + line);
            }

            // Parse HEX line: :LLAAAATT[DD...]CC
            uint8_t byteCount = std::stoi(line.substr(1, 2), nullptr, 16);
            uint16_t address = std::stoi(line.substr(3, 4), nullptr, 16);
            uint8_t recordType = std::stoi(line.substr(7, 2), nullptr, 16);

            uint32_t fullAddress = extendedAddress + address;


            switch (recordType) {
            case 0x00: // Data record
                for (int i = 0; i < byteCount; ++i) {
                    uint8_t data = std::stoi(line.substr(9 + i * 2, 2), nullptr, 16);
                    addressData[fullAddress + i] = data;
                    minAddress = (std::min)(minAddress, fullAddress + i);
                    maxAddress = (std::max)(maxAddress, fullAddress + i);
                }
                break;

            case 0x01: // End of file
                break;

            case 0x02: // Extended Segment Address
                if (byteCount == 2) {
                    extendedAddress = std::stoi(line.substr(9, 4), nullptr, 16) << 4;
                }
                break;

            case 0x04: // Extended Linear Address
                if (byteCount == 2) {
                    extendedAddress = std::stoi(line.substr(9, 4), nullptr, 16) << 16;
                }
                break;

            default:
                std::cout << "Warning: Unsupported record type 0x" << std::hex << (int)recordType << std::dec << std::endl;
                break;
            }
        }

        if (addressData.empty()) {
            throw std::runtime_error("No data found in HEX file");
        }

        // Convert to contiguous binary data
        std::vector<uint8_t> binaryData;
        std::cout << "HEX file parsed: Address range 0x" << std::hex << minAddress
            << " to 0x" << maxAddress << std::dec << std::endl;
        std::cout << "Total data size: " << (maxAddress - minAddress + 1) << " bytes" << std::endl;

        // Fill gaps with 0xFF (common for flash memory)
        for (uint32_t addr = minAddress; addr <= maxAddress; ++addr) {
            auto it = addressData.find(addr);
            if (it != addressData.end()) {
                binaryData.push_back(it->second);
            }
            else {
                binaryData.push_back(0xFF); // Fill gaps
            }
        }

        std::cout << "Address analysis:" << std::endl;
        std::cout << "ASW0 range: 0x09020000 - 0x093BFFFF" << std::endl;
        std::cout << "DS0 range:  0x093C0000 - 0x094BFFFF" << std::endl;
        std::cout << "File range: 0x" << std::hex << minAddress << " - 0x" << maxAddress << std::dec << std::endl;

        if (maxAddress > 0x093BFFFF) {
            std::cout << "WARNING: HEX file contains DS0 data!" << std::endl;
        }

        return binaryData;
    }

    bool RC40FlasherDevice::initialize() {
        TPCANStatus result = CAN_Initialize(config.canChannel, PCAN_BAUD_250K, 0, 0, 0);
        if (result == PCAN_ERROR_OK) {
            canInitialized = true;
            std::cout << "[" << config.controllerId << "] CAN initialized successfully" << std::endl;
            return true;
        }
        else {
            std::cerr << "[" << config.controllerId << "] CAN initialization failed: 0x"
                << std::hex << result << std::dec << std::endl;
            return false;
        }
    }

    bool pingECU(TPCANHandle channel) {
        TPCANStatus result = CAN_Initialize(channel, PCAN_BAUD_250K, 0, 0, 0);
        if (result != PCAN_ERROR_OK && result != PCAN_ERROR_CAUTION) {
            return false;
        }

        // Send tester present
        TPCANMsg msg;
        msg.ID = 0x18DA01FA;
        msg.LEN = 8;
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        msg.DATA[0] = 0x02;  // Length
        msg.DATA[1] = 0x3E;  // Tester Present
        msg.DATA[2] = 0x00;  // Sub-function
        for (int i = 3; i < 8; i++) msg.DATA[i] = 0x55;

        bool responsive = false;
        if (CAN_Write(channel, &msg) == PCAN_ERROR_OK) {
            // Wait for response
            for (int attempts = 0; attempts < 100; attempts++) {
                TPCANMsg responseMsg;
                TPCANTimestamp timestamp;
                if (CAN_Read(channel, &responseMsg, &timestamp) == PCAN_ERROR_OK &&
                    responseMsg.ID == 0x18DAFA01) {
                    responsive = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        CAN_Uninitialize(channel);
        return responsive;
    }

    void RC40FlasherDevice::cleanup() {
        if (canInitialized) {
            CAN_Uninitialize(config.canChannel);
            canInitialized = false;
            std::cout << "[" << config.controllerId << "] CAN channel closed" << std::endl;
        }
    }

    bool RC40FlasherDevice::sendFunctionalCommand(const std::vector<uint8_t>& command) {
        TPCANMsg msg;
        msg.ID = 0x18DB33F1;  // Use confirmed functional address
        msg.LEN = 8; // Always use 8-byte frames
        // Pad unused bytes with 0x55 (like official tool)
        for (int i = command.size() + 1; i < 8; ++i) {
            msg.DATA[i] = 0x55;
        }
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        msg.DATA[0] = static_cast<BYTE>(command.size());

        for (size_t i = 0; i < command.size(); ++i) {
            msg.DATA[i + 1] = command[i];
        }

        TPCANStatus result = CAN_Write(config.canChannel, &msg);
        return result == PCAN_ERROR_OK;
    }

    std::vector<uint8_t> RC40FlasherDevice::sendUDSRequestWithMultiFrame(const std::vector<uint8_t>& request, int timeoutMs) {
        if (!canInitialized) {
            throw std::runtime_error("CAN not initialized");
        }

        if (request.size() <= 7) {
            // Single frame transmission
            TPCANMsg msg;
            msg.ID = config.canIdRequest;
            msg.LEN = 8; // Always use 8-byte frames
            // Pad unused bytes with 0x55 (like official tool)
            for (int i = request.size() + 1; i < 8; ++i) {
                msg.DATA[i] = 0x55;
            }
            msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
            msg.DATA[0] = static_cast<BYTE>(request.size());
            for (size_t i = 0; i < request.size(); ++i) {
                msg.DATA[i + 1] = request[i];
            }

            if (CAN_Write(config.canChannel, &msg) != PCAN_ERROR_OK) {
                throw std::runtime_error("Failed to send single frame");
            }
        }
        else {
            // Multi-frame transmission
            std::cout << "[" << config.controllerId << "] Sending multi-frame request ("
                << request.size() << " bytes)" << std::endl;

            // Send First Frame
            TPCANMsg ffMsg;
            ffMsg.ID = config.canIdRequest;
            ffMsg.LEN = 8;
            ffMsg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
            ffMsg.DATA[0] = 0x10 | ((request.size() >> 8) & 0x0F);  // First frame + length high nibble
            ffMsg.DATA[1] = request.size() & 0xFF;                  // Length low byte

            // Fill first frame with up to 6 bytes of data
            size_t firstFrameDataSize = (std::min)(static_cast<size_t>(6), request.size());
            for (size_t i = 0; i < firstFrameDataSize; ++i) {
                ffMsg.DATA[i + 2] = request[i];
            }

            if (CAN_Write(config.canChannel, &ffMsg) != PCAN_ERROR_OK) {
                throw std::runtime_error("Failed to send first frame");
            }

            // Wait for Flow Control
            bool gotFlowControl = false;
            auto fcStartTime = std::chrono::steady_clock::now();

            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - fcStartTime).count() < 1000) {

                TPCANMsg fcMsg;
                TPCANTimestamp timestamp;
                if (CAN_Read(config.canChannel, &fcMsg, &timestamp) == PCAN_ERROR_OK &&
                    fcMsg.ID == config.canIdResponse) {

                    if (fcMsg.LEN >= 1 && (fcMsg.DATA[0] & 0xF0) == 0x30) {  // Flow Control
                        uint8_t flowStatus = fcMsg.DATA[0] & 0x0F;
                        if (flowStatus == 0x00) {  // Continue To Send
                            std::cout << "[" << config.controllerId << "] Received Flow Control - CTS" << std::endl;
                            gotFlowControl = true;
                            break;
                        }
                        else {
                            throw std::runtime_error("Flow control indicates wait or abort");
                        }
                    }
                }
                //std::this_thread::sleep_for(std::chrono::milliseconds(1)); // don't delay, make it go BRRRRRT
            }

            if (!gotFlowControl) {
                throw std::runtime_error("No flow control received");
            }

            // Send Consecutive Frames
            size_t bytesSent = firstFrameDataSize;
            uint8_t sequenceNumber = 1;

            while (bytesSent < request.size()) {
                TPCANMsg cfMsg;
                cfMsg.ID = config.canIdRequest;
                cfMsg.LEN = 8;
                cfMsg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
                cfMsg.DATA[0] = 0x20 | (sequenceNumber & 0x0F);  // Consecutive frame + sequence

                // Fill with up to 7 bytes of data
                size_t frameBytesToSend = (std::min)(static_cast<size_t>(7), request.size() - bytesSent);
                for (size_t i = 0; i < frameBytesToSend; ++i) {
                    cfMsg.DATA[i + 1] = request[bytesSent + i];
                }

                // Pad unused bytes
                for (size_t i = frameBytesToSend + 1; i < 8; ++i) {
                    cfMsg.DATA[i] = 0x00;
                }

                if (CAN_Write(config.canChannel, &cfMsg) != PCAN_ERROR_OK) {
                    throw std::runtime_error("Failed to send consecutive frame");
                }

                bytesSent += frameBytesToSend;
                sequenceNumber = (sequenceNumber + 1) & 0x0F;

                // Small delay between consecutive frames
                //std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            std::cout << "[" << config.controllerId << "] Multi-frame transmission complete" << std::endl;
        }

        // Now wait for response (same as before)...
        std::vector<uint8_t> fullResponse;
        auto startTime = std::chrono::steady_clock::now();

        while (true) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - startTime).count();

            if (elapsed > timeoutMs) {
                throw std::runtime_error("UDS request timeout");
            }

            TPCANMsg responseMsg;
            TPCANTimestamp timestamp;
            TPCANStatus result = CAN_Read(config.canChannel, &responseMsg, &timestamp);

            if (result == PCAN_ERROR_OK && responseMsg.ID == config.canIdResponse) {
                if (responseMsg.LEN > 0) {
                    uint8_t pci = responseMsg.DATA[0];

                    if ((pci & 0xF0) == 0x00) {
                        // Single Frame response
                        uint8_t length = pci & 0x0F;
                        for (int i = 1; i <= length && i < responseMsg.LEN; ++i) {
                            fullResponse.push_back(responseMsg.DATA[i]);
                        }

                        // Handle response pending
                        if (fullResponse.size() >= 3 && fullResponse[0] == 0x7F && fullResponse[2] == 0x78) {
                            std::cout << "[" << config.controllerId << "] Response pending..." << std::endl;
                            fullResponse.clear();
                            continue;
                        }

                        return fullResponse;
                    }
                    else if ((pci & 0xF0) == 0x10) {
                        // Multi-frame response (same handling as before)
                        uint16_t totalLength = ((pci & 0x0F) << 8) | responseMsg.DATA[1];

                        for (int i = 2; i < responseMsg.LEN; ++i) {
                            fullResponse.push_back(responseMsg.DATA[i]);
                        }

                        // Send Flow Control
                        TPCANMsg fcMsg;
                        fcMsg.ID = config.canIdRequest;
                        fcMsg.LEN = 8;
                        fcMsg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
                        fcMsg.DATA[0] = 0x30;  // Flow Control - CTS
                        fcMsg.DATA[1] = 0x00;  // Block Size
                        fcMsg.DATA[2] = 0x00;  // Separation Time
                        for (int i = 3; i < 8; i++) {
                            fcMsg.DATA[i] = 0x00; // add padding
                        }

                        CAN_Write(config.canChannel, &fcMsg);

                        // Receive consecutive frames
                        uint8_t expectedSequence = 1;
                        while (fullResponse.size() < totalLength) {
                            auto frameStartTime = std::chrono::steady_clock::now();
                            bool gotFrame = false;

                            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - frameStartTime).count() < 1000) {

                                TPCANMsg cfMsg;
                                TPCANTimestamp cfTs;
                                if (CAN_Read(config.canChannel, &cfMsg, &cfTs) == PCAN_ERROR_OK &&
                                    cfMsg.ID == config.canIdResponse) {

                                    uint8_t cfPci = cfMsg.DATA[0];
                                    if ((cfPci & 0xF0) == 0x20) {
                                        uint8_t sequence = cfPci & 0x0F;
                                        if (sequence == expectedSequence) {
                                            for (int i = 1; i < cfMsg.LEN && fullResponse.size() < totalLength; ++i) {
                                                fullResponse.push_back(cfMsg.DATA[i]);
                                            }
                                            expectedSequence = (expectedSequence + 1) & 0x0F;
                                            gotFrame = true;
                                            break;
                                        }
                                    }
                                }
                                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            }

                            if (!gotFrame) {
                                throw std::runtime_error("Timeout waiting for consecutive frame");
                            }
                        }

                        return fullResponse;
                    }
                }
            }
            else if (result != PCAN_ERROR_QRCVEMPTY) {
                throw std::runtime_error("CAN read error: 0x" + std::to_string(result));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::vector<uint8_t> RC40FlasherDevice::sendUDSRequest(const std::vector<uint8_t>& request, int timeoutMs) {
        if (!canInitialized) {
            throw std::runtime_error("CAN not initialized");
        }

        if (request.size() > 7) {
            throw std::runtime_error("Multi-frame UDS not implemented yet");
        }

        TPCANMsg msg;
        msg.ID = config.canIdRequest;
        msg.LEN = 8; // Always use 8-byte frames
        // Pad unused bytes with 0x55 (like official tool)
        for (int i = request.size() + 1; i < 8; ++i) {
            msg.DATA[i] = 0x55;
        }
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;  // Changed to extended

        // ISO-TP single frame: first byte is length
        msg.DATA[0] = static_cast<BYTE>(request.size());
        for (size_t i = 0; i < request.size(); ++i) {
            msg.DATA[i + 1] = request[i];
        }

        // Send message
        TPCANStatus result = CAN_Write(config.canChannel, &msg);
        if (result != PCAN_ERROR_OK) {
            throw std::runtime_error("Failed to send CAN message");
        }

        // Wait for response with proper 0x78 (pending) handling
        auto startTime = std::chrono::steady_clock::now();
        while (true) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - startTime).count();

            if (elapsed > timeoutMs) {
                throw std::runtime_error("UDS request timeout");
            }

            TPCANMsg responseMsg;
            TPCANTimestamp timestamp;
            result = CAN_Read(config.canChannel, &responseMsg, &timestamp);

            if (result == PCAN_ERROR_OK && responseMsg.ID == config.canIdResponse) {
                if (responseMsg.LEN > 1) {
                    std::vector<uint8_t> response;
                    for (int i = 1; i < responseMsg.LEN; ++i) {
                        response.push_back(responseMsg.DATA[i]);
                    }

                    // Handle response pending (0x7F XX 0x78)
                    if (response.size() >= 3 && response[0] == 0x7F && response[2] == 0x78) {
                        std::cout << "[" << config.controllerId << "] Response pending, continuing to wait..." << std::endl;
                        continue;  // Keep waiting for final response
                    }

                    return response;
                }
            }
            else if (result != PCAN_ERROR_QRCVEMPTY) {
                throw std::runtime_error("CAN read error: 0x" + std::to_string(result));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    bool RC40FlasherDevice::diagnosticSessionControl(uint8_t sessionType) {
        std::vector<uint8_t> request = { 0x10, sessionType };

        // First attempt
        try {
            auto response = sendUDSRequest(request, 10000);

            if (response.size() >= 2 && response[0] == 0x50) {
                std::cout << "[" << config.controllerId << "] Entered diagnostic session 0x"
                    << std::hex << (int)sessionType << std::dec << std::endl;
                return true;
            }
        }
        catch (const std::exception& e) {
            std::cout << "[" << config.controllerId << "] First attempt exception: " << e.what() << std::endl;
        }

        // Second attempt for programming mode only
        if (sessionType == 0x02) {
            std::cout << "[" << config.controllerId << "] Retrying programming mode..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            try {
                auto response = sendUDSRequest(request, 10000);

                if (response.size() >= 2 && response[0] == 0x50) {
                    std::cout << "[" << config.controllerId << "] Entered diagnostic session 0x"
                        << std::hex << (int)sessionType << std::dec << " on second attempt" << std::endl;
                    return true;
                }
            }
            catch (const std::exception& e) {
                std::cerr << "[" << config.controllerId << "] Second attempt failed: " << e.what() << std::endl;
            }
        }

        std::cout << "[" << config.controllerId << "] Diagnostic session failed after all attempts" << std::endl;
        return false;
    }

    std::vector<uint8_t> RC40FlasherDevice::securityAccessRequestSeed() {
        try {
            std::vector<uint8_t> request = { 0x27, 0x01 };
            auto response = sendUDSRequestWithMultiFrame(request, 5000);  // Use multi-frame version

            if (response.size() >= 18 && response[0] == 0x67 && response[1] == 0x01) {
                std::vector<uint8_t> seed(response.begin() + 2, response.begin() + 18);
                std::cout << "[" << config.controllerId << "] Received 16-byte security seed" << std::endl;
                return seed;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "[" << config.controllerId << "] Security seed request failed: "
                << e.what() << std::endl;
        }
        return {};
    }

    bool RC40FlasherDevice::securityAccessSendKey(const std::vector<uint8_t>& key) {
        if (key.size() != 16) {
            return false;
        }

        try {
            std::vector<uint8_t> request = { 0x27, 0x02 };
            request.insert(request.end(), key.begin(), key.end());

            auto response = sendUDSRequestWithMultiFrame(request, 5000);  // Use multi-frame version

            if (response.size() >= 2 && response[0] == 0x67 && response[1] == 0x02) {
                std::cout << "[" << config.controllerId << "] Security access unlocked successfully!" << std::endl;
                return true;
            }

            if (response.size() >= 3 && response[0] == 0x7F) {
                std::cout << "[" << config.controllerId << "] Security access denied - Error: 0x"
                    << std::hex << (int)response[2] << std::dec << std::endl;
                return false;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "[" << config.controllerId << "] Security access failed: "
                << e.what() << std::endl;
        }
        return false;
    }

    bool RC40FlasherDevice::eraseMemory(uint8_t areaId) {
        try {
            std::vector<uint8_t> request = { 0x31, 0x01, 0xFF, 0x00, 0x01, areaId };
            auto response = sendUDSRequestWithMultiFrame(request, 30000);

            // Check for positive response: 71 01 FF 00 00
            if (response.size() >= 5 &&
                response[0] == 0x71 &&    // Positive response to routine control
                response[1] == 0x01 &&    // Start routine echo
                response[2] == 0xFF &&    // Routine ID high byte
                response[3] == 0x00 &&    // Routine ID low byte  
                response[4] == 0x00) {    // Success status

                std::cout << "[" << config.controllerId << "] Erased memory area " << (int)areaId << std::endl;
                return true;
            }

            std::cout << "[" << config.controllerId << "] Erase failed - unexpected response" << std::endl;
            return false;

        }
        catch (const std::exception& e) {
            std::cerr << "[" << config.controllerId << "] Erase memory failed: " << e.what() << std::endl;
            return false;
        }
    }

    uint16_t RC40FlasherDevice::requestDownload(uint32_t address, uint32_t size) {
        try {
            std::vector<uint8_t> request = { 0x34, 0x00, 0x44 };

            // Add 4-byte address (big-endian)
            request.push_back((address >> 24) & 0xFF);
            request.push_back((address >> 16) & 0xFF);
            request.push_back((address >> 8) & 0xFF);
            request.push_back(address & 0xFF);

            // Add 4-byte size (big-endian)  
            request.push_back((size >> 24) & 0xFF);
            request.push_back((size >> 16) & 0xFF);
            request.push_back((size >> 8) & 0xFF);
            request.push_back(size & 0xFF);

            auto response = sendUDSRequestWithMultiFrame(request, 5000);  // Use multi-frame version

            if (response.size() >= 4 && response[0] == 0x74) {
                uint16_t maxBlockLength = (response[2] << 8) | response[3];
                std::cout << "[" << config.controllerId << "] Request download OK, max block: "
                    << maxBlockLength << std::endl;
                return maxBlockLength;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "[" << config.controllerId << "] Request download failed: "
                << e.what() << std::endl;
        }
        return 0;
    }

    bool RC40FlasherDevice::transferData(const std::vector<uint8_t>& data, uint16_t maxBlockSize) {
        size_t effectiveBlockSize = maxBlockSize - 2; // Account for service ID and counter
        size_t totalBlocks = (data.size() + effectiveBlockSize - 1) / effectiveBlockSize;

        for (size_t i = 0; i < totalBlocks; ++i) {
            uint8_t sequenceCounter = static_cast<uint8_t>((i + 1) & 0xFF);

            size_t offset = i * effectiveBlockSize;
            size_t blockSize = (effectiveBlockSize < (data.size() - offset)) ? effectiveBlockSize : (data.size() - offset);

            std::vector<uint8_t> request = { 0x36, sequenceCounter };
            request.insert(request.end(), data.begin() + offset, data.begin() + offset + blockSize);

            try {
                auto response = sendUDSRequestWithMultiFrame(request, 5000);  // Multi-frame version

                if (!(response.size() >= 2 && response[0] == 0x76 &&
                    response[1] == sequenceCounter)) {
                    std::cerr << "[" << config.controllerId << "] Transfer failed at block "
                        << (int)sequenceCounter << std::endl;
                    return false;
                }

                if ((i + 1) % 10 == 0) { // Progress update every 10 blocks
                    double progress = (double)(i + 1) / totalBlocks * 100.0;
                    std::cout << "[" << config.controllerId << "] Transfer progress: "
                        << std::fixed << std::setprecision(1) << progress << "%" << std::endl;
                }
            }
            catch (const std::exception& e) {
                std::cerr << "[" << config.controllerId << "] Transfer data error: "
                    << e.what() << std::endl;
                return false;
            }
        }

        return true;
    }

    // Add this to your RC40FlasherDevice class
    struct DataRegion {
        uint32_t address;
        std::vector<uint8_t> data;
    };

    std::vector<DataRegion> findDataRegions(const std::vector<uint8_t>& firmwareData, uint32_t baseAddress) {
        std::vector<DataRegion> regions;

        const size_t minRegionSize = 256; // Minimum bytes to justify separate transfer
        const size_t maxPaddingGap = 1024; // Skip gaps larger than this

        size_t regionStart = 0;
        bool inDataRegion = false;

        for (size_t i = 0; i < firmwareData.size(); ++i) {
            bool isData = (firmwareData[i] != 0xFF); // Assume 0xFF is padding

            if (isData && !inDataRegion) {
                // Start of new data region
                regionStart = i;
                inDataRegion = true;
            }
            else if (!isData && inDataRegion) {
                // End of data region - check if we should close it
                size_t paddingCount = 0;
                size_t j = i;

                // Count consecutive padding bytes
                while (j < firmwareData.size() && firmwareData[j] == 0xFF && paddingCount < maxPaddingGap) {
                    paddingCount++;
                    j++;
                }

                if (paddingCount >= maxPaddingGap || j >= firmwareData.size()) {
                    // Large gap or end of data - close current region
                    size_t regionSize = i - regionStart;

                    if (regionSize >= minRegionSize) {
                        DataRegion region;
                        region.address = baseAddress + regionStart;
                        region.data = std::vector<uint8_t>(
                            firmwareData.begin() + regionStart,
                            firmwareData.begin() + i
                        );
                        regions.push_back(region);

                        std::cout << "Data region: 0x" << std::hex << region.address
                            << " size: " << std::dec << region.data.size() << " bytes" << std::endl;
                    }

                    inDataRegion = false;
                    i = j - 1; // Skip the padding gap
                }
            }
        }

        // Handle final region
        if (inDataRegion) {
            size_t regionSize = firmwareData.size() - regionStart;
            if (regionSize >= minRegionSize) {
                DataRegion region;
                region.address = baseAddress + regionStart;
                region.data = std::vector<uint8_t>(
                    firmwareData.begin() + regionStart,
                    firmwareData.end()
                );
                regions.push_back(region);

                std::cout << "Final region: 0x" << std::hex << region.address
                    << " size: " << std::dec << region.data.size() << " bytes" << std::endl;
            }
        }

        return regions;
    }

    bool RC40FlasherDevice::transferDataSparse(uint8_t areaId, const std::vector<uint8_t>& firmwareData, uint32_t baseAddress) {
        // Find actual data regions (skip large padding areas)
        auto regions = findDataRegions(firmwareData, baseAddress);

        std::cout << "Found " << regions.size() << " data regions to transfer" << std::endl;

        size_t totalBytes = 0;
        for (const auto& region : regions) {
            totalBytes += region.data.size();
        }

        std::cout << "Transferring " << totalBytes << " bytes (skipping "
            << (firmwareData.size() - totalBytes) << " padding bytes)" << std::endl;

        // Transfer each region separately
        for (const auto& region : regions) {
            std::cout << "Transferring region at 0x" << std::hex << region.address
                << " (" << std::dec << region.data.size() << " bytes)" << std::endl;

            // Request download for this specific region
            uint16_t maxBlockSize = requestDownload(region.address, region.data.size());
            if (maxBlockSize == 0) {
                std::cout << "Request download failed for region 0x" << std::hex << region.address << std::endl;
                return false;
            }

            // Transfer the region data
            if (!transferData(region.data, maxBlockSize)) {
                std::cout << "Transfer failed for region 0x" << std::hex << region.address << std::endl;
                return false;
            }

            // Transfer exit for this region
            if (!requestTransferExit()) {
                std::cout << "Transfer exit failed for region 0x" << std::hex << region.address << std::endl;
                return false;
            }
        }

        return true;
    }

    bool RC40FlasherDevice::requestTransferExit() {
        try {
            std::vector<uint8_t> request = { 0x37 };
            auto response = sendUDSRequestWithMultiFrame(request, 5000);  // Multi-frame

            if (response.size() >= 1 && response[0] == 0x77) {
                std::cout << "[" << config.controllerId << "] Transfer exit successful" << std::endl;
                return true;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "[" << config.controllerId << "] Transfer exit failed: "
                << e.what() << std::endl;
        }
        return false;
    }

    bool RC40FlasherDevice::checkMemory(uint8_t areaId) {
        try {
            std::vector<uint8_t> request = { 0x31, 0x01, 0x02, 0x02, 0x01, areaId };
            auto response = sendUDSRequestWithMultiFrame(request, 10000);  // Multi-frame

            if (response.size() >= 5 && response[0] == 0x71 && response[4] == 0x00) {
                std::cout << "[" << config.controllerId << "] Memory check passed for area "
                    << (int)areaId << std::endl;
                return true;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "[" << config.controllerId << "] Memory check failed: "
                << e.what() << std::endl;
        }
        return false;
    }

    bool RC40FlasherDevice::ecuReset() {
        try {
            std::vector<uint8_t> request = { 0x11, 0x01 };
            auto response = sendUDSRequestWithMultiFrame(request, 2000);

            if (response.size() >= 2 && response[0] == 0x51) {
                std::cout << "[" << config.controllerId << "] ECU reset successful" << std::endl;
                return true;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "[" << config.controllerId << "] ECU reset failed: "
                << e.what() << std::endl;
        }
        return false;
    }

    const std::string& RC40FlasherDevice::getControllerId() const {
        return config.controllerId;
    }

    // Main flashing function
    bool RC40FlasherDevice::flashController(const std::map<std::string, std::string>& firmwareFiles,
        const std::vector<uint8_t>& password) {
        std::cout << "[" << config.controllerId << "] Starting flash process..." << std::endl;

        try {
            // Step 1: Preparation phase (FUNCTIONAL ADDRESSING)
            std::cout << "[" << config.controllerId << "] Preparation phase (functional addressing)..." << std::endl;

            if (!sendFunctionalCommand({ 0x10, 0x83 })) return false;  // Extended diagnostic
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            if (!sendFunctionalCommand({ 0x85, 0x82 })) return false;  // DTC OFF  
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            if (!sendFunctionalCommand({ 0x28, 0x81, 0x01 })) return false;  // Comm control
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // Step 2: Programming mode (PHYSICAL ADDRESSING)
            std::cout << "[" << config.controllerId << "] Entering programming mode (physical addressing)..." << std::endl;
            if (!diagnosticSessionControl(0x02)) return false; // Programming mode


            // Step 3: Security access
            auto seed = securityAccessRequestSeed();
            if (seed.empty()) return false;

            auto key = AES128Security::calculateKeyFromSeed(seed, password);
            if (!securityAccessSendKey(key)) return false;

            // Step 4: Flash each block
            for (const auto& block : config.blocks) {
                auto it = firmwareFiles.find(block.name);
                if (it == firmwareFiles.end()) {
                    std::cout << "[" << config.controllerId << "] Skipping block "
                        << block.name << " - no firmware provided" << std::endl;
                    continue;
                }

                // Load firmware file
                std::ifstream file(it->second, std::ios::binary);
                if (!file.is_open()) {
                    std::cerr << "[" << config.controllerId << "] Cannot open firmware file: "
                        << it->second << std::endl;
                    return false;
                }

                std::vector<uint8_t> firmwareData(
                    (std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>()
                );
                file.close();

                std::cout << "[" << config.controllerId << "] Flashing block "
                    << block.name << " (" << firmwareData.size() << " bytes)" << std::endl;

                // Flash sequence for this block
                if (!eraseMemory(block.areaId)) return false;

                uint16_t maxBlockSize = requestDownload(block.startAddress, (uint32_t)firmwareData.size());
                if (maxBlockSize == 0) return false;

                if (!transferData(firmwareData, maxBlockSize)) return false;
                if (!requestTransferExit()) return false;
                if (!checkMemory(block.areaId)) return false;
            }

            // Step 5: Finalization
            if (!ecuReset()) return false;

            std::this_thread::sleep_for(std::chrono::seconds(2)); // Wait for reboot

            std::cout << "[" << config.controllerId << "] Flash completed successfully!" << std::endl;
            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "[" << config.controllerId << "] Flash failed: " << e.what() << std::endl;
            return false;
        }
    }

    // Helper function to create controller configurations
    std::vector<ControllerConfig> createControllerConfigs(const std::string& variant,
        const std::vector<std::string>& controllerIds) {
        std::vector<ControllerConfig> configs;

        // Available PCAN channels
        std::vector<TPCANHandle> channels = {
            PCAN_USBBUS1, PCAN_USBBUS2, PCAN_USBBUS3, PCAN_USBBUS4,
            PCAN_USBBUS5, PCAN_USBBUS6, PCAN_USBBUS7, PCAN_USBBUS8
        };

        for (size_t i = 0; i < controllerIds.size() && i < channels.size(); ++i) {
            ControllerConfig config(variant, controllerIds[i], channels[i]);

            // Add memory blocks based on variant
            if (variant == "RC5-6/40") {
                config.blocks.emplace_back("CB", 0x00, 0x08FD8000, 0x00048000);
                config.blocks.emplace_back("ASW0", 0x01, 0x09020000, 0x003A0000);
                config.blocks.emplace_back("DS0", 0x02, 0x093C0000, 0x00100000);
            }
            else if (variant == "RC18-12/40" || variant == "RC27-18/40") {
                config.blocks.emplace_back("CB", 0x00, 0x80070000, 0x00050000);
                config.blocks.emplace_back("ASW0", 0x01, 0x800C0000, 0x00740000);
                config.blocks.emplace_back("DS0", 0x02, 0x80800000, 0x00100000);
            }

            configs.push_back(config);
        }

        return configs;
    }

    std::vector<uint8_t> AES128Security::calculateKeyFromSeed(const std::vector<uint8_t>& seed,
        const std::vector<uint8_t>& password) {
        if (seed.size() != 16) {
            throw std::invalid_argument("Seed must be exactly 16 bytes");
        }

        // Prepare 16-byte AES key from password  
        std::vector<uint8_t> aesKey(16, 0);
        size_t copySize = password.size() < 16 ? password.size() : 16;
        std::copy(password.begin(), password.begin() + copySize, aesKey.begin());

        // Allocate extra buffer space to prevent overruns
        std::vector<uint8_t> key(32, 0); // Double the size for safety

        EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
        if (!ctx) {
            throw std::runtime_error("Failed to create AES context");
        }

        try {
            // Disable padding to ensure exactly 16 bytes in/out
            if (EVP_EncryptInit_ex(ctx, EVP_aes_128_ecb(), nullptr, aesKey.data(), nullptr) != 1) {
                throw std::runtime_error("Failed to initialize AES encryption");
            }

            if (EVP_CIPHER_CTX_set_padding(ctx, 0) != 1) {
                throw std::runtime_error("Failed to disable padding");
            }

            int outLen = 0;
            if (EVP_EncryptUpdate(ctx, key.data(), &outLen, seed.data(), 16) != 1) {
                throw std::runtime_error("Failed to encrypt seed");
            }

            int finalLen = 0;
            if (EVP_EncryptFinal_ex(ctx, key.data() + outLen, &finalLen) != 1) {
                throw std::runtime_error("Failed to finalize AES encryption");
            }

            EVP_CIPHER_CTX_free(ctx);

            // Return only the first 16 bytes
            key.resize(16);
            return key;
        }
        catch (...) {
            EVP_CIPHER_CTX_free(ctx);
            throw;
        }
    }

    bool AES128Security::testAES() {
        std::cout << "Testing AES implementation..." << std::endl;

        try {
            std::vector<uint8_t> testSeed = {
                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10
            };
            std::vector<uint8_t> testPassword = {
                'T', 'e', 's', 't', 'P', 'a', 's', 's', 'w', 'o', 'r', 'd', '1', '2', '3', '4'
            };

            std::cout << "Calling calculateKeyFromSeed..." << std::endl;
            auto key = calculateKeyFromSeed(testSeed, testPassword);
            std::cout << "calculateKeyFromSeed returned successfully" << std::endl;

            std::cout << "Key size: " << key.size() << std::endl;

            if (key.size() != 16) {
                std::cout << "ERROR: Key size is not 16 bytes!" << std::endl;
                return false;
            }

            std::cout << "AES Test Key: ";
            for (size_t i = 0; i < key.size(); ++i) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)key[i];
            }
            std::cout << std::dec << std::endl;

            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "AES test failed with exception: " << e.what() << std::endl;
            return false;
        }
        catch (...) {
            std::cerr << "AES test failed with unknown exception" << std::endl;
            return false;
        }
    }

} // namespace RC40Flasher
