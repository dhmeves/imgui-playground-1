/**
 * @file RC40Flasher.h
 * @brief Core ECU flashing functionality for RC40 controllers
 *
 * Provides low-level UDS (Unified Diagnostic Services) communication
 * and flashing operations over CAN bus using PEAK PCAN hardware.
 */

#pragma once

#include <windows.h>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <stdexcept>
#include <fstream>
#include <chrono>
#include <thread>
#include <functional>

 // PEAK PCAN includes
#include "PCANBasic.h"

// OpenSSL includes for AES security
#include <openssl/aes.h>
#include <openssl/evp.h>

namespace RC40Flasher {

    /**
     * @struct MemoryBlock
     * @brief Defines a flash memory region in the ECU
     */
    struct MemoryBlock {
        std::string name;           ///< Block name (e.g., "CB", "ASW0", "DS0")
        uint8_t areaId;            ///< UDS area identifier
        uint32_t startAddress;     ///< Physical start address in flash
        uint32_t size;             ///< Size in bytes

        MemoryBlock(const std::string& n, uint8_t id, uint32_t addr, uint32_t sz);
    };

    /**
     * @struct ControllerConfig
     * @brief Configuration for a single ECU instance
     */
    struct ControllerConfig {
        std::string variant;            ///< Controller variant (e.g., "RC5-6/40")
        std::string controllerId;       ///< Unique identifier for logging
        TPCANHandle canChannel;         ///< PCAN channel handle
        uint32_t canIdRequest;          ///< CAN ID for requests (default: 0x18DA01FA)
        uint32_t canIdResponse;         ///< CAN ID for responses (default: 0x18DAFA01)
        std::vector<MemoryBlock> blocks;///< Memory blocks to flash

        ControllerConfig(const std::string& var, const std::string& id, TPCANHandle ch);
    };

    /**
     * @class AES128Security
     * @brief AES-128 security access key generation
     *
     * Implements seed/key challenge-response for UDS security access.
     */
    class AES128Security {
    public:
        /**
         * @brief Calculate security access key from seed using AES-128-ECB
         * @param seed 16-byte seed from ECU
         * @param password Password bytes for AES key derivation
         * @return 16-byte security access key
         */
        static std::vector<uint8_t> calculateKeyFromSeed(
            const std::vector<uint8_t>& seed,
            const std::vector<uint8_t>& password);

        /**
         * @brief Test AES implementation with known values
         * @return true if AES implementation works correctly
         */
        static bool testAES();
    };

    /**
     * @class RC40FlasherDevice
     * @brief Low-level ECU flashing device controller
     *
     * Handles CAN communication and UDS protocol for a single ECU.
     */
    class RC40FlasherDevice {
    private:
        ControllerConfig config;    ///< Configuration for this device
        bool canInitialized;        ///< CAN initialization status

    public:
        /**
         * @brief Construct flasher device with configuration
         * @param cfg Controller configuration
         */
        RC40FlasherDevice(const ControllerConfig& cfg);
        ~RC40FlasherDevice();

        /**
         * @brief Initialize CAN bus communication
         * @return true if CAN initialized successfully
         */
        bool initialize();

        /**
         * @brief Clean up and close CAN channel
         */
        void cleanup();

        /**
         * @brief Get controller identifier string
         * @return Controller ID for logging
         */
        const std::string& getControllerId() const;

        // ===== UDS Service Methods =====

        /**
         * @brief Send functional (broadcast) CAN command
         * @param command UDS command bytes
         * @return true if sent successfully
         */
        bool sendFunctionalCommand(const std::vector<uint8_t>& command);

        /**
         * @brief Send UDS request with multi-frame support (ISO-TP)
         * @param request UDS request bytes
         * @param timeoutMs Response timeout in milliseconds
         * @return Response bytes from ECU
         */
        std::vector<uint8_t> sendUDSRequestWithMultiFrame(
            const std::vector<uint8_t>& request, int timeoutMs);

        /**
         * @brief Send single-frame UDS request
         * @param request UDS request bytes (max 7 bytes)
         * @param timeoutMs Response timeout in milliseconds
         * @return Response bytes from ECU
         */
        std::vector<uint8_t> sendUDSRequest(
            const std::vector<uint8_t>& request, int timeoutMs = 1000);

        /**
         * @brief UDS Service 0x10: Diagnostic Session Control
         * @param sessionType Session type (0x01=default, 0x02=programming, 0x03=extended)
         * @return true if session changed successfully
         */
        bool diagnosticSessionControl(uint8_t sessionType);

        /**
         * @brief UDS Service 0x27 (0x01): Request security seed
         * @return 16-byte seed from ECU
         */
        std::vector<uint8_t> securityAccessRequestSeed();

        /**
         * @brief UDS Service 0x27 (0x02): Send security key
         * @param key 16-byte calculated key
         * @return true if security access granted
         */
        bool securityAccessSendKey(const std::vector<uint8_t>& key);

        /**
         * @brief UDS Service 0x31: Erase flash memory area
         * @param areaId Memory area identifier (0x00=CB, 0x01=ASW0, 0x02=DS0)
         * @return true if erase successful
         */
        bool eraseMemory(uint8_t areaId);

        /**
         * @brief UDS Service 0x34: Request download session
         * @param address Starting address for download
         * @param size Number of bytes to download
         * @return Maximum block size for transfer, or 0 on failure
         */
        uint16_t requestDownload(uint32_t address, uint32_t size);

        /**
         * @brief UDS Service 0x36: Transfer data blocks
         * @param data Firmware data to transfer
         * @param maxBlockSize Maximum bytes per block
         * @return true if all blocks transferred successfully
         */
        bool transferData(const std::vector<uint8_t>& data, uint16_t maxBlockSize);

        /**
         * @brief UDS Service 0x36: Transfer data with progress callback (sparse regions)
         *
         * Optimized transfer that skips 0xFF padding regions
         *
         * @param areaId Memory area identifier
         * @param firmwareData Complete firmware data
         * @param baseAddress Base address for firmware
         * @param progressCallback Optional callback for progress updates (0.0 to 1.0)
         * @return true if transfer successful
         */
        bool transferDataSparse(
            uint8_t areaId,
            const std::vector<uint8_t>& firmwareData,
            uint32_t baseAddress,
            std::function<void(float)> progressCallback = nullptr);

        /**
         * @brief UDS Service 0x37: Exit transfer session
         * @return true if exit successful
         */
        bool requestTransferExit();

        /**
         * @brief UDS Service 0x31: Verify programmed memory
         * @param areaId Memory area to verify
         * @return true if memory verification passed
         */
        bool checkMemory(uint8_t areaId);

        /**
         * @brief UDS Service 0x11: Reset ECU
         * @return true if reset command acknowledged
         */
        bool ecuReset();

        /**
         * @brief Flash controller with firmware files (UNUSED - kept for reference)
         * @param firmwareFiles Map of block names to file paths
         * @param password Security access password
         * @return true if flashing successful
         */
        bool flashController(
            const std::map<std::string, std::string>& firmwareFiles,
            const std::vector<uint8_t>& password);
    };

    // ===== Helper Functions =====

    /**
     * @brief Check if ECU responds on specified CAN channel
     * @param channel PCAN channel to test
     * @return true if ECU responds to tester present
     */
    bool pingECU(TPCANHandle channel);

    /**
     * @brief Parse Intel HEX file into binary firmware data
     * @param filename Path to .hex file
     * @return Binary firmware data with gaps filled with 0xFF
     */
    std::vector<uint8_t> parseIntelHexFile(const std::string& filename);

    /**
     * @brief Create controller configurations for multiple ECUs (UNUSED)
     * @param variant Controller variant name
     * @param controllerIds List of controller IDs
     * @return Vector of controller configurations
     */
    std::vector<ControllerConfig> createControllerConfigs(
        const std::string& variant,
        const std::vector<std::string>& controllerIds);

} // namespace RC40Flasher
