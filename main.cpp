// Dear ImGui: standalone example application for DirectX 11

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder).
// - Introduction, links and more at the top of imgui.cpp

#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"
#include <d3d11.h>
#include <tchar.h>
#include <math.h>
#include <fstream>
#include "kalman.h"
#include "FABRIK2D.h"
#include "fsc_iks.h"
//#include "fsciks_dan.h"
#include "sudoku.h" 
#include "motionController_2D.h"
#include "joystickHandling.h"

#include "RC40Flasher.h"
#include "ProductionFlasher.h"
#include <iostream>

//	ADD "OPEN/SAVE" NATIVE-WINDOWS DIALOG POPUPS
#include "nfd.h"			//	https://github.com/mlabbe/nativefiledialog
//	ADD JSON SUPPORT 
#include <json/json.h>			//	https://github.com/open-source-parsers/jsoncpp

// START - 3D PROJECTION
#include "ImGuizmo.h" // 3D projection

#define uint64 uint64_t
#define sint64 int64_t
#define uint32 uint32_t
#define sint32 int32_t
#define uint16 uint16_t
#define sint16 int16_t
#define uint8 uint8_t
#define sint8 int8_t

// --- FILE OPERATION STUFF ---
const size_t MAXIMUM_PATH_SIZE = 1024;	//	TYPICAL WINDOWS MAX IS 260 CHARS, CAN BE EXTENDED TO 32767 CHARS IN SPECIAL CASES...BUT I DON'T THINK THAT'S GOING TO BE AN ISSUE
std::string savePathStr = ""; // global cause we're going to reference it in a more general scope

bool useWindow = true;
int gizmoCount = 1;
static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::TRANSLATE);

Kalman kalman;

Sudoku sudoku;

TimersAndCalculations TaC;

const int NUM_JOINTS = 3;
int lengths[NUM_JOINTS] = { 39, 36, 18 };
Fabrik2D::AngularConstraint angularConstraints[NUM_JOINTS];
static Fabrik2D fabrik2D(NUM_JOINTS + 1, lengths, 1);

Fsciks fsciks;

float camDistance = 8.f;


float objectMatrix[4][16] = {
  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
  0.f, 1.f, 0.f, 0.f,
  0.f, 0.f, 1.f, 0.f,
  2.f, 0.f, 0.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
  0.f, 1.f, 0.f, 0.f,
  0.f, 0.f, 1.f, 0.f,
  2.f, 0.f, 2.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
  0.f, 1.f, 0.f, 0.f,
  0.f, 0.f, 1.f, 0.f,
  0.f, 0.f, 2.f, 1.f }
};

static const float identityMatrix[16] =
{ 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f };


const int MAX_NUM_SPNS = 32;
const int BITS_PER_BYTE = 8;

typedef enum
{
    TYPE_INT,
    TYPE_FLOAT,
    NUM_VAR_TYPES // Special value to represent the total number of DTC codes
} var_type;

typedef struct
{
    uint32_t spnNum;
    uint8_t byte; // we start counting this at 1 instead of 0, since non-programmers made the J1939 standard...
    uint8_t bit;  // we start counting this at 1 instead of 0, since non-programmers made the J1939 standard...
    uint8_t len;
    float scaling;
    int32_t offset;
    var_type varType;
} spn_info;

typedef struct
{
    uint8_t instanceNum;
    uint16_t boxNum;
    uint32_t pgn;
    uint8_t prio;
    uint8_t src;
    uint16_t timeout;
    uint16_t startTimeout;
    uint32_t lenMax;
    uint8_t data[8];
    spn_info spns[MAX_NUM_SPNS];
} can_isobus_info;

// Motion profile structure
typedef struct {
    float max_velocity;
    float max_acceleration;
    float max_jerk;
    float current_value;
    float target_value;
    float velocity;
    float acceleration;
    uint32_t last_update;
} MotionProfile;

void update_motion(MotionProfile* profile) {
    uint32_t now = millis();
    float dt = (now - profile->last_update) / 1000.0f; // Convert ms to seconds
    profile->last_update = now;

    float error = profile->target_value - profile->current_value;
    if (fabs(error) < 2 && fabs(profile->velocity) < 1) return; // Close enough

    // Adjust acceleration dynamically based on remaining distance
    float decel_distance = (profile->velocity * profile->velocity) / (2 * profile->max_acceleration);
    if (fabs(error) < decel_distance) {
        profile->acceleration = -profile->max_acceleration * (profile->velocity > 0 ? 1 : -1);
    }
    else {
        profile->acceleration = profile->max_acceleration * (error > 0 ? 1 : -1);
    }

    profile->velocity += profile->acceleration * dt;
    if (fabs(profile->velocity) > profile->max_velocity)
        profile->velocity = profile->max_velocity * (profile->velocity > 0 ? 1 : -1);

    profile->current_value += profile->velocity * dt;
}
/**
 * @brief Extracts a value of type uint16 at a given location and length from an array of uint8's, returns success
 *
 * @param[in] telegram[] array of bytes from CAN Telegram
 * @param[in] sizeOfTelegram size of array (number of elements)
 * @param[inout] output* extracted value
 * @param[in] spnConfig struct of type SPN_Config, location and length of requested value in array
 * @return success status of block.
 */

 //int ExtractValueFromCanTelegram(can_isobus_info messageData, int spnInfoIndex, uint64_t* output)
 //{
 //    uint8_t byteIndex = messageData.spns[spnInfoIndex].byte - 1;                                                              // These values start from 1, not 0
 //    uint8_t bitIndex = messageData.spns[spnInfoIndex].bit - 1;                                                                // These values start from 1, not 0
 //    if ((BITS_PER_BYTE * byteIndex + bitIndex + messageData.spns[spnInfoIndex].len) > (messageData.lenMax * BITS_PER_BYTE))   // Check if we are asking for something outside of telegram's allocation
 //        return -1;                                                                                                            // Return FSC_ERR if we are going to overrun the array
 //
 //    uint64_t mask = 0;
 //    uint8_t i = 0;
 //    for (i; i < messageData.spns[spnInfoIndex].len; i++)
 //    {
 //        mask |= (1 << i);
 //    }
 //    uint64_t val = 0;
 //    uint8_t numBytes = 1 + (bitIndex + messageData.spns[spnInfoIndex].len - 1) / BITS_PER_BYTE; // How many bytes does this information span?
 //    i = 0;
 //    for (i; i < numBytes; i++)
 //    {
 //        val |= messageData.data[byteIndex + i] << (BITS_PER_BYTE * i);
 //    }
 //    if (numBytes == 1) // If we are contained in one single byte, little-endianess makes things a little stupid, so read backwards
 //    {
 //        int8_t byteMaskShift = 8 - (bitIndex + messageData.spns[spnInfoIndex].len);
 //        *output = (val >> byteMaskShift) & mask;
 //    }
 //    else
 //    {
 //        *output = (val >> bitIndex) & mask;
 //    }
 //    return 0;
 //}




void resetPCANChannel() {
    std::cout << "Resetting PCAN channel..." << std::endl;

    // Force uninitialize all possible channels
    for (int i = PCAN_USBBUS1; i <= PCAN_USBBUS8; i++) {
        CAN_Uninitialize(static_cast<TPCANHandle>(i));
    }

    // Brief delay for driver reset
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void detectAvailablePCANChannels() {
    std::cout << "=== Detecting Available PCAN Channels ===" << std::endl;

    // PCAN-USB X6 channels (typically PCAN_USBBUS16 to PCAN_USBBUS21)
    std::vector<std::pair<TPCANHandle, std::string>> channels = {
        {PCAN_USBBUS1, "PCAN_USBBUS1"},
        {PCAN_USBBUS2, "PCAN_USBBUS2"},
        {PCAN_USBBUS3, "PCAN_USBBUS3"},
        {PCAN_USBBUS4, "PCAN_USBBUS4"},
        {PCAN_USBBUS5, "PCAN_USBBUS5"},
        {PCAN_USBBUS6, "PCAN_USBBUS6"},
        {PCAN_USBBUS7, "PCAN_USBBUS7"},
        {PCAN_USBBUS8, "PCAN_USBBUS8"},
        {PCAN_USBBUS9, "PCAN_USBBUS9"},
        {PCAN_USBBUS10, "PCAN_USBBUS10"},
        {PCAN_USBBUS11, "PCAN_USBBUS11"},
        {PCAN_USBBUS12, "PCAN_USBBUS12"},
        {PCAN_USBBUS13, "PCAN_USBBUS13"},
        {PCAN_USBBUS14, "PCAN_USBBUS14"},
        {PCAN_USBBUS15, "PCAN_USBBUS15"},
        {PCAN_USBBUS16, "PCAN_USBBUS16"}
    };

    std::vector<TPCANHandle> availableChannels;

    for (const auto& channel : channels) {
        TPCANStatus result = CAN_Initialize(channel.first, PCAN_BAUD_250K, 0, 0, 0);

        if (result == PCAN_ERROR_OK) {
            std::cout << channel.second << ": AVAILABLE" << std::endl;
            availableChannels.push_back(channel.first);
            CAN_Uninitialize(channel.first);  // Clean up
        }
        else if (result == PCAN_ERROR_CAUTION) {
            std::cout << channel.second << ": AVAILABLE (with warnings)" << std::endl;
            availableChannels.push_back(channel.first);
            CAN_Uninitialize(channel.first);
        }
        else {
            std::cout << channel.second << ": NOT AVAILABLE (Error: 0x"
                << std::hex << result << std::dec << ")" << std::endl;
        }
    }

    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Found " << availableChannels.size() << " available channels:" << std::endl;

    for (size_t i = 0; i < availableChannels.size(); ++i) {
        std::cout << "Channel " << (i + 1) << ": ";
        for (const auto& channel : channels) {
            if (channel.first == availableChannels[i]) {
                std::cout << channel.second << std::endl;
                break;
            }
        }
    }

    if (availableChannels.size() >= 6) {
        std::cout << "\nRecommended channels for 6 ECUs:" << std::endl;
        for (int i = 0; i < 6; ++i) {
            for (const auto& channel : channels) {
                if (channel.first == availableChannels[i]) {
                    std::cout << "ECU " << (i + 1) << ": " << channel.second << std::endl;
                    break;
                }
            }
        }
    }
}

void testPCANX6Configuration() {
    std::cout << "Testing PCAN X6 multi-channel configuration..." << std::endl;

    // Test channels typically used by PCAN-USB X6
    std::vector<TPCANHandle> testChannels = {
        PCAN_USBBUS1, PCAN_USBBUS2, PCAN_USBBUS3,
        PCAN_USBBUS4, PCAN_USBBUS5, PCAN_USBBUS6
    };

    std::vector<TPCANHandle> workingChannels;

    for (size_t i = 0; i < testChannels.size(); ++i) {
        TPCANStatus result = CAN_Initialize(testChannels[i], PCAN_BAUD_250K, 0, 0, 0);
        if (result == PCAN_ERROR_OK || result == PCAN_ERROR_CAUTION) {
            workingChannels.push_back(testChannels[i]);
            std::cout << "PCAN_USBBUS" << (16 + i) << ": OK" << std::endl;
            CAN_Uninitialize(testChannels[i]);
        }
        else {
            std::cout << "PCAN_USBBUS" << (16 + i) << ": Failed (0x"
                << std::hex << result << std::dec << ")" << std::endl;
        }
    }

    std::cout << "Working channels: " << workingChannels.size() << "/6" << std::endl;

    if (workingChannels.size() < 6) {
        std::cout << "Note: PCAN-USB X6 channels may start at different numbers." << std::endl;
        std::cout << "Run detectAvailablePCANChannels() to find actual channel numbers." << std::endl;
    }
}

void flashASW0AndDS0Split() {
    std::cout << "=== Flashing ASW0+DS0 Split (Official Tool Style) ===" << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_SPLIT_FLASH", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);
    if (!flasher.initialize()) return;

    // Load and split firmware data
    std::string hexFile = "firmware/rc5_6_40_asw0.hex";
    std::vector<uint8_t> firmwareData;

    try {
        firmwareData = RC40Flasher::parseIntelHexFile(hexFile);
        std::cout << "Total firmware loaded: " << firmwareData.size() << " bytes" << std::endl;
    }
    catch (const std::exception& e) {
        std::cout << "Cannot load firmware file: " << e.what() << std::endl;
        return;
    }

    try {
        // Send tester present first (like official tool)
        std::vector<uint8_t> testerPresent = { 0x3E, 0x00 };
        flasher.sendUDSRequest(testerPresent, 1000);

        // Use official tool sequence (no functional prep)
        if (!flasher.diagnosticSessionControl(0x02)) {
            std::cout << "Programming mode failed" << std::endl;
            return;
        }

        // Security access
        auto seed = flasher.securityAccessRequestSeed();
        std::string passwordStr = "DEF_PASSWORD_021";
        std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());
        auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

        if (!flasher.securityAccessSendKey(key)) {
            std::cout << "Security access failed" << std::endl;
            return;
        }


        // After security access success, add:
        std::cout << "Writing fingerprint data..." << std::endl;

        // Programming date (DDMMYYYY format)
        std::vector<uint8_t> dateCmd = { 0x2E, 0xF1, 0x99 };
        std::string progDate = "22092025"; // Today's date
        dateCmd.insert(dateCmd.end(), progDate.begin(), progDate.end());

        auto dateResp = flasher.sendUDSRequestWithMultiFrame(dateCmd, 2000);
        if (dateResp.size() >= 2 && dateResp[0] == 0x6E) {
            std::cout << "Fingerprint date: SUCCESS" << std::endl;
        }
        else {
            std::cout << "Fingerprint date: FAILED" << std::endl;
            return;
        }

        // Split at ASW0/DS0 boundary
        uint32_t asw0Start = 0x09020000;
        uint32_t asw0End = 0x093BFFFF;
        uint32_t ds0Start = 0x093C0000;

        uint32_t asw0Size = asw0End - asw0Start + 1;  // 0x3A0000 bytes
        uint32_t ds0Offset = ds0Start - asw0Start;    // Offset in firmware data

        // Extract ASW0 portion
        std::vector<uint8_t> asw0Data;
        if (firmwareData.size() >= asw0Size) {
            asw0Data = std::vector<uint8_t>(firmwareData.begin(), firmwareData.begin() + asw0Size);
        }
        else {
            asw0Data = firmwareData; // Use all data if smaller than expected
        }

        // Extract DS0 portion if it exists
        std::vector<uint8_t> ds0Data;
        if (firmwareData.size() > ds0Offset) {
            ds0Data = std::vector<uint8_t>(firmwareData.begin() + ds0Offset, firmwareData.end());
        }

        // After splitting the data:
        std::cout << "ASW0 data: " << asw0Data.size() << " bytes" << std::endl;

        // Pad ASW0 to full expected size (0x3A0000 = 3866624 bytes)
        const uint32_t expectedASW0Size = 0x3A0000;
        if (asw0Data.size() < expectedASW0Size) {
            std::cout << "Padding ASW0 from " << asw0Data.size() << " to " << expectedASW0Size << " bytes" << std::endl;
            asw0Data.resize(expectedASW0Size, 0xFF); // Pad with 0xFF (common for flash)
        }

        std::cout << "ASW0 data: " << asw0Data.size() << " bytes" << std::endl;
        std::cout << "DS0 data: " << ds0Data.size() << " bytes" << std::endl;

        // Flash ASW0 first (like official tool)
        std::cout << "\n=== FLASHING ASW0 ===" << std::endl;

        std::cout << "Erasing ASW0..." << std::endl;
        if (!flasher.eraseMemory(0x01)) {
            std::cout << "ASW0 erase failed" << std::endl;
            return;
        }

        //std::cout << "Request download ASW0..." << std::endl;
        //uint16_t maxBlockSize = flasher.requestDownload(asw0Start, (uint32_t)asw0Data.size());
        //if (maxBlockSize == 0) {
        //    std::cout << "ASW0 request download failed" << std::endl;
        //    return;
        //}

        //std::cout << "Transferring ASW0 data..." << std::endl;
        //if (!flasher.transferData(asw0Data, maxBlockSize)) {
        //    std::cout << "ASW0 transfer failed" << std::endl;
        //    return;
        //}

        //std::cout << "ASW0 transfer exit..." << std::endl;
        //if (!flasher.requestTransferExit()) {
        //    std::cout << "ASW0 transfer exit failed" << std::endl;
        //    return;
        //}

        std::cout << "Transferring ASW0 data (sparse)..." << std::endl;
        if (!flasher.transferDataSparse(0x01, asw0Data, asw0Start)) {
            std::cout << "ASW0 transfer failed" << std::endl;
            return;
        }

        std::cout << "ASW0 memory check..." << std::endl;
        if (!flasher.checkMemory(0x01)) {
            std::cout << "ASW0 memory check failed" << std::endl;
            return;
        }

        std::cout << "ASW0 flashing complete!" << std::endl;

        // Flash DS0 if data exists
        if (!ds0Data.empty()) {
            std::cout << "\n=== FLASHING DS0 ===" << std::endl;

            std::cout << "Erasing DS0..." << std::endl;
            if (!flasher.eraseMemory(0x02)) {
                std::cout << "DS0 erase failed" << std::endl;
                return;
            }

            //std::cout << "Request download DS0..." << std::endl;
            //maxBlockSize = flasher.requestDownload(ds0Start, (uint32_t)ds0Data.size());
            //if (maxBlockSize == 0) {
            //    std::cout << "DS0 request download failed" << std::endl;
            //    return;
            //}

            //std::cout << "Transferring DS0 data..." << std::endl;
            //if (!flasher.transferData(ds0Data, maxBlockSize)) {
            //    std::cout << "DS0 transfer failed" << std::endl;
            //    return;
            //}

            //std::cout << "DS0 transfer exit..." << std::endl;
            //if (!flasher.requestTransferExit()) {
            //    std::cout << "DS0 transfer exit failed" << std::endl;
            //    return;
            //}

            std::cout << "Transferring DS0 data (sparse)..." << std::endl;
            if (!flasher.transferDataSparse(0x02, ds0Data, ds0Start)) {
                std::cout << "DS0 transfer failed" << std::endl;
                return;
            }

            std::cout << "DS0 memory check..." << std::endl;
            if (!flasher.checkMemory(0x02)) {
                std::cout << "DS0 memory check failed" << std::endl;
                return;
            }

            std::cout << "DS0 flashing complete!" << std::endl;
        }
        else {
            std::cout << "No DS0 data found - skipping DS0 flash" << std::endl;
        }

        // Reset ECU
        std::cout << "\n=== FINALIZING ===" << std::endl;
        std::cout << "Resetting ECU..." << std::endl;
        flasher.ecuReset();

        std::cout << "\n*** FLASHING COMPLETE! ***" << std::endl;
        std::cout << "ASW0: " << asw0Data.size() << " bytes" << std::endl;
        if (!ds0Data.empty()) {
            std::cout << "DS0:  " << ds0Data.size() << " bytes" << std::endl;
        }

    }
    catch (const std::exception& e) {
        std::cout << "Flash error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void testOfficialToolSequence() {
    std::cout << "=== Testing Official Tool Sequence ===" << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_OFFICIAL", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;  // Confirmed from trace
    testConfig.canIdResponse = 0x18DAFA01; // Confirmed from trace

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) return;

    try {
        // Step 1: Send tester present (like official tool)
        std::cout << "Step 1: Tester present..." << std::endl;
        std::vector<uint8_t> testerPresent = { 0x3E, 0x00 };
        auto tpResponse = flasher.sendUDSRequest(testerPresent, 1000);

        if (tpResponse.size() >= 2 && tpResponse[0] == 0x7E) {
            std::cout << "Tester present: SUCCESS" << std::endl;
        }
        else {
            std::cout << "Tester present: FAILED" << std::endl;
            return;
        }

        // Step 2: Programming mode directly (no functional prep)
        std::cout << "Step 2: Programming mode..." << std::endl;
        if (!flasher.diagnosticSessionControl(0x02)) {
            std::cout << "Programming mode: FAILED" << std::endl;
            return;
        }
        std::cout << "Programming mode: SUCCESS" << std::endl;

        // Step 3: Security access
        std::cout << "Step 3: Security access..." << std::endl;
        auto seed = flasher.securityAccessRequestSeed();
        if (seed.empty()) {
            std::cout << "Security seed: FAILED" << std::endl;
            return;
        }
        std::cout << "Security seed: SUCCESS (" << seed.size() << " bytes)" << std::endl;

        std::string passwordStr = "DEF_PASSWORD_021";
        std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());
        auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

        if (!flasher.securityAccessSendKey(key)) {
            std::cout << "Security access: FAILED" << std::endl;
            return;
        }
        std::cout << "Security access: SUCCESS" << std::endl;

        std::cout << "*** READY FOR FLASHING ***" << std::endl;

        // Load firmware file
        std::string hexFile = "firmware/rc5_6_40_asw0.hex";
        std::vector<uint8_t> firmwareData;

        try {
            firmwareData = RC40Flasher::parseIntelHexFile(hexFile);
            std::cout << "Loaded ASW0 firmware: " << firmwareData.size() << " bytes" << std::endl;
        }
        catch (const std::exception& e) {
            std::cout << "Cannot load firmware file: " << e.what() << std::endl;
            return;
        }

        // Erase ASW0 only
        std::cout << "Step 1: Erasing ASW0..." << std::endl;
        if (!flasher.eraseMemory(0x01)) {
            std::cout << "ASW0 erase failed" << std::endl;
            return;
        }

        // Request download for ASW0
        std::cout << "Step 3: Request download for ASW0..." << std::endl;
        uint32_t asw0Address = 0x09020000;
        uint16_t maxBlockSize = flasher.requestDownload(asw0Address, (uint32_t)firmwareData.size());
        if (maxBlockSize == 0) return;

        // Transfer data
        std::cout << "Step 3: Transferring ASW0 data..." << std::endl;
        if (!flasher.transferData(firmwareData, maxBlockSize)) return;

        // Transfer exit
        std::cout << "Step 4: Transfer exit..." << std::endl;
        if (!flasher.requestTransferExit()) return;

        // Memory check
        std::cout << "Step 5: Memory verification..." << std::endl;
        if (!flasher.checkMemory(0x01)) return;

        std::cout << "=== ASW0 FLASHING COMPLETE ===" << std::endl;

    }
    catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void findWorkingPreparationSequence() {
    std::cout << "=== Finding Working Preparation Sequence ===" << std::endl;
    std::cout << "Testing different approaches to match official Rexroth tool behavior..." << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_PREP_TEST", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) {
        std::cout << "CAN initialization failed" << std::endl;
        return;
    }

    // Test different preparation approaches
    std::vector<std::string> approaches = {
        "No preparation - direct programming",
        "Minimal functional prep",
        "Full functional prep sequence",
        "Extended session first",
        "Different timing"
    };

    for (size_t i = 0; i < approaches.size(); ++i) {
        std::cout << "\n--- Approach " << (i + 1) << ": " << approaches[i] << " ---" << std::endl;

        try {
            bool success = false;

            switch (i) {
            case 0: // No preparation
                success = flasher.diagnosticSessionControl(0x02);
                break;

            case 1: // Minimal functional prep
                flasher.sendFunctionalCommand({ 0x10, 0x83 });
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                success = flasher.diagnosticSessionControl(0x02);
                break;

            case 2: // Full functional prep (your current approach)
                flasher.sendFunctionalCommand({ 0x10, 0x83 });
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                flasher.sendFunctionalCommand({ 0x85, 0x82 });
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                success = flasher.diagnosticSessionControl(0x02);
                break;

            case 3: // Extended session first
                if (flasher.diagnosticSessionControl(0x03)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    success = flasher.diagnosticSessionControl(0x02);
                }
                break;

            case 4: // Different timing
                flasher.sendFunctionalCommand({ 0x10, 0x83 });
                std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Longer delay
                flasher.sendFunctionalCommand({ 0x85, 0x82 });
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                success = flasher.diagnosticSessionControl(0x02);
                break;
            }

            if (success) {
                std::cout << "PROGRAMMING MODE: SUCCESS" << std::endl;

                // Test security access
                auto seed = flasher.securityAccessRequestSeed();
                if (!seed.empty()) {
                    std::cout << "SECURITY SEED: SUCCESS (" << seed.size() << " bytes)" << std::endl;

                    // Test with actual password
                    std::string passwordStr = "DEF_PASSWORD_021"; // Replace with real password
                    std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());
                    auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

                    if (flasher.securityAccessSendKey(key)) {
                        std::cout << "SECURITY ACCESS: SUCCESS" << std::endl;
                        std::cout << "*** WORKING SEQUENCE FOUND! ***" << std::endl;
                        std::cout << "Use approach: " << approaches[i] << std::endl;

                        flasher.cleanup();
                        return;
                    }
                    else {
                        std::cout << "SECURITY ACCESS: FAILED (wrong password?)" << std::endl;
                    }
                }
                else {
                    std::cout << "SECURITY SEED: FAILED" << std::endl;
                }
            }
            else {
                std::cout << "PROGRAMMING MODE: FAILED" << std::endl;
            }

        }
        catch (const std::exception& e) {
            std::cout << "EXCEPTION: " << e.what() << std::endl;
        }

        // Reset for next attempt
        flasher.cleanup();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!flasher.initialize()) {
            std::cout << "Failed to reinitialize for next test" << std::endl;
            break;
        }
    }

    std::cout << "\nNo working sequence found. Check password or CAN settings." << std::endl;
    flasher.cleanup();
}

void diagnoseECUState() {
    std::cout << "=== ECU State Diagnosis ===" << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_STATE_CHECK", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) {
        std::cout << "CAN initialization failed" << std::endl;
        return;
    }

    // Test 1: Check if ECU responds to default session
    std::cout << "Test 1: Default diagnostic session..." << std::endl;
    try {
        bool defaultSession = flasher.diagnosticSessionControl(0x01);
        std::cout << "Default session: " << (defaultSession ? "SUCCESS" : "FAILED") << std::endl;

        if (defaultSession) {
            // Test 2: Try extended diagnostic session
            std::cout << "Test 2: Extended diagnostic session..." << std::endl;
            bool extendedSession = flasher.diagnosticSessionControl(0x03);
            std::cout << "Extended session: " << (extendedSession ? "SUCCESS" : "FAILED") << std::endl;

            // Test 3: Try programming session without preparation
            std::cout << "Test 3: Direct programming session..." << std::endl;
            bool directProgramming = flasher.diagnosticSessionControl(0x02);
            std::cout << "Direct programming: " << (directProgramming ? "SUCCESS" : "FAILED") << std::endl;

            if (!directProgramming) {
                // Test 4: Try with minimal preparation
                std::cout << "Test 4: Programming with functional preparation..." << std::endl;

                // Just extended diagnostic functional
                flasher.sendFunctionalCommand({ 0x10, 0x83 });
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                bool preparedProgramming = flasher.diagnosticSessionControl(0x02);
                std::cout << "Prepared programming: " << (preparedProgramming ? "SUCCESS" : "FAILED") << std::endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cout << "Session test error: " << e.what() << std::endl;
    }

    // Test 5: Raw CAN communication test
    std::cout << "Test 5: Raw CAN communication check..." << std::endl;

    TPCANMsg msg;
    msg.ID = testConfig.canIdRequest;
    msg.LEN = 3;
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msg.DATA[0] = 0x02;  // Length
    msg.DATA[1] = 0x3E;  // Tester Present
    msg.DATA[2] = 0x00;  // Sub-function

    TPCANStatus writeResult = CAN_Write(testConfig.canChannel, &msg);
    std::cout << "Tester Present write: " << (writeResult == PCAN_ERROR_OK ? "SUCCESS" : "FAILED") << std::endl;

    if (writeResult == PCAN_ERROR_OK) {
        // Wait for response
        bool gotResponse = false;
        for (int attempts = 0; attempts < 200; attempts++) {
            TPCANMsg responseMsg;
            TPCANTimestamp timestamp;
            TPCANStatus readResult = CAN_Read(testConfig.canChannel, &responseMsg, &timestamp);

            if (readResult == PCAN_ERROR_OK && responseMsg.ID == testConfig.canIdResponse) {
                std::cout << "Tester Present response: ";
                for (int i = 0; i < responseMsg.LEN; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)responseMsg.DATA[i] << " ";
                }
                std::cout << std::dec << std::endl;
                gotResponse = true;

                // Check if it's a positive response to tester present (7E 00)
                if (responseMsg.LEN >= 2 && responseMsg.DATA[1] == 0x7E) {
                    std::cout << "ECU is responding normally to tester present" << std::endl;
                }
                else if (responseMsg.LEN >= 2 && responseMsg.DATA[1] == 0x7F) {
                    std::cout << "ECU sent negative response - Error code: 0x" << std::hex << (int)responseMsg.DATA[3] << std::dec << std::endl;
                }
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!gotResponse) {
            std::cout << "No response to Tester Present - ECU may not be communicating" << std::endl;
        }
    }

    // Test 6: Listen for any CAN traffic
    std::cout << "Test 6: Listening for any CAN traffic (5 seconds)..." << std::endl;
    auto startTime = std::chrono::steady_clock::now();
    int messageCount = 0;

    while (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - startTime).count() < 5) {

        TPCANMsg anyMsg;
        TPCANTimestamp timestamp;
        TPCANStatus readResult = CAN_Read(testConfig.canChannel, &anyMsg, &timestamp);

        if (readResult == PCAN_ERROR_OK) {
            messageCount++;
            std::cout << "CAN ID: 0x" << std::hex << std::setw(8) << std::setfill('0') << anyMsg.ID << " Data: ";
            for (int i = 0; i < anyMsg.LEN; ++i) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)anyMsg.DATA[i] << " ";
            }
            std::cout << std::dec << std::endl;

            if (messageCount >= 10) {
                std::cout << "Stopping after 10 messages..." << std::endl;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (messageCount == 0) {
        std::cout << "No CAN traffic detected - possible communication issue" << std::endl;
    }
    else {
        std::cout << "Detected " << messageCount << " CAN messages" << std::endl;
    }

    flasher.cleanup();

    std::cout << "\n=== DIAGNOSIS SUMMARY ===" << std::endl;
    std::cout << "If ECU responds to default session but not programming session," << std::endl;
    std::cout << "the CB flash may have changed the ECU's state requirements." << std::endl;
    std::cout << "Check if ECU needs power cycle or different preparation sequence." << std::endl;
}

void testASW0AfterECUPowerCycle() {
    std::cout << "\n=== Testing ASW0 After ECU State Check ===" << std::endl;
    std::cout << "*** IMPORTANT: Power cycle the ECU before running this test ***" << std::endl;
    std::cout << "Press Enter when ECU has been power cycled...";
    std::cin.get();

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_ASW0_AFTER_RESET", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) return;

    try {
        // Try the complete sequence fresh
        std::cout << "Step 1: Extended diagnostic session..." << std::endl;
        if (!flasher.diagnosticSessionControl(0x03)) {
            std::cout << "Extended session failed" << std::endl;
            return;
        }

        std::cout << "Step 2: Programming session..." << std::endl;
        if (!flasher.diagnosticSessionControl(0x02)) {
            std::cout << "Programming session failed even after power cycle" << std::endl;

            // Try with preparation commands
            std::cout << "Trying with preparation sequence..." << std::endl;
            flasher.sendFunctionalCommand({ 0x10, 0x83 });
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            flasher.sendFunctionalCommand({ 0x85, 0x82 });
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            if (flasher.diagnosticSessionControl(0x02)) {
                std::cout << "Programming session SUCCESS with preparation" << std::endl;

                // Continue with security access
                auto seed = flasher.securityAccessRequestSeed();
                if (!seed.empty()) {
                    std::cout << "Security seed obtained successfully" << std::endl;
                    std::cout << "ECU communication is working - ready for ASW0 flashing" << std::endl;
                }
            }
            else {
                std::cout << "Programming session still failed - may need different approach" << std::endl;
            }
            return;
        }

        std::cout << "Programming session SUCCESS" << std::endl;
        std::cout << "ECU is ready for ASW0 flashing" << std::endl;

    }
    catch (const std::exception& e) {
        std::cout << "Test error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void debugASW0FlashingIssues() {
    std::cout << "=== Debugging ASW0 Flashing Issues ===" << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_ASW0_DEBUG", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) {
        std::cout << "Failed to initialize CAN" << std::endl;
        return;
    }

    try {
        // Standard preparation sequence
        std::cout << "Step 1: Preparation sequence..." << std::endl;
        flasher.sendFunctionalCommand({ 0x10, 0x83 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x85, 0x82 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Programming mode
        std::cout << "Step 2: Enter programming mode..." << std::endl;
        if (!flasher.diagnosticSessionControl(0x02)) {
            std::cout << "FAILED: Cannot enter programming mode" << std::endl;
            return;
        }

        // Security access
        std::cout << "Step 3: Security access..." << std::endl;
        auto seed = flasher.securityAccessRequestSeed();
        if (seed.empty()) {
            std::cout << "FAILED: Cannot get security seed" << std::endl;
            return;
        }

        std::string passwordStr = "DEF_PASSWORD_021"; // Use your actual password
        std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());
        auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

        if (!flasher.securityAccessSendKey(key)) {
            std::cout << "FAILED: Security access denied" << std::endl;
            return;
        }

        std::cout << "SUCCESS: Security unlocked" << std::endl;

        // Add fingerprint data (as per reference document)
        std::cout << "Step 4: Writing fingerprint data..." << std::endl;
        try {
            // Tester serial number
            std::vector<uint8_t> serialCmd = { 0x2E, 0xF1, 0x98 };
            std::string serialNum = "TEST123456";
            serialCmd.insert(serialCmd.end(), serialNum.begin(), serialNum.end());

            auto serialResp = flasher.sendUDSRequestWithMultiFrame(serialCmd, 2000);
            if (serialResp.size() >= 2 && serialResp[0] == 0x6E) {
                std::cout << "Fingerprint serial: SUCCESS" << std::endl;
            }
            else {
                std::cout << "Fingerprint serial: FAILED" << std::endl;
            }

            // Programming date
            std::vector<uint8_t> dateCmd = { 0x2E, 0xF1, 0x99 };
            std::string progDate = "22092025"; // DDMMYYYY
            dateCmd.insert(dateCmd.end(), progDate.begin(), progDate.end());

            auto dateResp = flasher.sendUDSRequestWithMultiFrame(dateCmd, 2000);
            if (dateResp.size() >= 2 && dateResp[0] == 0x6E) {
                std::cout << "Fingerprint date: SUCCESS" << std::endl;
            }
            else {
                std::cout << "Fingerprint date: FAILED" << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cout << "Fingerprint data failed: " << e.what() << std::endl;
        }

        // Test ASW0 erase with extended timeout
        std::cout << "Step 5: Testing ASW0 erase (this may take longer)..." << std::endl;

        auto eraseStartTime = std::chrono::steady_clock::now();
        bool eraseResult = false;

        try {
            // Send erase command manually with extended timeout
            std::vector<uint8_t> eraseCmd = { 0x31, 0x01, 0xFF, 0x00, 0x01, 0x01 }; // ASW0 = 0x01
            auto eraseResp = flasher.sendUDSRequestWithMultiFrame(eraseCmd, 60000); // 60 second timeout

            if (eraseResp.size() >= 5 && eraseResp[0] == 0x71 && eraseResp[4] == 0x00) {
                eraseResult = true;
            }
        }
        catch (const std::exception& e) {
            std::cout << "ASW0 erase exception: " << e.what() << std::endl;
        }

        auto eraseEndTime = std::chrono::steady_clock::now();
        auto eraseDuration = std::chrono::duration_cast<std::chrono::seconds>(eraseEndTime - eraseStartTime);

        if (eraseResult) {
            std::cout << "ASW0 ERASE: SUCCESS (took " << eraseDuration.count() << " seconds)" << std::endl;

            // Now test request download for ASW0
            std::cout << "Step 6: Testing ASW0 request download..." << std::endl;

            uint32_t asw0Address = 0x09020000;
            uint32_t asw0Size = 0x003A0000; // Full ASW0 size

            uint16_t maxBlockSize = flasher.requestDownload(asw0Address, asw0Size);
            if (maxBlockSize > 0) {
                std::cout << "ASW0 REQUEST DOWNLOAD: SUCCESS" << std::endl;
                std::cout << "Max block size: " << maxBlockSize << " bytes" << std::endl;
                std::cout << "Estimated transfer blocks: " << (asw0Size / (maxBlockSize - 2)) << std::endl;
                std::cout << "Estimated transfer time: ~" << ((asw0Size / (maxBlockSize - 2)) * 50 / 1000) << " seconds" << std::endl;

                // Test with small dummy data to verify transfer mechanism
                std::cout << "Step 7: Testing small data transfer..." << std::endl;
                std::vector<uint8_t> testData(1024, 0xFF); // 1KB of test data

                if (flasher.transferData(testData, maxBlockSize)) {
                    std::cout << "TEST TRANSFER: SUCCESS" << std::endl;

                    if (flasher.requestTransferExit()) {
                        std::cout << "TRANSFER EXIT: SUCCESS" << std::endl;
                        std::cout << "*** ASW0 flash mechanism appears to be working! ***" << std::endl;
                        std::cout << "Issue is likely with actual firmware data or file parsing." << std::endl;
                    }
                }
            }
            else {
                std::cout << "ASW0 REQUEST DOWNLOAD: FAILED" << std::endl;
            }
        }
        else {
            std::cout << "ASW0 ERASE: FAILED (took " << eraseDuration.count() << " seconds)" << std::endl;
            std::cout << "This is likely the root cause of ASW0 flashing issues." << std::endl;
        }

    }
    catch (const std::exception& e) {
        std::cout << "Debug error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

// Enhanced ASW0 flashing function with proper error handling
void flashASW0WithProperSequence() {
    std::cout << "=== Flashing ASW0 with Proper Sequence ===" << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_ASW0_ENHANCED", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) return;

    try {
        // Preparation
        //flasher.sendFunctionalCommand({ 0x10, 0x83 });
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //flasher.sendFunctionalCommand({ 0x85, 0x82 });
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
        //std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (!flasher.diagnosticSessionControl(0x02)) return;

        // Security access
        auto seed = flasher.securityAccessRequestSeed();
        std::string passwordStr = "DEF_PASSWORD_021";
        std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());
        auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

        if (!flasher.securityAccessSendKey(key)) return;

        // Load firmware file
        std::string hexFile = "firmware/rc5_6_40_asw0.hex";
        std::vector<uint8_t> firmwareData;

        try {
            firmwareData = RC40Flasher::parseIntelHexFile(hexFile);
            std::cout << "Loaded ASW0 firmware: " << firmwareData.size() << " bytes" << std::endl;
        }
        catch (const std::exception& e) {
            std::cout << "Cannot load firmware file: " << e.what() << std::endl;
            return;
        }

        // Erase ASW0 only
        std::cout << "Step 1: Erasing ASW0..." << std::endl;
        if (!flasher.eraseMemory(0x01)) {
            std::cout << "ASW0 erase failed" << std::endl;
            return;
        }

        // Request download for ASW0
        std::cout << "Step 3: Request download for ASW0..." << std::endl;
        uint32_t asw0Address = 0x09020000;
        uint16_t maxBlockSize = flasher.requestDownload(asw0Address, (uint32_t)firmwareData.size());
        if (maxBlockSize == 0) return;

        // Transfer data
        std::cout << "Step 3: Transferring ASW0 data..." << std::endl;
        if (!flasher.transferData(firmwareData, maxBlockSize)) return;

        // Transfer exit
        std::cout << "Step 4: Transfer exit..." << std::endl;
        if (!flasher.requestTransferExit()) return;

        // Memory check
        std::cout << "Step 5: Memory verification..." << std::endl;
        if (!flasher.checkMemory(0x01)) return;

        std::cout << "=== ASW0 FLASHING COMPLETE ===" << std::endl;

        // Reset ECU
        flasher.ecuReset();

    }
    catch (const std::exception& e) {
        std::cout << "Enhanced ASW0 flash error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void testCBFirstApproach() {
    std::cout << "Testing CB-First Approach..." << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_CB_FIRST", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) return;

    try {
        // Standard preparation and unlock sequence
        flasher.sendFunctionalCommand({ 0x10, 0x83 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x85, 0x82 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (!flasher.diagnosticSessionControl(0x02)) return;

        auto seed = flasher.securityAccessRequestSeed();
        std::string passwordStr = "DEF_PASSWORD_021";
        std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());
        auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

        if (!flasher.securityAccessSendKey(key)) return;

        std::cout << "=== FLASHING CB FIRST ===" << std::endl;

        // Check if you have a CB hex file
        std::string cbHexFile = "firmware/rc5_6_40_cb.hex";
        std::ifstream cbFile(cbHexFile);
        if (cbFile.is_open()) {
            cbFile.close();

            auto cbData = RC40Flasher::parseIntelHexFile(cbHexFile);
            std::cout << "CB firmware loaded: " << cbData.size() << " bytes" << std::endl;

            // Flash CB
            if (flasher.eraseMemory(0x00)) {
                uint32_t cbAddress = 0x08FD8000; // CB address from reference
                uint16_t maxBlockSize = flasher.requestDownload(cbAddress, (uint32_t)cbData.size());

                if (maxBlockSize > 0) {
                    if (flasher.transferData(cbData, maxBlockSize) &&
                        flasher.requestTransferExit() &&
                        flasher.checkMemory(0x00)) {

                        std::cout << "CB flashed successfully!" << std::endl;
                        std::cout << "Now resetting to let new CB handle ASW0/DS0..." << std::endl;

                        flasher.ecuReset();
                        std::this_thread::sleep_for(std::chrono::seconds(3));

                        // Try ASW0/DS0 erase after CB update
                        // You'd need to restart the whole sequence here
                    }
                }
            }
        }
        else {
            std::cout << "No CB firmware file found at: " << cbHexFile << std::endl;
        }

    }
    catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void testSequentialErase() {
    std::cout << "Testing Sequential Erase (CB -> ASW0 -> DS0)..." << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_SEQ_ERASE", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) return;

    try {
        // Setup and unlock (same as before)
        flasher.sendFunctionalCommand({ 0x10, 0x83 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x85, 0x82 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (!flasher.diagnosticSessionControl(0x02)) return;

        auto seed = flasher.securityAccessRequestSeed();
        if (seed.empty()) return;

        std::string passwordStr = "DEF_PASSWORD_021";
        std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());
        auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

        if (!flasher.securityAccessSendKey(key)) return;

        std::cout << "Successfully unlocked - testing sequential erase..." << std::endl;

        // Test different erase sequences
        std::cout << "\n=== SEQUENCE 1: CB -> ASW0 -> DS0 ===" << std::endl;

        // Step 1: Erase CB (we know this works)
        std::cout << "Step 1: Erasing CB..." << std::endl;
        if (flasher.eraseMemory(0x00)) {
            std::cout << "CB erase: SUCCESS" << std::endl;

            // Step 2: Try ASW0 after CB is erased
            std::cout << "Step 2: Erasing ASW0 (after CB)..." << std::endl;
            if (flasher.eraseMemory(0x01)) {
                std::cout << "ASW0 erase: SUCCESS" << std::endl;

                // Step 3: Try DS0 after ASW0 is erased
                std::cout << "Step 3: Erasing DS0 (after ASW0)..." << std::endl;
                if (flasher.eraseMemory(0x02)) {
                    std::cout << "DS0 erase: SUCCESS" << std::endl;
                    std::cout << "*** ALL AREAS ERASED SUCCESSFULLY! ***" << std::endl;
                }
                else {
                    std::cout << "DS0 erase: FAILED" << std::endl;
                }
            }
            else {
                std::cout << "ASW0 erase: FAILED" << std::endl;
            }
        }

        // Alternative: Try erasing ASW0 and DS0 together
        std::cout << "\n=== ALTERNATIVE: Try Combined Erase Command ===" << std::endl;

        // Reset and unlock again
        flasher.cleanup();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (flasher.initialize()) {
            // Quick re-unlock
            flasher.sendFunctionalCommand({ 0x10, 0x83 });
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            flasher.sendFunctionalCommand({ 0x85, 0x82 });
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if (flasher.diagnosticSessionControl(0x02)) {
                seed = flasher.securityAccessRequestSeed();
                key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);
                flasher.securityAccessSendKey(key);

                // Try different erase command formats
                std::vector<std::vector<uint8_t>> combinedEraseCmds = {
                    {0x31, 0x01, 0xFF, 0x00, 0x02, 0x01, 0x02}, // Erase both ASW0 and DS0
                    {0x31, 0x01, 0xFF, 0x01, 0x01, 0x01},       // Alternative format
                    {0x31, 0x01, 0xFF, 0x00, 0x01, 0xFF},       // Erase all applications
                };

                for (size_t i = 0; i < combinedEraseCmds.size(); ++i) {
                    std::cout << "Trying combined erase format " << (i + 1) << ": ";
                    for (uint8_t b : combinedEraseCmds[i]) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                    }
                    std::cout << std::dec << std::endl;

                    try {
                        auto response = flasher.sendUDSRequestWithMultiFrame(combinedEraseCmds[i], 30000);
                        std::cout << "Response: ";
                        for (uint8_t b : response) {
                            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                        }
                        std::cout << std::dec << std::endl;

                        if (response.size() >= 2 && response[0] == 0x71) {
                            std::cout << "SUCCESS with combined erase format " << (i + 1) << "!" << std::endl;
                            break;
                        }
                    }
                    catch (const std::exception& e) {
                        std::cout << "Format " << (i + 1) << " failed: " << e.what() << std::endl;
                    }
                }
            }
        }

    }
    catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void debugRC40EraseOperation() {
    std::cout << "Debugging RC40 Erase Operation..." << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_DEBUG", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) return;

    try {
        // Preparation and security access
        flasher.sendFunctionalCommand({ 0x10, 0x83 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x85, 0x82 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (flasher.diagnosticSessionControl(0x02)) {
            auto seed = flasher.securityAccessRequestSeed();
            if (!seed.empty()) {
                std::string passwordStr = "DEF_PASSWORD_021";
                std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());
                auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

                if (flasher.securityAccessSendKey(key)) {
                    std::cout << "Successfully unlocked - now testing erase operations..." << std::endl;

                    // Test different area IDs to see which ones work
                    std::vector<std::pair<uint8_t, std::string>> testAreas = {
                        {0x00, "CB (Customer Boot)"},
                        {0x01, "ASW0 (Application Software)"},
                        {0x02, "DS0 (Data Section)"}
                    };

                    for (const auto& [areaId, description] : testAreas) {
                        std::cout << "\nTesting erase for Area " << (int)areaId << " (" << description << ")..." << std::endl;

                        try {
                            // Send erase command manually to get detailed response
                            std::vector<uint8_t> eraseCmd = { 0x31, 0x01, 0xFF, 0x00, 0x01, areaId };
                            auto response = flasher.sendUDSRequestWithMultiFrame(eraseCmd, 30000); // Long timeout

                            std::cout << "Erase response: ";
                            for (uint8_t b : response) {
                                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                            }
                            std::cout << std::dec << std::endl;

                            if (response.size() >= 2) {
                                if (response[0] == 0x71) {
                                    std::cout << "SUCCESS: Area " << (int)areaId << " erased successfully!" << std::endl;
                                }
                                else if (response[0] == 0x7F) {
                                    uint8_t service = response[1];
                                    uint8_t errorCode = response.size() >= 3 ? response[2] : 0x00;
                                    std::cout << "NEGATIVE RESPONSE:" << std::endl;
                                    std::cout << "  Service: 0x" << std::hex << (int)service << std::dec << std::endl;
                                    std::cout << "  Error Code: 0x" << std::hex << (int)errorCode << std::dec << " (";

                                    // Decode common error codes
                                    switch (errorCode) {
                                    case 0x12: std::cout << "subFunctionNotSupported"; break;
                                    case 0x13: std::cout << "incorrectMessageLengthOrInvalidFormat"; break;
                                    case 0x22: std::cout << "conditionsNotCorrect"; break;
                                    case 0x31: std::cout << "requestOutOfRange"; break;
                                    case 0x33: std::cout << "securityAccessDenied"; break;
                                    case 0x72: std::cout << "generalProgrammingFailure"; break;
                                    case 0x78: std::cout << "requestCorrectlyReceived-ResponsePending"; break;
                                    default: std::cout << "Unknown"; break;
                                    }
                                    std::cout << ")" << std::endl;
                                }
                            }
                        }
                        catch (const std::exception& e) {
                            std::cout << "Erase failed with exception: " << e.what() << std::endl;
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }

                    // Test if we can read any memory info
                    std::cout << "\nTesting memory info reads..." << std::endl;
                    try {
                        // Try to read some diagnostic identifiers
                        std::vector<std::vector<uint8_t>> testReads = {
                            {0x22, 0xF1, 0x90}, // Software version
                            {0x22, 0xF1, 0x91}, // Application version
                            {0x22, 0xF1, 0x92}, // Calibration version  
                            {0x22, 0xF1, 0x86}, // Active diagnostic session
                        };

                        for (const auto& readCmd : testReads) {
                            try {
                                auto readResp = flasher.sendUDSRequestWithMultiFrame(readCmd, 2000);
                                std::cout << "Read ";
                                for (uint8_t b : readCmd) std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                                std::cout << "-> ";
                                for (uint8_t b : readResp) std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                                std::cout << std::dec << std::endl;
                            }
                            catch (...) {
                                // Ignore read failures
                            }
                        }
                    }
                    catch (...) {
                        // Ignore diagnostic read failures
                    }
                }
            }
        }
    }
    catch (const std::exception& e) {
        std::cout << "Debug error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void testRC40ASW0AndDS0HexFlashing() {
    std::cout << "Testing RC40 ASW0+DS0 Combined HEX Flashing..." << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_COMBINED", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) return;

    try {
        // ... preparation and security access code same as before ...

        std::cout << "=== LOADING HEX FILE ===" << std::endl;
        std::string hexFile = "firmware/rc5_6_40_asw0.hex";
        auto firmwareData = RC40Flasher::parseIntelHexFile(hexFile);

        std::cout << "Total firmware size: " << firmwareData.size() << " bytes" << std::endl;

        // Split firmware data at ASW0/DS0 boundary
        uint32_t asw0Start = 0x09020000;
        uint32_t asw0End = 0x093BFFFF;
        uint32_t ds0Start = 0x093C0000;

        uint32_t asw0Size = asw0End - asw0Start + 1;      // 0x3A0000 bytes
        uint32_t ds0Offset = ds0Start - asw0Start;        // Offset in firmware data

        // Extract ASW0 portion (first part of firmware)
        std::vector<uint8_t> asw0Data(firmwareData.begin(), firmwareData.begin() + asw0Size);

        // Extract DS0 portion (remaining part)
        std::vector<uint8_t> ds0Data;
        if (firmwareData.size() > ds0Offset) {
            ds0Data = std::vector<uint8_t>(firmwareData.begin() + ds0Offset, firmwareData.end());
        }

        std::cout << "ASW0 portion: " << asw0Data.size() << " bytes" << std::endl;
        std::cout << "DS0 portion: " << ds0Data.size() << " bytes" << std::endl;

        // Flash ASW0 first
        std::cout << "=== FLASHING ASW0 ===" << std::endl;
        if (!flasher.eraseMemory(0x01)) { // ASW0 area ID
            std::cout << "Failed to erase ASW0" << std::endl;
            return;
        }

        uint16_t maxBlockSize = flasher.requestDownload(asw0Start, (uint32_t)asw0Data.size());
        if (maxBlockSize == 0) return;

        if (!flasher.transferData(asw0Data, maxBlockSize)) return;
        if (!flasher.requestTransferExit()) return;
        if (!flasher.checkMemory(0x01)) return;

        // Flash DS0 if there's data for it
        if (!ds0Data.empty()) {
            std::cout << "=== FLASHING DS0 ===" << std::endl;
            if (!flasher.eraseMemory(0x02)) { // DS0 area ID
                std::cout << "Failed to erase DS0" << std::endl;
                return;
            }

            maxBlockSize = flasher.requestDownload(ds0Start, (uint32_t)ds0Data.size());
            if (maxBlockSize == 0) return;

            if (!flasher.transferData(ds0Data, maxBlockSize)) return;
            if (!flasher.requestTransferExit()) return;
            if (!flasher.checkMemory(0x02)) return;
        }

        std::cout << "=== SUCCESS ===" << std::endl;
        flasher.ecuReset();

    }
    catch (const std::exception& e) {
        std::cout << "Flash error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void testRC40ASW0HexFlashing() {
    std::cout << "Testing RC40 ASW0 Flashing with Intel HEX File..." << std::endl;

    resetPCANChannel();

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_ASW0_HEX", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) {
        std::cout << "CAN initialization failed" << std::endl;
        return;
    }

    try {
        std::cout << "=== PREPARATION PHASE ===" << std::endl;
        // Functional addressing preparation
        flasher.sendFunctionalCommand({ 0x10, 0x83 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x85, 0x82 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::cout << "=== PROGRAMMING MODE ===" << std::endl;
        if (!flasher.diagnosticSessionControl(0x02)) {
            std::cout << "Failed to enter programming mode" << std::endl;
            return;
        }

        std::cout << "=== SECURITY ACCESS ===" << std::endl;
        auto seed = flasher.securityAccessRequestSeed();
        if (seed.empty()) {
            std::cout << "Failed to get security seed" << std::endl;
            return;
        }

        std::string passwordStr = "DEF_PASSWORD_021";
        std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());
        auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

        if (!flasher.securityAccessSendKey(key)) {
            std::cout << "Security access failed" << std::endl;
            return;
        }

        std::cout << "=== LOADING HEX FILE ===" << std::endl;

        // Parse Intel HEX file
        std::string hexFile = "firmware/rc5_6_40_asw0.hex";
        std::vector<uint8_t> firmwareData;
        
        try {
            firmwareData = RC40Flasher::parseIntelHexFile(hexFile);
        }
        catch (const std::exception& e) {
            std::cout << "ERROR: Failed to parse HEX file: " << e.what() << std::endl;
            std::cout << "Please place your ASW0 firmware file at: " << hexFile << std::endl;
            flasher.cleanup();
            return;
        }

        std::cout << "Parsed HEX file successfully: " << firmwareData.size() << " bytes" << std::endl;

        std::cout << "=== ASW0 FLASHING ===" << std::endl;

        // ASW0 memory layout for RC5-6/40
        uint8_t asw0AreaId = 0x01;
        uint32_t asw0Address = 0x09020000;

        std::cout << "Step 1: Erasing ASW0 memory area..." << std::endl;
        if (!flasher.eraseMemory(asw0AreaId)) {
            std::cout << "Failed to erase ASW0 memory" << std::endl;
            return;
        }

        std::cout << "Step 2: Request download..." << std::endl;
        uint16_t maxBlockSize = flasher.requestDownload(asw0Address, (uint32_t)firmwareData.size());
        if (maxBlockSize == 0) {
            std::cout << "Failed to request download" << std::endl;
            return;
        }

        std::cout << "Step 3: Transferring firmware data (" << firmwareData.size() << " bytes)..." << std::endl;
        std::cout << "Max block size: " << maxBlockSize << " bytes" << std::endl;

        // Calculate estimated time
        size_t totalBlocks = (firmwareData.size() + maxBlockSize - 3) / (maxBlockSize - 2);
        std::cout << "Estimated blocks to transfer: " << totalBlocks << std::endl;
        std::cout << "Estimated time: ~" << (totalBlocks * 50 / 1000) << " seconds" << std::endl;

        if (!flasher.transferData(firmwareData, maxBlockSize)) {
            std::cout << "Failed to transfer firmware data" << std::endl;
            return;
        }

        std::cout << "Step 4: Transfer exit..." << std::endl;
        if (!flasher.requestTransferExit()) {
            std::cout << "Failed to exit transfer" << std::endl;
            return;
        }

        std::cout << "Step 5: Memory verification..." << std::endl;
        if (!flasher.checkMemory(asw0AreaId)) {
            std::cout << "Memory verification failed!" << std::endl;
            return;
        }

        std::cout << "=== FINALIZATION ===" << std::endl;
        std::cout << "Resetting ECU..." << std::endl;
        if (!flasher.ecuReset()) {
            std::cout << "ECU reset failed" << std::endl;
            return;
        }

        std::cout << "Waiting for ECU reboot..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::cout << "\n*** FLASHING COMPLETE! ***" << std::endl;
        std::cout << "ASW0 (Application Software) has been successfully flashed from HEX file!" << std::endl;
        std::cout << "Firmware size: " << firmwareData.size() << " bytes" << std::endl;
        std::cout << "RC40 controller should now be running the new application." << std::endl;

    }
    catch (const std::exception& e) {
        std::cout << "Flash error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void testRC40FastSecurityAccess() {
    std::cout << "Testing RC40 Security Access with Faster Sequence..." << std::endl;

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_TEST", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) return;

    try {
        // Faster preparation sequence
        flasher.sendFunctionalCommand({ 0x10, 0x83 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x85, 0x82 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Programming mode
        if (flasher.diagnosticSessionControl(0x02)) {
            std::cout << "Programming mode entered successfully" << std::endl;

            // FAST security access - minimize delay between seed and key
            std::cout << "Performing FAST security access..." << std::endl;

            auto seed = flasher.securityAccessRequestSeed();
            if (!seed.empty() && seed.size() == 16) {
                // Try multiple potential passwords
                std::vector<std::string> testPasswords = {
                    "DEF_PASSWORD_021",
                    "DEFAULT_PASSWORD",
                    "RC40_DEFAULT_KEY",
                    "BOSCH_DEFAULT_21",
                    "000102030405060708090A0B0C0D0E0F", // Hex string
                    "REXROTH_FLASH_PW"
                };

                for (const auto& pwd : testPasswords) {
                    std::cout << "\nTrying password: " << pwd << std::endl;

                    std::vector<uint8_t> password;
                    if (pwd.length() == 32) {
                        // Convert hex string to bytes
                        for (size_t i = 0; i < pwd.length(); i += 2) {
                            std::string byteStr = pwd.substr(i, 2);
                            password.push_back(static_cast<uint8_t>(std::stoi(byteStr, nullptr, 16)));
                        }
                    }
                    else {
                        password = std::vector<uint8_t>(pwd.begin(), pwd.end());
                    }

                    auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

                    // Send key immediately after calculation
                    bool unlocked = flasher.securityAccessSendKey(key);

                    if (unlocked) {
                        std::cout << "SUCCESS! Correct password: " << pwd << std::endl;

                        // Test unlock by trying an erase
                        std::cout << "Testing erase to confirm unlock..." << std::endl;
                        bool eraseTest = flasher.eraseMemory(0x00);
                        std::cout << "Erase test: " << (eraseTest ? "SUCCESS" : "FAILED") << std::endl;

                        flasher.cleanup();
                        return;
                    }
                    else {
                        std::cout << "Failed with this password" << std::endl;

                        // Need to restart session for next attempt
                        flasher.cleanup();
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));

                        if (!flasher.initialize()) break;

                        // Re-enter programming mode
                        flasher.sendFunctionalCommand({ 0x10, 0x83 });
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        flasher.sendFunctionalCommand({ 0x85, 0x82 });
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));

                        if (!flasher.diagnosticSessionControl(0x02)) {
                            std::cout << "Failed to re-enter programming mode" << std::endl;
                            break;
                        }

                        // Get fresh seed
                        seed = flasher.securityAccessRequestSeed();
                        if (seed.empty()) {
                            std::cout << "Failed to get fresh seed" << std::endl;
                            break;
                        }
                    }
                }

                std::cout << "\nNone of the test passwords worked." << std::endl;
                std::cout << "You need the actual security password used when this RC40 firmware was built." << std::endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void testRC40SecurityUnlock() {
    std::cout << "Testing RC40 Security Key Calculation and Unlock..." << std::endl;

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_TEST", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) return;

    try {
        // Preparation sequence
        flasher.sendFunctionalCommand({ 0x10, 0x83 });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        flasher.sendFunctionalCommand({ 0x85, 0x82 });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Programming mode
        if (flasher.diagnosticSessionControl(0x02)) {
            // Get seed
            auto seed = flasher.securityAccessRequestSeed();
            if (!seed.empty() && seed.size() == 16) {
                std::cout << "Seed: ";
                for (uint8_t b : seed) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                }
                std::cout << std::dec << std::endl;

                // **CRITICAL: You need your actual security password here**
                std::cout << "\nEnter your RC40 security password: ";
                std::string passwordStr;
                std::getline(std::cin, passwordStr);

                if (passwordStr.empty()) {
                    // Use a test password for now - REPLACE WITH REAL PASSWORD
                    passwordStr = "TestPassword1234";
                    std::cout << "Using test password (this will likely fail on real hardware)" << std::endl;
                }

                std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());

                // Calculate key
                std::cout << "Calculating AES key..." << std::endl;
                auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

                std::cout << "Key:  ";
                for (uint8_t b : key) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                }
                std::cout << std::dec << std::endl;

                // Send key to unlock
                std::cout << "\nSending security key..." << std::endl;
                bool unlocked = flasher.securityAccessSendKey(key);

                if (unlocked) {
                    std::cout << "🎉 SUCCESS: RC40 CONTROLLER IS UNLOCKED!" << std::endl;
                    std::cout << "Ready for firmware flashing!" << std::endl;

                    // Test a simple erase command to verify unlock status
                    std::cout << "\nTesting erase command (CB block)..." << std::endl;
                    bool eraseResult = flasher.eraseMemory(0x00); // CB block
                    std::cout << "Erase test: " << (eraseResult ? "SUCCESS" : "FAILED") << std::endl;

                }
                else {
                    std::cout << "❌ FAILED: Security access denied" << std::endl;
                    std::cout << "This usually means wrong password or key calculation issue" << std::endl;
                }
            }
        }
    }
    catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void testRC40SecurityAccess() {
    std::cout << "Testing RC40 Security Access with Multi-Frame Support..." << std::endl;

    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_TEST", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) return;

    try {
        // Do the preparation sequence first
        flasher.sendFunctionalCommand({ 0x10, 0x83 });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        flasher.sendFunctionalCommand({ 0x85, 0x82 });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        flasher.sendFunctionalCommand({ 0x28, 0x81, 0x01 });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Enter programming mode
        std::cout << "Entering programming mode..." << std::endl;
        if (flasher.diagnosticSessionControl(0x02)) {
            std::cout << "Programming mode successful!" << std::endl;

            // Test multi-frame security access
            std::cout << "Requesting security seed..." << std::endl;
            auto seed = flasher.securityAccessRequestSeed();

            if (!seed.empty()) {
                std::cout << "SUCCESS: Received " << seed.size() << " byte seed: ";
                for (uint8_t b : seed) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
                }
                std::cout << std::dec << std::endl;

                if (seed.size() == 16) {
                    std::cout << "Perfect! 16-byte seed received - ready for key calculation!" << std::endl;
                }
            }
            else {
                std::cout << "Failed to get seed" << std::endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void testRC40CorrectSequence() {
    std::cout << "Testing RC40 with Correct Response Handling..." << std::endl;

    TPCANStatus result = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_250K, 0, 0, 0);
    if (result != PCAN_ERROR_OK) return;

    uint32_t functionalId = 0x18DB33F1;
    uint32_t physicalReq = 0x18DA01FA;
    uint32_t physicalResp = 0x18DAFA01;

    auto sendPhysicalWithPending = [&](const std::vector<uint8_t>& cmd, const std::string& description) -> bool {
        std::cout << "PHYSICAL: " << description << std::endl;

        TPCANMsg msg;
        msg.ID = physicalReq;
        msg.LEN = static_cast<BYTE>(cmd.size() + 1);
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        msg.DATA[0] = static_cast<BYTE>(cmd.size());
        for (size_t i = 0; i < cmd.size(); ++i) {
            msg.DATA[i + 1] = cmd[i];
        }

        if (CAN_Write(PCAN_USBBUS1, &msg) != PCAN_ERROR_OK) return false;

        bool gotPending = false;

        // Wait for response with proper pending handling
        for (int attempts = 0; attempts < 1000; attempts++) { // Longer timeout for pending
            TPCANMsg responseMsg;
            TPCANTimestamp timestamp;
            TPCANStatus readResult = CAN_Read(PCAN_USBBUS1, &responseMsg, &timestamp);

            if (readResult == PCAN_ERROR_OK && responseMsg.ID == physicalResp) {
                std::cout << "  Response: ";
                for (int i = 0; i < responseMsg.LEN; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)responseMsg.DATA[i] << " ";
                }
                std::cout << std::dec << std::endl;

                if (responseMsg.LEN >= 3) {
                    uint8_t service = responseMsg.DATA[1];

                    if (service == 0x7F) {  // Negative response
                        uint8_t errorCode = responseMsg.DATA[3];
                        if (errorCode == 0x78) {  // Response pending
                            std::cout << "  RESPONSE PENDING - continuing to wait..." << std::endl;
                            gotPending = true;
                            continue;  // Keep waiting for final response
                        }
                        else {
                            std::cout << "  NEGATIVE RESPONSE - Code: 0x" << std::hex << (int)errorCode << std::dec << std::endl;
                            return false;
                        }
                    }
                    else if ((service & 0x40) != 0) {  // Positive response
                        std::cout << "  POSITIVE RESPONSE" << std::endl;
                        return true;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (gotPending) {
            std::cout << "  TIMEOUT waiting for final response after pending" << std::endl;
        }
        else {
            std::cout << "  TIMEOUT - no response" << std::endl;
        }
        return false;
        };

    // Functional preparation
    auto sendFunctional = [&](const std::vector<uint8_t>& cmd) -> bool {
        TPCANMsg msg;
        msg.ID = functionalId;
        msg.LEN = static_cast<BYTE>(cmd.size() + 1);
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        msg.DATA[0] = static_cast<BYTE>(cmd.size());
        for (size_t i = 0; i < cmd.size(); ++i) {
            msg.DATA[i + 1] = cmd[i];
        }
        return CAN_Write(PCAN_USBBUS1, &msg) == PCAN_ERROR_OK;
        };

    std::cout << "=== PREPARATION PHASE ===" << std::endl;
    sendFunctional({ 0x10, 0x83 }); std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sendFunctional({ 0x85, 0x82 }); std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sendFunctional({ 0x28, 0x81, 0x01 }); std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "\n=== PROGRAMMING SESSION ===" << std::endl;
    bool progSession = sendPhysicalWithPending({ 0x10, 0x02 }, "Enter Programming Session");

    if (progSession) {
        std::cout << "\n=== SECURITY ACCESS ===" << std::endl;
        bool seedReq = sendPhysicalWithPending({ 0x27, 0x01 }, "Request Security Seed");

        if (seedReq) {
            std::cout << "\nSUCCESS: Ready for key calculation and flashing!" << std::endl;
        }
    }

    CAN_Uninitialize(PCAN_USBBUS1);
}

void testRC40MixedAddressingSequence() {
    std::cout << "Testing RC40 Mixed Addressing Sequence (Functional + Physical)..." << std::endl;

    TPCANStatus result = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_250K, 0, 0, 0);
    if (result != PCAN_ERROR_OK) return;

    // Based on your reference document, use functional addressing for preparation
    uint32_t functionalId = 0x18DB33F1;  // Use the first functional ID
    uint32_t physicalReq = 0x18DA01FA;   // Your confirmed working physical ID
    uint32_t physicalResp = 0x18DAFA01;

    auto sendFunctional = [&](const std::vector<uint8_t>& cmd, const std::string& description) -> bool {
        std::cout << "FUNCTIONAL: " << description << std::endl;

        TPCANMsg msg;
        msg.ID = functionalId;
        msg.LEN = static_cast<BYTE>(cmd.size() + 1);
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        msg.DATA[0] = static_cast<BYTE>(cmd.size());
        for (size_t i = 0; i < cmd.size(); ++i) {
            msg.DATA[i + 1] = cmd[i];
        }

        TPCANStatus writeResult = CAN_Write(PCAN_USBBUS1, &msg);
        if (writeResult == PCAN_ERROR_OK) {
            std::cout << "  Sent (no response expected)" << std::endl;
            return true;
        }
        std::cout << "  Send failed" << std::endl;
        return false;
        };

    auto sendPhysical = [&](const std::vector<uint8_t>& cmd, const std::string& description) -> bool {
        std::cout << "PHYSICAL: " << description << std::endl;

        TPCANMsg msg;
        msg.ID = physicalReq;
        msg.LEN = static_cast<BYTE>(cmd.size() + 1);
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        msg.DATA[0] = static_cast<BYTE>(cmd.size());
        for (size_t i = 0; i < cmd.size(); ++i) {
            msg.DATA[i + 1] = cmd[i];
        }

        if (CAN_Write(PCAN_USBBUS1, &msg) != PCAN_ERROR_OK) {
            std::cout << "  Send failed" << std::endl;
            return false;
        }

        // Wait for physical response
        for (int attempts = 0; attempts < 200; attempts++) {
            TPCANMsg responseMsg;
            TPCANTimestamp timestamp;
            TPCANStatus readResult = CAN_Read(PCAN_USBBUS1, &responseMsg, &timestamp);

            if (readResult == PCAN_ERROR_OK && responseMsg.ID == physicalResp) {
                std::cout << "  Response: ";
                for (int i = 0; i < responseMsg.LEN; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)responseMsg.DATA[i] << " ";
                }
                std::cout << std::dec << std::endl;

                // Check for positive response
                if (responseMsg.LEN >= 2 && (responseMsg.DATA[1] & 0x40) != 0) {
                    std::cout << "  SUCCESS" << std::endl;
                    return true;
                }
                else if (responseMsg.LEN >= 2 && responseMsg.DATA[1] == 0x7F) {
                    std::cout << "  NEGATIVE RESPONSE - Code: 0x" << std::hex << (int)responseMsg.DATA[3] << std::dec << std::endl;
                    return false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "  TIMEOUT" << std::endl;
        return false;
        };

    // Follow the exact sequence from your reference document
    std::cout << "\n=== PREPARATION PHASE (Functional) ===" << std::endl;

    // Step 1: Extended Diagnostic Session (Functional)
    bool step1 = sendFunctional({ 0x10, 0x83 }, "Extended Diagnostic Session");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // S3 timing

    // Step 2: Control DTC Settings (Functional)  
    bool step2 = sendFunctional({ 0x85, 0x82 }, "Control DTC Settings - DTC OFF");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Step 3: Communication Control (Functional)
    bool step3 = sendFunctional({ 0x28, 0x81, 0x01 }, "Communication Control - Disable Non-Diagnostic Messages");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "\n=== PROGRAMMING PHASE (Physical) ===" << std::endl;

    // Step 4: Programming Session (Physical - expects response)
    bool step4 = sendPhysical({ 0x10, 0x02 }, "Enter Programming Session");

    if (step4) {
        std::cout << "\n*** SUCCESS! RC40 is now in programming mode ***" << std::endl;

        // Test Security Access
        std::cout << "\n=== SECURITY ACCESS TEST ===" << std::endl;
        bool seedRequest = sendPhysical({ 0x27, 0x01 }, "Request Security Seed");

        if (seedRequest) {
            std::cout << "Ready for security key calculation and flashing!" << std::endl;
        }
    }
    else {
        std::cout << "\nProgramming session entry failed." << std::endl;
        std::cout << "This may be normal - some controllers need all preparation steps." << std::endl;
    }

    CAN_Uninitialize(PCAN_USBBUS1);
}

void testRC40FunctionalSequence() {
    std::cout << "Testing RC40 Functional Addressing Sequence..." << std::endl;

    TPCANStatus result = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_250K, 0, 0, 0);
    if (result != PCAN_ERROR_OK) return;

    // Test different functional addresses for extended CAN IDs
    std::vector<uint32_t> functionalIds = {
        0x18DB33F1,  // Common functional address
        0x18DBFFF1,  // Broadcast functional
        0x18DB01F1,  // Node-specific functional
        0x18DFFFFF,  // Alternative broadcast
    };

    for (uint32_t funcId : functionalIds) {
        std::cout << "Testing functional ID: 0x" << std::hex << funcId << std::dec << std::endl;

        TPCANMsg msg;
        msg.ID = funcId;
        msg.LEN = 3;
        msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
        msg.DATA[0] = 0x02;  // Length
        msg.DATA[1] = 0x10;  // Diagnostic Session Control
        msg.DATA[2] = 0x83;  // Extended Diagnostic Session

        TPCANStatus writeResult = CAN_Write(PCAN_USBBUS1, &msg);
        if (writeResult == PCAN_ERROR_OK) {
            std::cout << "Message sent successfully" << std::endl;

            // Functional addressing typically doesn't expect responses,
            // but let's listen for any traffic
            for (int attempts = 0; attempts < 100; attempts++) {
                TPCANMsg responseMsg;
                TPCANTimestamp timestamp;
                TPCANStatus readResult = CAN_Read(PCAN_USBBUS1, &responseMsg, &timestamp);

                if (readResult == PCAN_ERROR_OK) {
                    std::cout << "Response ID: 0x" << std::hex << std::setw(8) << std::setfill('0')
                        << responseMsg.ID << " Data: ";
                    for (int i = 0; i < responseMsg.LEN; ++i) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)responseMsg.DATA[i] << " ";
                    }
                    std::cout << std::dec << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        // Small delay between attempts
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    CAN_Uninitialize(PCAN_USBBUS1);
}

void testRC40ProgrammingSequence() {
    std::cout << "Testing RC40 Programming Sequence..." << std::endl;

    // Create controller with correct settings
    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "RC40_REAL", PCAN_USBBUS1);
    testConfig.canIdRequest = 0x18DA01FA;
    testConfig.canIdResponse = 0x18DAFA01;

    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (!flasher.initialize()) {
        std::cout << "Failed to initialize" << std::endl;
        return;
    }

    try {
        // Step 1: Extended Diagnostic Session
        std::cout << "Step 1: Extended Diagnostic Session..." << std::endl;
        bool result1 = flasher.diagnosticSessionControl(0x83);
        std::cout << "Result: " << (result1 ? "SUCCESS" : "FAILED") << std::endl;

        // Step 2: Programming Session 
        std::cout << "Step 2: Programming Session..." << std::endl;
        bool result2 = flasher.diagnosticSessionControl(0x02);
        std::cout << "Result: " << (result2 ? "SUCCESS" : "FAILED") << std::endl;

        if (result2) {
            // Step 3: Security Access - Request Seed
            std::cout << "Step 3: Security Access - Request Seed..." << std::endl;
            auto seed = flasher.securityAccessRequestSeed();
            if (!seed.empty()) {
                std::cout << "SUCCESS: Received " << seed.size() << " byte seed" << std::endl;

                // You'll need your actual security password here
                std::string passwordStr = "YourActualPassword"; // REPLACE THIS
                std::vector<uint8_t> password(passwordStr.begin(), passwordStr.end());

                auto key = RC40Flasher::AES128Security::calculateKeyFromSeed(seed, password);

                std::cout << "Step 4: Security Access - Send Key..." << std::endl;
                bool keyResult = flasher.securityAccessSendKey(key);
                std::cout << "Result: " << (keyResult ? "SUCCESS - UNLOCKED!" : "FAILED - Wrong password?") << std::endl;
            }
            else {
                std::cout << "FAILED to get seed" << std::endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }

    flasher.cleanup();
}

void testRC40ExtendedAddressing() {
    std::cout << "Testing RC40 with extended CAN ID (18DA01FAh)..." << std::endl;

    TPCANStatus result = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_250K, 0, 0, 0);
    if (result != PCAN_ERROR_OK) {
        std::cout << "Failed to initialize CAN" << std::endl;
        return;
    }

    // Test 1: Default Session (safe test)
    std::cout << "Test 1: Default Diagnostic Session..." << std::endl;
    TPCANMsg msg;
    msg.ID = 0x18DA01FA;                    // Extended ID from forum post
    msg.LEN = 3;
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;    // Important: Extended frame!
    msg.DATA[0] = 0x02;                     // ISO-TP single frame length
    msg.DATA[1] = 0x10;                     // UDS Diagnostic Session Control
    msg.DATA[2] = 0x01;                     // Default session

    TPCANStatus writeResult = CAN_Write(PCAN_USBBUS1, &msg);
    std::cout << "Write result: 0x" << std::hex << writeResult << std::dec << std::endl;

    if (writeResult == PCAN_ERROR_OK) {
        // Expected response ID: 18DAFA01 (swapped addresses)
        uint32_t expectedResponseId = 0x18DAFA01;

        std::cout << "Listening for response on 0x" << std::hex << expectedResponseId << std::dec << "..." << std::endl;

        for (int attempts = 0; attempts < 200; attempts++) {
            TPCANMsg responseMsg;
            TPCANTimestamp timestamp;
            TPCANStatus readResult = CAN_Read(PCAN_USBBUS1, &responseMsg, &timestamp);

            if (readResult == PCAN_ERROR_OK) {
                std::cout << "Response received!" << std::endl;
                std::cout << "ID: 0x" << std::hex << std::setw(8) << std::setfill('0') << responseMsg.ID << std::dec;
                std::cout << " Type: " << (responseMsg.MSGTYPE == PCAN_MESSAGE_EXTENDED ? "Extended" : "Standard");
                std::cout << " Len: " << (int)responseMsg.LEN << " Data: ";
                for (int i = 0; i < responseMsg.LEN; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)responseMsg.DATA[i] << " ";
                }
                std::cout << std::dec << std::endl;

                // Check if it's a positive response (50 01...)
                if (responseMsg.LEN >= 2 && responseMsg.DATA[1] == 0x50) {
                    std::cout << "SUCCESS: Positive UDS response received!" << std::endl;
                }

                CAN_Uninitialize(PCAN_USBBUS1);
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "No response received after 2 seconds." << std::endl;
    }

    // Test 2: Try the specific test command from forum
    std::cout << "\nTest 2: Forum test command (03 22 F1 92)..." << std::endl;
    msg.ID = 0x18DA01FA;
    msg.LEN = 4;
    msg.MSGTYPE = PCAN_MESSAGE_EXTENDED;
    msg.DATA[0] = 0x03;    // Length
    msg.DATA[1] = 0x22;    // Read Data By Identifier
    msg.DATA[2] = 0xF1;    // DID high byte
    msg.DATA[3] = 0x92;    // DID low byte

    writeResult = CAN_Write(PCAN_USBBUS1, &msg);
    if (writeResult == PCAN_ERROR_OK) {
        for (int attempts = 0; attempts < 200; attempts++) {
            TPCANMsg responseMsg;
            TPCANTimestamp timestamp;
            TPCANStatus readResult = CAN_Read(PCAN_USBBUS1, &responseMsg, &timestamp);

            if (readResult == PCAN_ERROR_OK) {
                std::cout << "Test command response!" << std::endl;
                std::cout << "ID: 0x" << std::hex << std::setw(8) << std::setfill('0') << responseMsg.ID
                    << " Data: ";
                for (int i = 0; i < responseMsg.LEN; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)responseMsg.DATA[i] << " ";
                }
                std::cout << std::dec << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    CAN_Uninitialize(PCAN_USBBUS1);
}

void testRC40PhysicalAddressing() {
    std::cout << "Testing RC40 physical addressing..." << std::endl;

    TPCANStatus result = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_250K, 0, 0, 0);
    if (result != PCAN_ERROR_OK) return;

    // Common ECU physical addresses
    std::vector<uint32_t> physicalIds = {
        0x700, 0x701, 0x702, 0x703, 0x704, 0x705, 0x706, 0x707,  // 0x70x range
        0x7E0, 0x7E1, 0x7E2, 0x7E3, 0x7E4, 0x7E5,              // Standard UDS range
        0x600, 0x601, 0x602, 0x603,                             // Alternative range
    };

    for (uint32_t requestId : physicalIds) {
        uint32_t expectedResponseId = requestId + 0x08;  // Common pattern: response = request + 8

        std::cout << "Trying physical ID 0x" << std::hex << requestId
            << " (expecting response on 0x" << expectedResponseId << ")" << std::dec << std::endl;

        TPCANMsg msg;
        msg.ID = requestId;
        msg.LEN = 3;
        msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
        msg.DATA[0] = 0x02;  // Length
        msg.DATA[1] = 0x10;  // Diagnostic Session Control  
        msg.DATA[2] = 0x01;  // Default Session (safe request)

        CAN_Write(PCAN_USBBUS1, &msg);

        // Wait for response
        for (int attempts = 0; attempts < 100; attempts++) {
            TPCANMsg responseMsg;
            TPCANTimestamp timestamp;
            TPCANStatus readResult = CAN_Read(PCAN_USBBUS1, &responseMsg, &timestamp);

            if (readResult == PCAN_ERROR_OK) {
                std::cout << "FOUND RESPONSE! Request ID: 0x" << std::hex << requestId
                    << " Response ID: 0x" << responseMsg.ID << std::dec << std::endl;
                std::cout << "Response data: ";
                for (int i = 0; i < responseMsg.LEN; ++i) {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)responseMsg.DATA[i] << " ";
                }
                std::cout << std::dec << std::endl;

                CAN_Uninitialize(PCAN_USBBUS1);
                return; // Success!
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::cout << "No physical responses found." << std::endl;
    CAN_Uninitialize(PCAN_USBBUS1);
}

void testRC40FunctionalAddressing() {
    std::cout << "Testing RC40 functional addressing at 250K..." << std::endl;

    TPCANStatus result = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_250K, 0, 0, 0);
    if (result != PCAN_ERROR_OK) {
        std::cout << "Failed to initialize CAN" << std::endl;
        return;
    }

    // UDS functional addressing uses 0x7DF for requests
    TPCANMsg msg;
    msg.ID = 0x7DF;  // Functional addressing ID
    msg.LEN = 3;
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD;

    // Test 1: Extended Diagnostic Session (from your reference doc)
    std::cout << "Sending Extended Diagnostic Session (functional)..." << std::endl;
    msg.DATA[0] = 0x02;  // ISO-TP length
    msg.DATA[1] = 0x10;  // Diagnostic Session Control
    msg.DATA[2] = 0x83;  // Extended Diagnostic Session

    TPCANStatus writeResult = CAN_Write(PCAN_USBBUS1, &msg);
    std::cout << "Write result: 0x" << std::hex << writeResult << std::dec << std::endl;

    // According to your document, functional addressing might not expect responses
    // But let's listen anyway for any physical responses
    std::cout << "Listening for any responses (2 seconds)..." << std::endl;
    auto startTime = std::chrono::steady_clock::now();
    bool gotResponse = false;

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - startTime).count() < 2000) {

        TPCANMsg responseMsg;
        TPCANTimestamp timestamp;
        TPCANStatus readResult = CAN_Read(PCAN_USBBUS1, &responseMsg, &timestamp);

        if (readResult == PCAN_ERROR_OK) {
            gotResponse = true;
            std::cout << "Response - ID: 0x" << std::hex << std::setw(3) << std::setfill('0')
                << responseMsg.ID << " Data: ";
            for (int i = 0; i < responseMsg.LEN; ++i) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)responseMsg.DATA[i] << " ";
            }
            std::cout << std::dec << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!gotResponse) {
        std::cout << "No responses (this may be normal for functional addressing)" << std::endl;
    }

    CAN_Uninitialize(PCAN_USBBUS1);
}

void testDifferentCANIDs_250K() {
    std::cout << "Testing different CAN IDs at 250K..." << std::endl;

    TPCANStatus result = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_250K, 0, 0, 0);
    if (result != PCAN_ERROR_OK) return;

    struct IDPair {
        uint32_t request;
        uint32_t response;
        const char* description;
    };

    std::vector<IDPair> idPairs = {
        {0x7E0, 0x7E8, "Standard UDS"},
        {0x700, 0x708, "Alt 1"},
        {0x600, 0x608, "Alt 2"},
        {0x123, 0x12B, "Sequential"},
        {0x200, 0x201, "Low Range"},
        {0x300, 0x301, "Mid Range"},
        // RC40 might use specific ranges
    };

    for (const auto& ids : idPairs) {
        std::cout << "Testing " << ids.description << " (0x" << std::hex
            << ids.request << " -> 0x" << ids.response << ")" << std::dec << std::endl;

        TPCANMsg msg;
        msg.ID = ids.request;
        msg.LEN = 3;
        msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
        msg.DATA[0] = 0x02;  // ISO-TP length
        msg.DATA[1] = 0x10;  // UDS Diagnostic Session Control
        msg.DATA[2] = 0x01;  // Default session

        TPCANStatus writeResult = CAN_Write(PCAN_USBBUS1, &msg);
        if (writeResult == PCAN_ERROR_OK) {
            // Wait for response
            for (int attempts = 0; attempts < 50; attempts++) {
                TPCANMsg responseMsg;
                TPCANTimestamp timestamp;
                TPCANStatus readResult = CAN_Read(PCAN_USBBUS1, &responseMsg, &timestamp);

                if (readResult == PCAN_ERROR_OK) {
                    if (responseMsg.ID == ids.response) {
                        std::cout << "SUCCESS! Response on expected ID 0x" << std::hex << responseMsg.ID << std::dec << std::endl;
                        std::cout << "Response data: ";
                        for (int i = 0; i < responseMsg.LEN; ++i) {
                            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)responseMsg.DATA[i] << " ";
                        }
                        std::cout << std::dec << std::endl;
                        CAN_Uninitialize(PCAN_USBBUS1);
                        return; // Found working IDs!
                    }
                    else {
                        std::cout << "Unexpected response ID: 0x" << std::hex << responseMsg.ID << std::dec << std::endl;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    CAN_Uninitialize(PCAN_USBBUS1);
    std::cout << "No responses found with tested ID pairs." << std::endl;
}


void testCANReceive() {
    std::cout << "Listening for CAN messages at 250K..." << std::endl;

    TPCANStatus result = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_250K, 0, 0, 0);
    if (result != PCAN_ERROR_OK) {
        std::cout << "Failed to initialize CAN" << std::endl;
        return;
    }

    std::cout << "Listening for 10 seconds..." << std::endl;
    auto startTime = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - startTime).count() < 10) {

        TPCANMsg msg;
        TPCANTimestamp timestamp;
        TPCANStatus readResult = CAN_Read(PCAN_USBBUS1, &msg, &timestamp);

        if (readResult == PCAN_ERROR_OK) {
            std::cout << "Received - ID: 0x" << std::hex << std::setw(3) << std::setfill('0') << msg.ID
                << " Len: " << std::dec << (int)msg.LEN << " Data: ";
            for (int i = 0; i < msg.LEN; ++i) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)msg.DATA[i] << " ";
            }
            std::cout << std::dec << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    CAN_Uninitialize(PCAN_USBBUS1);
    std::cout << "Listening complete." << std::endl;
}

void testDifferentBitrates() {
    std::vector<TPCANBaudrate> bitrates = {
        PCAN_BAUD_250K,   // Try 250kbps first
        PCAN_BAUD_500K,   // Current setting
        PCAN_BAUD_125K,   // Sometimes used
        PCAN_BAUD_1M      // Less common but possible
    };

    for (auto bitrate : bitrates) {
        std::cout << "Testing bitrate: ";
        switch (bitrate) {
        case PCAN_BAUD_125K: std::cout << "125K"; break;
        case PCAN_BAUD_250K: std::cout << "250K"; break;
        case PCAN_BAUD_500K: std::cout << "500K"; break;
        case PCAN_BAUD_1M: std::cout << "1M"; break;
        }
        std::cout << std::endl;

        // Initialize with this bitrate
        TPCANStatus result = CAN_Initialize(PCAN_USBBUS1, bitrate, 0, 0, 0);
        if (result == PCAN_ERROR_OK) {
            std::cout << "CAN initialized at this bitrate" << std::endl;

            // Try to send a simple message and see if we get a different error
            TPCANMsg msg;
            msg.ID = 0x7E0;
            msg.LEN = 3;
            msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
            msg.DATA[0] = 0x02;  // Length
            msg.DATA[1] = 0x10;  // UDS Service
            msg.DATA[2] = 0x01;  // Default session

            TPCANStatus writeResult = CAN_Write(PCAN_USBBUS1, &msg);
            std::cout << "Write result: 0x" << std::hex << writeResult << std::dec << std::endl;

            // Wait briefly and try to read
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            TPCANMsg responseMsg;
            TPCANTimestamp timestamp;
            TPCANStatus readResult = CAN_Read(PCAN_USBBUS1, &responseMsg, &timestamp);
            std::cout << "Read result: 0x" << std::hex << readResult << std::dec << std::endl;

            CAN_Uninitialize(PCAN_USBBUS1);
            std::cout << "---" << std::endl;
        }
    }
}

void testCANCommunication() {
    std::cout << "Testing CAN Communication..." << std::endl;

    // Create a simple test controller
    RC40Flasher::ControllerConfig testConfig("RC5-6/40", "TEST_001", PCAN_USBBUS1);
    RC40Flasher::RC40FlasherDevice flasher(testConfig);

    if (flasher.initialize()) {
        std::cout << "CAN hardware initialized successfully!" << std::endl;

        try {
            // Try a simple diagnostic session control request
            bool result = flasher.diagnosticSessionControl(0x01); // Default session
            std::cout << "Diagnostic session test: " << (result ? "SUCCESS" : "FAILED") << std::endl;
        }
        catch (const std::exception& e) {
            std::cout << "CAN communication test failed: " << e.what() << std::endl;
        }

        flasher.cleanup();
    }
    else {
        std::cout << "CAN hardware initialization failed!" << std::endl;
    }
}


int ParseJson(std::string JsonStr, Json::Value& parsedJson)
{
    int output = -1;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(JsonStr.c_str(), parsedJson);
    if (!parsingSuccessful)
    {
        printf("FAILED TO PARSE! %d\n", reader.getFormattedErrorMessages());
        return output;
    }
    return 0;
}

int InsertValueToCanTelegram(can_isobus_info* messageData, int spnInfoIndex, uint64_t input)
{
    // Extract SPN details
    uint8_t byteIndex = messageData->spns[spnInfoIndex].byte - 1; // Convert to zero-based index
    uint8_t bitIndex = messageData->spns[spnInfoIndex].bit - 1;   // Convert to zero-based index
    uint8_t length = messageData->spns[spnInfoIndex].len;         // Length in bits

    // Check if the requested operation exceeds message boundaries
    if ((BITS_PER_BYTE * byteIndex + bitIndex + length) > (messageData->lenMax * BITS_PER_BYTE)) {
        return -1; // Out of bounds error
    }

    // Mask the input value to ensure it fits within the required bit length
    uint64_t mask = (1ULL << length) - 1;
    input &= mask;

    // Iterate over each bit in the input value
    for (uint8_t i = 0; i < length; i++) {
        uint8_t bitPosition = bitIndex + i;                          // Absolute bit position from the start byte
        uint8_t targetByte = byteIndex + (bitPosition / BITS_PER_BYTE); // Target byte index
        uint8_t targetBit = bitPosition % BITS_PER_BYTE;             // Bit position within the byte

        // Clear the bit in the CAN message
        messageData->data[targetByte] &= ~(1 << targetBit);

        // Insert the bit from the input value
        messageData->data[targetByte] |= ((input >> i) & 1) << targetBit;
    }
    return 0; // Success
}


int ExtractValueFromCanTelegram(can_isobus_info messageData, int spnInfoIndex, uint64_t* output) {
    // Extract SPN details
    uint8_t byteIndex = messageData.spns[spnInfoIndex].byte - 1; // Convert to zero-based index
    uint8_t bitIndex = messageData.spns[spnInfoIndex].bit - 1;   // Convert to zero-based index
    uint8_t length = messageData.spns[spnInfoIndex].len;         // Length in bits

    // Bounds check to ensure we do not exceed message size
    if ((BITS_PER_BYTE * byteIndex + bitIndex + length) > (messageData.lenMax * BITS_PER_BYTE)) {
        return -1; // Out of bounds error
    }

    uint64_t value = 0;

    // Extract bits little-endian style
    uint8_t bitOffset = bitIndex;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t bitPosition = bitOffset + i;                        // Absolute bit position from start byte
        uint8_t sourceByte = byteIndex + (bitPosition / BITS_PER_BYTE); // Byte index in the message
        uint8_t sourceBit = bitPosition % BITS_PER_BYTE;            // Bit position within the byte

        // Extract bit and insert into output
        value |= ((messageData.data[sourceByte] >> sourceBit) & 1) << i;
    }

    *output = value;
    return 0; // Success
}
/*
int InsertValueToCanTelegram(can_isobus_info* messageData, int spnInfoIndex, uint64_t input)
{
    uint8_t byteIndex = messageData->spns[spnInfoIndex].byte - 1;                                                               // These values start from 1, not 0
    uint8_t bitIndex = messageData->spns[spnInfoIndex].bit - 1;                                                                 // These values start from 1, not 0
    if ((BITS_PER_BYTE * byteIndex + bitIndex + messageData->spns[spnInfoIndex].len) > (messageData->lenMax * BITS_PER_BYTE)) // Check if we are asking for something outside of telegram's allocation
        return -1;                                                                                                              // Return FSC_ERR if we are going to overrun the array

    uint64_t mask = 0;
    uint8_t i = 0;
    for (i; i < messageData->spns[spnInfoIndex].len; i++)
    {
        mask |= (1 << i);
    }
    uint64_t val = 0;
    uint8_t numBytes = 1 + (bitIndex + messageData->spns[spnInfoIndex].len - 1) / BITS_PER_BYTE; // How many bytes does this information span?
    i = 0;
    for (i; i < numBytes; i++)
    {
        uint8_t byteMask = 0;
        uint8_t byteInput = 0;
        uint8_t invMask = 0;
        int8_t byteMaskShift = messageData->spns[spnInfoIndex].len - ((BITS_PER_BYTE * (numBytes - i)) - bitIndex); // figure out how many bits to shift to convert a potential value over 8 bits into chunks of 8 bits
        if (byteMaskShift > 0)
        {
            byteMask = (mask >> byteMaskShift);
            byteInput = (input >> byteMaskShift) & byteMask;
        }
        else // if shift is negative, make it positive and shift left instead
        {
            byteMaskShift *= -1;
            byteMask = (mask << byteMaskShift);
            byteInput = (input << byteMaskShift) & byteMask;
        }
        invMask = ~byteMask;
        messageData->data[byteIndex + i] &= invMask; // remove previous data in the location we are writing to
        messageData->data[byteIndex + i] |= byteInput;
    }
    // *output = (val >> bitIndex) & mask;
    return 0;
}*/
void ScaleAndOffsetForTx_Float(uint64 inRawVal, spn_info spn, float* outVal)
{
    *outVal = (float)((inRawVal - spn.offset) / spn.scaling);
}

void ScaleAndOffsetForTx_Int(uint64 inRawVal, spn_info spn, int* outVal)
{
    *outVal = (int)((inRawVal - spn.offset) / spn.scaling);
}

void ScaleAndOffsetForTx(uint64 inRawVal, spn_info spn, void* outVal)
{
    switch (spn.varType)
    {
    case TYPE_INT:
        ScaleAndOffsetForTx_Int(inRawVal, spn, (int*)outVal);
        break;
    case TYPE_FLOAT:
        ScaleAndOffsetForTx_Float(inRawVal, spn, (float*)outVal);
        break;
    default: // do nothing for now...
        break;
    }
}

void ScaleAndOffsetForRx_Float(uint64 inRawVal, spn_info spn, float* outVal)
{
    *outVal = (float)(inRawVal * spn.scaling + spn.offset);
}

void ScaleAndOffsetForRx_Int(uint64 inRawVal, spn_info spn, int* outVal)
{
    *outVal = (int)(inRawVal * (int)spn.scaling + spn.offset);
}

void ScaleAndOffsetForRx(uint64 inRawVal, spn_info spn, void* outVal)
{
    switch (spn.varType)
    {
    case TYPE_INT:
        ScaleAndOffsetForRx_Int(inRawVal, spn, (int*)outVal);
        break;
    case TYPE_FLOAT:
        ScaleAndOffsetForRx_Float(inRawVal, spn, (float*)outVal);
        break;
    default: // do nothing for now...
        break;
    }
}

int ScaleOffsetAndInsertValueToCan(can_isobus_info* messageData, int spnInfoIndex, uint64 inVal)
{
    uint64 tempScaledVal;
    ScaleAndOffsetForTx(inVal, messageData->spns[spnInfoIndex], &tempScaledVal);
    return InsertValueToCanTelegram(messageData, spnInfoIndex, tempScaledVal);
}

int ExtractScaleAndOffsetValueFromCan(can_isobus_info* messageData, int spnInfoIndex, uint64* rawVal, void* scaledVal)
{
    int output = ExtractValueFromCanTelegram(*messageData, spnInfoIndex, rawVal);
    ScaleAndOffsetForRx(*rawVal, messageData->spns[spnInfoIndex], scaledVal);
    return output;
}

typedef enum
{
    SAFOUT_DEBUG1_INITIALIZED,
    SAFOUT_DEBUG1_CONFIGURED,
    SAFOUT_DEBUG1_LOCKED_CHNL,
    SAFOUT_DEBUG1_IDEV_CMP_HSLS,
    SAFOUT_DEBUG1_ACTIVE_OUT1,
    SAFOUT_DEBUG1_ACTIVE_OUT2,
    SAFOUT_DEBUG1_ACTIVE_OUT3,
    SAFOUT_DEBUG1_ACTIVE_OUT4,
    SAFOUT_DEBUG1_IGNORE_ERR,
    SAFOUT_DEBUG1_ISET_COM,
    SAFOUT_DEBUG1_ISET_OUT1,
    SAFOUT_DEBUG1_ISET_OUT2,
    SAFOUT_DEBUG1_ISET_OUT3,
    SAFOUT_DEBUG1_ISET_OUT4
} SAFOUT_DEBUG1_SPNs;
typedef enum
{
    SAFOUT_DEBUG2_DFC_OUT1,
    SAFOUT_DEBUG2_DFC_OUT2,
    SAFOUT_DEBUG2_DFC_OUT3,
    SAFOUT_DEBUG2_DFC_OUT4,
    SAFOUT_DEBUG2_IMEAS_OUT1,
    SAFOUT_DEBUG2_IMEAS_OUT2,
    SAFOUT_DEBUG2_IMEAS_OUT3,
    SAFOUT_DEBUG2_IMEAS_OUT4,
    SAFOUT_DEBUG2_IMEAS_OUTCOMM,
} SAFOUT_DEBUG2_SPNs;

enum safout_dfc_te : int
{
    SAFOUT_DFC_NO_FAULT,
    SAFOUT_DFC_SCB,
    SAFOUT_DFC_SCG,
    SAFOUT_DFC_OL,
    SAFOUT_DFC_OC
};

typedef struct
{
    safout_dfc_te dfcOut1;
    safout_dfc_te dfcOut2;
    safout_dfc_te dfcOut3;
    safout_dfc_te dfcOut4;
} safout_dfc_ts;

/* safout status flags */
#define SAFOUT_INITIALIZED_MASK 0x0001u /* initialization after startup */
#define SAFOUT_CONFIGURED_MASK 0x0002u  /* configured channel */
#define SAFOUT_ACTIVEOUT1_MASK 0x0010u  /* activated outputs */
#define SAFOUT_ACTIVEOUT2_MASK 0x0020u
#define SAFOUT_ACTIVEOUT3_MASK 0x0040u
#define SAFOUT_ACTIVEOUT4_MASK 0x0080u
#define SAFOUT_LOCKEDCHNL_MASK 0x0100u  /* locked channel */
#define SAFOUT_IDEVCMPHSLS_MASK 0x2000u /* maximal current deviation of the HS/LS compare is exceeded */
#define SAFOUT_IGNOREERROR_MASK 0x8000u /* ignore non critical faults  */

typedef struct
{
    bool initialized;
    bool configured;
    bool activeOut1;
    bool activeOut2;
    bool activeOut3;
    bool activeOut4;
    bool lockedChnl;
    bool iDevCmpHSLS;
    bool ignoreError;
} safout_status_ts;


can_isobus_info INFO_SAFOUT_DEBUG1 = {
    .instanceNum = 1,
    .boxNum = 3,
    .pgn = 0x0000,
    .prio = 3,
    .src = 0x00,
    .timeout = 500,
    .startTimeout = 5000,
    .lenMax = 8,
    .spns = {
        {.spnNum = 1, .byte = 1, .bit = 1, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 2, .byte = 1, .bit = 2, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 3, .byte = 1, .bit = 3, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 4, .byte = 1, .bit = 4, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 5, .byte = 1, .bit = 5, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 6, .byte = 1, .bit = 6, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 7, .byte = 1, .bit = 7, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 8, .byte = 1, .bit = 8, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 9, .byte = 2, .bit = 1, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 10, .byte = 2, .bit = 2, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 11, .byte = 3, .bit = 4, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 12, .byte = 4, .bit = 6, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 13, .byte = 5, .bit = 8, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 14, .byte = 7, .bit = 2, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT}} };

can_isobus_info INFO_SAFOUT_DEBUG2 = {
    .instanceNum = 1,
    .boxNum = 3,
    .pgn = 0x0000,
    .prio = 3,
    .src = 0x00,
    .timeout = 500,
    .startTimeout = 5000,
    .lenMax = 8,
    .spns = {
        {.spnNum = 15, .byte = 1, .bit = 1, .len = 3, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 16, .byte = 1, .bit = 4, .len = 3, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 17, .byte = 1, .bit = 7, .len = 3, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 18, .byte = 2, .bit = 2, .len = 3, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 19, .byte = 2, .bit = 5, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 20, .byte = 3, .bit = 7, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 21, .byte = 5, .bit = 1, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 22, .byte = 6, .bit = 3, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 23, .byte = 7, .bit = 5, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT}} };

typedef struct bds_safout_t {
    int chnl_u16 = 0;
    int status_b16 = 0;
    int setOutCo_u16 = 0;
    int setOut1_u16 = 0;
    int setOut2_u16 = 0;
    int setOut3_u16 = 0;
    int setOut4_u16 = 0;
    int iOutCo_u16 = 0;
    int iOut1_u16 = 0;
    int iOut2_u16 = 0;
    int iOut3_u16 = 0;
    int iOut4_u16 = 0;
    int iOutSum_u16 = 0;
    int diag_u32 = 0;
    int diag2_u32 = 0;
    int stDFC_u16 = 0;
    int stDFC_iDev_u16 = 0;
    int testSts_u16 = 0;
} bds_safout_ts;

typedef enum
{
    RBR_CAN_1_E = 0
    , RBR_CAN_2_E
    , RBR_CAN_3_E
    , RBR_CAN_4_E
    , RBR_CAN_NBR_E
} rbr_can_Chnl_te;

#define bds_can_Chnl_te rbr_can_Chnl_te

/* status flags for status_b16 */

#define PO_INITIALIZED_MASK              0x0001u    /* initialization after startup */
#define PO_CFG_POC_MASK           0x0002u    /* configuration of closed loop control */
#define PO_CFG_PERIOD_MASK            0x0004u    /* configuration of period */
#define PO_CFG_DITHER_MASK            0x0008u    /* configuration of dither */
#define PO_CFG_DEVIATION_MASK         0x0010u    /* configuration of deviation */
#define PO_CFG_PI_MASK                0x0020u    /* configuration of KpKi */
#define PO_LOCKED_SCB_MASK            0x0040u    /* locked by SCB...necessary for AOV outputs */
#define PO_LOCKED_SCG_MASK            0x0080u    /* locked by SCG...necessary for AOV outputs */
#define PO_LOCKED_OC_MASK             0x0100u    /* locked by over current */
#define PO_LOCKED_MASK               0x0200u    /* locked by error handler (faults are SCB, SCG, OL and OT) */
#define PO_LOCKED_MO_MASK             0x0400u    /* locked by Self-monitoring Error */
#define PO_LOCK_COU_TI_REACT_MASK 0x0800u    /* locked reactivation timer reload after a hot short circuit */
#define PO_TI_REACT_NOT_ELAPSTED_MASK 0x1000u    /* time for reactivation after a hot short circuit not elapsed */
#define PO_LINKED_MASK               0x2000u    /* linked at least with one other safout output */
#define PO_TST_PLS_DISABLE_MASK            0x4000u    /* Test pulses disable */
#define PO_DISABLE             0x8000u    /* output is disabled (avoid open load faults of unused outputs)*/


typedef struct
{
    uint16  status_b16;
    uint16 dc_u16;
    uint16 digits_u16;
    int iSet_u16;
    int iAct_u16;
    int iActRaw_mA_u16;
    uint16 stDFC_SCB_u16;
    uint16 stDFC_SCG_u16;
    uint16 stDFC_OL_u16;
    uint16 stDFC_OT_u16;
    uint16 stDFC_OC_u16;
    uint16 stDFC_OR_u16;
    bool unintendedSwitchONErr_l;
    bool lockedByINH_l;
} bds_out_po_ts;

typedef struct
{
    bool initialized;
    bool configured_POC;
    bool configured_Period;
    bool configured_Dither;
    bool configured_Deviation;
    bool configured_PI;
    bool locked_SCB;
    bool locked_SCG;
    bool locked_OC;
    bool locked;
    bool locked_MO;
    bool lockCouTiReact;
    bool TiReactNotElapsed;
    bool linked;
    bool testPulsesDisable;
    bool disable;
} propOut_status_ts;

typedef enum
{
    PO_DEBUG_INIT_OUT,
    PO_DEBUG_CFG_POC,
    PO_DEBUG_CFG_PERIOD,
    PO_DEBUG_CFG_DITHER,
    PO_DEBUG_CFG_DEVIATION,
    PO_DEBUG_CFG_PI,
    PO_DEBUG_LOCKED_SCB,
    PO_DEBUG_LOCKED_SCG,
    PO_DEBUG_LOCKED_OC,
    PO_DEBUG_LOCKED,
    PO_DEBUG_LOCKED_MO,
    PO_DEBUG_LOCK_COU_TI_REACT,
    PO_DEBUG_TI_REACT_NOT_ELAPSED,
    PO_DEBUG_LINKED,
    PO_DEBUG_TST_PLS_DIS,
    PO_DEBUG_DISABLED,
    PO_DEBUG_DFC_SCB,
    PO_DEBUG_DFC_SCG,
    PO_DEBUG_DFC_OL,
    PO_DEBUG_DFC_OT,
    PO_DEBUG_DFC_OC,
    PO_DEBUG_DFC_OR,
    PO_DEBUG_UNINTENDED_SWTCH_ON_ERR,
    PO_DEBUG_LOCKED_BY_INH,
    PO_DEBUG_I_SET,
    PO_DEBUG_I_ACT,
    PO_DEBUG_I_ACT_RAW_MA,
} PO_DEBUG_SPNs;

can_isobus_info INFO_PO_DEBUG = {
    .instanceNum = 1,
    .boxNum = 3,
    .pgn = 0x0000,
    .prio = 3,
    .src = 0x00,
    .timeout = 500,
    .startTimeout = 5000,
    .lenMax = 8,
    .spns = {
        {.spnNum = 0, .byte = 1, .bit = 1, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 1, .bit = 2, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 1, .bit = 3, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 1, .bit = 4, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 1, .bit = 5, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 1, .bit = 6, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 1, .bit = 7, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 1, .bit = 8, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 2, .bit = 1, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 2, .bit = 2, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 2, .bit = 3, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 2, .bit = 4, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 2, .bit = 5, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 2, .bit = 6, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 2, .bit = 7, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 2, .bit = 8, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 3, .bit = 1, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 3, .bit = 2, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 3, .bit = 3, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 3, .bit = 4, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 3, .bit = 5, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 3, .bit = 6, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 3, .bit = 7, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 3, .bit = 8, .len = 1, .scaling = 1, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 4, .bit = 1, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 5, .bit = 3, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT},
        {.spnNum = 0, .byte = 6, .bit = 5, .len = 10, .scaling = 4, .offset = 0, .varType = TYPE_INT}} };

boolean bds_dsm_DFC_ldf(uint16 Data)
{
    return ((((Data) & ((uint16)(1uL << 4))) != 0u));
}

/* proportional output status flags */
#define PROP_OUT_INITIALIZED_MASK 0x0001u          /* initialization after startup */
#define PROP_OUT_CFG_POC_MASK 0x0002u              /* configuration of closed loop control */
#define PROP_OUT_CFG_PERIOD_MASK 0x0004u           /* configuration of period */
#define PROP_OUT_CFG_DITHER_MASK 0x0008u           /* configuration of dither */
#define PROP_OUT_CFG_DEVIATION_MASK 0x0010u        /* configuration of deviation */
#define PROP_OUT_CFG_PI_MASK 0x0020u               /* configuration of KpKi */
#define PROP_OUT_LOCKED_SCB_MASK 0x0040u           /* locked by SCB...necessary for AOV outputs */
#define PROP_OUT_LOCKED_SCG_MASK 0x0080u           /* locked by SCG...necessary for AOV outputs */
#define PROP_OUT_LOCKED_OC_MASK 0x0100u            /* locked by over current */
#define PROP_OUT_LOCKED_MASK 0x0200u               /* locked by error handler (faults are SCB, SCG, OL and OT) */
#define PROP_OUT_LOCKED_MO_MASK 0x0400u            /* locked by Self-monitoring Error */
#define PROP_OUT_LOCK_COU_TI_REACT_MASK 0x0800u    /* locked reactivation timer reload after a hot short circuit */
#define PROP_OUT_TI_REACT_NOT_ELAPSED_MASK 0x1000u /* time for reactivation after a hot short circuit not elapsed */
#define PROP_OUT_LINKED_MASK 0x2000u               /* linked at least with one other safout output */
#define PROP_OUT_TEST_PULSE_DISABLE_MASK 0x4000u   /* Test pulses disable */
#define PROP_OUT_DISABLE_MASK 0x8000u              /* output is disabled (avoid open load faults of unused outputs)*/

void ExtractPropOutStatusBits(uint16 status, propOut_status_ts* statusStruct)
{
    statusStruct->initialized = (status & PROP_OUT_INITIALIZED_MASK) ? TRUE : FALSE;
    statusStruct->configured_POC = (status & PROP_OUT_CFG_POC_MASK) ? TRUE : FALSE;
    statusStruct->configured_Period = (status & PROP_OUT_CFG_PERIOD_MASK) ? TRUE : FALSE;
    statusStruct->configured_Dither = (status & PROP_OUT_CFG_DITHER_MASK) ? TRUE : FALSE;
    statusStruct->configured_Deviation = (status & PROP_OUT_CFG_DEVIATION_MASK) ? TRUE : FALSE;
    statusStruct->configured_PI = (status & PROP_OUT_CFG_PI_MASK) ? TRUE : FALSE;
    statusStruct->locked_SCB = (status & PROP_OUT_LOCKED_SCB_MASK) ? TRUE : FALSE;
    statusStruct->locked_SCG = (status & PROP_OUT_LOCKED_SCG_MASK) ? TRUE : FALSE;
    statusStruct->locked_OC = (status & PROP_OUT_LOCKED_OC_MASK) ? TRUE : FALSE;
    statusStruct->locked = (status & PROP_OUT_LOCKED_MASK) ? TRUE : FALSE;
    statusStruct->locked_MO = (status & PROP_OUT_LOCKED_MO_MASK) ? TRUE : FALSE;
    statusStruct->lockCouTiReact = (status & PROP_OUT_LOCK_COU_TI_REACT_MASK) ? TRUE : FALSE;
    statusStruct->TiReactNotElapsed = (status & PROP_OUT_TI_REACT_NOT_ELAPSED_MASK) ? TRUE : FALSE;
    statusStruct->linked = (status & PROP_OUT_LINKED_MASK) ? TRUE : FALSE;
    statusStruct->testPulsesDisable = (status & PROP_OUT_TEST_PULSE_DISABLE_MASK) ? TRUE : FALSE;
    statusStruct->disable = (status & PROP_OUT_DISABLE_MASK) ? TRUE : FALSE;
}

void Send_PropOut_DebugToCanTX(bds_out_po_ts outputPinStatus, bds_can_Chnl_te canChnl, uint32 canID1, uint8 canFormat)
{
    can_isobus_info tempInfoPODebug1 = INFO_PO_DEBUG;
    propOut_status_ts tempStatus_s;

    ExtractPropOutStatusBits(outputPinStatus.status_b16, &tempStatus_s);

    uint8 statSCB = bds_dsm_DFC_ldf(outputPinStatus.stDFC_SCB_u16);
    uint8 statSCG = bds_dsm_DFC_ldf(outputPinStatus.stDFC_SCG_u16);
    uint8 statOL = bds_dsm_DFC_ldf(outputPinStatus.stDFC_OL_u16);
    uint8 statOT = bds_dsm_DFC_ldf(outputPinStatus.stDFC_OT_u16);
    uint8 statOC = bds_dsm_DFC_ldf(outputPinStatus.stDFC_OC_u16);
    uint8 statOR = bds_dsm_DFC_ldf(outputPinStatus.stDFC_OR_u16);

    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_INIT_OUT, tempStatus_s.initialized);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_CFG_POC, tempStatus_s.configured_POC);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_CFG_PERIOD, tempStatus_s.configured_Period);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_CFG_DITHER, tempStatus_s.configured_Dither);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_CFG_DEVIATION, tempStatus_s.configured_Deviation);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_CFG_PI, tempStatus_s.configured_PI);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_LOCKED_SCB, tempStatus_s.locked_SCB);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_LOCKED_SCG, tempStatus_s.locked_SCG);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_LOCKED_OC, tempStatus_s.locked_OC);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_LOCKED, tempStatus_s.locked);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_LOCKED_MO, tempStatus_s.locked_MO);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_LOCK_COU_TI_REACT, tempStatus_s.lockCouTiReact);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_TI_REACT_NOT_ELAPSED, tempStatus_s.TiReactNotElapsed);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_LINKED, tempStatus_s.linked);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_TST_PLS_DIS, tempStatus_s.testPulsesDisable);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_DISABLED, tempStatus_s.disable);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_DFC_SCB, statSCB);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_DFC_SCG, statSCG);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_DFC_OL, statOL);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_DFC_OT, statOT);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_DFC_OC, statOC);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_DFC_OR, statOR);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_UNINTENDED_SWTCH_ON_ERR, outputPinStatus.unintendedSwitchONErr_l);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_LOCKED_BY_INH, outputPinStatus.lockedByINH_l);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_I_SET, outputPinStatus.iSet_u16);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_I_ACT, outputPinStatus.iAct_u16);
    ScaleOffsetAndInsertValueToCan(&tempInfoPODebug1, PO_DEBUG_I_ACT_RAW_MA, outputPinStatus.iActRaw_mA_u16);
    //bds_can_sendData(BDS_CAN_3_D, canID1, canFormat, 8, &tempInfoPODebug1.data);
    ImGui::Text("Debug:");
    ImGui::SameLine();
    for (int i = 0; i < 8; i++)
    {
        ImGui::Text("%02X ", tempInfoPODebug1.data[i]);
        ImGui::SameLine();
    }
    ImGui::Text("");
}

#define SAFOUT_SCB     0x0001u
#define SAFOUT_SCG     0x0002u
#define SAFOUT_OL     0x0004u
#define SAFOUT_OC     0x0008u

typedef enum
{
    DFC_OUT_COM,
    DFC_OUT_1,
    DFC_OUT_2,
    DFC_OUT_3,
    DFC_OUT_4,
    DFC_OUT_NUM,
} dfc_out_te;

void ExtractSafoutDFC(uint32 safoutDiag, safout_dfc_te* safoutDFCs_s)
{
    int i = 0;
    for (i; i < DFC_OUT_NUM; i++)
    {
        if (safoutDiag & SAFOUT_SCB << (4 * i))
            safoutDFCs_s[i] = SAFOUT_DFC_SCB;
        else if (safoutDiag & SAFOUT_SCG << (4 * i))
            safoutDFCs_s[i] = SAFOUT_DFC_SCG;
        else if (safoutDiag & SAFOUT_OL << (4 * i))
            safoutDFCs_s[i] = SAFOUT_DFC_OL;
        else if (safoutDiag & SAFOUT_OC << (4 * i))
            safoutDFCs_s[i] = SAFOUT_DFC_OC;
        else
            safoutDFCs_s[i] = SAFOUT_DFC_NO_FAULT;
    }
}

void ExtractSafoutStatusBits(uint16 status, safout_status_ts* statusStruct)
{
    statusStruct->initialized = (status & SAFOUT_INITIALIZED_MASK) ? TRUE : FALSE;
    statusStruct->configured = (status & SAFOUT_CONFIGURED_MASK) ? TRUE : FALSE;
    statusStruct->activeOut1 = (status & SAFOUT_ACTIVEOUT1_MASK) ? TRUE : FALSE;
    statusStruct->activeOut2 = (status & SAFOUT_ACTIVEOUT2_MASK) ? TRUE : FALSE;
    statusStruct->activeOut3 = (status & SAFOUT_ACTIVEOUT3_MASK) ? TRUE : FALSE;
    statusStruct->activeOut4 = (status & SAFOUT_ACTIVEOUT4_MASK) ? TRUE : FALSE;
    statusStruct->lockedChnl = (status & SAFOUT_LOCKEDCHNL_MASK) ? TRUE : FALSE;
    statusStruct->iDevCmpHSLS = (status & SAFOUT_IDEVCMPHSLS_MASK) ? TRUE : FALSE;
    statusStruct->ignoreError = (status & SAFOUT_IGNOREERROR_MASK) ? TRUE : FALSE;
}

void Send_SAFOUT_DebugToCanTX(bds_safout_ts outputPinStatus, bds_can_Chnl_te canChnl, uint32_t canID1, uint32_t canID2, uint8_t canFormat)
{
    can_isobus_info tempInfoSafoutDebug1 = INFO_SAFOUT_DEBUG1;
    can_isobus_info tempInfoSafoutDebug2 = INFO_SAFOUT_DEBUG2;
    safout_status_ts tempStatus_s;
    safout_dfc_te dfcOut[DFC_OUT_NUM];
    ImGui::Text("statusVar: %d", outputPinStatus.status_b16);
    ExtractSafoutStatusBits(outputPinStatus.status_b16, &tempStatus_s);
    ExtractSafoutDFC(outputPinStatus.diag_u32, dfcOut);

    ImGui::Text("initialized: %d", tempStatus_s.initialized);
    ImGui::Text("configured: %d", tempStatus_s.configured);
    ImGui::Text("activeOut1: %d", tempStatus_s.activeOut1);
    ImGui::Text("activeOut2: %d", tempStatus_s.activeOut2);
    ImGui::Text("activeOut3: %d", tempStatus_s.activeOut3);
    ImGui::Text("activeOut4: %d", tempStatus_s.activeOut4);
    ImGui::Text("lockedChnl: %d", tempStatus_s.lockedChnl);
    ImGui::Text("iDevCmpHSLS: %d", tempStatus_s.iDevCmpHSLS);
    ImGui::Text("ignoreError: %d", tempStatus_s.ignoreError);

    int tempScaledVal = 0;
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_INITIALIZED, tempStatus_s.initialized);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_CONFIGURED, tempStatus_s.configured);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_LOCKED_CHNL, tempStatus_s.lockedChnl);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_IDEV_CMP_HSLS, tempStatus_s.iDevCmpHSLS);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_IGNORE_ERR, tempStatus_s.ignoreError);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ACTIVE_OUT1, tempStatus_s.activeOut1);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ACTIVE_OUT2, tempStatus_s.activeOut2);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ACTIVE_OUT3, tempStatus_s.activeOut3);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ACTIVE_OUT4, tempStatus_s.activeOut4);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ISET_COM, outputPinStatus.setOutCo_u16);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ISET_OUT1, outputPinStatus.setOut1_u16);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ISET_OUT2, outputPinStatus.setOut2_u16);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ISET_OUT3, outputPinStatus.setOut3_u16);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ISET_OUT4, outputPinStatus.setOut4_u16);
    //bds_can_sendData(BDS_CAN_3_D, canID1, canFormat, 8, &tempInfoSafoutDebug1.data);
    ImGui::Text("Debug1:");
    ImGui::SameLine();
    for (int i = 0; i < 8; i++)
    {
        ImGui::Text("%02X ", tempInfoSafoutDebug1.data[i]);
        ImGui::SameLine();
    }
    ImGui::Text("");
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_DFC_OUT1, dfcOut[DFC_OUT_1]);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_DFC_OUT2, dfcOut[DFC_OUT_2]);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_DFC_OUT3, dfcOut[DFC_OUT_3]);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_DFC_OUT4, dfcOut[DFC_OUT_4]);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_IMEAS_OUT1, outputPinStatus.iOut1_u16);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_IMEAS_OUT2, outputPinStatus.iOut2_u16);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_IMEAS_OUT3, outputPinStatus.iOut3_u16);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_IMEAS_OUT4, outputPinStatus.iOut4_u16);
    ScaleOffsetAndInsertValueToCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_IMEAS_OUTCOMM, outputPinStatus.iOutCo_u16);
    //bds_can_sendData(BDS_CAN_3_D, canID2, canFormat, 8, &tempInfoSafoutDebug2.data);
    ImGui::Text("Debug2:");
    ImGui::SameLine();
    for (int i = 0; i < 8; i++)
    {
        ImGui::Text("%02X ", tempInfoSafoutDebug2.data[i]);
        ImGui::SameLine();
    }

    // test out extracting the values we inserted to make sure we did it right
    bds_safout_ts outputPinStatusTest;
    safout_status_ts statusTest;
    safout_dfc_ts dfcTest;
    uint64_t rawVal;
    int scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_INITIALIZED, &rawVal, &scaledVal);
    statusTest.initialized = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_CONFIGURED, &rawVal, &scaledVal);
    statusTest.configured = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_LOCKED_CHNL, &rawVal, &scaledVal);
    statusTest.lockedChnl = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_IDEV_CMP_HSLS, &rawVal, &scaledVal);
    statusTest.iDevCmpHSLS = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_IGNORE_ERR, &rawVal, &scaledVal);
    statusTest.ignoreError = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ACTIVE_OUT1, &rawVal, &scaledVal);
    statusTest.activeOut1 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ACTIVE_OUT2, &rawVal, &scaledVal);
    statusTest.activeOut2 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ACTIVE_OUT3, &rawVal, &scaledVal);
    statusTest.activeOut3 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ACTIVE_OUT4, &rawVal, &scaledVal);
    statusTest.activeOut4 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ISET_COM, &rawVal, &scaledVal);
    outputPinStatusTest.setOutCo_u16 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ISET_OUT1, &rawVal, &scaledVal);
    outputPinStatusTest.setOut1_u16 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ISET_OUT2, &rawVal, &scaledVal);
    outputPinStatusTest.setOut2_u16 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ISET_OUT3, &rawVal, &scaledVal);
    outputPinStatusTest.setOut3_u16 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug1, SAFOUT_DEBUG1_ISET_OUT4, &rawVal, &scaledVal);
    outputPinStatusTest.setOut4_u16 = scaledVal;

    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_DFC_OUT1, &rawVal, &scaledVal);
    dfcTest.dfcOut1 = (safout_dfc_te)scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_DFC_OUT2, &rawVal, &scaledVal);
    dfcTest.dfcOut2 = (safout_dfc_te)scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_DFC_OUT3, &rawVal, &scaledVal);
    dfcTest.dfcOut3 = (safout_dfc_te)scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_DFC_OUT4, &rawVal, &scaledVal);
    dfcTest.dfcOut4 = (safout_dfc_te)scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_IMEAS_OUT1, &rawVal, &scaledVal);
    outputPinStatusTest.iOut1_u16 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_IMEAS_OUT2, &rawVal, &scaledVal);
    outputPinStatusTest.iOut2_u16 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_IMEAS_OUT3, &rawVal, &scaledVal);
    outputPinStatusTest.iOut3_u16 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_IMEAS_OUT4, &rawVal, &scaledVal);
    outputPinStatusTest.iOut4_u16 = scaledVal;
    ExtractScaleAndOffsetValueFromCan(&tempInfoSafoutDebug2, SAFOUT_DEBUG2_IMEAS_OUTCOMM, &rawVal, &scaledVal);
    outputPinStatusTest.iOutCo_u16 = scaledVal;

    ImGui::Text("");
    ImGui::Text("statusTest.initialized: %d", statusTest.initialized);
    ImGui::Text("statusTest.configured: %d", statusTest.configured);
    ImGui::Text("statusTest.lockedChnl: %d", statusTest.lockedChnl);
    ImGui::Text("statusTest.iDevCmpHSLS: %d", statusTest.iDevCmpHSLS);
    ImGui::Text("statusTest.ignoreError: %d", statusTest.ignoreError);
    ImGui::Text("statusTest.activeOut1: %d", statusTest.activeOut1);
    ImGui::Text("statusTest.activeOut2: %d", statusTest.activeOut2);
    ImGui::Text("statusTest.activeOut3: %d", statusTest.activeOut3);
    ImGui::Text("statusTest.activeOut4: %d", statusTest.activeOut4);
    ImGui::Text("dfcTest.dfcOut1: %d", dfcTest.dfcOut1);
    ImGui::Text("dfcTest.dfcOut2: %d", dfcTest.dfcOut2);
    ImGui::Text("dfcTest.dfcOut3: %d", dfcTest.dfcOut3);
    ImGui::Text("dfcTest.dfcOut4: %d", dfcTest.dfcOut4);
    ImGui::Text("outputPinStatusTest.setOut1_u16: %d", outputPinStatusTest.setOut1_u16);
    ImGui::Text("outputPinStatusTest.setOut2_u16: %d", outputPinStatusTest.setOut2_u16);
    ImGui::Text("outputPinStatusTest.setOut3_u16: %d", outputPinStatusTest.setOut3_u16);
    ImGui::Text("outputPinStatusTest.setOut4_u16: %d", outputPinStatusTest.setOut4_u16);
    ImGui::Text("outputPinStatusTest.setOutCo_u16: %d", outputPinStatusTest.setOutCo_u16);
    ImGui::Text("outputPinStatusTest.iOut1_u16: %d", outputPinStatusTest.iOut1_u16);
    ImGui::Text("outputPinStatusTest.iOut2_u16: %d", outputPinStatusTest.iOut2_u16);
    ImGui::Text("outputPinStatusTest.iOut3_u16: %d", outputPinStatusTest.iOut3_u16);
    ImGui::Text("outputPinStatusTest.iOut4_u16: %d", outputPinStatusTest.iOut4_u16);
    ImGui::Text("outputPinStatusTest.iOutCo_u16: %d", outputPinStatusTest.iOutCo_u16);
}





void EditTransform(float* cameraView, float* cameraProjection, float* matrix, bool editTransformDecomposition);
void Perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar, float* m16);
void Frustum(float left, float right, float bottom, float top, float znear, float zfar, float* m16);
void LookAt(const float* eye, const float* at, const float* up, float* m16);
void Normalize(const float* a, float* r);
float Dot(const float* a, const float* b);
void Cross(const float* a, const float* b, float* r);

int WriteOutFile(std::string str, uint8_t* iv, const char* path, uint16_t crc, long len)
{
    int output = -1;
    std::ofstream outfile(path, std::ios::binary);
    //long aesLen = TaC.GetAESLength(len);
    if (outfile.is_open())
    {
        outfile << "/* WARNING: ALTERATIONS TO THIS FILE MAY RESULT IN CORRUPTION */\n/* EDIT AT YOUR OWN RISK */" << std::endl; // Write the encryption heading
        //ADDING IV TO FILE
        //outfile << encStrIvLine << std::endl;
        //for (int i = 0; i < AES_BLOCKLEN; i++)
        //{
        //    outfile << std::dec << std::to_string(iv[i]) << std::endl;// << std::endl;
        //}
        outfile << std::endl;
        // ADDING LENGTH OF JSON TO FILE
        //outfile << encStrLengthLine << std::endl;
        outfile << len << std::endl;
        // ADDING CRC OF (PLAINTEXT) JSON TO FILE
        //outfile << encStrCrcLine << std::endl;
        //outfile << crc << std::endl;
        // ADDING START OF ENCRYPTION STRING TO FILE
        //outfile << encStrTriggerLine << std::endl;
        // ADDING ENCRYPTED STRING TO FILE
        //std::string s = "";
        //std::string s(encryptedStr.begin(), encryptedStr.end());
        //long len = s.length();
        printf("string length: %d\n", len);
        outfile.write(str.c_str(), len);
        outfile.close();
        output = 0;
    }
    return output;
}

// END - 3D PROJECTION

#include "TimersAndCalculations.h" // fsc library

//START - PCAN - USB READING
#include "03_ManualRead.h"

#include <math.h>
//#define PI 3.14159265


//#define FSC_MAHONY // Pick what algo you want for IMU algorithm
#define FSC_MADGWICK // Pick what algo you want for IMU algorithm

#if defined(FSC_MAHONY)
#include "fsc_mahony.h"
fsc_mahony imu;
#elif defined(FSC_MADGWICK)
#include "fsc_madgwick.h"
fsc_madgwick imu;
#endif

//ManualRead CAN;


Quaternion imuC = { 1.0f, 0.0f, 0.0f, 0.0f };

Quaternion AverageQuaternion(struct Quaternion* cumulative, Quaternion newRotation, Quaternion firstRotation, int addAmount);
Quaternion NormalizeQuaternion(Quaternion q);
Quaternion InverseSignQuaternion(Quaternion q);
bool AreQuaternionsClose(Quaternion q1, Quaternion q2);
float qDot(Quaternion q1, Quaternion q2);


//Get an average (mean) from more than two quaternions (with two, slerp would be used).
//Note: this only works if all the quaternions are relatively close together.
//Usage:
//-Cumulative is an external Vector4 which holds all the added x y z and w components.
//-newRotation is the next rotation to be added to the average pool
//-firstRotation is the first quaternion of the array to be averaged
//-addAmount holds the total amount of quaternions which are currently added
//This function returns the current average quaternion
Quaternion AverageQuaternion(Quaternion cumulative[], Quaternion newRotation, Quaternion firstRotation, int addAmount)
{
    //float w = 0.0f;
    //float x = 0.0f;
    //float y = 0.0f;
    //float z = 0.0f;
    Quaternion avg = { 0, 0, 0, 0 };

    for (int i = 0; i < addAmount; i++)
    {
        if (!AreQuaternionsClose(cumulative[i], cumulative[0])) {

            cumulative[i] = InverseSignQuaternion(cumulative[i]);
        }
        avg.w += cumulative[i].w;
        avg.x += cumulative[i].x;
        avg.y += cumulative[i].y;
        avg.z += cumulative[i].z;
    }

    avg.w *= (1.0 / 10);
    avg.x *= (1.0 / 10);
    avg.y *= (1.0 / 10);
    avg.z *= (1.0 / 10);


    NormalizeQuaternion(avg);

    //Before we add the new rotation to the average (mean), we have to check whether the quaternion has to be inverted. Because
    //q and -q are the same rotation, but cannot be averaged, we have to make sure they are all the same.
    //if (!AreQuaternionsClose(newRotation, cumulative[0])) {

    //    newRotation = InverseSignQuaternion(newRotation);
    //}
    //Quaternion q;
    //Average the values
    //float addDet = 1.0f / (float)addAmount;
    //cumulative.w += newRotation.w;
    //q.w = cumulative.w * addDet;
    //cumulative.x += newRotation.x;
    //q.x = cumulative.x * addDet;
    //cumulative.y += newRotation.y;
    //q.y = cumulative.y * addDet;
    //cumulative.z += newRotation.z;
    //q.z = cumulative.z * addDet;

    //note: if speed is an issue, you can skip the normalization step
    //NormalizeQuaternion(avg);
    return avg;// NormalizeQuaternion(x, y, z, w);
}

Quaternion NormalizeQuaternion(Quaternion q)
{
    Quaternion newQ = q;
    float lengthD = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    newQ.w *= lengthD;
    newQ.x *= lengthD;
    newQ.y *= lengthD;
    newQ.z *= lengthD;

    return newQ;
}

//Changes the sign of the quaternion components. This is not the same as the inverse.
Quaternion InverseSignQuaternion(Quaternion q)
{
    Quaternion inverse = { -q.x, -q.y, -q.z, -q.w };
    return inverse;
}

//Returns true if the two input quaternions are close to each other. This can
//be used to check whether or not one of two quaternions which are supposed to
//be very similar but has its component signs reversed (q has the same rotation as
//-q)
bool AreQuaternionsClose(Quaternion q1, Quaternion q2)
{
    float dot = qDot(q1, q2);

    if (dot < 0.0f)
        return false;
    else
        return true;
}

float qDot(Quaternion q1, Quaternion q2)
{
    float dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
    return acos(dot);
}







const TPCANHandle PcanHandle1 = PCAN_USBBUS1;
//TPCANMsg CANMsg;
//TPCANTimestamp CANTimeStamp;


#define SENSOR_MM7_C_TELEGRAM_1_ID 0x374 // ACTUAL SENSOR C
#define SENSOR_MM7_C_TELEGRAM_2_ID 0x378
#define SENSOR_MM7_C_TELEGRAM_3_ID 0x37C

//#define SENSOR_MM7_C_TELEGRAM_1_ID 0x174 // JUST FOR TESTING - TODO - RM: REMOVE WHEN FINISHED TESTING
//#define SENSOR_MM7_C_TELEGRAM_2_ID 0x178
//#define SENSOR_MM7_C_TELEGRAM_3_ID 0x17C

//#define SENSOR_MM7_C_TELEGRAM_1_ID 0x274 // JUST FOR TESTING - TODO - RM: REMOVE WHEN FINISHED TESTING
//#define SENSOR_MM7_C_TELEGRAM_2_ID 0x278
//#define SENSOR_MM7_C_TELEGRAM_3_ID 0x27C

typedef struct // Same as Codesys SPN configuration, location of byte, bit inside that byte, and how long the value is
{
    uint8_t byte;
    uint8_t bit;
    uint8_t len;
} SPN_Config;

const SPN_Config SENSOR_MM7_TX1_YAW_RATE = { 0, 0, 16 };
const SPN_Config SENSOR_MM7_TX1_CLU_STAT = { 2, 0, 4 };
const SPN_Config SENSOR_MM7_TX1_YAW_RATE_STAT = { 2, 4, 4 };
const SPN_Config SENSOR_MM7_TX1_TEMP_RATE_Z = { 3, 0, 8 };
const SPN_Config SENSOR_MM7_TX1_AY = { 4, 0, 16 };
const SPN_Config SENSOR_MM7_TX1_MSG_CNT = { 6, 0, 4 };
const SPN_Config SENSOR_MM7_TX1_AY_STAT = { 6, 4, 4 };
const SPN_Config SENSOR_MM7_TX1_CRC = { 7, 0, 8 };

const SPN_Config SENSOR_MM7_TX2_ROLL_RATE = { 0, 0, 16 };
const SPN_Config SENSOR_MM7_TX2_CLU_STAT5 = { 2, 0, 4 };
const SPN_Config SENSOR_MM7_TX2_ROLL_RATE_STAT = { 2, 4, 4 };
const SPN_Config SENSOR_MM7_TX2_CLU_DIAG = { 3, 0, 8 };
const SPN_Config SENSOR_MM7_TX2_AX = { 4, 0, 16 };
const SPN_Config SENSOR_MM7_TX2_MSG_CNT = { 6, 0, 4 };
const SPN_Config SENSOR_MM7_TX2_AX_STAT = { 6, 4, 4 };
const SPN_Config SENSOR_MM7_TX2_CRC = { 7, 0, 8 };

const SPN_Config SENSOR_MM7_TX3_PITCH_RATE = { 0, 0, 16 };
const SPN_Config SENSOR_MM7_TX3_HW_INDEX = { 2, 0, 4 };
const SPN_Config SENSOR_MM7_TX3_PITCH_RATE_STAT = { 2, 4, 4 };
const SPN_Config SENSOR_MM7_TX3_RESERVED = { 3, 0, 8 };
const SPN_Config SENSOR_MM7_TX3_AZ = { 4, 0, 16 };
const SPN_Config SENSOR_MM7_TX3_MSG_CNT = { 6, 0, 4 };
const SPN_Config SENSOR_MM7_TX3_AZ_STAT = { 6, 4, 4 };
const SPN_Config SENSOR_MM7_TX3_CRC = { 7, 0, 8 };


float GetAngularRateFromMM7Raw(uint16_t input);
float GetAccelerationFromMM7Raw(uint16_t input);
int16_t GetTemperatureFromMM7Raw(uint8_t input);
int ExtractBoolFromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, bool* output, SPN_Config spnConfig);
int ExtractUint16FromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, uint16_t* output, SPN_Config spnConfig);
int ExtractUint8FromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, uint8_t* output, SPN_Config spnConfig);


float CalculateRollEuler(float x, float y, float z)
{
    float miu = 0.001;
    int sign = 1;
    if (z < 0)
        sign = -1;
    float yaw = (180.0 / PI) * atan2(-y, sign * sqrt(z * z + miu * x * x));
    {
        //if (z < 0)
        //{
        //    yaw = -yaw;
        //    if (yaw > 0)
        //        yaw = 180 - yaw;
        //    else
        //        yaw = -180 - yaw;
        //}
    }
    return yaw;
}

float CalculateYawEuler(float x, float y, float z)
{
    float miu = 0.001;
    int sign = 1;
    if (x < 0)
        sign = -1;
    return (180.0 / PI) * atan2(-y, sign * sqrt(x * x + miu * z * z));
}

float CalculatePitchEuler(float x, float y, float z)
{
    //float miu = 0.001;
    //int sign = 1;
    //if (x < 0)
    //    sign = -1;
    float pitch = -(180.0 / PI) * atan2(x, sqrt(y * y + z * z));
    {
        //if (z < 0)
        //{
        //    if (pitch > 0)
        //        pitch = 180 - pitch;
        //    else
        //        pitch = -180 - pitch;
        //}
    }
    return pitch;
}


/**
 * @brief millisecond-time-based function that calculates an output given a current value, a setpoint, value range,
 * and time parameters. The output will follow a maximum slope calculated by using the min/max values and the rampTime
 *
 * @param[in,out] *currentlVal - passes current value in and the calculated new value out.
 * @param[in] setpoint - Where `*currentVal` should be after ramp is complete
 * @param[in] min - minimum value that `*currentVal` can be
 * @param[in] max - maximum value that `*currentVal` can be
 * @param[in,out] *prevTime - pointer that keeps track of the last time we calculated a value
 * @param[in] rampTime - (ms) how long the ramp should take from `min` to `max`
 * @return
 */
double RampScale(double currentVal, double setpoint, double min, double max, uint64_t* prevTime, uint64_t rampTime, bool* finishedRamp)
{
    *finishedRamp = FALSE;
    double minVal = min;//(min < max) ? min : max;	//	FIND MIN/MAX VALUES - INCASE WE ARE INVERSELY SCALING
    double maxVal = max;// (min > max) ? min : max;
    uint64_t now = millis();
    uint64_t timeSince = now - *prevTime;
    *prevTime = now;
    if (rampTime == 0) // let's not divide by 0
    {
        rampTime = 1;
    }
    double maxIncrement = (((maxVal - minVal) / rampTime) * (double)timeSince); // FLOATS MAKE NEGATIVE NUMBERS SAD!
    //printf("maxIncrement: %f\n", maxIncrement);
    double increment = setpoint - currentVal;
    double absIncrement = increment;
    bool goUp;
    if (increment == 0)
    {
        *finishedRamp = TRUE; // if we aren't incrementing, we're done ramping
        return currentVal;
    }
    if (increment > 0) // have to increase to reach setpoint
    {
        goUp = TRUE;
    }
    else // have to decrease to reach setpoint
    {
        absIncrement = -increment;
        goUp = FALSE;
    }

    if (absIncrement < maxIncrement)
    {
        currentVal += increment;
    }
    else if (goUp)
    {
        currentVal += maxIncrement;
    }
    else
    {
        currentVal -= maxIncrement;
    }

    if (currentVal > maxVal) // limit output to between inMin and inMax
    {
        currentVal = maxVal;
        *finishedRamp = TRUE; // if we are clipping, we're done ramping
    }
    if (currentVal < minVal)
    {
        currentVal = minVal;
        *finishedRamp = TRUE;
    }

    if (currentVal == setpoint) // if we are exactly equal, we're done ramping
    {
        *finishedRamp = TRUE;
    }
    return currentVal;
}

#define CMD_NO_CHANGE 0
#define CMD_DECREMENT 1
#define CMD_INCREMENT 2

/**
 * @brief Increments/Decrements a given value by a certain amount up or down given a tri-state bool
 *      Also clips to a min/max value.
 *
 * @param[inout] value* pointer to value that will be incremented/decremented
 * @param[in] increDecrement tri-state input, 1 = decrement, 2 = increment, anything else = don't change
 * @param[in] incrementVal How much to increment/decrement
 * @param[in] minRange minimum value to clip `value` to if decremented below
 * @param[in] maxRange maximum value to clip `value` to if incremented above
 */
void IncrementValue(int16_t* value, uint8_t increDecrement, uint16_t incrementVal, int32_t minRange, int32_t maxRange)
{
    // sint32 output = input; // Set output to input so we don't send garbage if we don't change anything.
    if (increDecrement == CMD_DECREMENT)
    {
        *value -= incrementVal;
        if (*value < minRange) // clip value if outside maxRange
        {
            *value = minRange;
        }
    }
    else if (increDecrement == CMD_INCREMENT)
    {
        *value += incrementVal;
        if (*value > maxRange) // clip value if outside maxRange
        {
            *value = maxRange;
        }
    }
}


typedef struct
{
    // INPUTS
    int16_t joystickVal;
    int16_t posMin_mA;
    int16_t posMax_mA;
    int16_t negMin_mA;
    int16_t negMax_mA;
    int16_t rampStartTime;
    int16_t rampStopTime;
    int16_t deadband;
    int16_t controlModeMult;

    // PRIVATE/INTERNAL
    int8_t setpoint;        // calculated setpoint (ramped)
    int8_t currentSetpoint; // calculated end-of-ramp setpoint
    int8_t prevSetpoint;    // setpoint from previous loop
    uint64_t prevTime;
    bool finishedRamp;

    // OUTPUTS
    int16_t outputPositive_mA;
    int16_t outputNegative_mA;

} rampedOutput_ts;


#define COIL_CURRENT_0_MA 0

#define MIN_SINT8_SETPNT -127//(int8_t)-127
#define MAX_SINT8_SETPNT 127//(int8_t)127

#define NEG_THOUSAND -1000.0f
#define NEG_HUNDRED -100.0f
#define NEG_TWO -2.0f
#define ZERO_FLT 0.0f
#define ONE_HALF 0.5f
#define ONE_FLT 1.0f
#define THREE_HALFS 1.5f
#define TWO 2.0f
#define FOUR 4.0f
#define EIGHT 8.0f
#define POS_HUNDRED 100.0f
#define POS_THOUSAND 1000.0f

#define SCANRECO_LINK_STATUS_RUNNING 0x01
#define SCANRECO_WDT_TIMEOUT 1000
#define SCANRECO_INPUT_IDLE 127
#define SCANRECO_INPUT_MIN 0
#define SCANRECO_INPUT_MAX 254
#define SCANRECO_INPUT_BTN_NOT_PRESSED 0x00
#define SCANRECO_INPUT_BTN_PRESSED 0x01

void RampedOutput(rampedOutput_ts* rampedVals)
{
    // Sensitivity % F-R
    int16_t sensitivity_H = 127 + ((127 * rampedVals->deadband) / 100); // 10% GAP sensitivity max (127-140)
    int16_t sensitivity_L = 127 - ((127 * rampedVals->deadband) / 100); // 10% GAP sensitivity max (115-127)
    bool inNeutral = false;

    // Adjust for deadband and control aggression
    if (rampedVals->joystickVal > 127)//sensitivity_H)
    {
        rampedVals->currentSetpoint = (int8_t)scale(rampedVals->joystickVal, SCANRECO_INPUT_IDLE/*sensitivity_H*/, SCANRECO_INPUT_MAX, ZERO_FLT, (MAX_SINT8_SETPNT * rampedVals->controlModeMult) / POS_HUNDRED, TRUE);
    }
    else if (rampedVals->joystickVal < 127)//sensitivity_L)
    {
        rampedVals->currentSetpoint = (int8_t)scale(rampedVals->joystickVal, SCANRECO_INPUT_IDLE/*sensitivity_L*/, SCANRECO_INPUT_MIN, ZERO_FLT, (MIN_SINT8_SETPNT * rampedVals->controlModeMult) / POS_HUNDRED, TRUE);
    }
    else
    {
        inNeutral = true;
        rampedVals->currentSetpoint = 0;
    }

    // Ramp setpoint
    if (inNeutral) // stop ramp time
    {
        rampedVals->setpoint = RampScale(rampedVals->prevSetpoint, rampedVals->currentSetpoint, MIN_SINT8_SETPNT, MAX_SINT8_SETPNT, &rampedVals->prevTime, rampedVals->rampStopTime, &rampedVals->finishedRamp);
    }
    else // start ramp time
    {
        rampedVals->setpoint = RampScale(rampedVals->prevSetpoint, rampedVals->currentSetpoint, MIN_SINT8_SETPNT, MAX_SINT8_SETPNT, &rampedVals->prevTime, rampedVals->rampStartTime, &rampedVals->finishedRamp);
    }

    // since setpoint is an int, make sure our difference isn't too small to change, and if it is increment/decrement by 1
    if (rampedVals->setpoint == rampedVals->prevSetpoint && rampedVals->setpoint != rampedVals->currentSetpoint)
    {
        int16_t setpoint = rampedVals->setpoint;
        if (rampedVals->setpoint < rampedVals->currentSetpoint)
        {
            IncrementValue(&setpoint, CMD_INCREMENT, 1, MIN_SINT8_SETPNT, MAX_SINT8_SETPNT);
        }
        else if (rampedVals->setpoint > rampedVals->currentSetpoint)
        {
            IncrementValue(&setpoint, CMD_DECREMENT, 1, MIN_SINT8_SETPNT, MAX_SINT8_SETPNT);
        }
        rampedVals->setpoint = setpoint;
    }

    // Apply proportional current to coils based on setpoint
    if (rampedVals->setpoint > 0)
    {
        rampedVals->outputPositive_mA = (int16_t)scale(rampedVals->setpoint, 0, MAX_SINT8_SETPNT, rampedVals->posMin_mA, rampedVals->posMax_mA, TRUE);
        rampedVals->outputNegative_mA = COIL_CURRENT_0_MA;
    }
    else if (rampedVals->setpoint < 0)
    {
        rampedVals->outputPositive_mA = COIL_CURRENT_0_MA;
        rampedVals->outputNegative_mA = (int16_t)scale(rampedVals->setpoint, 0, MIN_SINT8_SETPNT, rampedVals->negMin_mA, rampedVals->negMax_mA, TRUE);
    }
    else
    {
        rampedVals->outputPositive_mA = COIL_CURRENT_0_MA;
        rampedVals->outputNegative_mA = COIL_CURRENT_0_MA;
    }

    // canDebugBuf6[0] = rampedVals->setpoint;
    // canDebugBuf6[1] = rampedVals->currentSetpoint;
    // canDebugBuf6[2] = rampedVals->prevSetpoint;
    // canDebugBuf6[3] = rampedVals->finishedRamp;3

    rampedVals->prevSetpoint = rampedVals->setpoint; // make sure to update prevSetpoint
}

typedef struct {
    double position;
    double velocity;
    double acceleration;
    double jerk;
    bool inDecelPhase;
    bool targetReached;
} MotionProfile_ts;

typedef struct {
    // Configuration
    double maxVelocity;
    double maxAcceleration;
    double maxJerk;
    double dt;
    double positionTolerance;
    double velocityTolerance;

    // State variables
    double currentPos;
    double currentVel;
    double currentAccel;
    double targetPos;

    // Deceleration detection
    bool decelStarted;
    double decelStartPos;
    double decelStartVel;
} TrajectoryCalculator;

// Helper function to clamp a value between min and max
double clamp(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// Helper function for absolute value
double fabs_custom(double x) {
    return (x < 0) ? -x : x;
}

// Initialize trajectory calculator
void trajectory_init(TrajectoryCalculator* calc, double maxVel, double maxAccel, double maxJerk, double timeStep) {
    calc->maxVelocity = maxVel;
    calc->maxAcceleration = maxAccel;
    calc->maxJerk = maxJerk;
    calc->dt = timeStep;
    calc->positionTolerance = 0.01;
    calc->velocityTolerance = 0.01;

    // Initialize state
    calc->currentPos = 0.0;
    calc->currentVel = 0.0;
    calc->currentAccel = 0.0;
    calc->targetPos = 0.0;
    calc->decelStarted = false;
    calc->decelStartPos = 0.0;
    calc->decelStartVel = 0.0;
}

// Reset trajectory calculator
void trajectory_reset(TrajectoryCalculator* calc) {
    calc->currentPos = 0.0;
    calc->currentVel = 0.0;
    calc->currentAccel = 0.0;
    calc->targetPos = 0.0;
    calc->decelStarted = false;
    calc->decelStartPos = 0.0;
    calc->decelStartVel = 0.0;
}

// Set target position
void trajectory_setTarget(TrajectoryCalculator* calc, double target) {
    // If target has changed significantly, reset deceleration state
    if (fabs_custom(target - calc->targetPos) > calc->positionTolerance) {
        calc->decelStarted = false;
        calc->decelStartPos = 0.0;
        calc->decelStartVel = 0.0;
    }
    calc->targetPos = target;
}

// Set current state
void trajectory_setCurrentState(TrajectoryCalculator* calc, double pos, double vel, double accel) {
    calc->currentPos = pos;
    calc->currentVel = vel;
    calc->currentAccel = accel;
}

// Calculate stopping distance given current velocity and acceleration
double trajectory_calculateStoppingDistance(TrajectoryCalculator* calc, double velocity, double acceleration) {
    if (fabs_custom(acceleration) < 1e-6) {
        // If no acceleration, use maximum deceleration
        return (velocity * velocity) / (2.0 * calc->maxAcceleration);
    }

    // Time to decelerate to zero velocity
    double timeToStop = fabs_custom(velocity) / calc->maxAcceleration;

    // Distance during deceleration: s = v*t - 0.5*a*t^2
    double stoppingDistance = fabs_custom(velocity) * timeToStop -
        0.5 * calc->maxAcceleration * timeToStop * timeToStop;

    return stoppingDistance;
}

// Predict if we'll overshoot with current trajectory
bool trajectory_willOvershoot(TrajectoryCalculator* calc, double lookaheadSteps) {
    double futurePos = calc->currentPos;
    double futureVel = calc->currentVel;
    double futureAccel = calc->currentAccel;

    // Simulate forward trajectory
    int maxSteps = (int)lookaheadSteps;
    for (int i = 0; i < maxSteps; i++) {
        // Update using current acceleration
        futurePos += futureVel * calc->dt + 0.5 * futureAccel * calc->dt * calc->dt;
        futureVel += futureAccel * calc->dt;

        // Check if we'll overshoot
        double remainingDistance = calc->targetPos - futurePos;
        double stoppingDistance = trajectory_calculateStoppingDistance(calc, futureVel, futureAccel);

        if ((remainingDistance > 0 && futureVel > 0 && stoppingDistance > remainingDistance) ||
            (remainingDistance < 0 && futureVel < 0 && stoppingDistance > fabs_custom(remainingDistance))) {
            return true;
        }
    }

    return false;
}



// Main trajectory calculation function
MotionProfile_ts trajectory_calculateNextPoint(TrajectoryCalculator* calc) {
    MotionProfile_ts profile;

    double error = calc->targetPos - calc->currentPos;
    double errorDirection = (error > 0) ? 1.0 : -1.0;

    // Check if we're close enough to target
    if (fabs_custom(error) < calc->positionTolerance && fabs_custom(calc->currentVel) < calc->velocityTolerance) {
        profile.position = calc->targetPos;
        profile.velocity = 0.0;
        profile.acceleration = 0.0;
        profile.jerk = 0.0;
        profile.inDecelPhase = false;
        profile.targetReached = true;
        return profile;
    }

    // Check if we're moving away from target - if so, reset decel state and accelerate toward target
    bool movingAwayFromTarget = ((error > 0 && calc->currentVel < -calc->velocityTolerance) ||
        (error < 0 && calc->currentVel > calc->velocityTolerance));

    if (movingAwayFromTarget) {
        calc->decelStarted = false;  // Reset decel state if moving wrong direction
    }

    // Determine if we should start decelerating
    bool shouldDecelerate = false;

    // Don't decelerate if we're moving away from target
    if (!movingAwayFromTarget) {
        // Calculate stopping distance - account for jerk limiting if enabled
        double stoppingDistance = 0.0;

        // If jerk limiting is enabled, we need more distance to change acceleration
        if (calc->maxJerk > 0) {
            // Determine the direction we need to decelerate
            double decelDirection = (calc->currentVel > 0) ? -1.0 : 1.0;
            double targetDecelAccel = decelDirection * calc->maxAcceleration;

            // How much do we need to change acceleration?
            double accelChangeNeeded = fabs_custom(targetDecelAccel - calc->currentAccel);

            // Time to change from current acceleration to max deceleration
            double timeToMaxDecel = accelChangeNeeded / calc->maxJerk;

            // Distance traveled during acceleration change (using current velocity and acceleration)
            double accelChangeDistance = calc->currentVel * timeToMaxDecel +
                0.5 * calc->currentAccel * timeToMaxDecel * timeToMaxDecel;

            // Velocity at end of acceleration change period
            double velAfterAccelChange = calc->currentVel + calc->currentAccel * timeToMaxDecel;

            // Additional stopping distance after reaching max deceleration
            double additionalStopDistance = 0.0;
            if (fabs_custom(velAfterAccelChange) > 1e-6) {
                additionalStopDistance = (velAfterAccelChange * velAfterAccelChange) / (2.0 * calc->maxAcceleration);
            }

            // Total stopping distance is the sum
            stoppingDistance = fabs_custom(accelChangeDistance) + additionalStopDistance;

            // Debug: only trigger if we're actually moving toward target and stopping distance is reasonable
            bool movingTowardTarget = ((error > 0 && calc->currentVel > 0) || (error < 0 && calc->currentVel < 0));

            if (movingTowardTarget && fabs_custom(error) <= stoppingDistance * 1.1) {
                shouldDecelerate = true;
            }
        }
        else {
            // No jerk limiting - use simple stopping distance calculation
            if (fabs_custom(calc->currentVel) > 1e-6) {
                stoppingDistance = (calc->currentVel * calc->currentVel) / (2.0 * calc->maxAcceleration);
            }

            double distanceToTarget = fabs_custom(error);

            // Main deceleration trigger: do we need to start decelerating to stop at target?
            if (distanceToTarget <= stoppingDistance) {
                shouldDecelerate = true;
            }

            // Additional check: if we're moving toward target and very close, start decelerating
            bool movingTowardTarget = ((error > 0 && calc->currentVel > 0) || (error < 0 && calc->currentVel < 0));
            if (movingTowardTarget && distanceToTarget <= stoppingDistance * 1.1) {
                shouldDecelerate = true;
            }
        }

        // Lookahead check: will we overshoot if we don't decelerate now?
        if (trajectory_willOvershoot(calc, 1.0)) {
            shouldDecelerate = true;
        }

        // Additional safety: don't start deceleration too early if we're still far from target
        if (fabs_custom(error) > 10.0 && fabs_custom(calc->currentVel) < 1.0) {
            shouldDecelerate = false;  // Don't decelerate when far away and moving slowly
        }
    }

    // Calculate desired acceleration
    double desiredAccel = 0.0;

    if (shouldDecelerate || calc->decelStarted) {
        calc->decelStarted = true;

        // Calculate precise deceleration to hit target exactly
        // Using: v_f^2 = v_i^2 + 2*a*d, solve for a to get v_f = 0 at target
        if (fabs_custom(error) > calc->positionTolerance) {
            // Calculate exact acceleration needed to reach target with zero velocity
            desiredAccel = -(calc->currentVel * calc->currentVel) / (2.0 * error);

            // Only limit if the calculated acceleration exceeds our capability
            if (fabs_custom(desiredAccel) > calc->maxAcceleration) {
                desiredAccel = (desiredAccel > 0) ? calc->maxAcceleration : -calc->maxAcceleration;
            }
        }
        else {
            // Very close to target, decelerate to zero quickly
            desiredAccel = -calc->currentVel / calc->dt;
            desiredAccel = clamp(desiredAccel, -calc->maxAcceleration, calc->maxAcceleration);
        }

        profile.inDecelPhase = true;
    }
    else {
        // Acceleration phase - use full acceleration toward target
        if (fabs_custom(calc->currentVel) < calc->maxVelocity) {
            desiredAccel = errorDirection * calc->maxAcceleration;
        }
        else {
            desiredAccel = 0.0; // Coast at max velocity
        }

        profile.inDecelPhase = false;
    }

    // Safety check: if we're moving away from target, always accelerate toward it
    if (movingAwayFromTarget) {
        desiredAccel = errorDirection * calc->maxAcceleration;
        profile.inDecelPhase = false;
        calc->decelStarted = false;
    }

    // Apply jerk limiting if specified
    if (calc->maxJerk > 0) {
        double jerkLimit = calc->maxJerk * calc->dt;
        double accelChange = desiredAccel - calc->currentAccel;

        if (fabs_custom(accelChange) > jerkLimit) {
            // Limit the acceleration change rate
            double limitedAccelChange = (accelChange > 0) ? jerkLimit : -jerkLimit;
            desiredAccel = calc->currentAccel + limitedAccelChange;

            // If we're in deceleration phase and jerk limiting is preventing us from decelerating fast enough,
            // be more aggressive to prevent overshoot
            if (profile.inDecelPhase && fabs_custom(error) < calc->positionTolerance * 10.0) {
                // Very close to target - use larger jerk limit for final approach
                double emergencyJerkLimit = calc->maxJerk * calc->dt * 3.0; // 3x normal jerk limit
                if (fabs_custom(accelChange) > emergencyJerkLimit) {
                    limitedAccelChange = (accelChange > 0) ? emergencyJerkLimit : -emergencyJerkLimit;
                    desiredAccel = calc->currentAccel + limitedAccelChange;
                }
            }
        }

        profile.jerk = (desiredAccel - calc->currentAccel) / calc->dt;
    }
    else {
        profile.jerk = 0.0;
    }

    // Update state using kinematic equations
    double newAccel = desiredAccel;
    double newVel = calc->currentVel + newAccel * calc->dt;
    double newPos = calc->currentPos + calc->currentVel * calc->dt + 0.5 * newAccel * calc->dt * calc->dt;

    // Velocity limiting
    if (fabs_custom(newVel) > calc->maxVelocity) {
        newVel = (newVel > 0) ? calc->maxVelocity : -calc->maxVelocity;
        newAccel = (newVel - calc->currentVel) / calc->dt;
    }

    // Final position check - only clamp if we actually overshoot
    if ((error > 0 && newPos > calc->targetPos) || (error < 0 && newPos < calc->targetPos)) {
        // We've overshot - clamp to target and stop
        newPos = calc->targetPos;
        newVel = 0.0;
        newAccel = 0.0;
    }

    // Update internal state
    calc->currentPos = newPos;
    calc->currentVel = newVel;
    calc->currentAccel = newAccel;

    // Fill profile
    profile.position = newPos;
    profile.velocity = newVel;
    profile.acceleration = newAccel;
    profile.targetReached = (fabs_custom(calc->targetPos - newPos) < calc->positionTolerance &&
        fabs_custom(newVel) < calc->velocityTolerance);

    return profile;
}

// Getter functions
double trajectory_getCurrentPosition(TrajectoryCalculator* calc) { return calc->currentPos; }
double trajectory_getCurrentVelocity(TrajectoryCalculator* calc) { return calc->currentVel; }
double trajectory_getCurrentAcceleration(TrajectoryCalculator* calc) { return calc->currentAccel; }
double trajectory_getTarget(TrajectoryCalculator* calc) { return calc->targetPos; }

// Setter functions for tuning
void trajectory_setPositionTolerance(TrajectoryCalculator* calc, double tolerance) {
    calc->positionTolerance = tolerance;
}

void trajectory_setVelocityTolerance(TrajectoryCalculator* calc, double tolerance) {
    calc->velocityTolerance = tolerance;
}

// Basic 2D vector structure
typedef struct {
    double x, y;
} Vec2;

// Motion constraints
typedef struct {
    double max_vel;          // Maximum velocity magnitude (units/sec)
    double max_accel;        // Maximum acceleration magnitude (units/sec^2)
    double position_tol;     // Position tolerance for "at target" (units)
    double velocity_tol;     // Velocity tolerance for "settled" (units/sec)
    double homing_distance;  // Distance at which to switch to homing mode (units)
    double homing_gain;      // Proportional gain for final homing (typically < 1.0)
} Constraints;

// Motion states
typedef enum {
    STATE_ACCELERATING,
    STATE_CRUISING,
    STATE_DECELERATING,
    STATE_HOMING,
    STATE_SETTLED
} MotionState_ts;

// Current state of the mover
typedef struct {
    Vec2 pos;
    Vec2 vel;
    MotionState_ts motion_state;
    int settled_count;       // Counter for debouncing settled state
} State;

// Calculate stopping distance given current speed and max deceleration
//double calculate_stopping_distance(double current_speed, double max_accel) {
//    // Using: d = v^2 / (2*a)
//    return (current_speed * current_speed) / (2.0 * max_accel);
//}

// Check if we're "at target"
//bool is_at_target(Vec2 pos, Vec2 target_pos, double tolerance) {
//    double dx = target_pos.x - pos.x;
//    double dy = target_pos.y - pos.y;
//    return (dx * dx + dy * dy) <= (tolerance * tolerance);
//}

// Check if velocity is near zero
//bool is_stopped(Vec2 vel, double tolerance) {
//    return (vel.x * vel.x + vel.y * vel.y) <= (tolerance * tolerance);
//}



// Update state based on acceleration command
void update_state(State* state, Vec2 accel_cmd, double dt) {
    // Update velocity
    state->vel.x += accel_cmd.x * dt;
    state->vel.y += accel_cmd.y * dt;

    // Update position
    state->pos.x += state->vel.x * dt;
    state->pos.y += state->vel.y * dt;
}

// Get state name for debugging
const char* get_state_name(MotionState_ts state) {
    switch (state) {
    case STATE_ACCELERATING: return "ACCELERATING";
    case STATE_CRUISING: return "CRUISING";
    case STATE_DECELERATING: return "DECELERATING";
    case STATE_HOMING: return "HOMING";
    case STATE_SETTLED: return "SETTLED";
    default: return "UNKNOWN";
    }
}




typedef struct
{
    // INPUTS
    double inputVal;
    double outputMin;
    double outputMax;
    double maxVelocity;
    double maxAccel;
    double maxDecel;
    double maxJerk;
    double dt;
    bool inNeutral;
    bool limitVelocity;
    bool limitAccel;
    bool limitJerk;

    float posMin;
    float posMax;
    float negMin;
    float negMax;
    float rampStartTime;
    float rampStopTime;
    float deadband;
    //float controlModeMult;

    // PRIVATE/INTERNAL
    double setpoint;        // calculated setpoint (ramped)
    double currentSetpoint; // calculated end-of-ramp setpoint
    double prevSetpoint;    // setpoint from previous loop
    double prevPrevSetpoint; // setpoint from 2 loops ago
    uint64_t prevTime;
    bool finishedRamp;

    // OUTPUTS
    float output;
    int16_t outputPositive_mA;
    int16_t outputNegative_mA;

} ramp_ts;

// Ramp() - one-dimensional function that will ramp an output based on an input setpoint and current setpoint
void Ramp(ramp_ts* ramp_s)
{
    static bool pause = false;
    // clip setpoint to min and max output values
    ramp_s->currentSetpoint = scale(ramp_s->inputVal, ramp_s->outputMin, ramp_s->outputMax, ramp_s->outputMin, ramp_s->outputMax, TRUE);

    // Ramp setpoint - jerk
    if (ramp_s->limitJerk)
    {
        //ramp_s->setpoint = ramp_s->currentSetpoint;
    }
    else
    {
        //ramp_s->setpoint = ramp_s->currentSetpoint;
    }

    // Ramp setpoint - acceleration
    if (ramp_s->limitAccel)// && ramp_s->maxAccel != 0.0f) // if maxAccel is 0, we definitely don't want to use it
    {
        static double unlimitedAccel;
        static double acceleration;
        static double currentVelocity;
        //static double error;
        static double stopPosError;
        static double maxP2;
        static double stopTimeGivenCurrentVelocity;
        static double accelDirection;
        static double stopDisplacmentAtMaxAccel;
        static double stopPosAtMaxAccel;
        static double accel;
        static double decelTrigger;
        static double setpointTolerance;
        static double futureStopPosAtMaxAccel;
        static double futureVelocity;
        static double futureStopPosError;
        static int errorDirection;
        static bool startDecel = false;
        static int stopPosErrorDirection;

        static bool oneShot = false;

        static TrajectoryCalculator calc;
        static bool initialized = false;
        if (ImGui::Button("Step Once"))
        {
            oneShot = true;
        }
        if (oneShot)
        {
            if (pause)
            {
                pause = false;
            }
            else
            {
                pause = true;
                oneShot = false;
            }
        }
        ImGui::Checkbox("pause", &pause);
        if (!pause)
        {
            // Calculate current velocity and acceleration from setpoint history
            double currentVelocity = (ramp_s->prevSetpoint - ramp_s->prevPrevSetpoint) / ramp_s->dt;
            double currentAccel = (ramp_s->currentSetpoint - 2.0 * ramp_s->prevSetpoint + ramp_s->prevPrevSetpoint) / (ramp_s->dt * ramp_s->dt);
            if (!initialized)
            {
                //trajectory_init(&calc, ramp_s->maxVelocity, ramp_s->maxAccel, ramp_s->maxJerk, ramp_s->dt);
                trajectory_init(&calc, ramp_s->maxVelocity, ramp_s->maxAccel, ramp_s->maxJerk, ramp_s->dt);
                initialized = true;
            }

            // Set target and current state
            //trajectory_setTarget(&calc, ramp_s->currentSetpoint);
            //trajectory_setCurrentState(&calc, ramp_s->prevSetpoint, currentVelocity, currentAccel);

            // Calculate next point
            //MotionProfile_ts profile = trajectory_calculateNextPoint(&calc);
            //ramp_s->setpoint = profile.position;



            //trajectory_init(&calc, 10000.0, 5000.0, 0000.0, 0.01);

            // Set target position
            trajectory_setTarget(&calc, ramp_s->currentSetpoint);
            //trajectory_setCurrentState(&calc, ramp_s->prevSetpoint, currentVelocity, currentAccel);
            calc.maxVelocity = ramp_s->maxVelocity;
            calc.maxAcceleration = ramp_s->maxAccel;
            calc.maxJerk = ramp_s->maxJerk;
            MotionProfile_ts profile = trajectory_calculateNextPoint(&calc);

            double error = calc.targetPos - profile.position;
            double stoppingDist = (profile.velocity * profile.velocity) / (2.0 * calc.maxAcceleration);

            ramp_s->setpoint = profile.position;
            ImGui::Text("Pos=%.3f, Vel=%.3f, Accel=%.3f, Decel=%d, Done=%d, Error=%.3f, StopDist=%.3f\n",
                profile.position, profile.velocity, profile.acceleration,
                profile.inDecelPhase, profile.targetReached, error, stoppingDist);
            /*
            float v1 = (ramp_s->prevSetpoint - ramp_s->prevPrevSetpoint) * ramp_s->dt;
            float v2 = (ramp_s->currentSetpoint - ramp_s->prevSetpoint) * ramp_s->dt;

            unlimitedAccel = (ramp_s->currentSetpoint - 2.0 * ramp_s->prevSetpoint + ramp_s->prevPrevSetpoint) / ramp_s->dt;

            currentVelocity = (ramp_s->prevSetpoint - ramp_s->prevPrevSetpoint) / ramp_s->dt;

            error = ramp_s->prevSetpoint - ramp_s->currentSetpoint;
            errorDirection = (error < 0.0) ? -1 : 1;

            // use Vf = Vi + a * t to find time it will take to slow down
            stopTimeGivenCurrentVelocity = (0 - currentVelocity) / ramp_s->maxAccel;
            accelDirection = 1.0;
            if (stopTimeGivenCurrentVelocity < 0.0) // negative times are a no-no
            {
                stopTimeGivenCurrentVelocity *= -1.0;
                accelDirection = -1.0;
            }

            // then use s(t) = v * t + 0.5 * a * t ^2 given that t to find out if that stopping position is within a certain tolerance to start slowing down
            //float stopPosGivenCurrentVelocity = (currentVelocity * stopTimeGivenCurrentVelocity) + 0.5f * (ramp_s->maxAccel * accelDirection) * stopTimeGivenCurrentVelocity * stopTimeGivenCurrentVelocity;
            stopDisplacmentAtMaxAccel = (0.0 - currentVelocity * currentVelocity) / (2.0 * ramp_s->maxAccel * accelDirection);
            stopPosAtMaxAccel = ramp_s->prevSetpoint + stopDisplacmentAtMaxAccel;


             //TODO - RM: IF WE CHECK IF WE WILL BE EQUAL OR OVERSHOOT SETPOINT, JUST CHECK CURRENT VELOCITY DIRECTION AND DECEL UNTIL WE REACH SETPOINT???
            stopPosError = stopPosAtMaxAccel - ramp_s->currentSetpoint;
            stopPosErrorDirection = (stopPosError < 0.0) ? -1 : 1;

            if (stopPosErrorDirection != errorDirection)
            {
                startDecel = true;
            }
            else
            {
                startDecel = false;
            }

            //startDecel = false;
            // If we're close enough to the setpoint, just start slowing down
            setpointTolerance = 0.1;
            decelTrigger = error + stopPosAtMaxAccel;
            bool decelTriggerSign = decelTrigger > 0.0f ? true : false;
            static bool prevDecelTriggerSign = decelTriggerSign;
            float toleranceFactor = 2.0 * abs(decelTrigger / 1.0); // adjust tolerance based on how far away we are from setpoint??
            //setpointTolerance *= toleranceFactor; // probably not a good idea - should probably figure out another way to make sure we latch our decel
            if (setpointTolerance < 1.0)
            {
                setpointTolerance = 1.0;
            }
            if ((stopPosError < setpointTolerance && stopPosError > -setpointTolerance))// || decelTriggerSign != prevDecelTriggerSign)
            {
                startDecel = true;
            }
            prevDecelTriggerSign = decelTrigger > 0.0f ? true : false;

            acceleration = ramp_s->maxAccel;
            static float accelDivisor = 1;
            ImGui::SliderFloat("acceldivisor", &accelDivisor, 0.1f, 100.0f, "%.1f");
            if (abs(unlimitedAccel) < ramp_s->maxAccel / accelDivisor)
            {
                acceleration = abs(unlimitedAccel);
                //maxP2 = ramp_s->currentSetpoint;
                //ramp_s->prevSetpoint = maxP2; // this fixes the slap back when we manually set our point below accel threshold
            }

            if (errorDirection > 0)
            {
                if (startDecel)
                {
                    acceleration = ramp_s->maxAccel;
                }
                else
                {
                    acceleration *= -1.0f;
                }
            }
            else
            {
                if (startDecel)
                {
                    acceleration = -ramp_s->maxAccel;
                }
            }


            if (abs(acceleration) < ramp_s->maxAccel / 20.0) // if our acceleration is less than 10x our max accel, just go straight to the point
            {
                maxP2 = ramp_s->currentSetpoint;
                ramp_s->prevSetpoint = ramp_s->currentSetpoint;
            }
            else
            {
                maxP2 = ramp_s->dt * ramp_s->dt * acceleration + 2.0 * ramp_s->prevSetpoint - ramp_s->prevPrevSetpoint;
            }

            // RECURRSSSSSIOOOONNNNN - check if our next point will shoot past given our current acceleration - if so, start deceling early
            // use Vf = Vi + a * t to find time it will take to slow down
            futureVelocity = (maxP2 - ramp_s->prevSetpoint) / ramp_s->dt;
            float stopTimeGivenFutureVelocity = (0 - futureVelocity) / ramp_s->maxAccel;
            float futureAccelDirection = 1.0;
            if (stopTimeGivenFutureVelocity < 0.0) // negative times are a no-no
            {
                stopTimeGivenFutureVelocity *= -1.0;
                futureAccelDirection = -1.0;
            }

            // then use s(t) = v * t + 0.5 * a * t ^2 given that t to find out if that stopping position is within a certain tolerance to start slowing down
            //float stopPosGivenCurrentVelocity = (currentVelocity * stopTimeGivenCurrentVelocity) + 0.5f * (ramp_s->maxAccel * accelDirection) * stopTimeGivenCurrentVelocity * stopTimeGivenCurrentVelocity;
            float futureStopDisplacementAtMaxAccel = (0.0 - futureVelocity * futureVelocity) / (2.0 * ramp_s->maxAccel * futureAccelDirection);
            futureStopPosAtMaxAccel = maxP2 + futureStopDisplacementAtMaxAccel;


            futureStopPosError = futureStopPosAtMaxAccel - ramp_s->currentSetpoint;
            int futureStopPosErrorDirection = (futureStopPosError < 0.0) ? -1 : 1;

            float futurePosError = maxP2 - ramp_s->currentSetpoint;
            float reqdVelocityToStopPerfectly = sqrt(-2.0 * ramp_s->maxAccel * futurePosError);

            if ((stopPosError > setpointTolerance || stopPosError < -setpointTolerance) && futureStopPosErrorDirection != errorDirection && !((maxP2 < ramp_s->currentSetpoint - 10.0f) && (maxP2 > ramp_s->currentSetpoint + 10.0f))) // We don't wanna change acceleration when close to setpoint
            {
                //acceleration *= -1.0f;
                acceleration = (reqdVelocityToStopPerfectly - currentVelocity) / ramp_s->dt;
            }


            maxP2 = ramp_s->dt * ramp_s->dt * acceleration + 2.0 * ramp_s->prevSetpoint - ramp_s->prevPrevSetpoint;

            // RECURRSSSSSIOOOONNNNN - check if our next point will shoot past given our current acceleration - if so, start deceling early
            // use Vf = Vi + a * t to find time it will take to slow down
            futureVelocity = (maxP2 - ramp_s->prevSetpoint) / ramp_s->dt;
            stopTimeGivenFutureVelocity = (0 - futureVelocity) / ramp_s->maxAccel;
            futureAccelDirection = 1.0;
            if (stopTimeGivenFutureVelocity < 0.0) // negative times are a no-no
            {
                stopTimeGivenFutureVelocity *= -1.0;
                futureAccelDirection = -1.0;
            }

            // then use s(t) = v * t + 0.5 * a * t ^2 given that t to find out if that stopping position is within a certain tolerance to start slowing down
            //float stopPosGivenCurrentVelocity = (currentVelocity * stopTimeGivenCurrentVelocity) + 0.5f * (ramp_s->maxAccel * accelDirection) * stopTimeGivenCurrentVelocity * stopTimeGivenCurrentVelocity;
            futureStopDisplacementAtMaxAccel = (0.0 - futureVelocity * futureVelocity) / (2.0 * ramp_s->maxAccel * futureAccelDirection);
            futureStopPosAtMaxAccel = maxP2 + futureStopDisplacementAtMaxAccel;


            ramp_s->setpoint = maxP2;
            accel = (ramp_s->setpoint - 2.0 * ramp_s->prevSetpoint + ramp_s->prevPrevSetpoint) / (ramp_s->dt * ramp_s->dt);
            */
        }
        ImGui::Text("ramp_s->currentSetpoint: %.2f", ramp_s->currentSetpoint);
        ImGui::Text("ramp_s->prevSetpoint: %.2f", ramp_s->prevSetpoint);
        //ImGui::Text("maxP2: %.2f", maxP2);
        //ImGui::Text("ramp_s->prevPrevSetpoint: %.2f", ramp_s->prevPrevSetpoint);
        //ImGui::Text("stopTimeGivenCurrentVelocity: %.2f", stopTimeGivenCurrentVelocity);
        //ImGui::Text("stopPosAtMaxAccel: %.2f", stopPosAtMaxAccel);
        //ImGui::Text("futureStopPosAtMaxAccel: %.2f", futureStopPosAtMaxAccel);
        //ImGui::Text("velocity: %.2f", currentVelocity);
        //ImGui::Text("futureVelocity: %.2f", futureVelocity);
        //ImGui::Text("unlimitedAccel: %.2f", unlimitedAccel);
        //ImGui::Text("Accel: %.2f", accel);
        //ImGui::Text("startDecel: %d", startDecel);
        //ImGui::Text("error: %.2f", error);
        //ImGui::Text("stopPosError: %.2f", stopPosError);
        //ImGui::Text("futureStopPosError: %.2f", futureStopPosError);
        //ImGui::Text("decelTrigger: %.2f", decelTrigger);
        //ImGui::Text("setpointTolerance: %.2f", setpointTolerance);
        //ImGui::Text("errorDirection: %d", errorDirection);
        //ImGui::Text("stopPosErrorDirection: %d", stopPosErrorDirection);
        //ImGui::Text("acceleration: %f", acceleration);
    }
    else
    {
        //ramp_s->setpoint = ramp_s->currentSetpoint;
    }

    // Ramp setpoint - velocity
    if (ramp_s->limitVelocity)
    {
        float setpoint = ramp_s->currentSetpoint; // use the currentSetpoint by default
        if ((ramp_s->limitAccel && ramp_s->maxAccel != 0.0f) || (ramp_s->limitJerk && ramp_s->maxJerk != 0.0f))
        {
            setpoint = ramp_s->setpoint; // if we already have calculated values from another limit, use the setpoint calculated from that
        }
        float error = setpoint - ramp_s->prevSetpoint;
        int errorDirection = (error < 0.0f) ? -1 : 1;
        error = fabs(error);
        if (error > (ramp_s->maxVelocity * ramp_s->dt))
        {
            if (errorDirection > 0)
            {
                ramp_s->setpoint = ramp_s->prevSetpoint + (ramp_s->maxVelocity * ramp_s->dt);
            }
            else
            {
                ramp_s->setpoint = ramp_s->prevSetpoint - (ramp_s->maxVelocity * ramp_s->dt);
            }
        }
        else
        {
            ramp_s->setpoint = setpoint;// ramp_s->currentSetpoint;
        }
    }
    else
    {
        //ramp_s->setpoint = ramp_s->currentSetpoint;
    }

    if (!pause)
    {
        ramp_s->output = ramp_s->setpoint;
        ramp_s->prevPrevSetpoint = ramp_s->prevSetpoint; // make sure to update prevSetpoint
        ramp_s->prevSetpoint = ramp_s->output; // make sure to update prevSetpoint
    }
}

typedef struct
{
    // INPUTS
    Vec2 inputVal;
    double maxVelocity;
    double maxAccel;
    double maxDecel;
    double maxJerk;
    double dt;
    bool inNeutral;
    bool limitVelocity;
    bool limitAccel;
    bool limitJerk;

    // OUTPUTS
    Vec2 output;

} ramp2D_ts;

// Ramp2D() - two-dimensional function that will ramp an output based on an input setpoint and current setpoint
void Ramp2D(ramp2D_ts* ramp2D_s)
{
    static bool pause = false;
    // clip setpoint to min and max output 
    //ramp_s->currentSetpoint = scale(ramp_s->inputVal, ramp_s->outputMin, ramp_s->outputMax, ramp_s->outputMin, ramp_s->outputMax, TRUE);

    // Ramp setpoint - jerk
    if (ramp2D_s->limitJerk)
    {
        //ramp_s->setpoint = ramp_s->currentSetpoint;
    }
    else
    {
        //ramp_s->setpoint = ramp_s->currentSetpoint;
    }

    // Ramp setpoint - acceleration
    if (ramp2D_s->limitAccel)// && ramp_s->maxAccel != 0.0f) // if maxAccel is 0, we definitely don't want to use it
    {
        static double unlimitedAccel;
        static double acceleration;
        static double currentVelocity;
        static double error;
        static double stopPosError;
        static double maxP2;
        static double stopTimeGivenCurrentVelocity;
        static double accelDirection;
        static double stopDisplacmentAtMaxAccel;
        static double stopPosAtMaxAccel;
        static double accel;
        static double decelTrigger;
        static double setpointTolerance;
        static double futureStopPosAtMaxAccel;
        static double futureVelocity;
        static double futureStopPosError;
        static int errorDirection;
        static bool startDecel = false;
        //static int stopPosErrorDirection;

        static bool oneShot = false;

        //static TrajectoryCalculator2D calc2D;
        static bool initialized = false;
        if (ImGui::Button("Step Once"))
        {
            oneShot = true;
        }
        if (oneShot)
        {
            if (pause)
            {
                pause = false;
            }
            else
            {
                pause = true;
                oneShot = false;
            }
        }
        ImGui::Checkbox("pause", &pause);
        if (!pause)
        {
            // Calculate current velocity and acceleration from setpoint history
            //double currentVelocity = (ramp_s->prevSetpoint - ramp_s->prevPrevSetpoint) / ramp_s->dt;
            //double currentAccel = (ramp_s->currentSetpoint - 2.0 * ramp_s->prevSetpoint + ramp_s->prevPrevSetpoint) / (ramp_s->dt * ramp_s->dt);
            static State current;
            static Vec2 target_vel;  // Stationary target
            if (!initialized)
            {
                // Initialize state
                current = {
                    .pos = {0.0, 0.0},
                    .vel = {0.0, 0.0},
                    .motion_state = STATE_ACCELERATING,
                    .settled_count = 0
                };

                target_vel = { 0.0, 0.0 };  // Stationary target
                initialized = true;
            }

            Vec2 target_pos = { ramp2D_s->inputVal.x, ramp2D_s->inputVal.y };
            // Set target position
            //trajectory2D_setTarget(&calc2D, ramp2D_s->inputVal);
            //trajectory_setCurrentState(&calc, ramp_s->prevSetpoint, currentVelocity, currentAccel);
            Constraints limits = {
                .max_vel = ramp2D_s->maxVelocity,
                .max_accel = ramp2D_s->maxAccel,
                .position_tol = 0.01,     // 0.01 units (1% of a unit)
                .velocity_tol = 0.05,     // 0.05 units/sec
                .homing_distance = 0.5,   // Switch to homing within 0.5 units
                .homing_gain = 2.0        // P-gain for homing
            };

            //Vec2 accel = compute_control(&current, target_pos, target_vel, &limits, ramp2D_s->dt);

            //// Update state
            //update_state(&current, accel, ramp2D_s->dt);
            ////MotionProfile2D profile = trajectory2D_calculateNextPoint(&calc2D);


            //ramp2D_s->output = Vec2(current.pos.x, current.pos.y);
            //double dist = sqrt(pow(target_pos.x - current.pos.x, 2) + pow(target_pos.y - current.pos.y, 2));
            //double speed = vec2_magnitude(current.vel);

            //ImGui::Text("pos: %.3f/%.3f, vel: %.3f/%.3f|%.3f, trgt: %.3f/%.3f, dist: %.3f\n",
            //    current.pos.x, current.pos.y,
            //    current.vel.x, current.vel.y, speed,
            //    target_pos.x, target_pos.y, dist);
        }
    }


    if (!pause)
    {
        //ramp_s->output = ramp_s->setpoint;
        //ramp_s->prevPrevSetpoint = ramp_s->prevSetpoint; // make sure to update prevSetpoint
        //ramp_s->prevSetpoint = ramp_s->output; // make sure to update prevSetpoint
    }
}

// Example usage function
void trajectory_example() {
    TrajectoryCalculator calc;

    // Initialize trajectory calculator
    // maxVel=10, maxAccel=5, maxJerk=2, dt=0.01 (10ms)
    trajectory_init(&calc, 10000.0, 1000.0, 5.0, 0.01);

    // Set target position
    trajectory_setTarget(&calc, 100.0);

    // Simulation loop
    //for (int i = 0; i < 2000; i++) {
        //MotionProfile_ts profile = trajectory_calculateNextPoint(&calc);

        // Print current state (or send to your motor controller)
        //printf("Step %d: Pos=%.3f, Vel=%.3f, Accel=%.3f, Decel=%d, Done=%d\n",
        //    i, profile.position, profile.velocity, profile.acceleration,
        //    profile.inDecelPhase, profile.targetReached);

        //if (profile.targetReached) {
        //    printf("Target reached!\n");
        //    break;
        //}
    //}
}
// Example usage function
void trajectory2D_example()
{
    // Initialize constraints
    Constraints limits = {
        .max_vel = 10.0,          // 10 units/sec
        .max_accel = 5.0,         // 5 units/sec^2
        .position_tol = 0.01,     // 0.01 units (1% of a unit)
        .velocity_tol = 0.05,     // 0.05 units/sec
        .homing_distance = 0.5,   // Switch to homing within 0.5 units
        .homing_gain = 2.0        // P-gain for homing
    };

    // Initialize state
    State current = {
        .pos = {0.0, 0.0},
        .vel = {0.0, 0.0},
        .motion_state = STATE_ACCELERATING,
        .settled_count = 0
    };

    // Target
    Vec2 target_pos = { 20.0, 15.0 };
    Vec2 target_vel = { 0.0, 0.0 };  // Stationary target

    // Simulation parameters
    double dt = 0.01;  // 10ms
    double time = 0.0;

    printf("Time,PosX,PosY,VelX,VelY,Speed,TargetX,TargetY,Distance,State\n");

    // Run simulation
    for (int i = 0; i < 800; i++) {  // 8 seconds
        // Get distance to target
        double dist = sqrt(pow(target_pos.x - current.pos.x, 2) +
            pow(target_pos.y - current.pos.y, 2));

        // Compute control
        //Vec2 accel = compute_control(&current, target_pos, target_vel, &limits, dt);

        // Update state
        //update_state(&current, accel, dt);

        // Print state (every 10th iteration = 100ms)
        if (i % 10 == 0) {
            //double speed = vec2_magnitude(current.vel);
            //printf("%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s\n",
            //    time, current.pos.x, current.pos.y,
            //    current.vel.x, current.vel.y, speed,
            //    target_pos.x, target_pos.y, dist,
            //    get_state_name(current.motion_state));
        }

        // Move target after 4 seconds to test tracking
        if (i == 400) {
            target_pos.x = 5.0;
            target_pos.y = 25.0;
            printf("# Target moved to (5, 25)\n");
        }

        // Small target movement at 6 seconds to test settling
        if (i == 600) {
            target_pos.x = 5.2;
            target_pos.y = 25.1;
            printf("# Target nudged to (5.2, 25.1)\n");
        }

        time += dt;
    }
}

float MM7_C_YAW_RATE;
float MM7_C_ROLL_RATE;
float MM7_C_PITCH_RATE;
float MM7_C_AX;
float MM7_C_AY;
float MM7_C_AZ;
int16_t MM7_C_TEMP;
//END - PCAN - USB READING

// Data
static ID3D11Device* g_pd3dDevice = nullptr;
static ID3D11DeviceContext* g_pd3dDeviceContext = nullptr;
static IDXGISwapChain* g_pSwapChain = nullptr;
static UINT                     g_ResizeWidth = 0, g_ResizeHeight = 0;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;

// Forward declarations of helper functions
bool CreateDeviceD3D(HWND hWnd);
void CleanupDeviceD3D();
void CreateRenderTarget();
void CleanupRenderTarget();
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Main code
int main(int, char**)
{
    std::cout << "RC40 Flasher Dependencies Test" << std::endl;
    std::cout << "================================" << std::endl;

    // Test OpenSSL
    std::cout << "Testing OpenSSL AES..." << std::endl;
    try {
        bool aesOk = RC40Flasher::AES128Security::testAES();
        std::cout << "OpenSSL AES: " << (aesOk ? "OK" : "FAILED") << std::endl;
    }
    catch (const std::exception& e) {
        std::cout << "OpenSSL AES: FAILED - " << e.what() << std::endl;
    }

    // Test PCAN
    std::cout << "Testing PCAN Basic..." << std::endl;
    TPCANStatus result = CAN_Initialize(PCAN_USBBUS1, PCAN_BAUD_250K, 0, 0, 0);
    if (result == PCAN_ERROR_OK) {
        std::cout << "PCAN Basic: OK" << std::endl;
        CAN_Uninitialize(PCAN_USBBUS1);
    }
    else {
        std::cout << "PCAN Basic: " << (result == PCAN_ERROR_NODRIVER ? "OK (No hardware)" : "FAILED")
            << " - Code: 0x" << std::hex << result << std::dec << std::endl;
    }

    std::cout << "\nSetup verification complete!" << std::endl;

    //testDifferentBitrates();
    //testCANCommunication();
    //testCANReceive();
    //testDifferentCANIDs_250K();
    //testRC40FunctionalAddressing();
    //testRC40PhysicalAddressing();
    //testRC40ExtendedAddressing();
    //testRC40ProgrammingSequence();
    //testRC40FunctionalSequence();
    //testRC40MixedAddressingSequence();
    //testRC40CorrectSequence();
    //testRC40SecurityAccess();
    //testRC40SecurityUnlock();
    //testRC40FastSecurityAccess();
    //testRC40ASW0HexFlashing();
    //testRC40ASW0AndDS0HexFlashing();
    //debugRC40EraseOperation();
    //testSequentialErase();
    //testCBFirstApproach();
    //debugASW0FlashingIssues();
    //diagnoseECUState();
    //testASW0AfterECUPowerCycle();
    //findWorkingPreparationSequence();
    //testOfficialToolSequence();
    //flashASW0AndDS0Split(); // THIS ACTUALLY FLASHES A CONTROLLER!
    //detectAvailablePCANChannels;
    //RC40Flasher::testChannelDetection();
    //RC40Flasher::testCBVersionCheck();
    RC40Flasher::testAutoDetectMultiFlash(); // THIS ACTUALLY FLASHES MULTIPLE CONTROLLERS CONCURRENTLY!
    //RC40Flasher::testProductionLineFlashing();

    while (true)
    {
        ; // do nothing while we debug CAN flashing
    }
    // Create application window
    //ImGui_ImplWin32_EnableDpiAwareness();
    WNDCLASSEXW wc = { sizeof(wc), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr, L"ImGui Example", nullptr };
    ::RegisterClassExW(&wc);
    HWND hwnd = ::CreateWindowW(wc.lpszClassName, L"Dear ImGui DirectX11 Example", WS_OVERLAPPEDWINDOW, 100, 100, 1280, 800, nullptr, nullptr, wc.hInstance, nullptr);

    // Initialize Direct3D
    if (!CreateDeviceD3D(hwnd))
    {
        CleanupDeviceD3D();
        ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
        return 1;
    }

    // Show the window
    ::ShowWindow(hwnd, SW_MAXIMIZE);
    ::UpdateWindow(hwnd);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

    // Load Fonts
    ImFontConfig config;
    config.OversampleH = 5;	//	THIS DOESN'T SEEM TO DO MUCH TO BE HONEST
    config.RasterizerMultiply = 1.5;	//	THIS MAKES TEXT APPEAR MORE CLEAR, GOOD ENOUGH FOR NOW...MAY WANT TO USE FREE TYPE IMPLEMENTATION INSTEAD
    config.PixelSnapH = true;
    //io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeuib.TTF", 24.0f, &config);	//	SEGOE IS WINDOWS 10/11 FONT
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    ImFont* bodyTextFont = io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.TTF", 20.0f, nullptr);
    ImFont* headerTextFont = io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeuib.TTF", 34.0f, nullptr);
    //IM_ASSERT(font != nullptr);

    // Our state
    bool show_pcan_window = false;
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    bool done = false;
    while (!done)
    {
        const uint32_t NUM_RATES = 1000;
        static float rollRates[NUM_RATES], rollRateAvg, rollRateMin, rollRateMax, pitchRates[NUM_RATES], pitchRateAvg, pitchRateMin, pitchRateMax, yawRates[NUM_RATES], yawRateAvg, yawRateMin, yawRateMax;

        // START - GRAB CAN DATA
        TPCANMsg CANMsg;
        TPCANTimestamp CANTimeStamp;

        // We execute the "Read" function of the PCANBasic   
        //printf("YAW RATE: %f\tPITCH RATE: %f\tROLL RATE: %f\t\n", MM7_C_YAW_RATE, MM7_C_PITCH_RATE, MM7_C_ROLL_RATE);
        //printf("AY: %f\tAX: %f\tAZ: %f\t\n", MM7_C_AY, MM7_C_AX, MM7_C_AZ);
        //printf("\n~~~~~~~~~~~~~~~~~~~~\n");
        TPCANStatus stsResult = /*CAN_Read(PcanHandle1, &CANMsg, &CANTimeStamp); //*/ PCAN_ERROR_QRCVEMPTY;
        if (stsResult != PCAN_ERROR_QRCVEMPTY)
        {
            switch (CANMsg.ID)
            {
            case SENSOR_MM7_C_TELEGRAM_1_ID:
            {
                uint16_t yawRate;
                uint16_t acceleration;
                uint8_t temperature;
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &yawRate, SENSOR_MM7_TX1_YAW_RATE);
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &acceleration, SENSOR_MM7_TX1_AY);
                ExtractUint8FromCanTelegram(CANMsg.DATA, 8, &temperature, SENSOR_MM7_TX1_TEMP_RATE_Z);

                MM7_C_YAW_RATE = GetAngularRateFromMM7Raw(yawRate);
                MM7_C_AY = GetAccelerationFromMM7Raw(acceleration);
                MM7_C_TEMP = GetTemperatureFromMM7Raw(temperature);
                break;
            }
            case SENSOR_MM7_C_TELEGRAM_2_ID:
            {
                uint16_t rollRate;
                uint16_t acceleration;
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &rollRate, SENSOR_MM7_TX2_ROLL_RATE);
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &acceleration, SENSOR_MM7_TX2_AX);

                MM7_C_ROLL_RATE = GetAngularRateFromMM7Raw(rollRate);
                MM7_C_AX = GetAccelerationFromMM7Raw(acceleration);
                break;
            }
            case SENSOR_MM7_C_TELEGRAM_3_ID:
            {

                uint16_t pitchRate;
                uint16_t acceleration;
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &pitchRate, SENSOR_MM7_TX3_PITCH_RATE);
                ExtractUint16FromCanTelegram(CANMsg.DATA, 8, &acceleration, SENSOR_MM7_TX3_AZ);
                MM7_C_PITCH_RATE = GetAngularRateFromMM7Raw(pitchRate);
                MM7_C_AZ = GetAccelerationFromMM7Raw(acceleration);
                break;
            }
            default:
            {
                break;
            }
            }
            imu.updateIMU(&imuC, MM7_C_PITCH_RATE, -MM7_C_ROLL_RATE, MM7_C_YAW_RATE, -MM7_C_AY, -MM7_C_AX, MM7_C_AZ, .0019); // TODO - NOTE THAT SAMPLE RATE IS TRIPLED (0.0033 INSTEAD OF 0.01) DUE TO GETTING 3 AXISES IN 1 MESSAGE EACH!!

            static uint32_t counter = 0;
            rollRates[counter] = MM7_C_ROLL_RATE;
            pitchRates[counter] = MM7_C_PITCH_RATE;
            yawRates[counter] = MM7_C_YAW_RATE;

            double rollRatesSum = 0, pitchRatesSum = 0, yawRatesSum = 0;
            for (int i = 0; i < NUM_RATES; i++)
            {
                rollRatesSum += rollRates[i];
                pitchRatesSum += pitchRates[i];
                yawRatesSum += yawRates[i];
            }
            rollRateAvg = rollRatesSum / NUM_RATES;
            pitchRateAvg = pitchRatesSum / NUM_RATES;
            yawRateAvg = yawRatesSum / NUM_RATES;

            counter++;
            if (counter > NUM_RATES - 1)
            {
                counter = 0;
            }



            if (MM7_C_ROLL_RATE < rollRateMin)
                rollRateMin = MM7_C_ROLL_RATE;
            if (MM7_C_ROLL_RATE > rollRateMax)
                rollRateMax = MM7_C_ROLL_RATE;
            if (MM7_C_PITCH_RATE < pitchRateMin)
                pitchRateMin = MM7_C_PITCH_RATE;
            if (MM7_C_PITCH_RATE > pitchRateMax)
                pitchRateMax = MM7_C_PITCH_RATE;
            if (MM7_C_YAW_RATE < yawRateMin)
                yawRateMin = MM7_C_YAW_RATE;
            if (MM7_C_YAW_RATE > yawRateMax)
                yawRateMax = MM7_C_YAW_RATE;

            //imu.updateIMU(0,0,0, MM7_C_ROLL_RATE, MM7_C_PITCH_RATE, MM7_C_YAW_RATE, MM7_C_AX, MM7_C_AY, MM7_C_AZ); // TODO - NOTE THAT SAMPLE RATE IS TRIPLED (0.0033 INSTEAD OF 0.01) DUE TO GETTING 3 AXISES IN 1 MESSAGE EACH!!
        }
        // END - GRAB CAN DATA

        // START - IMGUI LOOP
        static uint64_t prevRenderTime = 0;
        const uint64_t renderTimeout = FRAME_TIME;  //  16ms is about 60fps
        static bool renderFrame = true;
        if (Timer(prevRenderTime, renderTimeout, true))
        {
            renderFrame = !renderFrame;
        }
        // Poll and handle messages (inputs, window resize, etc.)
        // See the WndProc() function below for our to dispatch events to the Win32 backend.
        if (renderFrame)
        {
            MSG msg;
            while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
            {
                ::TranslateMessage(&msg);
                ::DispatchMessage(&msg);
                if (msg.message == WM_QUIT)
                    done = true;
            }
            if (done)
                break;

            // Handle window resize (we don't resize directly in the WM_SIZE handler)
            if (g_ResizeWidth != 0 && g_ResizeHeight != 0)
            {
                CleanupRenderTarget();
                g_pSwapChain->ResizeBuffers(0, g_ResizeWidth, g_ResizeHeight, DXGI_FORMAT_UNKNOWN, 0);
                g_ResizeWidth = g_ResizeHeight = 0;
                CreateRenderTarget();
            }

            // Start the Dear ImGui frame
            ImGui_ImplDX11_NewFrame();
            ImGui_ImplWin32_NewFrame();
            ImGui::NewFrame();

            struct StationState {
                int id;
                std::string status;
                float progress;
                std::vector<std::string> logs;
                bool isFlashing;
                bool lastFlashSuccess;
            };

            static std::vector<StationState> stations;

            StationState state;
            state.id = 1;
            state.status = "Idle";
            state.progress = 0.0f;
            state.isFlashing = false;
            state.lastFlashSuccess = false;
            stations.push_back(state);

            {
                ImGui::Begin("ECU Production Flasher");

                ImGui::Text("Stations: %zu", 6);
                ImGui::Separator();

                // Station grid
                for (auto& station : stations) {
                    ImGui::PushID(station.id);

                    // Station box
                    ImVec4 boxColor = station.isFlashing ? ImVec4(1.0f, 1.0f, 0.0f, 0.3f) :
                        station.lastFlashSuccess ? ImVec4(0.0f, 1.0f, 0.0f, 0.3f) :
                        ImVec4(0.5f, 0.5f, 0.5f, 0.3f);

                    ImGui::PushStyleColor(ImGuiCol_ChildBg, boxColor);
                    ImGui::BeginChild("Station", ImVec2(300, 200), true);

                    ImGui::Text("Station %d", station.id);
                    ImGui::Separator();
                    ImGui::Text("Status: %s", station.status.c_str());

                    if (station.isFlashing) {
                        ImGui::ProgressBar(station.progress, ImVec2(-1, 0));
                    }

                    ImGui::Spacing();
                    ImGui::Text("Recent logs:");
                    ImGui::BeginChild("Logs", ImVec2(0, 100), true);
                    for (const auto& log : station.logs) {
                        ImGui::TextWrapped("%s", log.c_str());
                    }
                    if (!station.logs.empty()) {
                        ImGui::SetScrollHereY(1.0f);
                    }
                    ImGui::EndChild();

                    ImGui::EndChild();
                    ImGui::PopStyleColor();

                    ImGui::PopID();

                    if ((station.id % 3) != 0) ImGui::SameLine(); // 3 columns
                }

                ImGui::Separator();

                // Control buttons
                static bool isRunning = false;
                if (!isRunning) {
                    if (ImGui::Button("Start Production Line")) {
                        //start();
                        isRunning = true;
                    }
                }
                else {
                    if (ImGui::Button("Stop Production Line")) {
                        //stop();
                        isRunning = false;
                    }
                }

                ImGui::End();
            }



            // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
            if (show_pcan_window)
                //ImGui::ShowDemoWindow(&show_pcan_window);
                ImGui::SetNextWindowSize(ImVec2(350, 350), ImGuiCond_Appearing);
            {
                ImGui::Begin("Dual Track", (bool*)true);
                ImGui::Button("JOYSTICK");
                if (ImGui::IsItemActive())
                    ImGui::GetForegroundDrawList()->AddLine(io.MouseClickedPos[0], io.MousePos, ImGui::GetColorU32(ImGuiCol_Button), 4.0f); // Draw a line between the button and the mouse cursor

                // Drag operations gets "unlocked" when the mouse has moved past a certain threshold
                // (the default threshold is stored in io.MouseDragThreshold). You can request a lower or higher
                // threshold using the second parameter of IsMouseDragging() and GetMouseDragDelta().
                ImVec2 value_raw = ImGui::GetMouseDragDelta(0, 0.0f);
                ImVec2 value_with_lock_threshold = ImGui::GetMouseDragDelta(0);
                ImVec2 mouse_delta = io.MouseDelta;
                //ImGui::Text("GetMouseDragDelta(0):");
                //ImGui::Text("  w/ default threshold: (%.1f, %.1f)", value_with_lock_threshold.x, value_with_lock_threshold.y);
                ImGui::Text("(x,y): (%.1f, %.1f)", value_raw.x, value_raw.y);
                int leftVal = value_raw.x;
                int rightVal = -value_raw.y; // negative so that Y-up is positive

                const float DEG_PER_RAD = 57.2958f;
                double radius = sqrt(leftVal * leftVal + rightVal * rightVal);
                if (radius > 100)
                    radius = 100;
                double theta = atan2(leftVal, rightVal) * DEG_PER_RAD; // X value first because we want up to be 0deg

                if (theta < -180.)
                    theta = -180.;
                if (theta > 180.)
                    theta = 180.;
                ImGui::Text("radius: %f", radius);
                ImGui::Text("theta: %f", theta);

                float leftTrack = 0;
                float rightTrack = 0;
                int index = 0;

                //radius = 100; // TODO - RM: JUST FOCUS ON THETA FOR RIGHT NOW
                //leftTrack = radius * (45 - theta * 90) / 45;
                //rightTrack = 100, 2 * radius + leftTrack;

                if (theta >= 0. && theta <= 90.) // FRONT-RIGHT
                {
                    index = 1;
                    leftTrack = radius;
                    rightTrack = radius * cos(2 * theta / DEG_PER_RAD);
                }
                else if (theta <= 0. && theta >= -90.) // FRONT-LEFT
                {
                    index = 2;
                    leftTrack = radius * cos(2 * theta / DEG_PER_RAD);
                    rightTrack = radius;
                }
                else if (theta >= 90. && theta <= 180.) // BACK-RIGHT
                {
                    index = 3;
                    leftTrack = -radius * cos(2 * theta / DEG_PER_RAD);
                    rightTrack = -radius;
                }
                else if (theta <= -90. && theta >= -180.) // BACK-LEFT
                {
                    index = 4;
                    leftTrack = -radius;
                    rightTrack = -radius * cos(2 * theta / DEG_PER_RAD);
                }

                float leftAbs = abs(leftTrack);
                float rightAbs = abs(rightTrack);
                float thetaCalc = 0;

                if (leftAbs < rightAbs)
                {
                    if (leftTrack < rightTrack)
                    {
                        thetaCalc = 1; // FRONT-LEFT
                        thetaCalc = -DEG_PER_RAD * (acos(leftTrack / rightTrack)) / 2.0;
                    }
                    else
                    {
                        thetaCalc = 2; // BACK-RIGHT
                        thetaCalc = 90.0 + DEG_PER_RAD * (acos(-leftTrack / rightTrack)) / 2.0;
                    }
                }
                else if (rightAbs < leftAbs)
                {
                    if (leftTrack < rightTrack)
                    {
                        thetaCalc = 3; // BACK-LEFT
                        thetaCalc = -90.0 - DEG_PER_RAD * (acos(-rightTrack / leftTrack)) / 2.0;
                    }
                    else
                    {
                        thetaCalc = 4; // FRONT-RIGHT
                        thetaCalc = DEG_PER_RAD * (acos(rightTrack / leftTrack)) / 2.0;
                    }
                }
                else
                {
                    if (leftTrack > 0.0)
                    {
                        if (rightTrack > 0.0)
                        {
                            thetaCalc = 0.; // foward
                        }
                        else
                        {
                            thetaCalc = 90.; // right
                        }
                    }
                    else
                    {
                        if (rightTrack > 0.0)
                        {
                            thetaCalc = -90.; // left
                        }
                        else
                        {
                            thetaCalc = 180; // back
                        }
                    }
                }
                //ImGui::Text("index: %d", index);
                ImGui::Text("thetaCalc: %f", thetaCalc);
                ImGui::Text("LEFT");
                ImGui::SameLine();
                ImGui::Text("   RIGHT");

                // IN ABSOLUTE VALUE
                ImGui::SameLine();
                ImGui::Text("LEFT");
                ImGui::SameLine();
                ImGui::Text("   RIGHT");

                ImGui::VSliderFloat("##int", ImVec2(50, 200), &leftTrack, -100.f, 100.f);
                ImGui::SameLine();
                ImGui::VSliderFloat("##int", ImVec2(50, 200), &rightTrack, -100.f, 100.f);
                ImGui::SameLine();
                ImGui::VSliderFloat("##int", ImVec2(50, 200), &leftAbs, -100.f, 100.f);
                ImGui::SameLine();
                ImGui::VSliderFloat("##int", ImVec2(50, 200), &rightAbs, -100.f, 100.f);
                ImGui::End();
            }

            // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
            {
                ImGui::Begin("PCAN USB");                          // Create a window called "Hello, world!" and append into it.TPCANMsg CANMsg;
                const int NUM_QUAT = 10;
                static bool firstLoop = true;
                static int quatCounter = 0;
                static Quaternion cumulative[NUM_QUAT];
                static Quaternion firstQuaternion;
                Quaternion currQuaternion;
                Quaternion averageQuaternion = { 0,0,0,0 };

                imu.getQuaternion(imuC, &currQuaternion.w, &currQuaternion.x, &currQuaternion.y, &currQuaternion.z);
                cumulative[quatCounter] = imuC;
                if (firstLoop)
                {
                    firstLoop = false;
                    firstQuaternion = imuC;// currQuaternion;
                }
                else
                {
                    averageQuaternion = AverageQuaternion(cumulative, imuC/*currQuaternion*/, firstQuaternion, NUM_QUAT);
                }
                quatCounter++;
                if (quatCounter >= NUM_QUAT)
                {
                    quatCounter = 0;
                }
                //imuC = averageQuaternion;
                imu.setQuaternion(&imuC, currQuaternion.w, currQuaternion.x, currQuaternion.y, currQuaternion.z);

                ImGui::PlotLines("rollRates", rollRates, IM_ARRAYSIZE(rollRates), 0, 0, -170.0f, 170.0f, ImVec2(0, 50.0f));
                ImGui::PlotLines("yawRates", yawRates, IM_ARRAYSIZE(rollRates), 0, 0, -170.0f, 170.0f, ImVec2(0, 50.0f));
                ImGui::PlotLines("pitchRates", pitchRates, IM_ARRAYSIZE(rollRates), 0, 0, -170.0f, 170.0f, ImVec2(0, 50.0f));

                ImGui::Text("YAW RATE: %3.f\tPITCH RATE: %3.f\tROLL RATE: %3.f\t\n", MM7_C_YAW_RATE, MM7_C_PITCH_RATE, MM7_C_ROLL_RATE);
                ImGui::Text("AX:\t%f\tAT:\t%f\tAZ:\t%f\t\n", MM7_C_AX, MM7_C_AY, MM7_C_AZ);
                float gravX, gravY, gravZ;
                imu.getGravityVector(&gravX, &gravY, &gravZ);
                ImGui::Text("gX:\t%f\tgY:\t%f\tgZ:\t%f\t\n", gravX, gravY, gravZ);
                float qW, qX, qY, qZ;
                imu.getQuaternion(imuC, &qW, &qX, &qY, &qZ);
                ImGui::Text("qW: %f\tqX: %f\tqY: %f\tqZ: %f\t\n", qW, qX, qY, qZ);
                ImGui::Text("cqW: %f\tcqX: %f\tcqY: %f\tcqZ: %f\t\n", averageQuaternion.w, averageQuaternion.x, averageQuaternion.y, averageQuaternion.z);
                //static float rollRateMin, rollRateMax, pitchRateMin, pitchRateMax, yawRateMin, yawRateMax;

                ImGui::Text("rollRateMaxDiff: %f\n", rollRateMax - rollRateMin);
                ImGui::Text("pitchRateMaxDiff: %f\n", pitchRateMax - pitchRateMin);
                ImGui::Text("yawRateMaxDiff: %f\n", yawRateMax - yawRateMin);

                ImGui::Text("rollRateMin: %f\n", rollRateMin);
                ImGui::Text("pitchRateMin: %f\n", pitchRateMin);
                ImGui::Text("yawRateMin: %f\n", yawRateMin);

                ImGui::Text("rollRateMax: %f\n", rollRateMax);
                ImGui::Text("pitchRateMax: %f\n", pitchRateMax);
                ImGui::Text("yawRateMax: %f\n", yawRateMax);

                ImGui::Text("rollRateAvg: %f\n", rollRateAvg);
                ImGui::Text("pitchRateAvg: %f\n", pitchRateAvg);
                ImGui::Text("yawRateAvg: %f\n", yawRateAvg);
                // START - 3D DISPLAY

                int matId = 0;
                ImGuizmo::SetID(matId);
                // start - disregard this stuff?
                float matrixTranslation[3], matrixRotation[3], matrixScale[3];
                ImGuizmo::DecomposeMatrixToComponents(objectMatrix[matId], matrixTranslation, matrixRotation, matrixScale);
                ImGui::InputFloat3("Tr", matrixTranslation);
                ImGui::InputFloat3("Rt", matrixRotation);
                ImGui::InputFloat3("Sc", matrixScale);
                ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, objectMatrix[matId]);
                // end - disregard this stuff?
                //imu.updateIMU(MM7_C_YAW_RATE, MM7_C_PITCH_RATE, MM7_C_ROLL_RATE, MM7_C_AX, MM7_C_AY, MM7_C_AZ, 0.01); // this is done when we get data
                float yaw;// = imu.getYaw();
                float roll;// = imu.getRoll();
                float pitch;// = imu.getPitch();
                Quaternion grav;// = imu.getPitch();

                imu.computeIMUAngles(imuC, &roll, &pitch, &yaw, &grav);

                //if (MM7_C_AZ < 0)
                //{
                //    if (pitch < 0)
                //        pitch = -180 - pitch;
                //    else
                //        pitch = 180 - pitch;
                //}

                //float yaw = CalculateYawEuler(MM7_C_AX, MM7_C_AY, MM7_C_AZ);
                //float roll = CalculateRollEuler(MM7_C_AX, MM7_C_AY, MM7_C_AZ);
                //float pitch = CalculatePitchEuler(MM7_C_AX, MM7_C_AY, MM7_C_AZ);

                ImGui::Text("gX:\t%f\tgY:\t%f\tgZ:\t%f\t\n", grav.x, grav.y, grav.z);

                ImGui::Text("YAW: %f", yaw);
                ImGui::Text("ROLL: %f", roll);
                ImGui::Text("PITCH: %f", pitch);
                matrixRotation[0] = roll;
                matrixRotation[1] = yaw;
                matrixRotation[2] = pitch;

                ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, objectMatrix[matId]);
                static float cameraView[16] =
                { 1.f, 0.f, 0.f, 0.f,
                  0.f, 1.f, 0.f, 0.f,
                  0.f, 0.f, 1.f, 0.f,
                  0.f, 0.f, 0.f, 1.f };

                static int lastUsing = 0;

                static float cameraProjection[16];

                static float fov = 27.f;
                Perspective(fov, io.DisplaySize.x / io.DisplaySize.y, 0.1f, 100.f, cameraProjection);

                static float camYAngle = 165.f / 180.f * 3.14159f;
                static float camXAngle = 32.f / 180.f * 3.14159f;

                bool viewDirty = false;
                static bool firstFrame = true;


                viewDirty |= ImGui::SliderFloat("Distance", &camDistance, 1.f, 10.f);
                if (viewDirty || firstFrame)
                {
                    float eye[] = { cosf(camYAngle) * cosf(camXAngle) * camDistance, sinf(camXAngle) * camDistance, sinf(camYAngle) * cosf(camXAngle) * camDistance };
                    float at[] = { 0.f, 0.f, 0.f };
                    float up[] = { 0.f, 1.f, 0.f };
                    LookAt(eye, at, up, cameraView);
                    firstFrame = false;
                }

                EditTransform(cameraView, cameraProjection, objectMatrix[matId], lastUsing == matId);
                if (ImGuizmo::IsUsing())
                {
                    lastUsing = matId;
                }
                // END - 3D DISPLAY

                ImGui::End();
            }

            static bool show_PID_window = true;
            // 3. Show a PID loop window
            if (show_PID_window)
            {
                ImGui::SetNextWindowSize(ImVec2(500, 500), ImGuiCond_Appearing);
                ImGui::Begin("PID Window", &show_PID_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    if (ImGui::Button("Convert DBC to JSON"))
                    {
                        // RUNS DBCC.EXE TO CONVERT OUR `input.dbc` INTO A .JSON FILE!
                        LPCSTR open = "runas";
                        LPCSTR executable = "dbcc.exe";
                        LPCSTR parameters = "-j input.dbc";
                        LPCSTR path = "dbcc\\";
                        //LPCSTR filepath = "dbcc\\putty.exe";
                        printf("%d\n", ShellExecuteA(NULL, open, executable, parameters, path, SW_SHOWNORMAL)); // RETURNS 42 IF GOOD!
                    }
                    if (ImGui::Button("Open Putty!"))
                    {
                        // RUNS DBCC.EXE TO CONVERT OUR `input.dbc` INTO A .JSON FILE!
                        LPCSTR open = "runas";
                        LPCSTR executable = "dbcc\\putty.exe";
                        //LPCSTR path = "dbcc\\";
                        //LPCSTR filepath = "dbcc\\putty.exe";
                        printf("%d\n", ShellExecuteA(NULL, open, executable, NULL, NULL, SW_SHOWNORMAL)); // RETURNS 42 IF GOOD!
                    }
                    if (ImGui::Button("Create an arbitrary C-File!"))
                    {
                        std::string string = "test_check_check";
                        uint8_t iv[] = { 1,2 };
                        const char path[] = "test.c";
                        uint16_t crc = 1234;
                        long len = string.length();
                        WriteOutFile(string, iv, path, crc, len);
                    }
                    ImGui::Text("Hello from PID window!");
                    static int joystickVal = 0;
                    ImGui::SliderInt("Joystick Val", &joystickVal, SCANRECO_INPUT_MIN, SCANRECO_INPUT_MAX);
                    static int startRampTime = 0;
                    ImGui::SliderInt("Start Ramp Time (ms)", &startRampTime, 0, 5000);
                    static int stopRampTime = 0;
                    ImGui::SliderInt("Stop Ramp Time (ms)", &stopRampTime, 0, 5000);

                    static rampedOutput_ts aux1Vals;
                    aux1Vals.controlModeMult = 100; // percentage to ramp to
                    aux1Vals.deadband = 0;
                    aux1Vals.joystickVal = joystickVal;
                    aux1Vals.posMin_mA = 800;  // TODO - RM: MAKE THIS A PARAMETER
                    aux1Vals.posMax_mA = 1600; // TODO - RM: MAKE THIS A PARAMETER
                    aux1Vals.negMin_mA = 800;  // TODO - RM: MAKE THIS A PARAMETER
                    aux1Vals.negMax_mA = 1600; // TODO - RM: MAKE THIS A PARAMETER
                    aux1Vals.rampStartTime = startRampTime;
                    aux1Vals.rampStopTime = stopRampTime;
                    RampedOutput(&aux1Vals);
                    int outputFwd;
                    int outputRev;
                    outputFwd = aux1Vals.outputPositive_mA;
                    outputRev = aux1Vals.outputNegative_mA;

                    if (outputFwd)
                    {
                        ImGui::PushStyleColor(ImGuiCol_PlotLines, (ImVec4)ImColor(1.f, 0.f, 0.f));
                    }
                    else if (outputRev)
                    {
                        ImGui::PushStyleColor(ImGuiCol_PlotLines, (ImVec4)ImColor(0.f, 1.f, 0.f));
                    }
                    else
                    {
                        ImGui::PushStyleColor(ImGuiCol_PlotLines, (ImVec4)ImColor(1.f, 1.f, 1.f));
                    }

                    const int NUM_VALUES = 90;
                    static float values[NUM_VALUES] = {};
                    static int values_offset = 0;
                    float average = 0.0f;
                    static int counter = 0;

                    static uint64_t prevUpdateVal = 0;
                    const uint64_t updateValTimeout = 50;
                    if (io.MousePos.y > -10000 && io.MousePos.y < 10000)
                    {
                        if (Timer(prevUpdateVal, updateValTimeout, true))
                        {
                            if (outputFwd)
                            {
                                values[counter] = outputFwd;
                            }
                            else
                            {
                                values[counter] = outputRev;
                            }
                            values[counter] = aux1Vals.setpoint;
                            if (counter < NUM_VALUES - 1)
                            {
                                counter++;
                            }
                            // push everything back
                            if (counter == NUM_VALUES - 1)
                            {
                                for (int i = 1; i < NUM_VALUES; i++)
                                {
                                    values[i - 1] = values[i];
                                }
                            }
                        }
                    }
                    for (int n = 0; n < IM_ARRAYSIZE(values); n++)
                        average += values[n];
                    average /= (float)IM_ARRAYSIZE(values);
                    char overlay[32];
                    sprintf(overlay, "val %.0f", values[counter]);

                    if (ImGui::BeginTable("table_nested1", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable))
                    {
                        ImGui::TableSetupColumn("Graph");
                        ImGui::TableSetupColumn("Values");
                        ImGui::TableHeadersRow();

                        ImGui::TableNextColumn();

                        float minVal = -127;// 0;// getMin(values, NUM_VALUES);
                        float maxVal = 127;// aux1Vals.posMax_mA;// getMax(values, NUM_VALUES);
                        const float PLOT_HEIGHT = 160.0f;
                        ImGui::PlotLines("##Lines", values, IM_ARRAYSIZE(values), values_offset, overlay, minVal, maxVal, ImVec2(0, PLOT_HEIGHT));


                        //ImGui::Text("A0 Row 0");
                        ImGui::TableNextColumn();
                        //ImGui::Text("A1 Row 0");
                        {
                            float rows_height = 80 / 3;
                            if (ImGui::BeginTable("table_nested2", 1, ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable))
                            {
                                ImGui::TableSetupColumn("B0");

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                ImGui::Text("max: %d", (int)maxVal);

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                if (counter < NUM_VALUES - 1)
                                {
                                    ImGui::Text("val: %d", (int)values[counter - 1]);
                                }
                                else
                                {
                                    ImGui::Text("val: %d", (int)values[counter]);
                                }

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                ImGui::Text("min: %d", (int)minVal);

                                ImGui::EndTable();
                            }
                        }
                        //ImGui::TableNextColumn(); ImGui::Text("A0 Row 1");
                        //ImGui::TableNextColumn(); ImGui::Text("A1 Row 1");
                        ImGui::EndTable();
                    }
                    ImGui::PopStyleColor();
                    //ImGui::PlotLines("##Lines", values, IM_ARRAYSIZE(values), values_offset, overlay, getMin(values, NUM_VALUES), getMax(values, NUM_VALUES), ImVec2(0, 80.0f));
                    //ImGui::SameLine();
                    ImGui::Text("max");
                }
                if (ImGui::Button("Close Me"))
                    show_PID_window = false;
                ImGui::End();
            }


            static bool show_ramp_window = true;
            // 3. Show a PID loop window
            if (show_ramp_window)
            {
                ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiCond_Always);
                ImGui::Begin("Curve Window", &show_ramp_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    static bool limitVelocity = false;
                    static bool limitAcceleration = true;
                    static bool limitJerk = false;

                    ImGui::Text("Hello from Curve window!");
                    ImGui::Checkbox("limit Velocity", &limitVelocity);
                    ImGui::SameLine();
                    ImGui::Checkbox("limit Acceleration", &limitAcceleration);
                    ImGui::SameLine();
                    ImGui::Checkbox("limit Jerk", &limitJerk);
                    static int joystickVal = 0;
                    ImGui::SliderInt("##Joystick Val", &joystickVal, -110, 110, "input: %d");
                    //static int stopRampTime = 0;
                    //ImGui::SliderInt("Stop Ramp Time (ms)", &stopRampTime, 0, 5000);


                    static ramp_ts aux1Vals;


                    static float maxVelocity = 10000;
                    static float maxAcceleration = 0;
                    static float maxJerk = 0;

                    if (ImGui::Button("Reset Data"))
                    {
                        aux1Vals.prevSetpoint = 0.0f;
                        aux1Vals.prevPrevSetpoint = 0.0f;
                        maxAcceleration = 0.0f;
                        maxVelocity = 0.0f;
                    }

                    if (ImGui::Button("1.0 unit/s"))
                        maxVelocity = 1.0f;
                    ImGui::SameLine();
                    if (ImGui::Button("5.0 unit/s"))
                        maxVelocity = 5.0f;
                    ImGui::SameLine();
                    if (ImGui::Button("25.0 unit/s"))
                        maxVelocity = 25.0f;
                    ImGui::SameLine();
                    if (ImGui::Button("100.0 unit/s"))
                        maxVelocity = 100.0f;

                    if (ImGui::Button("1.0 unit/s^2"))
                        maxAcceleration = 1.0f;
                    ImGui::SameLine();
                    if (ImGui::Button("5.0 unit/s^2"))
                        maxAcceleration = 5.0f;
                    ImGui::SameLine();
                    if (ImGui::Button("10.0 unit/s^2"))
                        maxAcceleration = 10.0f;
                    ImGui::SameLine();
                    if (ImGui::Button("25.0 unit/s^2"))
                        maxAcceleration = 25.0f;
                    ImGui::SameLine();
                    if (ImGui::Button("100.0 unit/s^2"))
                        maxAcceleration = 100.0f;
                    ImGui::SameLine();
                    if (ImGui::Button("1000.0 unit/s^2"))
                        maxAcceleration = 1000.0f;

                    //if (limitVelocity)
                        ImGui::SliderFloat("##max velocity", &maxVelocity, 0, 100, "Max Velocity: %.1f units/s");
                    //if (limitAcceleration)
                        ImGui::SliderFloat("##max acceleration", &maxAcceleration, 0, 100, "Max Acceleration: %.1f units/s2");
                    //if (limitJerk)
                        ImGui::SliderFloat("##max jerk", &maxJerk, 0, 5, "Max Jerk: %.1f units/s3");

                    aux1Vals.inputVal = joystickVal;
                    aux1Vals.maxVelocity = maxVelocity;
                    aux1Vals.maxAccel = maxAcceleration;
                    aux1Vals.maxJerk = maxJerk;
                    aux1Vals.dt = 1.0f / FRAME_RATE;// 0.016; // loop time
                    aux1Vals.outputMin = -100.0;
                    aux1Vals.outputMax = 100.0;
                    aux1Vals.limitVelocity = limitVelocity;
                    aux1Vals.limitAccel = limitAcceleration;
                    aux1Vals.limitJerk = limitJerk;

                    Ramp(&aux1Vals);
                    static bool firstLoop = true;
                    if (firstLoop)
                    {
                        trajectory_example(); // TODO - RM: DELETE WHEN COMPLETE
                        trajectory2D_example();
                        firstLoop = false;
                    }
                    int output = aux1Vals.output;
                    ImGui::PushStyleColor(ImGuiCol_PlotLines, (ImVec4)ImColor(1.f, 0.f, 0.f));

                    const int NUM_VALUES = 90;
                    static float values[NUM_VALUES] = {};
                    static int values_offset = 0;
                    float average = 0.0f;
                    static int counter = 0;

                    static uint64_t prevUpdateVal = 0;
                    const uint64_t updateValTimeout = 50;
                    if (io.MousePos.y > -10000 && io.MousePos.y < 10000)
                    {
                        if (Timer(prevUpdateVal, updateValTimeout, true))
                        {
                            values[counter] = output;
                            if (counter < NUM_VALUES - 1)
                            {
                                counter++;
                            }
                            // push everything back
                            if (counter == NUM_VALUES - 1)
                            {
                                for (int i = 1; i < NUM_VALUES; i++)
                                {
                                    values[i - 1] = values[i];
                                }
                            }
                        }
                    }
                    for (int n = 0; n < IM_ARRAYSIZE(values); n++)
                        average += values[n];
                    average /= (float)IM_ARRAYSIZE(values);
                    char overlay[32];
                    sprintf(overlay, "val %.0f", values[counter]);

                    if (ImGui::BeginTable("table_nested1", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable))
                    {
                        ImGui::TableSetupColumn("Graph");
                        ImGui::TableSetupColumn("Values");
                        ImGui::TableHeadersRow();

                        ImGui::TableNextColumn();

                        float minVal = -127;// 0;// getMin(values, NUM_VALUES);
                        float maxVal = 127;// aux1Vals.posMax_mA;// getMax(values, NUM_VALUES);
                        const float PLOT_HEIGHT = 160.0f;
                        ImGui::PlotLines("##Lines", values, IM_ARRAYSIZE(values), values_offset, overlay, minVal, maxVal, ImVec2(0, PLOT_HEIGHT));


                        //ImGui::Text("A0 Row 0");
                        ImGui::TableNextColumn();
                        //ImGui::Text("A1 Row 0");
                        {
                            float rows_height = 80 / 3;
                            if (ImGui::BeginTable("table_nested2", 1, ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable))
                            {
                                ImGui::TableSetupColumn("B0");

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                ImGui::Text("max: %d", (int)maxVal);

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                if (counter < NUM_VALUES - 1)
                                {
                                    ImGui::Text("val: %d", (int)values[counter - 1]);
                                }
                                else
                                {
                                    ImGui::Text("val: %d", (int)values[counter]);
                                }

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                ImGui::Text("min: %d", (int)minVal);

                                ImGui::EndTable();
                            }
                        }
                        //ImGui::TableNextColumn(); ImGui::Text("A0 Row 1");
                        //ImGui::TableNextColumn(); ImGui::Text("A1 Row 1");
                        ImGui::EndTable();
                    }
                    ImGui::PopStyleColor();
                    //ImGui::PlotLines("##Lines", values, IM_ARRAYSIZE(values), values_offset, overlay, getMin(values, NUM_VALUES), getMax(values, NUM_VALUES), ImVec2(0, 80.0f));
                    //ImGui::SameLine();
                    ImGui::Text("max");
                }
                if (ImGui::Button("Close Me"))
                    show_ramp_window = false;
                ImGui::End();
            }

            static bool show_kalman_window = true;
            // 3. Show a PID loop window
            if (show_kalman_window)
            {
                ImGui::SetNextWindowSize(ImVec2(1200, 750), ImGuiCond_Appearing);
                ImGui::Begin("Kalman Window", &show_kalman_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    static kalman_ts kalman_s;
                    static bool firstLoop = true;
                    // MAKE SOME INITIAL VALUES FOR KALMAN FILTER
                    if (firstLoop)
                    {
                        firstLoop = false;
                        kalman_s.err_measure = 1;
                        kalman_s.err_estimate = 1;
                    }
                    ImGui::SliderFloat("Kalman Q", &kalman_s.q, 0, 1, "%.4f");
                    //kalman._q = 0.11;


                    //ImGui::Text("Hello from Kalman window!");
                    static int rateVal = 0;
                    const int MAX_RATE = 7000; // mV/s
                    const int MAX_RATE_60FPS = 7000 / FRAME_RATE;
                    ImGui::SliderInt("Joystick Val", &rateVal, 0, MAX_RATE, "%d mV/s"); // mV/s
                    float rate = (float)rateVal / (float)FRAME_RATE;
                    const int LVDT_MAX = 4500; // mV
                    const int LVDT_MIN = 500; // mV
                    static int dir_fwd = 1; // 1 = increasing, -1 = decreasing

                    static float joystickVal = 0;
                    joystickVal += rate * dir_fwd;
                    if (joystickVal > LVDT_MAX)
                        dir_fwd = -1;
                    if (joystickVal < LVDT_MIN)
                        dir_fwd = 1;

                    //ImGui::SliderFloat("Kalman Q", &kalman._q, 0, 10);
                    // fake noise generator
                    int noiseLim = 250; // 
                    int noise = (rand() % noiseLim) - (noiseLim / 2); // -50 ~ +50
                    int noisyJoystickVal = (int)joystickVal + noise;

                    ImGui::Text("noise: %d", (int)noise);

                    const int NUM_VALUES = 500;
                    static float values[NUM_VALUES] = {};
                    static float kalmanFilterVals[NUM_VALUES] = {};
                    static int values_offset = 0;
                    float average = 0.0f;
                    static int counter = 0;

                    static uint64_t prevUpdateVal = 0;
                    const uint64_t updateValTimeout = 50;
                    if (io.MousePos.y > -10000 && io.MousePos.y < 10000)
                    {
                        if (Timer(prevUpdateVal, updateValTimeout, true))
                        {
                            values[counter] = noisyJoystickVal;
                            kalmanFilterVals[counter] = kalman.updateEstimate(noisyJoystickVal, &kalman_s);
                            if (counter < NUM_VALUES - 1)
                            {
                                counter++;
                            }
                            // push everything back
                            if (counter == NUM_VALUES - 1)
                            {
                                for (int i = 1; i < NUM_VALUES; i++)
                                {
                                    values[i - 1] = values[i];
                                    kalmanFilterVals[i - 1] = kalmanFilterVals[i];
                                }
                            }
                        }
                    }
                    for (int n = 0; n < IM_ARRAYSIZE(values); n++)
                        average += values[n];
                    average /= (float)IM_ARRAYSIZE(values);
                    char overlay[32];
                    sprintf(overlay, "val %.0f", values[counter]);
                    char kalmanoverlay[32];
                    sprintf(kalmanoverlay, "val %.0f", kalmanFilterVals[counter]);

                    if (ImGui::BeginTable("table_nested1", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable))
                    {
                        ImGui::TableSetupColumn("Graph");
                        ImGui::TableSetupColumn("Values");
                        ImGui::TableHeadersRow();

                        ImGui::TableNextColumn();

                        float minVal = 0;// 0;// getMin(values, NUM_VALUES);
                        float maxVal = 5000;// aux1Vals.posMax_mA;// getMax(values, NUM_VALUES);
                        const float PLOT_HEIGHT = 250.0f;
                        // raw value with noise
                        ImGui::PlotLines("##Lines", values, IM_ARRAYSIZE(values), values_offset, overlay, minVal, maxVal, ImVec2(0, PLOT_HEIGHT));

                        // kalman filter
                        ImGui::PlotLines("##Lines", kalmanFilterVals, IM_ARRAYSIZE(kalmanFilterVals), values_offset, kalmanoverlay, minVal, maxVal, ImVec2(0, PLOT_HEIGHT));


                        //ImGui::Text("A0 Row 0");
                        ImGui::TableNextColumn();
                        //ImGui::Text("A1 Row 0");
                        {
                            float rows_height = 80 / 3;
                            if (ImGui::BeginTable("table_nested2", 1, ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable))
                            {
                                ImGui::TableSetupColumn("B0");

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                ImGui::Text("max: %d", (int)maxVal);

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                if (counter < NUM_VALUES - 1)
                                {
                                    ImGui::Text("val: %d", (int)values[counter - 1]);
                                }
                                else
                                {
                                    ImGui::Text("val: %d", (int)values[counter]);
                                }

                                ImGui::TableNextRow(ImGuiTableRowFlags_None, rows_height);
                                ImGui::TableNextColumn();
                                ImGui::Text("min: %d", (int)minVal);

                                ImGui::EndTable();
                            }
                        }
                        //ImGui::TableNextColumn(); ImGui::Text("A0 Row 1");
                        //ImGui::TableNextColumn(); ImGui::Text("A1 Row 1");
                        ImGui::EndTable();
                    }
                    //ImGui::PopStyleColor();
                    //ImGui::PlotLines("##Lines", values, IM_ARRAYSIZE(values), values_offset, overlay, getMin(values, NUM_VALUES), getMax(values, NUM_VALUES), ImVec2(0, 80.0f));
                    //ImGui::SameLine();
                    ImGui::Text("max");
                }
                if (ImGui::Button("Close Me"))
                    show_kalman_window = false;
                ImGui::End();
            }

            static bool show_CAN_PropOut_window = true;
            // 3. Show a CAN endianess playground window
            if (show_CAN_PropOut_window)
            {
                ImGui::SetNextWindowSize(ImVec2(1200, 800), ImGuiCond_Appearing);
                ImGui::Begin("PROP OUT DEBUG", &show_CAN_PropOut_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    static bds_out_po_ts outputPinStatus;
                    static propOut_status_ts statusStruct;

                    bool statSCB = bds_dsm_DFC_ldf(outputPinStatus.stDFC_SCB_u16);
                    bool statSCG = bds_dsm_DFC_ldf(outputPinStatus.stDFC_SCG_u16);
                    bool statOL = bds_dsm_DFC_ldf(outputPinStatus.stDFC_OL_u16);
                    bool statOT = bds_dsm_DFC_ldf(outputPinStatus.stDFC_OT_u16);
                    bool statOC = bds_dsm_DFC_ldf(outputPinStatus.stDFC_OC_u16);
                    bool statOR = bds_dsm_DFC_ldf(outputPinStatus.stDFC_OR_u16);

                    ImGui::Checkbox("Initialized", &statusStruct.initialized);
                    ImGui::Checkbox("configured_POC", &statusStruct.configured_POC);
                    ImGui::Checkbox("configured_Period", &statusStruct.configured_Period);
                    ImGui::Checkbox("configured_Dither", &statusStruct.configured_Dither);
                    ImGui::Checkbox("configured_Deviation", &statusStruct.configured_Deviation);
                    ImGui::Checkbox("configured_PI", &statusStruct.configured_PI);
                    ImGui::Checkbox("locked_SCB", &statusStruct.locked_SCB);
                    ImGui::Checkbox("locked_SCG", &statusStruct.locked_SCG);
                    ImGui::Checkbox("locked_OC", &statusStruct.locked_OC);
                    ImGui::Checkbox("locked", &statusStruct.locked);
                    ImGui::Checkbox("locked_MO", &statusStruct.locked_MO);
                    ImGui::Checkbox("lockCouTiReact", &statusStruct.lockCouTiReact);
                    ImGui::Checkbox("TiReactNotElapsed", &statusStruct.TiReactNotElapsed);
                    ImGui::Checkbox("linked", &statusStruct.linked);
                    ImGui::Checkbox("testPulsesDisable", &statusStruct.testPulsesDisable);
                    ImGui::Checkbox("disable", &statusStruct.disable);
                    ImGui::Checkbox("statSCB", &statSCB);
                    ImGui::Checkbox("statSCG", &statSCG);
                    ImGui::Checkbox("statOL", &statOL);
                    ImGui::Checkbox("statOT", &statOT);
                    ImGui::Checkbox("statOC", &statOC);
                    ImGui::Checkbox("statOR", &statOR);
                    ImGui::Checkbox("unintendedSwitchONErr_l", &outputPinStatus.unintendedSwitchONErr_l);
                    ImGui::Checkbox("lockedByINH_l", &outputPinStatus.lockedByINH_l);

                    if (statSCB)
                        outputPinStatus.stDFC_SCB_u16 = 1 << 4;
                    else
                        outputPinStatus.stDFC_SCB_u16 = 0;
                    if (statSCG)
                        outputPinStatus.stDFC_SCG_u16 = 1 << 4;
                    else
                        outputPinStatus.stDFC_SCG_u16 = 0;
                    if (statOL)
                        outputPinStatus.stDFC_OL_u16 = 1 << 4;
                    else
                        outputPinStatus.stDFC_OL_u16 = 0;
                    if (statOT)
                        outputPinStatus.stDFC_OT_u16 = 1 << 4;
                    else
                        outputPinStatus.stDFC_OT_u16 = 0;
                    if (statOC)
                        outputPinStatus.stDFC_OC_u16 = 1 << 4;
                    else
                        outputPinStatus.stDFC_OC_u16 = 0;
                    if (statOR)
                        outputPinStatus.stDFC_OR_u16 = 1 << 4;
                    else
                        outputPinStatus.stDFC_OR_u16 = 0;

                    outputPinStatus.status_b16 = 0;
                    if (statusStruct.initialized)
                        outputPinStatus.status_b16 |= PO_INITIALIZED_MASK;
                    if (statusStruct.configured_POC)
                        outputPinStatus.status_b16 |= PO_CFG_POC_MASK;
                    if (statusStruct.configured_Period)
                        outputPinStatus.status_b16 |= PO_CFG_PERIOD_MASK;
                    if (statusStruct.configured_Dither)
                        outputPinStatus.status_b16 |= PO_CFG_DITHER_MASK;
                    if (statusStruct.configured_Deviation)
                        outputPinStatus.status_b16 |= PO_CFG_DEVIATION_MASK;
                    if (statusStruct.configured_PI)
                        outputPinStatus.status_b16 |= PO_CFG_PI_MASK;
                    if (statusStruct.locked_SCB)
                        outputPinStatus.status_b16 |= PO_LOCKED_SCB_MASK;
                    if (statusStruct.locked_SCG)
                        outputPinStatus.status_b16 |= PO_LOCKED_SCG_MASK;
                    if (statusStruct.locked_OC)
                        outputPinStatus.status_b16 |= PO_LOCKED_OC_MASK;
                    if (statusStruct.locked)
                        outputPinStatus.status_b16 |= PO_LOCKED_MASK;
                    if (statusStruct.locked_MO)
                        outputPinStatus.status_b16 |= PO_LOCKED_MO_MASK;
                    if (statusStruct.lockCouTiReact)
                        outputPinStatus.status_b16 |= PO_LOCK_COU_TI_REACT_MASK;
                    if (statusStruct.TiReactNotElapsed)
                        outputPinStatus.status_b16 |= PO_TI_REACT_NOT_ELAPSTED_MASK;
                    if (statusStruct.linked)
                        outputPinStatus.status_b16 |= PO_LINKED_MASK;
                    if (statusStruct.testPulsesDisable)
                        outputPinStatus.status_b16 |= PO_TST_PLS_DISABLE_MASK;
                    if (statusStruct.disable)
                        outputPinStatus.status_b16 |= PO_DISABLE;


                    ImGui::SliderInt("iOut1", &outputPinStatus.iSet_u16, 0, 4000);
                    ImGui::SliderInt("iOut2", &outputPinStatus.iAct_u16, 0, 4000);
                    ImGui::SliderInt("iOut3", &outputPinStatus.iActRaw_mA_u16, 0, 4000);

                    Send_PropOut_DebugToCanTX(outputPinStatus, RBR_CAN_1_E, 0x42, 0);
                }
                ImGui::End();
            }

            static bool show_sudoku_window = true;
            static bool allCellsPencilled = false;
            // 3. Show a CAN endianess playground window
            if (show_sudoku_window)
            {
                //static int sudokuArr[9][9];
                static bool firstTime = true;
                if (firstTime)
                {
                    firstTime = false;
                    for (int row = 0; row < sudoku.NUM_ROWS; row++)
                    {
                        for (int column = 0; column < sudoku.NUM_COLUMNS; column++)
                        {
                            sudoku.gameVals_s[row][column].realVal = 0;
                            sudoku.gameVals_s[row][column].givenVal = false;

                            for (int val = 0; val < sudoku.NUM_VALUES; val++)
                            {
                                sudoku.gameVals_s[row][column].pencilledVals[val] = false;
                            }
                        }
                    }
                    allCellsPencilled = sudoku.PencilAllCells(sudoku.gameVals_s);
                }
                ImGui::SetNextWindowSize(ImVec2(850, 1000), ImGuiCond_Appearing);
                ImGui::Begin("Sudoku Solver", &show_sudoku_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    static bool editCells = false; // enable/disable drag sliders for editing cells
                    ImGui::Text(savePathStr.c_str());
                    if (ImGui::Button("New Game"))
                    {
                        ImGui::OpenPopup("New Game Confirmation");
                    }
                    // Always center this window when appearing
                    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
                    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

                    if (ImGui::BeginPopupModal("New Game Confirmation", NULL, ImGuiWindowFlags_AlwaysAutoResize))
                    {
                        ImGui::Text("Are you sure you want to start a new game?");
                        if (ImGui::Button("OK", ImVec2(120, 0)))
                        {
                            firstTime = true; // reset all values
                            ImGui::CloseCurrentPopup();
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Cancel", ImVec2(120, 0)))
                        {
                            ImGui::CloseCurrentPopup();
                        }
                        ImGui::EndPopup();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Save Game"))
                    {
                        Json::Value sudokuJsonFile;
                        Json::StreamWriterBuilder builder;
                        builder["indentation"] = "    ";	//	4 spaces for indent
                        nfdchar_t* savePath = (nfdchar_t*)std::malloc(MAXIMUM_PATH_SIZE);	//	ALLOCATE ENOUGH MEMORY FOR AS LONG AS A PATH AS WE EVER EXPECT
                        nfdresult_t result = NFD_ERROR;	//	DEFAULT TO ERROR STATE CAUSE THAT'S WHAT NDF_SaveDialog DOES! :D
                        result = NFD_SaveDialog(sudoku.fileExt, NULL, &savePath);
                        printf("result: %d\n", result);
                        if (result == NFD_OKAY)	//	IF WE ALREADY SAVED AS, JUST SAVE IT WITHOUT PROMPTING NFD
                        {
                            sudoku.SerializeSudokuGameData(sudoku.gameVals_s, sudokuJsonFile);
                            printf("Success!\n");
                            printf("%s\n", savePath);
                            LPCSTR extensionFound = PathFindExtensionA(savePath);
                            printf("EXTENSION FOUND: %s\n", extensionFound);
                            LPCSTR SUDOKU_EXTENSION = sudoku.fileExtDot;
                            if (strcmp(extensionFound, SUDOKU_EXTENSION))
                            {
                                printf("WE FOUND DIFFERENCES!\n");
                                printf("strlen total: %d\n", strlen(savePath) + strlen(SUDOKU_EXTENSION));
                                savePathStr = savePath + (std::string)SUDOKU_EXTENSION;
                            }
                            else
                            {
                                savePathStr = savePath;	//	IF THE CORRECT EXTENSION ALREADY EXISTS, JUST PLAIN COPY IT!
                            }
                            std::string jsonStr = Json::writeString(builder, sudokuJsonFile);	//	CONVERT JSON TO STRING FOR WRITING
                            long len = size(jsonStr);

                            std::ofstream outfile(savePathStr.c_str(), std::ios::binary);
                            if (outfile.is_open())
                            {
                                outfile.write(jsonStr.c_str(), len);
                                outfile.close();
                            }
                        }
                        else if (result == NFD_CANCEL)
                        {
                            printf("User pressed cancel.\n");
                        }
                        else
                        {
                            printf("Error: %s\n", NFD_GetError());
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Load Game"))
                    {
                        //	START NATIVE FILE DIALOG OPEN
                        const nfdchar_t* defaultPath = (nfdchar_t*)"::{031E4825-7B94-4DC3-B131-E946B44C8DD5}\Documents.library-ms";
                        nfdchar_t* openPath = NULL;
                        nfdresult_t result = NFD_OpenDialog(sudoku.fileExt, NULL/*defaultPath*/, &openPath);
                        if (result == NFD_OKAY)
                        {
                            //bool fileVerified = false;
                            printf("%s\n", openPath);
                            {
                                std::ifstream infile(openPath);
                                std::stringstream jsonStr;
                                jsonStr << infile.rdbuf();
                                Json::Value readJson;
                                if (ParseJson(jsonStr.str(), readJson) == 0)
                                {
                                    printf("JSON encode/decode SUCCESS!\n");
                                }
                                sudoku.DeserializeSudokuGameData(readJson, sudoku.gameVals_s);
                                savePathStr = openPath;
                            }
                            sudoku.PencilAllCells(sudoku.gameVals_s);
                        }
                        else if (result == NFD_CANCEL)
                        {
                            printf("User pressed cancel.\n");
                        }
                        else
                        {
                            printf("Error: %s\n", NFD_GetError());
                        }
                    }
                    ImGui::SameLine();
                    ImGui::Checkbox("Edit Cell Values", &editCells);
                    //if (ImGui::Button("Fast Pencil"))
                    //{
                    //    allCellsPencilled = sudoku.PencilAllCells(sudoku.gameVals_s);
                    //}
                    static int highlightedVals = 0;
                    static ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoHostExtendX;
                    if (ImGui::BeginTable("table1", sudoku.NUM_COLUMNS, flags))
                    {
                        for (int i = 0; i < sudoku.NUM_COLUMNS; i++)
                        {
                            //sudoku.CheckRow(sudoku.gameVals_s, 1, 1);
                            //sudoku.CheckColumn(sudoku.gameVals_s, 1, 1);
                            ImGui::TableSetupColumn("one", ImGuiTableColumnFlags_WidthFixed, 80.0f); // Default to 100.0f
                        }
                        for (int row = 0; row < sudoku.NUM_ROWS; row++)
                        {
                            ImGui::TableNextRow(ImGuiTableRowFlags_None, 80.0f);

                            // Fill cells
                            for (int column = 0; column < sudoku.NUM_COLUMNS; column++)
                            {
                                ImGui::TableSetColumnIndex(column);

                                ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                                ImU32 row_bg_color_drk = ImGui::GetColorU32(ImVec4(0.8f, 0.8f, 0.8f, 1.0f));
                                ImU32 row_bg_color_hilite = ImGui::GetColorU32(ImVec4(0.717f, 0.717f, 1.0f, 1.0f));
                                ImU32 row_bg_color_err = ImGui::GetColorU32(ImVec4(1.0f, 0.8f, 0.8f, 1.0f));
                                if (((column < 3 || column > 5) && (row < 3 || row > 5)) || ((column > 2 && column < 6) && (row > 2 && row < 6)))
                                {
                                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, row_bg_color_drk);
                                }
                                else
                                {
                                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, row_bg_color);
                                }

                                if (highlightedVals == sudoku.gameVals_s[row][column].realVal && sudoku.gameVals_s[row][column].realVal) // highlight cell value
                                {
                                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, row_bg_color_hilite);
                                }

                                if (sudoku.CheckForDuplicateVals(sudoku.gameVals_s, row, column))
                                {
                                    ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, row_bg_color_err);
                                }

                                //ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
                                ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));

                                float dragIntCursorPosX = 0;
                                float dragIntCursorPosY = 0;

                                ImVec2 cellSize = ImGui::GetContentRegionAvail(); // Get available cell size
                                float dragWidgetWidth = ImGui::CalcItemWidth();
                                float dragWidgetHeight = 0;// ImGui::GetFrameHeight();

                                float posX = (cellSize.x - dragWidgetWidth) * 0.5f; // Center horizontally
                                float posY = 20;// (cellSize.y - dragWidgetHeight) * 0.5f; // Center vertically

                                posX = posX > 0 ? posX : 0; // Prevent negative offset
                                posY = posY > 0 ? posY : 0;

                                dragIntCursorPosX = ImGui::GetCursorPosX() + posX;
                                dragIntCursorPosY = ImGui::GetCursorPosY() + posY;

                                if (editCells || sudoku.gameVals_s[row][column].realVal)
                                {
                                    ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
                                    ImGui::SetCursorPosX(dragIntCursorPosX);  // Adjust horizontal position
                                    ImGui::SetCursorPosY(dragIntCursorPosY);  // Adjust vertical position

                                    int nameNum = column + row * sudoku.NUM_COLUMNS;

                                    ImVec4 blankCol = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
                                    ImVec4 givenTextCol = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
                                    ImVec4 normalTextCol = ImVec4(0.0f, 0.0f, 1.0f, 1.0f);
                                    if (!sudoku.gameVals_s[row][column].realVal) // if the value is a zero, don't show it
                                    {
                                        ImGui::PushStyleColor(ImGuiCol_Text, blankCol);
                                    }
                                    else
                                    {
                                        if (sudoku.gameVals_s[row][column].givenVal)
                                        {
                                            ImGui::PushStyleColor(ImGuiCol_Text, givenTextCol);
                                        }
                                        else
                                        {
                                            ImGui::PushStyleColor(ImGuiCol_Text, normalTextCol);
                                        }
                                    }
                                    std::string name = std::string("##") + std::to_string(nameNum);
                                    if (sudoku.gameVals_s[row][column].givenVal && !editCells)
                                    {
                                        ImGui::BeginDisabled();
                                    }
                                    ImGui::PushFont(headerTextFont);
                                    if (ImGui::DragInt(name.c_str(), &sudoku.gameVals_s[row][column].realVal, 0.05f, 0, 9, "%d", ImGuiSliderFlags_AlwaysClamp))
                                    {
                                        if (editCells && sudoku.gameVals_s[row][column].realVal) // if we are inputting starting values, and the starting value isn't 0
                                        {
                                            sudoku.gameVals_s[row][column].givenVal = true;
                                        }
                                        else
                                        {
                                            sudoku.gameVals_s[row][column].givenVal = false;
                                        }
                                        allCellsPencilled = sudoku.PencilAllCells(sudoku.gameVals_s);
                                    }
                                    ImGui::PopFont();
                                    if (sudoku.gameVals_s[row][column].givenVal && !editCells)
                                    {
                                        ImGui::EndDisabled();
                                    }
                                    ImGui::PopStyleColor();
                                    ImGui::PopStyleColor();
                                }
                                else
                                {
                                    ImVec2 button1_pos = ImGui::GetCursorScreenPos();
                                    ImVec2 button2_pos = ImVec2(button1_pos.x + 14, button1_pos.y + 25);

                                    ImGui::SetNextItemAllowOverlap();
                                    ImGui::SetCursorScreenPos(button2_pos);

                                    int nameNum = column + row * sudoku.NUM_COLUMNS;

                                    ImVec4 blankCol = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
                                    ImVec4 normalTextCol = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
                                    if (!sudoku.gameVals_s[row][column].realVal) // if the value is a zero, don't show it
                                    {
                                        ImGui::PushStyleColor(ImGuiCol_Text, blankCol);
                                    }
                                    else
                                    {
                                        ImGui::PushStyleColor(ImGuiCol_Text, normalTextCol);
                                    }
                                    ImGui::PushFont(headerTextFont);
                                    std::string name = std::string("##") + std::to_string(nameNum);
                                    if (ImGui::DragInt(name.c_str(), &sudoku.gameVals_s[row][column].realVal, 0.05f, 0, 9))
                                    {
                                        allCellsPencilled = sudoku.PencilAllCells(sudoku.gameVals_s);
                                    }
                                    ImGui::PopFont();
                                    ImGui::PopStyleColor();

                                    ImGui::SetCursorScreenPos(button1_pos);
                                    static ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingMask_ | ImGuiTableFlags_NoHostExtendX;
                                    if (ImGui::BeginTable("pencilValsTable", sudoku.NUM_COLUMNS_BOX, flags))
                                    {
                                        for (int i = 0; i < sudoku.NUM_COLUMNS_BOX; i++)
                                        {
                                            //ImGui::TableSetupColumn("one", ImGuiTableColumnFlags_WidthFixed, 16.0f); // Default to 100.0f
                                        }
                                        for (int cellRow = 0; cellRow < sudoku.NUM_ROWS_BOX; cellRow++)
                                        {
                                            ImVec2 textSize = ImGui::CalcTextSize("8");  // Get text size
                                            ImGui::TableNextRow(ImGuiTableRowFlags_None, textSize.y + 2);
                                            for (int cellColumn = 0; cellColumn < sudoku.NUM_COLUMNS_BOX; cellColumn++)
                                            {
                                                int pencilNumIdx = cellColumn + cellRow * sudoku.NUM_COLUMNS_BOX;
                                                int pencilNum = pencilNumIdx + 1;
                                                ImGui::TableSetColumnIndex(cellColumn);
                                                ImU32 row_bg_color = ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
                                                if (highlightedVals == pencilNum)
                                                {
                                                    if (sudoku.gameVals_s[row][column].pencilledVals[pencilNum])
                                                    {
                                                        row_bg_color = ImGui::GetColorU32(ImVec4(0.717f, 0.717f, 1.0f, 1.0f)); // only highlight if pencilled
                                                    }
                                                }
                                                ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, row_bg_color);
                                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
                                                ImVec2 cellSize = ImGui::GetContentRegionAvail(); // Get available cell size

                                                float posX = (cellSize.x - textSize.x) * 0.5f; // Center horizontally
                                                float posY = 0;// (cellSize.y - textSize.y) * 0.5f; // Center vertically

                                                posX = posX > 0 ? posX : 0; // Prevent negative offset
                                                posY = posY > 0 ? posY : 0;

                                                ImGui::SetCursorPosY(ImGui::GetCursorPosY() + posY);  // Adjust vertical position
                                                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + posX);  // Adjust horizontal position

                                                int nameNum = cellColumn + cellRow * sudoku.NUM_COLUMNS_BOX;
                                                std::string name = std::string("##") + std::to_string(nameNum);
                                                if (sudoku.gameVals_s[row][column].pencilledVals[pencilNum])
                                                {
                                                    ImGui::Text("%d", pencilNum);
                                                }

                                                //ImGui::Text("%c%c", 'A' + row, '0' + column);
                                                ImGui::PopStyleColor();
                                            }
                                        }
                                        ImGui::EndTable();
                                    }
                                }
                                //ImGui::Text("%c%c", 'A' + row, '0' + column);
                                ImGui::PopStyleColor();
                            }
                        }
                        ImGui::EndTable();
                        //static int rowSlider = 0;
                        //static int columnSlider = 0;
                        //static int valueSlider = 0;
                        //ImGui::SliderInt("row", &rowSlider, 0, 8);
                        //ImGui::SliderInt("column", &columnSlider, 0, 8);
                        //ImGui::SliderInt("value", &valueSlider, 0, 9);
                        //ImGui::Text("CheckIfOnlyValInRowPencilled: %d", sudoku.CheckIfOnlyValInRowPencilled(sudoku.gameVals_s, rowSlider, columnSlider, valueSlider));
                        //ImGui::Text("CheckIfOnlyValInColumnPencilled: %d", sudoku.CheckIfOnlyValInColumnPencilled(sudoku.gameVals_s, rowSlider, columnSlider, valueSlider));
                        //ImGui::Text("CheckIfOnlyValInBoxPencilled: %d", sudoku.CheckIfOnlyValInBoxPencilled(sudoku.gameVals_s, rowSlider, columnSlider, valueSlider));
                        static int numSimpleSolveIterations = 0;
                        if (ImGui::Button("Solve Simple!"))
                        {
                            bool solving = true;
                            int count = 0;
                            while (solving && count < 100) // let's cap the number of iterations to 100
                            {
                                solving = sudoku.SolveSimple(sudoku.gameVals_s); // SolveSimple() returns false when it didn't make any moves
                                count++;
                            }
                            numSimpleSolveIterations = count;
                        }
                        ImGui::Text("number of iterations to solve: %d", numSimpleSolveIterations);

                        if (ImGui::Button("Fast Pencil"))
                        {
                            sudoku.PencilAllCells(sudoku.gameVals_s);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("Fast Complex Pencil"))
                        {
                            sudoku.PencilAllCells(sudoku.gameVals_s);
                            int pencilsRemoved = 1;
                            while (pencilsRemoved) // keep iterating through in case we find an opening
                            {
                                pencilsRemoved = 0;
                                pencilsRemoved += sudoku.CheckBoxRowPencilledVals(sudoku.gameVals_s);
                                pencilsRemoved += sudoku.CheckBoxColumnPencilledVals(sudoku.gameVals_s);
                                pencilsRemoved += sudoku.CheckOutsideBoxRowPencilledVals(sudoku.gameVals_s);
                                pencilsRemoved += sudoku.CheckOutsideBoxColumnPencilledVals(sudoku.gameVals_s);
                            }
                        }
                        if (ImGui::Button("SolveRowPencilledVals"))
                        {
                            sudoku.CheckBoxRowPencilledVals(sudoku.gameVals_s);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("SolveColumnPencilledVals"))
                        {
                            sudoku.CheckBoxColumnPencilledVals(sudoku.gameVals_s);
                        }
                        if (ImGui::Button("SolveOutsideRowPencilledVals"))
                        {
                            sudoku.CheckOutsideBoxRowPencilledVals(sudoku.gameVals_s);
                        }
                        ImGui::SameLine();
                        if (ImGui::Button("SolveOutsideColumnPencilledVals"))
                        {
                            sudoku.CheckOutsideBoxColumnPencilledVals(sudoku.gameVals_s);
                        }
                        ImGui::SliderInt("highlight Value", &highlightedVals, 0, 9);
                        //static int value = 0;
                        //ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
                        //ImGui::DragInt("#drag int", &value, 0.05f, 0, 9);
                        //ImGui::PopStyleColor();
                    }

                }
                ImGui::End();
            }

            static bool show_kinematics_toggle_window = true;
            // 3. Show a CAN endianess playground window
            if (show_kinematics_toggle_window)
            {
                ImGui::SetNextWindowSize(ImVec2(600, 750), ImGuiCond_Appearing);
                ImGui::Begin("Kinematics", &show_kinematics_toggle_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    Fabrik2D::AngularConstraint angularConstraints[3];
                    angularConstraints[0].min_angle = -360 / RAD_TO_DEG;// -90 / RAD_TO_DEG;
                    angularConstraints[0].max_angle = 360 / RAD_TO_DEG;//90 / RAD_TO_DEG;
                    angularConstraints[1].min_angle = -360 / RAD_TO_DEG;
                    angularConstraints[1].max_angle = 360 / RAD_TO_DEG;
                    angularConstraints[2].min_angle = 2.0 * -PI;// -90 / RAD_TO_DEG;
                    angularConstraints[2].max_angle = 2.0 * PI;// 90 / RAD_TO_DEG;
                    fabrik2D.SetAngularConstraints(angularConstraints);
                    fabrik2D.setTolerance(0.5f);
                    ImDrawList* draw_list = ImGui::GetWindowDrawList();
                    static float thickness = 2.0f;
                    static float toolAngle;
                    ImGui::SliderFloat("tool angle Kinematics", &toolAngle, 0, 360);

                    ImGuiIO& io = ImGui::GetIO();
                    static ImVec2 targetPosition = ImVec2(50.0f, -10.0f);
                    ImGui::Button("Tool Joystick");
                    if (ImGui::IsItemActive())
                    {
                        toolAngle += (5 * io.MouseWheel); // rotate 5 degrees per mouse scroll just for fun
                        if (toolAngle > 360)
                            toolAngle -= 360;
                        if (toolAngle < -360)
                            toolAngle += 360;
                        ImGui::GetForegroundDrawList()->AddLine(io.MouseClickedPos[0], io.MousePos, ImGui::GetColorU32(ImGuiCol_Button), 4.0f); // Draw a line between the button and the mouse cursor
                        targetPosition = ImGui::GetMouseDragDelta(0, 0.0f);
                        targetPosition = ImVec2(targetPosition.x, -targetPosition.y); // invert y cause it's upside down in GUIs
                    }
                    // If you have a controller hooked up, Press A and use the left joystick to control the "joystick" on screen :)
                    else if (ImGui::IsKeyDown(ImGuiKey_GamepadFaceDown))
                    {
                        ImVec2 gamePadJoystick = ImGui::GetKeyMagnitude2d(ImGuiKey_GamepadLStickLeft, ImGuiKey_GamepadLStickRight, ImGuiKey_GamepadLStickUp, ImGuiKey_GamepadLStickDown);
                        targetPosition = ImVec2(gamePadJoystick.x * 100.0, -gamePadJoystick.y * 100.0);
                    }
                    else
                    {
                        targetPosition = ImVec2(0.0f, 0.0f);
                    }



                    //static double prevToolSetpointX = 0;
                    //static uint64_t prevToolTimeX = 0;
                    //static bool finishedMovementX = false;
                    //double toolSetpointX = RampScale(prevToolSetpointX, value_raw.x, 10, 200, &prevToolTimeX, 2000, &finishedMovementX);
                    //prevToolSetpointX = toolSetpointX;

                    //static double prevToolSetpointY = 0;
                    //static uint64_t prevToolTimeY = 0;
                    //static bool finishedMovementY = false;
                    //double toolSetpointY = RampScale(prevToolSetpointY, value_raw.y, 10, 200, &prevToolTimeY, 2000, &finishedMovementY);
                    //prevToolSetpointY = toolSetpointY;

                    //static float ang = 0;
                    //static int counter = 0;
                    //uint64_t prevTime = 0;
                    //if (Timer(prevTime, 10, true))
                    //{
                    //    ang = ((int)(ang + 1)) % 360;
                    //}
                    //float radius = 30;
                    //float x_offset = 100;
                    //float y_offset = 150;
                    // Move x and y in a circular motion
                    //float x = x_offset + radius * cos(ang * 1000 / 57296);
                    //float y = y_offset + radius * sin(ang * 1000 / 57296);
                    //fabrik2D.solve(toolSetpointX.current_value, toolSetpointY.current_value, toolAngle / RAD_TO_DEG, lengths);
                    //int ikReturn = fabrik2D.solve(value_raw.x, value_raw.y, toolAngle / RAD_TO_DEG, lengths);
                    //int ikReturn = fabrik2D.solve2(value_raw.x, value_raw.y, 0, toolAngle / RAD_TO_DEG, lengths);
                    //fabrik2D.solve(inverseKinematics.x, inverseKinematics.y, toolAngle / RAD_TO_DEG, lengths);
                    //draw_list->AddLine(ImVec2(startPos.x + fabrik2D.getX(0), startPos.y - fabrik2D.getY(0)), ImVec2(startPos.x + fabrik2D.getX(1), startPos.y - fabrik2D.getY(1)), col, th);
                    //draw_list->AddLine(ImVec2(startPos.x + fabrik2D.getX(1), startPos.y - fabrik2D.getY(1)), ImVec2(startPos.x + fabrik2D.getX(2), startPos.y - fabrik2D.getY(2)), col, th);

                    //draw_list->AddLine(ImVec2(startPos.x + fabrik2D.getX(2), startPos.y - fabrik2D.getY(2)), ImVec2(startPos.x + fabrik2D.getX(3), startPos.y - fabrik2D.getY(3)), col2, th);
                    //*0 if FABRIK could not converge
                    //    * 1 if FABRIK converged to the set threshold
                    //    * 2 if FABRIK converged with a higher tolerance value




                    ImVec2 startPos = ImGui::GetCursorScreenPos();
                    ImVec2 windowSize;
                    windowSize.x = ImGui::GetWindowWidth();
                    windowSize.y = ImGui::GetWindowHeight();
                    startPos.x = startPos.x + windowSize.x / 2;
                    startPos.y = startPos.y + 150;// windowSize.y / 2;

                    static MotionProfile toolSetpointX;
                    static bool firstLoopX = true;
                    if (firstLoopX)
                    {
                        firstLoopX = false;
                        toolSetpointX.max_velocity = 20;
                        toolSetpointX.max_acceleration = 25;
                        toolSetpointX.last_update = 0;
                    }
                    toolSetpointX.target_value = targetPosition.x;
                    update_motion(&toolSetpointX);
                    toolSetpointX.last_update = toolSetpointX.current_value;

                    static MotionProfile toolSetpointY;
                    static bool firstLoopY = true;
                    if (firstLoopY)
                    {
                        firstLoopY = false;
                        toolSetpointY.max_velocity = 20;
                        toolSetpointY.max_acceleration = 25;
                        toolSetpointY.last_update = 0;
                    }
                    toolSetpointY.target_value = targetPosition.y;
                    update_motion(&toolSetpointY);
                    toolSetpointY.last_update = toolSetpointY.current_value;

                    static float maxVel = 80.0f;
                    static float maxAccel = 130.0f;
                    static float maxJerk = 0;
                    static float posTolerance = 1.0;
                    static bool constrainAngles = false;

                    ImGui::SliderFloat("max velocity", &maxVel, 0, 1000);
                    ImGui::SliderFloat("max acceleration", &maxAccel, 0, 1000);
                    //ImGui::SliderFloat("max jerk", &maxJerk, 0, 1000);
                    ImGui::SliderFloat("position Tolerance", &posTolerance, 0.0, 1.0);
                    ImGui::Checkbox("Constrain Angles", &constrainAngles);

                    //ramp2D_ts ramp;
                    //ramp.dt = 1.0 / FRAME_RATE;
                    //ramp.inputVal = Vec2(targetPosition.x, targetPosition.y);
                    //ramp.limitAccel = true;
                    //ramp.maxVelocity = maxVel;
                    //ramp.maxAccel = maxAccel;
                    //ramp.maxJerk = maxJerk;

                    //Ramp2D(&ramp);
                    static joystk_handlr_ts joystick;



                    static mc2D_state_t controller;
                    mc2D_vec2_t start_pos = { 73.0, 10.0 };
                    mc2D_vec2_t start_vel = { 0.0, 0.0 };

                    /* Set up constraints */
                    mc2D_constraints_t constraints = {
                        .max_vel = maxVel,
                        .max_accel = maxAccel,
                        .position_tol = posTolerance,     // 0.01 units (1% of a unit)
                        .velocity_tol = 0.05,     // 0.05 units/sec
                        .homing_distance = 0.5,   // Switch to homing within 0.5 units
                        .homing_gain = 2.0,        // P-gain for homing
                        .settle_cycles = 5
                    };

                    static bool firstLoop = true;
                    if (firstLoop)
                    {
                        firstLoop = false;
                        // Initialize state
                        controller = {
                            .pos = {73.0, 10.0},
                            .vel = {0.0, 0.0},
                            .state = MC_STATE_ACCELERATING,
                            .settled_count = 0
                        };

                        /* Configure joystick for safe operation */
                        joystick.config.deadzone = 0.0;      /* 15% deadzone for worn joysticks */
                        joystick.config.expo = 2.5;           /* Good balance of fine/coarse control */
                        joystick.config.max_vel_scale = 0.3;  /* Start conservative at 30% speed */
                        joystick.config.mode = JOY_MODE_VELOCITY;

                        /* Initialize joystick controller */
                        joy_init(&joystick, mc2D_vec2_t(73.0, 10.0));
                        /* Set workspace limits (e.g., for a 200x200mm workspace) */
                        mc2D_vec2_t workspace_min = { -100.0, -100.0 };
                        mc2D_vec2_t workspace_max = { 100.0, 100.0 };
                        joy_set_workspace_limits(&joystick, workspace_min, workspace_max);

                        //target_vel = { 0.0, 0.0 };  // Stationary target
                    }

                    joy_input_ts joystk_input;
                    joystk_input.x = targetPosition.x / 100.0;
                    joystk_input.y = targetPosition.y / 100.0;
                    joystk_input.enable = true;
                    joy_update(&joystick, joystk_input, controller.pos, controller.vel, &constraints, 1.0 / FRAME_RATE);

                    mc2D_vec2_t target_pos = joy_get_target_position(&joystick);
                    mc2D_vec2_t target_vel = joy_get_target_velocity(&joystick);

                    static mc2D_vec2_t prevTarget_Pos = target_pos;
                    //mc2D_vec2_t target_pos = mc2D_vec2_t(targetPosition.x, targetPosition.y);
                    //mc2D_vec2_t target_vel = { 0.0, 0.0 };  /* Stationary target */

                    /* Compute control */
                    //mc2D_vec2_t accel = mc2D_compute_control(&controller, target_pos, target_vel, &constraints, 1.0 / FRAME_RATE);


                    ImVec2 motionTarget = ImVec2(controller.pos.x, controller.pos.y);
                    //ImVec2 motionTarget = ImVec2(ramp.output.x, ramp.output.y);


                    Fsciks::Arm arm;

                    arm.joints[0].length = 0;
                    arm.joints[1].length = lengths[0];
                    arm.joints[2].length = lengths[1];
                    arm.joints[3].length = lengths[2];

                    arm.joints[0].angularConstraint.min_angle = -8.45; // all angle constraints are confirmed via solidworks
                    arm.joints[0].angularConstraint.max_angle = 50.73;
                    arm.joints[1].angularConstraint.min_angle = -56.45;
                    arm.joints[1].angularConstraint.max_angle = -131.64;
                    arm.joints[2].angularConstraint.min_angle = -35.14;
                    arm.joints[2].angularConstraint.max_angle = 92.03;

                    //arm.joints[0].angularConstraint.min_angle = -180.0f; // all angle constraints are confirmed via solidworks
                    //arm.joints[0].angularConstraint.max_angle = 180.0f;
                    //arm.joints[1].angularConstraint.min_angle = -180.0f;
                    //arm.joints[1].angularConstraint.max_angle = 180.0f;
                    //arm.joints[2].angularConstraint.min_angle = - 180.0f;
                    //arm.joints[2].angularConstraint.max_angle = 180.0f;

                    arm.joints[0].x = 0.0f; // first joint is what we are considering the origin
                    arm.joints[0].y = 0.0f; // first joint is what we are considering the origin

                    mc2D_vec2_t target = mc2D_vec2_t(motionTarget.x, motionTarget.y);
                    mc2D_vec2_t original_target = target;
                    float angleConstraintTolerance = 0.5f;

                    // Track limit state
                    static joint_limit_state_t limit_state = { 0 };

                    // Check if any joints are near limits
                    bool near_limits = false;
                    for (int i = 0; i < NUM_LINKS; i++) {
                        if (limit_state.at_limit[i]) {
                            near_limits = true;
                            break;
                        }
                    }

                    // Reduce speed near limits for smooth deceleration
                    mc2D_constraints_t dynamic_constraints = constraints;
                    if (near_limits) {
                        dynamic_constraints.max_vel *= 0.3;    // Slow down to 30%
                        dynamic_constraints.max_accel *= 0.5;  // Gentler acceleration
                    }

                    Fsciks::Arm tempArm = arm;
                    tempArm.joints[3].x = target_pos.x;
                    tempArm.joints[3].y = target_pos.y;
                    if (fsciks.calcArm(&tempArm) != CONVERGES || !fsciks.check_arm_angles(tempArm)) // if we don't converge or comply with angular constraints, use previous position
                    {
                        controller.pos.x = arm.joints[3].x;
                        controller.pos.y = arm.joints[3].y;
                        controller.vel = (mc2D_vec2_t)(0.0f, 0.0f);
                        controller.state = MC_STATE_ACCELERATING;
                        controller.settled_count = 0;

                        target_pos.x = arm.joints[3].x;
                        target_pos.y = arm.joints[3].y;
                        //joy_init(&joystick, (mc2D_vec2_t)(arm.joints[3].x, arm.joints[3].y));
                        joy_sync_to_position(&joystick, (mc2D_vec2_t)(arm.joints[3].x, arm.joints[3].y));
                        //controller.pos = prevTarget_Pos;
                        //controller.vel = (mc2D_vec2_t)(0.0f, 0.0f);
                        //controller.state = MC_STATE_ACCELERATING;
                        //controller.settled_count = 0;

                        //arm.joints[3].x = prevTarget_Pos.x;
                        //arm.joints[3].y = prevTarget_Pos.y;
                        //joy_init(&joystick, prevTarget_Pos);
                        //joy_sync_to_position(&joystick, prevTarget_Pos);
                    }
                    else
                    {
                        arm.joints[3].x = target_pos.x;
                        arm.joints[3].y = target_pos.y;
                    }

                    IK_CONVERGENCE_E ik_result = CONVERGES;// fsciks.validate_and_constrain_target_with_decel(arm, &target, target_pos, target_vel, &dynamic_constraints, &limit_state); //  TODO - RM: LET'S ROLL IT OURSELVES!

                    // If target was modified, sync joystick to prevent runaway
                    //if (fabs(target.x - original_target.x) > 0.001 || fabs(target.y - original_target.y) > 0.001)
                    //{
                    //    controller.pos.x = arm.joints[3].x;
                    //    controller.pos.y = arm.joints[3].y;
                    //    controller.vel = (mc2D_vec2_t)(0.0f, 0.0f);
                    //    controller.state = MC_STATE_ACCELERATING;
                    //    controller.settled_count = 0;

                    //    target_pos.x = arm.joints[3].x;
                    //    target_pos.y = arm.joints[3].y;
                    //    joy_init(&joystick, target_pos);
                    //    //joy_sync_to_position(&joystick, target);
                    //}

                    //if (ik_result != CONVERGES) {
                    //    // Can't reach target - stop at current position
                    //    controller.pos.x = arm.joints[3].x;
                    //    controller.pos.y = arm.joints[3].y;
                    //    controller.vel = (mc2D_vec2_t)(0.0f, 0.0f);
                    //    controller.state = MC_STATE_ACCELERATING;
                    //    controller.settled_count = 0;

                    //    target_pos.x = arm.joints[3].x;
                    //    target_pos.y = arm.joints[3].y;
                    //    joy_init(&joystick, target_pos);
                    //    //joy_sync_to_position(&joystick, target_pos);
                    //}

                    // If target was constrained, sync joystick
                    mc2D_vec2_t temp = { target_pos.x - original_target.x, target_pos.y - original_target.y };
                    if (sqrt(temp.x * temp.x + temp.y * temp.y) > 0.1)
                    {
                        //controller.pos.x = arm.joints[3].x;
                        //controller.pos.y = arm.joints[3].y;
                        //controller.vel = (mc2D_vec2_t)(0.0f, 0.0f);
                        //controller.state = MC_STATE_ACCELERATING;
                        //controller.settled_count = 0;

                        //joy_init(&joystick, target);
                        //joy_sync_to_position(&joystick, target_pos);
                    }

                    /* Compute control */
                    mc2D_vec2_t accel = mc2D_compute_control(&controller, target_pos, target_vel, &dynamic_constraints, 1.0 / FRAME_RATE);

                    /* Update state */
                    mc2D_update_state(&controller, accel, 1.0 / FRAME_RATE);

                    //target = controller.pos;

                    arm.joints[3].x = target.x;
                    arm.joints[3].y = target.y;

                    arm.gripperAngle = toolAngle / RAD_TO_DEG;

                    fsciks.fsciks_init(&arm);

                    IK_CONVERGENCE_E p2Converges = fsciks.calcP2(&arm);
                    IK_CONVERGENCE_E p1Converges = fsciks.calcP1(&arm);

                    std::string ikReturnText = "";
                    switch (p2Converges)
                    {
                    case CONVERGES:
                        ikReturnText = "Could converge";
                        break;
                    case CONVERGENCE_NOT_POSSIBLE:
                        ikReturnText = "Could not converge";
                        break;
                    }

                    // Draw Arm
                    ImVec4 colf = ImVec4(0.0f, 1.0f, 0.4f, 1.0f);
                    ImU32 colArm = ImColor(colf);
                    ImVec4 colf2 = ImVec4(1.0f, 0.1f, 0.1f, 1.0f);
                    ImU32 colGripper = ImColor(colf2);
                    draw_list->AddLine(ImVec2(startPos.x + arm.joints[0].x, startPos.y - arm.joints[0].y), ImVec2(startPos.x + arm.joints[1].x, startPos.y - arm.joints[1].y), colArm, thickness);
                    draw_list->AddLine(ImVec2(startPos.x + arm.joints[1].x, startPos.y - arm.joints[1].y), ImVec2(startPos.x + arm.joints[2].x, startPos.y - arm.joints[2].y), colArm, thickness);
                    draw_list->AddLine(ImVec2(startPos.x + arm.joints[2].x, startPos.y - arm.joints[2].y), ImVec2(startPos.x + arm.joints[3].x, startPos.y - arm.joints[3].y), colGripper, thickness);

                    const int numPoints_polygon = 20;
                    ImVec2 boundsPolygon[numPoints_polygon];
                    // Draw the bounds
                    boundsPolygon[0] = startPos + ImVec2(0, 0);
                    boundsPolygon[1] = startPos + ImVec2(0, 60);
                    boundsPolygon[2] = startPos + ImVec2(60, 60);
                    boundsPolygon[3] = startPos + ImVec2(60, 0);
                    boundsPolygon[4] = startPos + ImVec2(0, 0);
                    boundsPolygon[5] = startPos + ImVec2(0, 0);
                    boundsPolygon[6] = startPos + ImVec2(0, 0);
                    boundsPolygon[7] = startPos + ImVec2(0, 0);
                    boundsPolygon[8] = startPos + ImVec2(0, 0);
                    boundsPolygon[9] = startPos + ImVec2(0, 0);
                    boundsPolygon[10] = startPos + ImVec2(0, 0);
                    boundsPolygon[11] = startPos + ImVec2(0, 0);
                    boundsPolygon[12] = startPos + ImVec2(0, 0);
                    boundsPolygon[13] = startPos + ImVec2(0, 0);
                    boundsPolygon[14] = startPos + ImVec2(0, 0);
                    boundsPolygon[15] = startPos + ImVec2(0, 0);
                    boundsPolygon[16] = startPos + ImVec2(0, 0);
                    boundsPolygon[17] = startPos + ImVec2(0, 0);
                    boundsPolygon[18] = startPos + ImVec2(0, 0);
                    boundsPolygon[19] = startPos + ImVec2(0, 0);

                    for (int i = 0; i < numPoints_polygon; i++)
                    {
                        arm.goZone.pnt[i].x = boundsPolygon[i].x;
                        arm.goZone.pnt[i].y = boundsPolygon[i].y;
                    }
                    fsciks.precalcPolygonValues(arm.goZone); // ideally this is ran once, but since I want to keep the complexPolygon inside the scope of this window it'll re-calc every time.
                    
                    bool pointInBounds = fsciks.pointInPolygon(arm.goZone, numPoints_polygon, point_ts(startPos.x + motionTarget.x, startPos.y - motionTarget.y));
                    float distanceToEdge = fsciks.distance_to_polygon(arm.goZone, numPoints_polygon, point_ts(startPos.x + motionTarget.x, startPos.y - motionTarget.y));

                    ImVec4 colfInsideBounds = ImVec4(0.0f, 1.0f, 0.0f, 0.2f);
                    ImVec4 colfNearBounds = ImVec4(1.0f, 0.3f, 0.0f, 0.2f);
                    ImVec4 colfOutsideBounds = ImVec4(1.0f, 0.0f, 0.0f, 0.2f);
                    ImU32 boundsCol;// = pointInBounds ? ImColor(colfInsideBounds) : ImColor(colfOutsideBounds);
                    if (distanceToEdge > 0.0f)
                    {
                        boundsCol = ImColor(colfOutsideBounds);
                    }
                    else if (distanceToEdge > -5.0f) // turn yellow when we're near the bounds edge - could also affect max velocity/accel??
                    {
                        boundsCol = ImColor(colfNearBounds);
                    }
                    else
                    {
                        boundsCol = ImColor(colfInsideBounds);
                    }
                    draw_list->AddConcavePolyFilled(boundsPolygon, numPoints_polygon, boundsCol);

                    // text debugging
                    ImGui::Text("pos=(%.3f, %.3f), speed=%.3f, state=%s\n",
                        controller.pos.x, controller.pos.y,
                        mc2D_get_speed(&controller),
                        mc2D_get_state_name(controller.state));
                    ImGui::Text("ikReturn: %s", ikReturnText.c_str());
                    ImGui::Text("X/Y Target: %.2f/%.2f", targetPosition.x, targetPosition.y);
                    ImGui::Text("X/Y Setpoint: %.2f/%.2f", motionTarget.x, motionTarget.y);
                    ImGui::Text("X/Y Setpoint (joint constrainted): %.2f/%.2f", target.x, target.y);
                    ImGui::Text("target_pos: %.2f/%.2f", target_pos.x, target_pos.y);
                    ImGui::Text("target_vel: %.2f/%.2f", target_vel.x, target_vel.y);
                    //ImGui::SameLine();
                    ImGui::Text("angle0: %.2f° | %s, [%.2f|%.2f]", fsciks.getAngle(arm, 0) * RAD_TO_DEG, fsciks.check_arm_angle(arm, 0) ? "angles good" : "angles not good", arm.joints[0].angularConstraint.min_angle, arm.joints[0].angularConstraint.max_angle);
                    ImGui::Text("angle1: %.2f° | %s, [%.2f|%.2f]", fsciks.getAngle(arm, 1) * RAD_TO_DEG, fsciks.check_arm_angle(arm, 1) ? "angles good" : "angles not good", arm.joints[1].angularConstraint.min_angle, arm.joints[1].angularConstraint.max_angle);
                    ImGui::Text("angle2: %.2f° | %s, [%.2f|%.2f]", fsciks.getAngle(arm, 2) * RAD_TO_DEG, fsciks.check_arm_angle(arm, 2) ? "angles good" : "angles not good", arm.joints[2].angularConstraint.min_angle, arm.joints[2].angularConstraint.max_angle);
                    //ImGui::SameLine();
                    ImGui::Text("POS0: %.2f/%.2f", arm.joints[0].x, arm.joints[0].y);
                    ImGui::SameLine();
                    ImGui::Text("POS1: %.2f/%.2f", arm.joints[1].x, arm.joints[1].y);
                    ImGui::SameLine();
                    ImGui::Text("POS2: %.2f/%.2f", arm.joints[2].x, arm.joints[2].y);
                    ImGui::SameLine();
                    ImGui::Text("POS3: %.2f/%.2f", arm.joints[3].x, arm.joints[3].y);

                    ImGui::Text("End-effector joint is %s shape", pointInBounds ? "inside" : "outside");
                    prevTarget_Pos = target_pos; // save last known good state
                }
                ImGui::End();
            }
            static bool show_geometry_toggle_window = true;
            // 3. Show a CAN endianess playground window
            if (show_geometry_toggle_window)
            {
                ImGui::SetNextWindowSize(ImVec2(500, 600), ImGuiCond_Appearing);
                ImGui::Begin("Geometry", &show_geometry_toggle_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    ImDrawList* draw_list = ImGui::GetWindowDrawList();
                    ImVec4 colf = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
                    ImU32 col = ImColor(colf);

                    ImVec2 startPos = ImGui::GetCursorScreenPos();
                    ImVec2 windowSize;
                    windowSize.x = ImGui::GetWindowWidth();
                    windowSize.y = ImGui::GetWindowHeight();
                    startPos.x = startPos.x + windowSize.x / 2;
                    startPos.y = startPos.y + 100;// windowSize.y / 2;

                    const int numPoints_polygon = 20;
                    ImVec2 complexPolygon[numPoints_polygon];

                    // Draw an angry pac-man
                    complexPolygon[0] = startPos + ImVec2(0, 0);
                    complexPolygon[1] = startPos + ImVec2(90, 0);
                    complexPolygon[2] = startPos + ImVec2(100, 40);
                    complexPolygon[3] = startPos + ImVec2(70, 40);
                    complexPolygon[4] = startPos + ImVec2(60, 20);
                    complexPolygon[5] = startPos + ImVec2(40, 20);
                    complexPolygon[6] = startPos + ImVec2(30, 40);
                    complexPolygon[7] = startPos + ImVec2(0, 40);
                    complexPolygon[8] = startPos + ImVec2(0, 70);
                    complexPolygon[9] = startPos + ImVec2(30, 100);
                    complexPolygon[10] = startPos + ImVec2(40, 60);
                    complexPolygon[11] = startPos + ImVec2(60, 60);
                    complexPolygon[12] = startPos + ImVec2(70, 100);
                    complexPolygon[13] = startPos + ImVec2(70, 120);
                    complexPolygon[14] = startPos + ImVec2(-20, 120);
                    complexPolygon[15] = startPos + ImVec2(-10, 0);
                    complexPolygon[16] = startPos + ImVec2(0, 0);
                    complexPolygon[17] = startPos + ImVec2(0, 0);
                    complexPolygon[18] = startPos + ImVec2(0, 0);
                    complexPolygon[19] = startPos + ImVec2(0, 0);

                    polygon_ts polygon1;
                    polygon_ts polygon2;

                    for (int i = 0; i < numPoints_polygon; i++)
                    {
                        polygon1.pnt[i].x = complexPolygon[i].x;
                        polygon1.pnt[i].y = complexPolygon[i].y;
                    }
                    fsciks.precalcPolygonValues(polygon1); // ideally this is ran once, but since I want to keep the complexPolygon inside the scope of this window it'll re-calc every time.

                    bool pointInPolygon = fsciks.pointInPolygon(polygon1, numPoints_polygon, point_ts(ImGui::GetMousePos().x, ImGui::GetMousePos().y));
                    float distanceToEdge = fsciks.distance_to_polygon(polygon1, numPoints_polygon, point_ts(ImGui::GetMousePos().x, ImGui::GetMousePos().y));


                    draw_list->AddConcavePolyFilled(complexPolygon, numPoints_polygon, col);

                    // Draw a cross
                    startPos = startPos + ImVec2(0, 150); // offset cross by 150 in y direction
                    complexPolygon[0] = startPos + ImVec2(0, 30);
                    complexPolygon[1] = startPos + ImVec2(35, 30);
                    complexPolygon[2] = startPos + ImVec2(35, 0);
                    complexPolygon[3] = startPos + ImVec2(65, 0);
                    complexPolygon[4] = startPos + ImVec2(65, 30);
                    complexPolygon[5] = startPos + ImVec2(100, 30);
                    complexPolygon[6] = startPos + ImVec2(100, 60);
                    complexPolygon[7] = startPos + ImVec2(65, 60);
                    complexPolygon[8] = startPos + ImVec2(65, 145);
                    complexPolygon[9] = startPos + ImVec2(35, 145);
                    complexPolygon[10] = startPos + ImVec2(35, 60);
                    complexPolygon[11] = startPos + ImVec2(0, 60);
                    complexPolygon[12] = startPos + ImVec2(0, 30);
                    complexPolygon[13] = startPos + ImVec2(0, 30);
                    complexPolygon[14] = startPos + ImVec2(0, 30);
                    complexPolygon[15] = startPos + ImVec2(0, 30);
                    complexPolygon[16] = startPos + ImVec2(0, 30);
                    complexPolygon[17] = startPos + ImVec2(0, 30);
                    complexPolygon[18] = startPos + ImVec2(0, 30);
                    complexPolygon[19] = startPos + ImVec2(0, 30);

                    for (int i = 0; i < numPoints_polygon; i++)
                    {
                        polygon2.pnt[i].x = complexPolygon[i].x;
                        polygon2.pnt[i].y = complexPolygon[i].y;
                    }
                    fsciks.precalcPolygonValues(polygon2); // ideally this is ran once, but since I want to keep the complexPolygon inside the scope of this window it'll re-calc every time.

                    pointInPolygon |= fsciks.pointInPolygon(polygon2, numPoints_polygon, point_ts(ImGui::GetMousePos().x, ImGui::GetMousePos().y));
                    float distanceToEdge2 = fsciks.distance_to_polygon(polygon2, numPoints_polygon, point_ts(ImGui::GetMousePos().x, ImGui::GetMousePos().y));

                    float minDistance = (distanceToEdge > distanceToEdge2) ? distanceToEdge2 : distanceToEdge;

                    ImGui::Text("Cursor: %.0f/%.0f", ImGui::GetMousePos().x, ImGui::GetMousePos().y);
                    ImGui::Text("Cursor is %s shape", pointInPolygon ? "inside" : "outside");
                    ImGui::Text("Cursor is %.0f away from edge", minDistance);
                    ImGui::Text("Cursor is %.0f away from poly1", distanceToEdge);
                    ImGui::Text("Cursor is %.0f away from poly2", distanceToEdge2);

                    draw_list->AddConcavePolyFilled(complexPolygon, numPoints_polygon, col);
                }
                ImGui::End();
            }

            static bool show_lamp_toggle_window = true;
            // 3. Show a CAN endianess playground window
            if (show_lamp_toggle_window)
            {
                ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiCond_Appearing);
                ImGui::Begin("Lamp Toggle DEBUG", &show_lamp_toggle_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    typedef enum
                    {
                        LAMP_OFF,
                        LAMP_LEFT,
                        LAMP_RIGHT
                    } lampStatus;

                    static lampStatus lampOnStatus = LAMP_OFF; // 0 is neither, 1 is left, 2 is right

                    static bool leftPushButtonReleased = false;
                    static bool rightPushButtonReleased = false;

                    ImGui::Button("Left Button");
                    if (ImGui::IsItemActive())
                    {
                        if (lampOnStatus == LAMP_LEFT) // if left lamp is already ON
                        {
                            if (leftPushButtonReleased) // turn off lamp only if have released the button
                            {
                                lampOnStatus = LAMP_OFF;
                            }
                        }
                        else
                        {
                            if (leftPushButtonReleased) // turn off lamp only if have released the button
                            {
                                lampOnStatus = LAMP_LEFT; // if our right lamp isn't already on, turn on the right lamp
                            }
                        }
                        leftPushButtonReleased = false;
                    }
                    else
                    {
                        leftPushButtonReleased = true;
                    }
                    ImGui::SameLine();
                    ImGui::Button("Right Button");
                    if (ImGui::IsItemActive())
                    {
                        if (lampOnStatus == LAMP_RIGHT) // if right lamp is already ON
                        {
                            if (rightPushButtonReleased) // turn off lamp only if have released the button
                            {
                                lampOnStatus = LAMP_OFF;
                            }
                        }
                        else
                        {
                            if (rightPushButtonReleased) // turn off lamp only if have released the button
                            {
                                lampOnStatus = LAMP_RIGHT; // if our right lamp isn't already on, turn on the right lamp
                            }
                        }
                        rightPushButtonReleased = false;
                    }
                    else
                    {
                        rightPushButtonReleased = true;
                    }

                    if (lampOnStatus == LAMP_LEFT) // TURN LEFT LAMP ON
                    {
                        ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.5, 0.6f, 0.6f));
                        ImGui::Button("LEFT LAMP");
                        ImGui::PopStyleColor();
                    }
                    else // TURN LEFT LAMP OFF
                    {
                        ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.5, 0.6f, 0.2f));
                        ImGui::Button("LEFT LAMP");
                        ImGui::PopStyleColor();
                    }
                    ImGui::SameLine();

                    if (lampOnStatus == LAMP_RIGHT) // TURN RIGHT LAMP ON
                    {
                        ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.5, 0.6f, 0.6f));
                        ImGui::Button("RIGHT LAMP");
                        ImGui::PopStyleColor();
                    }
                    else // TURN RIGHT LAMP OFF
                    {
                        ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.5, 0.6f, 0.2f));
                        ImGui::Button("RIGHT LAMP");
                        ImGui::PopStyleColor();
                    }
                }
                ImGui::End();
            }
            static bool show_atan2_window = true;
            // 3. Show a CAN endianess playground window
            if (show_atan2_window)
            {
                ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiCond_Appearing);
                ImGui::Begin("atan2 DEBUG", &show_atan2_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    float fscAtan2 = 0;
                    float realAtan2 = 0;
                    static float inputValY = 0;
                    static float inputValX = 0;
                    ImGui::SliderFloat("input Y", &inputValY, -1, 1);
                    ImGui::SliderFloat("input X", &inputValX, -1, 1);
                    fscAtan2 = fsc_atan2f(inputValY, inputValX) * RAD_TO_DEG;
                    realAtan2 = atan2(inputValY, inputValX) * RAD_TO_DEG;
                    ImGui::Text("fsc_atan2: %.1f °", fscAtan2);
                    ImGui::Text("act_atan2: %.1f °", realAtan2);
                }
                ImGui::End();
            }

            static bool show_SAFOUT_DEBUG_window = true;
            // 3. Show a CAN endianess playground window
            if (show_SAFOUT_DEBUG_window)
            {
                ImGui::SetNextWindowSize(ImVec2(1200, 800), ImGuiCond_Appearing);
                ImGui::Begin("SAFOUT DEBUG", &show_SAFOUT_DEBUG_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    static bds_safout_ts outputPinStatus;
                    static safout_status_ts statusStruct;
                    static safout_dfc_ts dfcStruct;

                    outputPinStatus.status_b16 = 0;
                    ImGui::Checkbox("Initialized", &statusStruct.initialized);
                    ImGui::Checkbox("configured", &statusStruct.configured);
                    ImGui::Checkbox("activeOut1", &statusStruct.activeOut1);
                    ImGui::Checkbox("activeOut2", &statusStruct.activeOut2);
                    ImGui::Checkbox("activeOut3", &statusStruct.activeOut3);
                    ImGui::Checkbox("activeOut4", &statusStruct.activeOut4);
                    ImGui::Checkbox("lockedChnl", &statusStruct.lockedChnl);
                    ImGui::Checkbox("iDevCmpHSLS", &statusStruct.iDevCmpHSLS);
                    ImGui::Checkbox("ignoreError", &statusStruct.ignoreError);

                    if (statusStruct.initialized)
                        outputPinStatus.status_b16 |= SAFOUT_INITIALIZED_MASK;
                    if (statusStruct.configured)
                        outputPinStatus.status_b16 |= SAFOUT_CONFIGURED_MASK;
                    if (statusStruct.activeOut1)
                        outputPinStatus.status_b16 |= SAFOUT_ACTIVEOUT1_MASK;
                    if (statusStruct.activeOut2)
                        outputPinStatus.status_b16 |= SAFOUT_ACTIVEOUT2_MASK;
                    if (statusStruct.activeOut3)
                        outputPinStatus.status_b16 |= SAFOUT_ACTIVEOUT3_MASK;
                    if (statusStruct.activeOut4)
                        outputPinStatus.status_b16 |= SAFOUT_ACTIVEOUT4_MASK;
                    if (statusStruct.lockedChnl)
                        outputPinStatus.status_b16 |= SAFOUT_LOCKEDCHNL_MASK;
                    if (statusStruct.iDevCmpHSLS)
                        outputPinStatus.status_b16 |= SAFOUT_IDEVCMPHSLS_MASK;
                    if (statusStruct.ignoreError)
                        outputPinStatus.status_b16 |= SAFOUT_IGNOREERROR_MASK;

                    outputPinStatus.diag_u32 = 0;
                    static int tempDFC1 = 0;
                    static int tempDFC2 = 0;
                    static int tempDFC3 = 0;
                    static int tempDFC4 = 0;
                    ImGui::SliderInt("DFC_Out1", &tempDFC1, 0, 4);
                    dfcStruct.dfcOut1 = (safout_dfc_te)tempDFC1;
                    ImGui::SliderInt("DFC_Out2", &tempDFC2, 0, 4);
                    dfcStruct.dfcOut2 = (safout_dfc_te)tempDFC2;
                    ImGui::SliderInt("DFC_Out3", &tempDFC3, 0, 4);
                    dfcStruct.dfcOut3 = (safout_dfc_te)tempDFC3;
                    ImGui::SliderInt("DFC_Out4", &tempDFC4, 0, 4);
                    dfcStruct.dfcOut4 = (safout_dfc_te)tempDFC4;

                    if (dfcStruct.dfcOut1 == SAFOUT_DFC_SCB)
                        outputPinStatus.diag_u32 |= SAFOUT_SCB << 4;
                    if (dfcStruct.dfcOut1 == SAFOUT_DFC_SCG)
                        outputPinStatus.diag_u32 |= SAFOUT_SCG << 4;
                    if (dfcStruct.dfcOut1 == SAFOUT_DFC_OL)
                        outputPinStatus.diag_u32 |= SAFOUT_OL << 4;
                    if (dfcStruct.dfcOut1 == SAFOUT_DFC_OC)
                        outputPinStatus.diag_u32 |= SAFOUT_OC << 4;

                    if (dfcStruct.dfcOut2 == SAFOUT_DFC_SCB)
                        outputPinStatus.diag_u32 |= (SAFOUT_SCB << 8);
                    if (dfcStruct.dfcOut2 == SAFOUT_DFC_SCG)
                        outputPinStatus.diag_u32 |= (SAFOUT_SCG << 8);
                    if (dfcStruct.dfcOut2 == SAFOUT_DFC_OL)
                        outputPinStatus.diag_u32 |= (SAFOUT_OL << 8);
                    if (dfcStruct.dfcOut2 == SAFOUT_DFC_OC)
                        outputPinStatus.diag_u32 |= (SAFOUT_OC << 8);

                    if (dfcStruct.dfcOut3 == SAFOUT_DFC_SCB)
                        outputPinStatus.diag_u32 |= (SAFOUT_SCB << 12);
                    if (dfcStruct.dfcOut3 == SAFOUT_DFC_SCG)
                        outputPinStatus.diag_u32 |= (SAFOUT_SCG << 12);
                    if (dfcStruct.dfcOut3 == SAFOUT_DFC_OL)
                        outputPinStatus.diag_u32 |= (SAFOUT_OL << 12);
                    if (dfcStruct.dfcOut3 == SAFOUT_DFC_OC)
                        outputPinStatus.diag_u32 |= (SAFOUT_OC << 12);

                    if (dfcStruct.dfcOut4 == SAFOUT_DFC_SCB)
                        outputPinStatus.diag_u32 |= (SAFOUT_SCB << 16);
                    if (dfcStruct.dfcOut4 == SAFOUT_DFC_SCG)
                        outputPinStatus.diag_u32 |= (SAFOUT_SCG << 16);
                    if (dfcStruct.dfcOut4 == SAFOUT_DFC_OL)
                        outputPinStatus.diag_u32 |= (SAFOUT_OL << 16);
                    if (dfcStruct.dfcOut4 == SAFOUT_DFC_OC)
                        outputPinStatus.diag_u32 |= (SAFOUT_OC << 16);

                    ImGui::SliderInt("iOut1", &outputPinStatus.iOut1_u16, 0, 4000);
                    ImGui::SliderInt("iOut2", &outputPinStatus.iOut2_u16, 0, 4000);
                    ImGui::SliderInt("iOut3", &outputPinStatus.iOut3_u16, 0, 4000);
                    ImGui::SliderInt("iOut4", &outputPinStatus.iOut4_u16, 0, 4000);
                    ImGui::SliderInt("iOutCo", &outputPinStatus.iOutCo_u16, 0, 4000);
                    ImGui::SliderInt("iSet1", &outputPinStatus.setOut1_u16, 0, 4000);
                    ImGui::SliderInt("iSet2", &outputPinStatus.setOut2_u16, 0, 4000);
                    ImGui::SliderInt("iSet3", &outputPinStatus.setOut3_u16, 0, 4000);
                    ImGui::SliderInt("iSet4", &outputPinStatus.setOut4_u16, 0, 4000);
                    ImGui::SliderInt("iSetCo", &outputPinStatus.setOutCo_u16, 0, 4000);
                    Send_SAFOUT_DebugToCanTX(outputPinStatus, RBR_CAN_1_E, 0x69, 0x420, 0);
                }
                ImGui::End();
            }

            // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
            if (show_demo_window)
                ImGui::ShowDemoWindow(&show_demo_window);

            // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
            {
                static float f = 0.0f;
                static int counter = 0;

                ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.
                const uint8_t NAME_NUM_CHARS = 8;
                uint8_t originalName[NAME_NUM_CHARS];
                originalName[0] = 'S';
                originalName[1] = 'e';
                originalName[2] = 't';
                originalName[3] = 'u';
                originalName[4] = 'p';
                originalName[5] = ' ';
                originalName[6] = 'A';
                originalName[7] = 0;

                uint8_t newName[NAME_NUM_CHARS];
                newName[0] = 'S';
                newName[1] = 'e';
                newName[2] = 't';
                newName[3] = 'u';
                newName[4] = 'p';
                newName[5] = ' ';
                newName[6] = 'B';
                newName[7] = 0;

                ImGui::Text("memcmp result: %d", memcmp(originalName, newName, NAME_NUM_CHARS));
                ImGui::Text("before memcpy: %s", originalName);
                memcpy(originalName, newName, NAME_NUM_CHARS);
                ImGui::Text("memcpy result: %s", originalName);

                ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
                ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
                ImGui::Checkbox("Another Window", &show_another_window);

                ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
                ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

                if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
                    counter++;
                ImGui::SameLine();
                ImGui::Text("counter = %d", counter);

                ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
                static uint64 prevButtonPress = millis();
                static bool myBool = false;
                if (ImGui::Button("Click for 2s + 2s delay"))
                {
                    prevButtonPress = millis() + 1000;
                }
                static int onTime = 1000;
                static int offTime = 500;
                ImGui::SliderInt("ON TIME", &onTime, 1, 2000);
                ImGui::SliderInt("OFF TIME", &offTime, 1, 2000);
                static uint64_t prevPeriodTime = millis();
                static uint64_t prevOnTime = millis();
                if (Timer(prevPeriodTime, onTime + offTime, true))
                {
                    myBool = true;
                    prevOnTime = millis();
                }
                if (Timer(prevOnTime, onTime, false))
                {
                    myBool = false;
                }
                ImGui::Text("myBool: %d", myBool);

                static uint64_t prevIncrementTime = 0;
                static uint64_t partialIncrementTime = 0;
                static uint64_t partialNonIncrementTime = 0;
                static int incTimeout = 1;
                static int64_t adjustedTimeout = incTimeout;
                static bool machineRunning = false;
                static bool firstLoop = true;
                ImGui::Checkbox("Machine Running", &machineRunning);
                ImGui::SliderInt("Increment Time", &incTimeout, 0, 10);

                static int valueToBeIncremented = 0;
                static bool prevMachineRunning = false;
                if (machineRunning)
                {
                    if (firstLoop || machineRunning != prevMachineRunning) // one shot when running changes
                    {
                        firstLoop = false;
                        partialIncrementTime += partialNonIncrementTime; // sum all partials times together so we always count properly
                    }
                    uint64_t adjustedTimeout = incTimeout * MS_PER_S * S_PER_MIN + partialIncrementTime;// partialNonIncrementTime;
                    if (Timer(prevIncrementTime, adjustedTimeout, true))
                    {
                        valueToBeIncremented += (incTimeout * S_PER_MIN);
                        partialNonIncrementTime = 0;
                        partialIncrementTime = 0;
                    }
                }
                else
                {
                    static uint64_t start = 0;
                    if (firstLoop || machineRunning != prevMachineRunning) // one shot when running changes
                    {
                        firstLoop = false;
                        start = millis();
                    }
                    partialNonIncrementTime = millis() - start; // TODO - RM: THIS ONLY TAKES INTO ACCOUNT ONE TOGGLE BETWEEN machineRunning = true AND machineRunning = false!!
                }
                prevMachineRunning = machineRunning;
                uint32_t actualOperatingHours = valueToBeIncremented + ((millis() - prevIncrementTime - partialNonIncrementTime) / MS_PER_S);
                ImGui::Text("incremented value: %d", valueToBeIncremented);
                ImGui::Text("prevIncrementTime: %d", prevIncrementTime);
                ImGui::Text("partialIncrementTime: %d", partialIncrementTime);
                ImGui::Text("actualOperatingHours: %d", actualOperatingHours);
                float testFloat = 1.23f;
                ImGui::Text("float: %f, (uint64)float: %f", testFloat, (uint64_t)testFloat);
                ImGui::End();
            }

            // 3. Show another simple window.
            if (show_another_window)
            {
                ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                ImGui::Text("Hello from another window!");
                if (ImGui::Button("Close Me"))
                    show_another_window = false;
                ImGui::End();
            }


            static bool show_scale_to_curve_window = true;
            // 3. Show a CAN endianess playground window
            if (show_scale_to_curve_window)
            {
                ImGui::SetNextWindowSize(ImVec2(1200, 800), ImGuiCond_Appearing);
                ImGui::Begin("Scale to curve", &show_scale_to_curve_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    static int resistance = 0;
                    static float waveform = 0;
                    ImGui::SliderInt("resistance", &resistance, -1, 6000, "%d ohm");
                    ImGui::SliderFloat("waveformVal", &waveform, -1.0, 3.0);
                    const int CURVE_LEN = 20;
                    static CURVE_T ERT120_CURVE = {
                        {5428, 5381, 5305, 5190, 5023, 4790, 4486, 4112, 3682, 3220, 2753, 2307, 1905, 1554, 1259, 1016, 820, 662, 536, 436},
                        {-40,  -30,  -20,  -10,  0,    10,   20,   30,   40,   50,   60,   70,   80,   90,   100,  110,  120, 130, 140, 150},
                        CURVE_LEN
                    };

                    const int NUM_TEST_PNTS = 50;
                    // Sine wave: 2 full periods over 2 seconds
                    const CURVE_T sine_wave = {
                        {
                            0.000, 0.041, 0.082, 0.122, 0.163, 0.204, 0.245, 0.286, 0.327, 0.367,
                            0.408, 0.449, 0.490, 0.531, 0.571, 0.612, 0.653, 0.694, 0.735, 0.776,
                            0.816, 0.857, 0.898, 0.939, 0.980, 1.020, 1.061, 1.102, 1.143, 1.184,
                            1.224, 1.265, 1.306, 1.347, 1.388, 1.429, 1.469, 1.510, 1.551, 1.592,
                            1.633, 1.673, 1.714, 1.755, 1.796, 1.837, 1.878, 1.918, 1.959, 2.000
                        },
                        {
                            0.000, 0.254, 0.491, 0.696, 0.855, 0.959, 0.999, 0.975, 0.887, 0.740,
                            0.546, 0.315, 0.064, -0.191, -0.434, -0.648, -0.820, -0.938, -0.995, -0.987,
                            -0.914, -0.782, -0.598, -0.375, -0.128, 0.128, 0.375, 0.598, 0.782, 0.914,
                            0.987, 0.995, 0.938, 0.820, 0.648, 0.434, 0.191, -0.064, -0.315, -0.546,
                            -0.740, -0.887, -0.975, -0.999, -0.959, -0.855, -0.696, -0.491, -0.254, -0.000
                        },
                        NUM_TEST_PNTS
                    };

                    // Square wave: 2 periods, 1s high, 1s low
                    const CURVE_T square_wave = {
                        {
                            0.000, 0.041, 0.082, 0.122, 0.163, 0.204, 0.245, 0.286, 0.327, 0.367,
                            0.408, 0.449, 0.490, 0.531, 0.571, 0.612, 0.653, 0.694, 0.735, 0.776,
                            0.816, 0.857, 0.898, 0.939, 0.980, 1.020, 1.061, 1.102, 1.143, 1.184,
                            1.224, 1.265, 1.306, 1.347, 1.388, 1.429, 1.469, 1.510, 1.551, 1.592,
                            1.633, 1.673, 1.714, 1.755, 1.796, 1.837, 1.878, 1.918, 1.959, 2.000
                        },
                        {
                            1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000,
                            1.000, 1.000, 1.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,
                            0.000, 0.000, 0.000, 0.000, 0.000, 1.000, 1.000, 1.000, 1.000, 1.000,
                            1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 0.000, 0.000, 0.000,
                            0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000
                        },
                        NUM_TEST_PNTS
                    };

                    // Triangle wave: 0 → 1 → 0 repeated for 2 cycles
                    const CURVE_T triangle_wave = {
                        {
                            0.000, 0.041, 0.082, 0.122, 0.163, 0.204, 0.245, 0.286, 0.327, 0.367,
                            0.408, 0.449, 0.490, 0.531, 0.571, 0.612, 0.653, 0.694, 0.735, 0.776,
                            0.816, 0.857, 0.898, 0.939, 0.980, 1.020, 1.061, 1.102, 1.143, 1.184,
                            1.224, 1.265, 1.306, 1.347, 1.388, 1.429, 1.469, 1.510, 1.551, 1.592,
                            1.633, 1.673, 1.714, 1.755, 1.796, 1.837, 1.878, 1.918, 1.959, 2.000
                        },
                        {
                            0.000, 0.163, 0.327, 0.490, 0.653, 0.816, 0.980, 1.143, 1.306, 1.469,
                            1.633, 1.796, 1.959, 1.878, 1.714, 1.551, 1.388, 1.224, 1.061, 0.898,
                            0.735, 0.571, 0.408, 0.245, 0.082, 0.082, 0.245, 0.408, 0.571, 0.735,
                            0.898, 1.061, 1.224, 1.388, 1.551, 1.714, 1.878, 1.959, 1.796, 1.633,
                            1.469, 1.306, 1.143, 0.980, 0.816, 0.653, 0.490, 0.327, 0.163, 0.000
                        },
                        NUM_TEST_PNTS
                    };

                    // Sawtooth wave: Repeats 0 → 1 twice over 2 seconds
                    const CURVE_T sawtooth_wave = {
                        {
                            0.000, 0.041, 0.082, 0.122, 0.163, 0.204, 0.245, 0.286, 0.327, 0.367,
                            0.408, 0.449, 0.490, 0.531, 0.571, 0.612, 0.653, 0.694, 0.735, 0.776,
                            0.816, 0.857, 0.898, 0.939, 0.980, 1.020, 1.061, 1.102, 1.143, 1.184,
                            1.224, 1.265, 1.306, 1.347, 1.388, 1.429, 1.469, 1.510, 1.551, 1.592,
                            1.633, 1.673, 1.714, 1.755, 1.796, 1.837, 1.878, 1.918, 1.959, 2.000
                        },
                        {
                            0.000, 0.041, 0.082, 0.122, 0.163, 0.204, 0.245, 0.286, 0.327, 0.367,
                            0.408, 0.449, 0.490, 0.531, 0.571, 0.612, 0.653, 0.694, 0.735, 0.776,
                            0.816, 0.857, 0.898, 0.939, 0.980, 0.020, 0.061, 0.102, 0.143, 0.184,
                            0.224, 0.265, 0.306, 0.347, 0.388, 0.429, 0.469, 0.510, 0.551, 0.592,
                            0.633, 0.673, 0.714, 0.755, 0.796, 0.837, 0.878, 0.918, 0.959, 0.000
                        },
                        NUM_TEST_PNTS
                    };

                    double temperature = scaleToCurve(ERT120_CURVE, resistance, true);
                    ImGui::Text("Resistance: %d", resistance);
                    ImGui::Text("Temperature: %f", temperature);

                    double output = scaleToCurve(triangle_wave, waveform, false);
                    ImGui::Text("waveform: %f", waveform);
                    ImGui::Text("output: %f", output);

                    double scaleTest = scale(resistance, 0, 1000, 0, 1000, false);
                    ImGui::Text("scaleTest: %f", scaleTest);

                    //char a = 'a';
                    //char bigA = 'A';
                    //char one = '1';
                    //ImGui::Text("a: %c", a);
                    //ImGui::Text("a toupper(): %c", toupper(a));
                    //ImGui::Text("A: %c", toupper(bigA));
                    //ImGui::Text("A toupper(): %c", toupper(bigA));
                    //ImGui::Text("one: %c", toupper(one));
                    //ImGui::Text("one toupper(): %c", toupper(one));

                    //const int NUM_HASH_CHARS = 7;
                    //char hash_chars[] = { 'a', '1', 'f', '3', '4', '8', '7' };
                    //ImGui::Text("BEFORE");
                    //for (int i = 0; i < NUM_HASH_CHARS; i++)
                    //{
                    //    ImGui::Text("%c ", hash_chars[i]); ImGui::SameLine();
                    //    hash_chars[i] = toupper(hash_chars[i]);
                    //}
                    //ImGui::Text("");
                    //ImGui::Text("AFTER");

                    //for (int i = 0; i < NUM_HASH_CHARS; i++)
                    //{
                    //    ImGui::Text("%c ", hash_chars[i]); ImGui::SameLine();
                    //}

                }
                ImGui::End();
            }
            {
                static bool show_scale_window = true;
                // 3. Show a CAN endianess playground window
                if (show_scale_window)
                {
                    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_Appearing);
                    ImGui::Begin("Scale", &show_scale_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                    {
                        static float input = 0;
                        static float minIn = 0;
                        static float maxIn = 0;
                        static float minOut = 0;
                        static float maxOut = 0;
                        static bool clipOutput = 0;
                        ImGui::SliderFloat("input", &input, -10000, 10000, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("minIn", &minIn, -10000, 10000, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("maxIn", &maxIn, -10000, 10000, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("minOut", &minOut, -10000, 10000, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("maxOut", &maxOut, -10000, 10000, "%3f", ImGuiSliderFlags_None);
                        ImGui::Checkbox("clipOutput", &clipOutput);
                        double output = scale(input, minIn, maxIn, minOut, maxOut, clipOutput);
                        ImGui::Text("output: %f", output);
                    }
                    ImGui::End();
                }

                static bool show_scale2_window = true;
                // 3. Show a CAN endianess playground window
                if (show_scale2_window)
                {
                    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_Appearing);
                    ImGui::Begin("Scale2", &show_scale2_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                    {
                        static float input = 0;
                        static float minIn = 0;
                        static float maxIn = 0;
                        static float minOut = 0;
                        static float maxOut = 0;
                        static bool clipOutput = 0;
                        ImGui::SliderFloat("input", &input, -10000, 10000, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("minIn", &minIn, -10000, 10000, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("maxIn", &maxIn, -10000, 10000, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("minOut", &minOut, -10000, 10000, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("maxOut", &maxOut, -10000, 10000, "%3f", ImGuiSliderFlags_None);
                        ImGui::Checkbox("clipOutput", &clipOutput);
                        double output = scale(input, minIn, maxIn, minOut, maxOut, clipOutput);
                        ImGui::Text("output: %f", output);
                    }
                    ImGui::End();
                }
            }

            // Rendering
            ImGui::Render();
            const float clear_color_with_alpha[4] = { clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w };
            g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
            g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear_color_with_alpha);
            ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

            //g_pSwapChain->Present(1, 0); // Present with vsync
            g_pSwapChain->Present(0, 0); // Present without vsync
        }
        renderFrame = false;
        // END - IMGUI LOOP
    }

    // Cleanup
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();

    CleanupDeviceD3D();
    ::DestroyWindow(hwnd);
    ::UnregisterClassW(wc.lpszClassName, wc.hInstance);

    return 0;
}

// Helper functions

bool CreateDeviceD3D(HWND hWnd)
{
    // Setup swap chain
    DXGI_SWAP_CHAIN_DESC sd;
    ZeroMemory(&sd, sizeof(sd));
    sd.BufferCount = 2;
    sd.BufferDesc.Width = 0;
    sd.BufferDesc.Height = 0;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1;
    sd.SampleDesc.Quality = 0;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    UINT createDeviceFlags = 0;
    //createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
    D3D_FEATURE_LEVEL featureLevel;
    const D3D_FEATURE_LEVEL featureLevelArray[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0, };
    HRESULT res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
    if (res == DXGI_ERROR_UNSUPPORTED) // Try high-performance WARP software driver if hardware is not available.
        res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
    if (res != S_OK)
        return false;

    CreateRenderTarget();
    return true;
}

void CleanupDeviceD3D()
{
    CleanupRenderTarget();
    if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
    if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
    if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
}

void CreateRenderTarget()
{
    ID3D11Texture2D* pBackBuffer;
    g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
    g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
    pBackBuffer->Release();
}

void CleanupRenderTarget()
{
    if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
}

// Forward declare message handler from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Win32 message handler
// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg)
    {
    case WM_SIZE:
        if (wParam == SIZE_MINIMIZED)
            return 0;
        g_ResizeWidth = (UINT)LOWORD(lParam); // Queue resize
        g_ResizeHeight = (UINT)HIWORD(lParam);
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
            return 0;
        break;
    case WM_DESTROY:
        ::PostQuitMessage(0);
        return 0;
    }
    return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}



/**
 * @brief Returns angular rate in degrees/s, LSB = 0.005 deg/s
 *
 * @param[in] input raw value from MM7 sensor (YAW_RATE, ROLL_RATE, PITCH_RATE)
 * @return angular rate in deg/s
 */
float GetAngularRateFromMM7Raw(uint16_t input)
{
    if (input == 0xFFFF)
    {
        return -999.9F; // Sensor will never send 0xFFFF except in error state, output an equally outrageous value
    }
    float output = (input - 0x8000) * 0.005;
    return output;
}

/**
 * @brief Returns acceleration in m/s^2, LSB = 0.00125 m/s^2
 *
 * @param[in] input raw value from MM7 sensor (AX, AY, AZ)
 * @return Acceleration in m/s^2
 */
float GetAccelerationFromMM7Raw(uint16_t input)
{
    if (input == 0xFFFF)
    {
        return -999.9F; // Sensor will never send 0xFFFF except in error state, output an equally outrageous value
    }
    float output = (input - 0x8000) * 0.00125;
    return output;
}

/**
 * @brief Returns temperature of MM7 sensor value in degrees Celsius
 *
 * @param[in] input raw value from MM7 sensor (TEMP_RATE_Z)
 * @return temperature in degrees Celsius
 */
int16_t GetTemperatureFromMM7Raw(uint8_t input)
{
    if (input == 0xFF)
    {
        return -999; // 0xFF means CRC-error or temperature is below -50C
    }
    if (input == 0xC9)
    {
        return 999; // 0xC9 means temperature is above 150C
    }
    int16_t output = input - 50;
    return output;
}

/**
 * @brief Extracts a value of type bool at a given location and length from an array of uint8's, returns success
 *
 * @param[in] telegram[] array of bytes from CAN Telegram
 * @param[in] sizeOfTelegram size of array (number of elements)
 * @param[inout] output* extracted value
 * @param[in] spnConfig struct of type SPN_Config, location and length of requested value in array
 * @return success status of block.
 */
int ExtractBoolFromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, bool* output, SPN_Config spnConfig)
{
    if ((8 * spnConfig.byte + spnConfig.bit + spnConfig.len) > (sizeOfTelegram * 8)) // Check if we are asking for something outside of telegram's allocation
        return -1;	// Return -1 if we overrun the array?

    int mask = 0;
    int i = 0;
    for (i; i < spnConfig.len; i++)
    {
        mask |= (1 << i);
    }
    int val = (telegram[spnConfig.byte] >> spnConfig.bit) & mask;
    if (val)
        *output = TRUE;
    else
        *output = FALSE;
    return 0;
}

/**
 * @brief Extracts a value of type uint16 at a given location and length from an array of uint8's, returns success
 *
 * @param[in] telegram[] array of bytes from CAN Telegram
 * @param[in] sizeOfTelegram size of array (number of elements)
 * @param[inout] output* extracted value
 * @param[in] spnConfig struct of type SPN_Config, location and length of requested value in array
 * @return success status of block.
 */
int ExtractUint16FromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, uint16_t* output, SPN_Config spnConfig)
{
    if ((8 * spnConfig.byte + spnConfig.bit + spnConfig.len) > (sizeOfTelegram * 8)) // Check if we are asking for something outside of telegram's allocation
        return -1;	// Return -1 if we overrun the array?

    int mask = 0;
    int i = 0;
    for (i; i < spnConfig.len; i++)
    {
        mask |= (1 << i);
    }
    int val = 0;
    uint8_t numBytes = 1 + (spnConfig.bit + spnConfig.len) / 8;
    i = 0;
    for (i; i < numBytes; i++)
    {
        val |= telegram[spnConfig.byte + i] << (8 * i);
    }
    *output = (val >> spnConfig.bit) & mask;
    return 0;
}

/**
 * @brief Extracts a value of type uint8 at a given location and length from an array of uint8's, returns success
 *
 * @param[in] telegram[] array of bytes from CAN Telegram
 * @param[in] sizeOfTelegram size of array (number of elements)
 * @param[inout] output* extracted value
 * @param[in] spnConfig struct of type SPN_Config, location and length of requested value in array
 * @return success status of block.
 */
int ExtractUint8FromCanTelegram(uint8_t telegram[], uint8_t sizeOfTelegram, uint8_t* output, SPN_Config spnConfig)
{
    if ((8 * spnConfig.byte + spnConfig.bit + spnConfig.len) > (sizeOfTelegram * 8)) // Check if we are asking for something outside of telegram's allocation
        return -1;	// Return -1 if we overrun the array?

    int mask = 0;
    int i = 0;
    for (i; i < spnConfig.len; i++)
    {
        mask |= (1 << i);
    }
    int val = 0;
    uint8_t numBytes = 1 + (spnConfig.bit + spnConfig.len) / 8;
    i = 0;
    for (i; i < numBytes; i++)
    {
        val |= telegram[spnConfig.byte + i] << (8 * i);
    }
    *output = (val >> spnConfig.bit) & mask;
    return 0;
}


void EditTransform(float* cameraView, float* cameraProjection, float* matrix, bool editTransformDecomposition)
{
    static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::LOCAL);
    static bool useSnap = false;
    static float snap[3] = { 1.f, 1.f, 1.f };
    static float bounds[] = { -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f };
    static float boundsSnap[] = { 0.1f, 0.1f, 0.1f };
    static bool boundSizing = false;
    static bool boundSizingSnap = false;

    if (editTransformDecomposition)
    {
        if (ImGui::IsKeyPressed(ImGuiKey_T))
            mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
        if (ImGui::IsKeyPressed(ImGuiKey_E))
            mCurrentGizmoOperation = ImGuizmo::ROTATE;
        if (ImGui::IsKeyPressed(ImGuiKey_R)) // r Key
            mCurrentGizmoOperation = ImGuizmo::SCALE;
        if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
            mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
            mCurrentGizmoOperation = ImGuizmo::ROTATE;
        ImGui::SameLine();
        if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
            mCurrentGizmoOperation = ImGuizmo::SCALE;
        if (ImGui::RadioButton("Universal", mCurrentGizmoOperation == ImGuizmo::UNIVERSAL))
            mCurrentGizmoOperation = ImGuizmo::UNIVERSAL;
        float matrixTranslation[3], matrixRotation[3], matrixScale[3];
        //ImGuizmo::DecomposeMatrixToComponents(matrix, matrixTranslation, matrixRotation, matrixScale);
        //ImGui::InputFloat3("Tr", matrixTranslation);
        //ImGui::InputFloat3("Rt", matrixRotation);
        //ImGui::InputFloat3("Sc", matrixScale);
        //ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, matrix);

        if (mCurrentGizmoOperation != ImGuizmo::SCALE)
        {
            if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL))
                mCurrentGizmoMode = ImGuizmo::LOCAL;
            ImGui::SameLine();
            if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD))
                mCurrentGizmoMode = ImGuizmo::WORLD;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_S))
            useSnap = !useSnap;
        ImGui::Checkbox("##UseSnap", &useSnap);
        ImGui::SameLine();

        switch (mCurrentGizmoOperation)
        {
        case ImGuizmo::TRANSLATE:
            ImGui::InputFloat3("Snap", &snap[0]);
            break;
        case ImGuizmo::ROTATE:
            ImGui::InputFloat("Angle Snap", &snap[0]);
            break;
        case ImGuizmo::SCALE:
            ImGui::InputFloat("Scale Snap", &snap[0]);
            break;
        }
        ImGui::Checkbox("Bound Sizing", &boundSizing);
        if (boundSizing)
        {
            ImGui::PushID(3);
            ImGui::Checkbox("##BoundSizing", &boundSizingSnap);
            ImGui::SameLine();
            ImGui::InputFloat3("Snap", boundsSnap);
            ImGui::PopID();
        }
    }

    ImGuiIO& io = ImGui::GetIO();
    float viewManipulateRight = io.DisplaySize.x;
    float viewManipulateTop = 0;
    static ImGuiWindowFlags gizmoWindowFlags = 0;
    if (useWindow)
    {
        ImGui::SetNextWindowSize(ImVec2(800, 400), ImGuiCond_Appearing);
        ImGui::SetNextWindowPos(ImVec2(400, 20), ImGuiCond_Appearing);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, (ImVec4)ImColor(0.35f, 0.3f, 0.3f));
        ImGui::Begin("Gizmo", 0, gizmoWindowFlags);
        ImGuizmo::SetDrawlist();
        float windowWidth = (float)ImGui::GetWindowWidth();
        float windowHeight = (float)ImGui::GetWindowHeight();
        ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, windowWidth, windowHeight);
        viewManipulateRight = ImGui::GetWindowPos().x + windowWidth;
        viewManipulateTop = ImGui::GetWindowPos().y;
        ImGuiWindow* window = ImGui::GetCurrentWindow();
        gizmoWindowFlags = ImGui::IsWindowHovered() && ImGui::IsMouseHoveringRect(window->InnerRect.Min, window->InnerRect.Max) ? ImGuiWindowFlags_NoMove : 0;
    }
    else
    {
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    }

    ImGuizmo::DrawGrid(cameraView, cameraProjection, identityMatrix, 100.f);
    ImGuizmo::DrawCubes(cameraView, cameraProjection, &objectMatrix[0][0], gizmoCount);
    ImGuizmo::Manipulate(cameraView, cameraProjection, mCurrentGizmoOperation, mCurrentGizmoMode, matrix, NULL, useSnap ? &snap[0] : NULL, boundSizing ? bounds : NULL, boundSizingSnap ? boundsSnap : NULL);

    ImGuizmo::ViewManipulate(cameraView, camDistance, ImVec2(viewManipulateRight - 128, viewManipulateTop), ImVec2(128, 128), 0x10101010);

    if (useWindow)
    {
        ImGui::End();
        ImGui::PopStyleColor(1);
    }
}


void Perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar, float* m16)
{
    float ymax, xmax;
    ymax = znear * tanf(fovyInDegrees * 3.141592f / 180.0f);
    xmax = ymax * aspectRatio;
    Frustum(-xmax, xmax, -ymax, ymax, znear, zfar, m16);
}


void Frustum(float left, float right, float bottom, float top, float znear, float zfar, float* m16)
{
    float temp, temp2, temp3, temp4;
    temp = 2.0f * znear;
    temp2 = right - left;
    temp3 = top - bottom;
    temp4 = zfar - znear;
    m16[0] = temp / temp2;
    m16[1] = 0.0;
    m16[2] = 0.0;
    m16[3] = 0.0;
    m16[4] = 0.0;
    m16[5] = temp / temp3;
    m16[6] = 0.0;
    m16[7] = 0.0;
    m16[8] = (right + left) / temp2;
    m16[9] = (top + bottom) / temp3;
    m16[10] = (-zfar - znear) / temp4;
    m16[11] = -1.0f;
    m16[12] = 0.0;
    m16[13] = 0.0;
    m16[14] = (-temp * zfar) / temp4;
    m16[15] = 0.0;
}


void LookAt(const float* eye, const float* at, const float* up, float* m16)
{
    float X[3], Y[3], Z[3], tmp[3];

    tmp[0] = eye[0] - at[0];
    tmp[1] = eye[1] - at[1];
    tmp[2] = eye[2] - at[2];
    Normalize(tmp, Z);
    Normalize(up, Y);

    Cross(Y, Z, tmp);
    Normalize(tmp, X);

    Cross(Z, X, tmp);
    Normalize(tmp, Y);

    m16[0] = X[0];
    m16[1] = Y[0];
    m16[2] = Z[0];
    m16[3] = 0.0f;
    m16[4] = X[1];
    m16[5] = Y[1];
    m16[6] = Z[1];
    m16[7] = 0.0f;
    m16[8] = X[2];
    m16[9] = Y[2];
    m16[10] = Z[2];
    m16[11] = 0.0f;
    m16[12] = -Dot(X, eye);
    m16[13] = -Dot(Y, eye);
    m16[14] = -Dot(Z, eye);
    m16[15] = 1.0f;
}

void Normalize(const float* a, float* r)
{
    float il = 1.f / (sqrtf(Dot(a, a)) + FLT_EPSILON);
    r[0] = a[0] * il;
    r[1] = a[1] * il;
    r[2] = a[2] * il;
}

float Dot(const float* a, const float* b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Cross(const float* a, const float* b, float* r)
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}
