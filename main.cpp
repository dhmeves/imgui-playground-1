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
#include "sudoku.h"

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

const int NUM_JOINTS = 3;
int lengths[NUM_JOINTS] = { 100, 100, 25 };
Fabrik2D::AngularConstraint angularConstraints[NUM_JOINTS];
static Fabrik2D fabrik2D(4, lengths, 100);


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
                ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_Appearing);
                ImGui::Begin("Kinematics", &show_kinematics_toggle_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
                {
                    Fabrik2D::AngularConstraint angularConstraints[3];
                    angularConstraints[0].min_angle = -PI;// -90 / RAD_TO_DEG;
                    angularConstraints[0].max_angle = PI;//90 / RAD_TO_DEG;
                    angularConstraints[1].min_angle = -PI;// -90 / RAD_TO_DEG;
                    angularConstraints[1].max_angle = PI;// 90 / RAD_TO_DEG;
                    angularConstraints[2].min_angle = -PI;// -90 / RAD_TO_DEG;
                    angularConstraints[2].max_angle = PI;// 90 / RAD_TO_DEG;
                    fabrik2D.SetAngularConstraints(angularConstraints);
                    fabrik2D.setTolerance(0.5f);
                    ImDrawList* draw_list = ImGui::GetWindowDrawList();
                    static ImVec2 armPoints[4];
                    static ImVec4 colf = ImVec4(1.0f, 1.0f, 0.4f, 1.0f);
                    const ImU32 col = ImColor(colf);
                    static float thickness = 2.0f;
                    float th = thickness;
                    static ImVec2 inverseKinematics;
                    static float toolAngle;
                    ImGui::SliderFloat("thickness", &thickness, 0, 100);
                    ImGui::SliderFloat("point0", &armPoints[0].x, 0, 100);
                    ImGui::SliderFloat("point1", &armPoints[0].y, 0, 100);
                    ImGui::SliderFloat("point2", &armPoints[1].x, 0, 100);
                    ImGui::SliderFloat("point3", &armPoints[1].y, 0, 100);
                    ImGui::SliderFloat("X-Inverse Kinematics", &inverseKinematics.x, 0, 200);
                    ImGui::SliderFloat("Y-Inverse Kinematics", &inverseKinematics.y, 0, 200);

                    ImGui::Button("Tool Joystick");
                    if (ImGui::IsItemActive())
                        ImGui::GetForegroundDrawList()->AddLine(io.MouseClickedPos[0], io.MousePos, ImGui::GetColorU32(ImGuiCol_Button), 4.0f); // Draw a line between the button and the mouse cursor
                    ImVec2 value_raw = ImGui::GetMouseDragDelta(0, 0.0f);

                    ImGui::SliderFloat("tool angle Kinematics", &toolAngle, 0, 360);
                    ImGui::Text("angle0 %f", fabrik2D.getAngle(0) * RAD_TO_DEG);
                    ImGui::Text("angle1 %f", fabrik2D.getAngle(1) * RAD_TO_DEG);
                    ImGui::Text("angle2 %f", fabrik2D.getAngle(2) * RAD_TO_DEG);
                    ImVec2 startPos = ImGui::GetCursorScreenPos();
                    ImVec2 windowSize;
                    windowSize.x = ImGui::GetWindowWidth();
                    windowSize.y = ImGui::GetWindowHeight();
                    startPos.x = startPos.x + windowSize.x / 2;
                    startPos.y = startPos.y + 200;// windowSize.y / 2;

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

                    static MotionProfile toolSetpointX;
                    static bool firstLoopX = true;
                    if (firstLoopX)
                    {
                        firstLoopX = false;
                        toolSetpointX.max_velocity = 200;
                        toolSetpointX.max_acceleration = 250;
                        toolSetpointX.last_update = 0;
                    }
                    toolSetpointX.target_value = value_raw.x;
                    update_motion(&toolSetpointX);
                    //toolSetpointX.last_update = toolSetpointX.current_value;

                    static MotionProfile toolSetpointY;
                    static bool firstLoopY = true;
                    if (firstLoopY)
                    {
                        firstLoopY = false;
                        toolSetpointY.max_velocity = 200;
                        toolSetpointY.max_acceleration = 250;
                        toolSetpointY.last_update = 0;
                    }
                    toolSetpointY.target_value = value_raw.y;
                    update_motion(&toolSetpointY);
                    //toolSetpointY.last_update = toolSetpointY.current_value;

                    static float ang = 0;
                    static int counter = 0;
                    uint64_t prevTime = 0;
                    if (Timer(prevTime, 10, true))
                    {
                        ang = ((int)(ang + 1)) % 360;
                    }
                    float radius = 30;
                    float x_offset = 100;
                    float y_offset = 150;
                    // Move x and y in a circular motion
                    float x = x_offset + radius * cos(ang * 1000 / 57296);
                    float y = y_offset + radius * sin(ang * 1000 / 57296);
                    draw_list->AddLine(ImVec2(startPos.x + armPoints[0].x, startPos.y + armPoints[0].y), ImVec2(startPos.x + armPoints[1].x, startPos.y + armPoints[1].y), col, th);
                    //fabrik2D.solve(toolSetpointX.current_value, toolSetpointY.current_value, toolAngle / RAD_TO_DEG, lengths);
                    fabrik2D.solve(value_raw.x, value_raw.y, toolAngle / RAD_TO_DEG, lengths);
                    //fabrik2D.solve(inverseKinematics.x, inverseKinematics.y, toolAngle / RAD_TO_DEG, lengths);
                    draw_list->AddLine(ImVec2(startPos.x + fabrik2D.getX(0), startPos.y + fabrik2D.getY(0)), ImVec2(startPos.x + fabrik2D.getX(1), startPos.y + fabrik2D.getY(1)), col, th);
                    draw_list->AddLine(ImVec2(startPos.x + fabrik2D.getX(1), startPos.y + fabrik2D.getY(1)), ImVec2(startPos.x + fabrik2D.getX(2), startPos.y + fabrik2D.getY(2)), col, th);
                    draw_list->AddLine(ImVec2(startPos.x + fabrik2D.getX(2), startPos.y + fabrik2D.getY(2)), ImVec2(startPos.x + fabrik2D.getX(3), startPos.y + fabrik2D.getY(3)), col, th);
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
                        ImGui::SliderFloat("input", &input, -100, 100, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("minIn", &minIn, -100, 100, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("maxIn", &maxIn, -100, 100, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("minOut", &minOut, -100, 100, "%3f", ImGuiSliderFlags_None);
                        ImGui::SliderFloat("maxOut", &maxOut, -100, 100, "%3f", ImGuiSliderFlags_None);
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
