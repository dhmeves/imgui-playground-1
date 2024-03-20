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

// START - 3D PROJECTION
#include "ImGuizmo.h" // 3D projection
bool useWindow = true;
int gizmoCount = 1;
static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::TRANSLATE);


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

void EditTransform(float* cameraView, float* cameraProjection, float* matrix, bool editTransformDecomposition);
void Perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar, float* m16);
void Frustum(float left, float right, float bottom, float top, float znear, float zfar, float* m16);
void LookAt(const float* eye, const float* at, const float* up, float* m16);
void Normalize(const float* a, float* r);
float Dot(const float* a, const float* b);
void Cross(const float* a, const float* b, float* r);
// END - 3D PROJECTION

#include "TimersAndCalculations.h" // fsc library

//START - PCAN - USB READING
#include "03_ManualRead.h"

#include <math.h>
#define PI 3.14159265

ManualRead CAN;

const TPCANHandle PcanHandle1 = PCAN_USBBUS1;
//TPCANMsg CANMsg;
//TPCANTimestamp CANTimeStamp;


#define SENSOR_MM7_C_TELEGRAM_1_ID 0x374
#define SENSOR_MM7_C_TELEGRAM_2_ID 0x378
#define SENSOR_MM7_C_TELEGRAM_3_ID 0x37C

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
    ::ShowWindow(hwnd, SW_SHOWDEFAULT);
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
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != nullptr);

    // Our state
    bool show_pcan_window = true;
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    bool done = false;
    while (!done)
    {


        // START - GRAB CAN DATA
        TPCANMsg CANMsg;
        TPCANTimestamp CANTimeStamp;

        // We execute the "Read" function of the PCANBasic   
        //printf("YAW RATE: %f\tPITCH RATE: %f\tROLL RATE: %f\t\n", MM7_C_YAW_RATE, MM7_C_PITCH_RATE, MM7_C_ROLL_RATE);
        //printf("AY: %f\tAX: %f\tAZ: %f\t\n", MM7_C_AY, MM7_C_AX, MM7_C_AZ);
        //printf("\n~~~~~~~~~~~~~~~~~~~~\n");
        TPCANStatus stsResult = CAN_Read(PcanHandle1, &CANMsg, &CANTimeStamp);
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
        }
        // END - GRAB CAN DATA

        // START - IMGUI LOOP
        static uint64_t prevRenderTime = 0;
        const uint64_t renderTimeout = 16;  //  16ms is about 60fps
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
                ImGui::ShowDemoWindow(&show_pcan_window);

            // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
            {
                ImGui::Begin("PCAN USB");                          // Create a window called "Hello, world!" and append into it.TPCANMsg CANMsg;
                ImGui::Text("YAW RATE: %f\tPITCH RATE: %f\tROLL RATE: %f\t\n", MM7_C_YAW_RATE, MM7_C_PITCH_RATE, MM7_C_ROLL_RATE);
                ImGui::Text("AY: %f\tAX: %f\tAZ: %f\t\n", MM7_C_AY, MM7_C_AX, MM7_C_AZ);


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


                float yaw = CalculateYawEuler(MM7_C_AX, MM7_C_AY, MM7_C_AZ);
                float roll = CalculateRollEuler(MM7_C_AX, MM7_C_AY, MM7_C_AZ);
                float pitch = CalculatePitchEuler(MM7_C_AX, MM7_C_AY, MM7_C_AZ);

                ImGui::Text("yaw: %f", yaw);
                ImGui::Text("roll: %f", roll);
                ImGui::Text("pitch: %f", pitch);
                matrixRotation[0] = roll;
                //matrixRotation[1] = yaw; // yaw doesn't work right now cause we aren't combining everything :)
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









            // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
            if (show_demo_window)
                ImGui::ShowDemoWindow(&show_demo_window);

            // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
            {
                static float f = 0.0f;
                static int counter = 0;

                ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

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
