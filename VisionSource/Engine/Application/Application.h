/*
 *  Copyright (c) 2021 Theophilus Eriata
*/

#pragma once

#include <chrono>
#include <vector>

#include "Common/interface/RefCntAutoPtr.hpp"

#include "Graphics/GraphicsEngine/interface/RenderDevice.h"
#include "Graphics/GraphicsEngine/interface/DeviceContext.h"
#include "Graphics/GraphicsEngine/interface/SwapChain.h"

#include "BasicMath.hpp"
#include "GLFW/glfw3.h"

namespace Vision
{

using namespace Diligent;


class Application
{
public:
    Application();
    virtual ~Application();

    //
    // Public API
    //

    IEngineFactory* GetEngineFactory() { return m_pDevice->GetEngineFactory(); }
    IRenderDevice*  GetDevice() { return m_pDevice; }
    IDeviceContext* GetContext() { return m_pImmediateContext; }
    ISwapChain*     GetSwapChain() { return m_pSwapChain; }

    void Quit();

    //
    // Interface
    //

    virtual bool        Initialize() = 0;
    virtual const Char* GetSampleName() const { return m_AppTitle.c_str(); }

    virtual void Start()               = 0;
    virtual void PreUpdate()           = 0;
    virtual void FixedUpdate(float dt) = 0;
    virtual void Update(float dt)      = 0;
    virtual void PostUpdate()          = 0;
    virtual void Render()              = 0;

    enum class Key
    {
        Unknown    = GLFW_KEY_UNKNOWN,
        Space      = GLFW_KEY_SPACE,
        Apostrophe = GLFW_KEY_APOSTROPHE,
        Comma      = GLFW_KEY_COMMA,
        Minus      = GLFW_KEY_MINUS,
        Period     = GLFW_KEY_PERIOD,
        Slash      = GLFW_KEY_SLASH,

        SemiColon = GLFW_KEY_SEMICOLON,
        Equal     = GLFW_KEY_EQUAL,

        LeftBracket  = GLFW_KEY_LEFT_BRACKET,
        BackSlash    = GLFW_KEY_BACKSLASH,
        RightBracket = GLFW_KEY_RIGHT_BRACKET,
        GraveAccent  = GLFW_KEY_GRAVE_ACCENT,
        World1       = GLFW_KEY_WORLD_1,
        World2       = GLFW_KEY_WORLD_2,
        Escape       = GLFW_KEY_ESCAPE,
        Enter        = GLFW_KEY_ENTER,
        Tab          = GLFW_KEY_TAB,
        BackSpace    = GLFW_KEY_BACKSPACE,
        Insert       = GLFW_KEY_INSERT,
        Delete       = GLFW_KEY_DELETE,

        Right    = GLFW_KEY_RIGHT,
        Left     = GLFW_KEY_LEFT,
        Down     = GLFW_KEY_DOWN,
        Up       = GLFW_KEY_UP,
        PageUp   = GLFW_KEY_PAGE_UP,
        PageDown = GLFW_KEY_PAGE_DOWN,

        Home        = GLFW_KEY_HOME,
        End         = GLFW_KEY_END,
        CapsLock    = GLFW_KEY_CAPS_LOCK,
        ScrollLock  = GLFW_KEY_SCROLL_LOCK,
        NumLock     = GLFW_KEY_NUM_LOCK,
        PrintScreen = GLFW_KEY_PRINT_SCREEN,
        Pause       = GLFW_KEY_PAUSE,

        // Function Keys
        F1  = GLFW_KEY_F1,
        F2  = GLFW_KEY_F2,
        F3  = GLFW_KEY_F3,
        F4  = GLFW_KEY_F4,
        F5  = GLFW_KEY_F5,
        F6  = GLFW_KEY_F6,
        F7  = GLFW_KEY_F7,
        F8  = GLFW_KEY_F8,
        F9  = GLFW_KEY_F9,
        F10 = GLFW_KEY_F10,
        F11 = GLFW_KEY_F11,
        F12 = GLFW_KEY_F12,
        F13 = GLFW_KEY_F13,
        f14 = GLFW_KEY_F14,
        F15 = GLFW_KEY_F15,
        F16 = GLFW_KEY_F16,
        F17 = GLFW_KEY_F17,
        F18 = GLFW_KEY_F18,
        F19 = GLFW_KEY_F19,
        F20 = GLFW_KEY_F20,
        F21 = GLFW_KEY_F21,
        F22 = GLFW_KEY_F22,
        F23 = GLFW_KEY_F23,
        F24 = GLFW_KEY_F24,
        F25 = GLFW_KEY_F25,


        // Number Keys
        Key0 = GLFW_KEY_0,
        Key1 = GLFW_KEY_1,
        Key2 = GLFW_KEY_2,
        Key3 = GLFW_KEY_3,
        Key4 = GLFW_KEY_4,
        Key5 = GLFW_KEY_5,
        Key6 = GLFW_KEY_6,
        Key7 = GLFW_KEY_7,
        Key8 = GLFW_KEY_8,
        Key9 = GLFW_KEY_9,

        // Numpad Keys
        NumKey0 = GLFW_KEY_KP_0,
        NumKey1 = GLFW_KEY_KP_1,
        NumKey2 = GLFW_KEY_KP_2,
        NumKey3 = GLFW_KEY_KP_3,
        NumKey4 = GLFW_KEY_KP_4,
        NumKey5 = GLFW_KEY_KP_5,
        NumKey6 = GLFW_KEY_KP_6,
        NumKey7 = GLFW_KEY_KP_7,
        NumKey8 = GLFW_KEY_KP_8,
        NumKey9 = GLFW_KEY_KP_9,

        // Characters
        A = GLFW_KEY_A,
        B = GLFW_KEY_B,
        C = GLFW_KEY_C,
        D = GLFW_KEY_D,
        E = GLFW_KEY_E,
        F = GLFW_KEY_F,
        G = GLFW_KEY_G,
        H = GLFW_KEY_H,
        I = GLFW_KEY_I,
        J = GLFW_KEY_J,
        K = GLFW_KEY_K,
        L = GLFW_KEY_L,
        M = GLFW_KEY_M,
        N = GLFW_KEY_N,
        O = GLFW_KEY_O,
        P = GLFW_KEY_P,
        Q = GLFW_KEY_Q,
        R = GLFW_KEY_R,
        S = GLFW_KEY_S,
        T = GLFW_KEY_T,
        U = GLFW_KEY_U,
        V = GLFW_KEY_V,
        W = GLFW_KEY_W,
        X = GLFW_KEY_X,
        Y = GLFW_KEY_Y,
        Z = GLFW_KEY_Z,

        Decimal     = GLFW_KEY_KP_DECIMAL,
        Divide      = GLFW_KEY_KP_DIVIDE,
        Multiply    = GLFW_KEY_KP_MULTIPLY,
        Subtract    = GLFW_KEY_KP_SUBTRACT,
        Add         = GLFW_KEY_KP_ADD,
        NumpadEnter = GLFW_KEY_KP_ENTER,
        NumpadEqual = GLFW_KEY_KP_EQUAL,

        LeftShift    = GLFW_KEY_LEFT_SHIFT,
        LeftControl  = GLFW_KEY_LEFT_CONTROL,
        LeftAlt      = GLFW_KEY_LEFT_ALT,
        LeftSuper    = GLFW_KEY_LEFT_SUPER,
        RightShift   = GLFW_KEY_RIGHT_SHIFT,
        RightControl = GLFW_KEY_RIGHT_CONTROL,
        RightAlt     = GLFW_KEY_RIGHT_ALT,
        RightSuper   = GLFW_KEY_RIGHT_SUPER,
        Menu         = GLFW_KEY_MENU,


        // mouse buttons
        MB_Left   = GLFW_MOUSE_BUTTON_LEFT,
        MB_Right  = GLFW_MOUSE_BUTTON_RIGHT,
        MB_Middle = GLFW_MOUSE_BUTTON_MIDDLE,
    };
    enum class KeyState
    {
        Release = GLFW_RELEASE,
        Press   = GLFW_PRESS,
        Repeat  = GLFW_REPEAT,
    };
    bool         GetKeyEvent(Key key, KeyState state);
    bool         GetKeyEvent(Key key);
    virtual void KeyEvent(Key key, KeyState state) = 0;
    virtual void MouseEvent(float2 pos)            = 0;
    float2       GetMousePosition();
    float        GetMouseX();
    float        GetMouseY();

protected:
    std::string                        m_AppTitle;
    Uint32                             m_AdapterId   = 0;
    ADAPTER_TYPE                       m_AdapterType = ADAPTER_TYPE_UNKNOWN;
    std::string                        m_AdapterDetailsString;
    bool                               m_bVSync          = false;
    bool                               m_bFullScreenMode = false;
    bool                               m_bShowUI         = true;

private:
    bool CreateWindow(const char* Title, int Width, int Height, int GlfwApiHint);
    bool InitEngine(RENDER_DEVICE_TYPE DevType);
    bool ProcessCommandLine(const char* CmdLine, RENDER_DEVICE_TYPE& DevType);
    void Run();
    void OnKeyEvent(Key key, KeyState state);
    void UpdateAdaptersDialog();

    static void GLFW_ResizeCallback(GLFWwindow* wnd, int w, int h);
    static void GLFW_KeyCallback(GLFWwindow* wnd, int key, int, int state, int);
    static void GLFW_MouseButtonCallback(GLFWwindow* wnd, int button, int state, int);
    static void GLFW_CursorPosCallback(GLFWwindow* wnd, double xpos, double ypos);
    static void GLFW_MouseWheelCallback(GLFWwindow* wnd, double dx, double dy);

    friend int ApplicationMain(const char* cmdLine);

private:
    RefCntAutoPtr<IRenderDevice>  m_pDevice;
    RefCntAutoPtr<IDeviceContext> m_pImmediateContext;
    RefCntAutoPtr<ISwapChain>     m_pSwapChain;
    GLFWwindow*                   m_Window = nullptr;

    GraphicsAdapterInfo             m_AdapterAttribs;
    std::vector<DisplayModeAttribs> m_DisplayModes;

    struct ActiveKey
    {
        Key      key;
        KeyState state;
    };
    std::vector<ActiveKey> m_ActiveKeys;

    using TClock   = std::chrono::high_resolution_clock;
    using TSeconds = std::chrono::duration<float>;

    TClock::time_point m_LastUpdate = {};
};

Application* CreateGLFWApp();

} // namespace Star
