// Live telemetry: one small UDP datagram per frame to 127.0.0.1 (dev/telemetry reads it).
// Fire and forget on a non-blocking socket; nothing waits, and with nobody listening the
// datagrams are simply dropped. Kept in its own file: winsock2.h must precede windows.h.
#include <winsock2.h>
#include <ws2tcpip.h>
#include <xinput.h>
#pragma comment(lib, "ws2_32.lib")

namespace {
SOCKET telemetry_socket = INVALID_SOCKET;
sockaddr_in telemetry_destination{};
int telemetry_open_port = 0;
bool telemetry_failed = false;
}

bool telemetry_send(int port, const char* data, int length) {
    if (port <= 0 || port > 65535 || telemetry_failed) return false;
    if (telemetry_socket == INVALID_SOCKET || telemetry_open_port != port) {
        static bool started = false;
        if (!started) {
            WSADATA wsa{};
            if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { telemetry_failed = true; return false; }
            started = true;
        }
        if (telemetry_socket == INVALID_SOCKET) {
            telemetry_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (telemetry_socket == INVALID_SOCKET) { telemetry_failed = true; return false; }
            u_long non_blocking = 1;
            ioctlsocket(telemetry_socket, FIONBIO, &non_blocking);
        }
        telemetry_destination = {};
        telemetry_destination.sin_family = AF_INET;
        telemetry_destination.sin_port = htons(static_cast<u_short>(port));
        telemetry_destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        telemetry_open_port = port;
    }
    return sendto(telemetry_socket, data, length, 0, reinterpret_cast<const sockaddr*>(&telemetry_destination),
                  sizeof(telemetry_destination)) == length;
}

// The first connected XInput gamepad, for recording how a player flies by hand: thumb
// sticks (-1..1, up and right positive), triggers (0..1) and the button bits. Loaded on
// demand; with no gamepad the slots are polled again only every 2 s (polling an empty
// slot is slow).
bool gamepad_read(float values[6], unsigned& buttons) {
    using GetState = DWORD(WINAPI*)(DWORD, XINPUT_STATE*);
    static GetState get_state = nullptr;
    static bool loaded = false;
    static int slot = -1;
    static ULONGLONG next_scan = 0;
    if (!loaded) {
        loaded = true;
        HMODULE module = LoadLibraryW(L"xinput1_4.dll");
        if (!module) module = LoadLibraryW(L"xinput9_1_0.dll");
        if (module) get_state = reinterpret_cast<GetState>(GetProcAddress(module, "XInputGetState"));
    }
    if (!get_state) return false;
    XINPUT_STATE state{};
    if (slot >= 0 && get_state(static_cast<DWORD>(slot), &state) != ERROR_SUCCESS) slot = -1;
    if (slot < 0) {
        const ULONGLONG now = GetTickCount64();
        if (now < next_scan) return false;
        next_scan = now + 2000;
        for (int i = 0; i < 4 && slot < 0; ++i)
            if (get_state(static_cast<DWORD>(i), &state) == ERROR_SUCCESS) slot = i;
        if (slot < 0) return false;
    }
    const XINPUT_GAMEPAD& g = state.Gamepad;
    values[0] = g.sThumbLX / 32767.0f; values[1] = g.sThumbLY / 32767.0f;
    values[2] = g.sThumbRX / 32767.0f; values[3] = g.sThumbRY / 32767.0f;
    values[4] = g.bLeftTrigger / 255.0f; values[5] = g.bRightTrigger / 255.0f;
    buttons = g.wButtons;
    return true;
}
