#include "carla_bridge_client.hpp"
#include <iostream>
#include <cstring>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
#else
  #include <sys/socket.h>
  #include <arpa/inet.h>
  #include <unistd.h>
#endif

namespace av::hal {

CarlaBridgeClient::CarlaBridgeClient() {
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    fallback_state_.is_valid = true;
    fallback_state_.velocity_m_s = 0.0;
}

CarlaBridgeClient::~CarlaBridgeClient() {
    disconnect();
#ifdef _WIN32
    WSACleanup();
#endif
}

void CarlaBridgeClient::socket_close() {
#ifdef _WIN32
    if (sock_ != ~0ULL) {
        closesocket(static_cast<SOCKET>(sock_));
        sock_ = ~0ULL;
    }
#else
    if (sock_ >= 0) {
        close(sock_);
        sock_ = -1;
    }
#endif
    connected_ = false;
}

bool CarlaBridgeClient::connect(const std::string& host, int port) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    host_ = host;
    port_ = port;

#ifdef _WIN32
    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) return false;
    sock_ = static_cast<uintptr_t>(s);
#else
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ < 0) return false;
#endif

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr);

    // Timeout 1 giây
#ifdef _WIN32
    DWORD timeout = 1000;
    setsockopt(static_cast<SOCKET>(sock_), SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    setsockopt(static_cast<SOCKET>(sock_), SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
    if (::connect(static_cast<SOCKET>(sock_), (sockaddr*)&server_addr, sizeof(server_addr)) != 0) {
        socket_close();
        return false;
    }
#else
    struct timeval tv{1, 0};
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if (::connect(sock_, (sockaddr*)&server_addr, sizeof(server_addr)) != 0) {
        socket_close();
        return false;
    }
#endif

    connected_ = true;
    std::cout << "[CARLA BRIDGE CLIENT] Đã kết nối thành công tới CARLA Bridge tại "
              << host << ":" << port << std::endl;
    return true;
}

void CarlaBridgeClient::disconnect() {
    std::lock_guard<std::mutex> lock(io_mutex_);
    socket_close();
}

bool CarlaBridgeClient::read_vehicle_state(types::VehicleStateData& state,
                                          std::vector<perception::RawActorData>& out_actors) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    out_actors.clear();

    if (!connected_) {
        if (sim_fallback_) {
            state = fallback_state_;
            return true;
        }
        return false;
    }

    // Đọc header 8 bytes: magic "CARL" (4 bytes) + uint32 payload_len (4 bytes)
    char header[8];
#ifdef _WIN32
    int n = recv(static_cast<SOCKET>(sock_), header, 8, MSG_WAITALL);
#else
    int n = recv(sock_, header, 8, MSG_WAITALL);
#endif
    if (n != 8 || std::memcmp(header, "CARL", 4) != 0) {
        // Mất kết nối hoặc sai header
        return false;
    }

    uint32_t payload_len = 0;
    std::memcpy(&payload_len, header + 4, sizeof(uint32_t));
    std::vector<char> buf(payload_len);
#ifdef _WIN32
    int recvd = recv(static_cast<SOCKET>(sock_), buf.data(), payload_len, MSG_WAITALL);
#else
    int recvd = recv(sock_, buf.data(), payload_len, MSG_WAITALL);
#endif
    if (recvd != static_cast<int>(payload_len)) return false;

    // Unpack State
    size_t offset = 0;
    if (payload_len < sizeof(types::VehicleStateData)) return false;
    std::memcpy(&state, buf.data(), sizeof(types::VehicleStateData));
    offset += sizeof(types::VehicleStateData);

    // Unpack Actors
    if (offset + sizeof(uint32_t) <= payload_len) {
        uint32_t num_actors = 0;
        std::memcpy(&num_actors, buf.data() + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);

        for (uint32_t i = 0; i < num_actors; ++i) {
            if (offset + sizeof(perception::RawActorData) <= payload_len) {
                perception::RawActorData act;
                std::memcpy(&act, buf.data() + offset, sizeof(perception::RawActorData));
                offset += sizeof(perception::RawActorData);
                out_actors.push_back(act);
            }
        }
    }

    fallback_state_ = state;
    return true;
}

bool CarlaBridgeClient::apply_actuator_control(double throttle, double brake, double steer) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (!connected_) {
        // Giả lập động học xe trong chế độ fallback
        double a = (throttle > 0.0) ? (throttle * config::MAX_ACCELERATION_MS2) : (brake * config::MAX_BRAKING_MS2);
        fallback_state_.velocity_m_s = std::max(0.0, fallback_state_.velocity_m_s + a * 0.05);
        fallback_state_.yaw_rad += (fallback_state_.velocity_m_s / config::WHEELBASE_L) * std::tan(steer * config::MAX_WHEEL_ANGLE_RAD) * 0.05;
        fallback_state_.x += fallback_state_.velocity_m_s * std::cos(fallback_state_.yaw_rad) * 0.05;
        fallback_state_.y += fallback_state_.velocity_m_s * std::sin(fallback_state_.yaw_rad) * 0.05;
        fallback_state_.current_steer = steer;
        fallback_state_.acceleration_m_s2 = a;
        return true;
    }

    // Packet: Magic "ACTR" (4 bytes) + throttle (8) + brake (8) + steer (8) = 28 bytes packed
#pragma pack(push, 1)
    struct ActuatorPkt {
        char magic[4]{'A', 'C', 'T', 'R'};
        double throttle{0.0};
        double brake{0.0};
        double steer{0.0};
    } pkt;
#pragma pack(pop)
    pkt.throttle = throttle;
    pkt.brake = brake;
    pkt.steer = steer;

#ifdef _WIN32
    int sent = send(static_cast<SOCKET>(sock_), (const char*)&pkt, sizeof(pkt), 0);
#else
    int sent = send(sock_, (const char*)&pkt, sizeof(pkt), 0);
#endif
    return (sent == sizeof(pkt));
}

} // namespace av::hal
