#include "carla_bridge_client.hpp"
#include <iostream>
#include <cstring>
#include <algorithm>

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
#else
  #include <sys/socket.h>
  #include <sys/ioctl.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <unistd.h>
#endif

namespace av::hal {

#pragma pack(push, 1)
struct BridgeMapCamHeader {
    double map_road_min{-0.45};
    double map_road_max{3.90};
    uint8_t map_is_yellow_solid{1};
    uint8_t is_junction{0};
    uint8_t has_cam_frame{0};
    uint8_t reserved{0};
    uint16_t cam_w{types::CAM_SHM_W};
    uint16_t cam_h{types::CAM_SHM_H};
};

struct ActuatorTelemetryPkt {
    char magic[4]{'A', 'C', 'T', 'R'};
    double throttle{0.0};
    double brake{0.0};
    double steer{0.0};
    double wheel_angle_rad{0.0};
    double accel_cmd{0.0};
    double v_ref_m_s{10.0};
    double jerk_m_s3{0.0};
    double road_min_m{-0.45};
    double road_max_m{3.90};
    double s_ego_m{0.0};
    double s_total_m{0.0};
    double d_yellow_m{1.75};
    double detect_ms{0.0};
    uint32_t current_idx{0};
    uint32_t total_idx{0};
    uint32_t obs_count{0};
    int32_t active_obs_id{-1};
    int8_t bypass_side{0};
    uint8_t is_emergency{0};
    uint8_t is_roadblock{0};
    uint8_t yellow_line_type{2};
    uint8_t yellow_source{2};
    double horizon_x[types::MPC_HORIZON]{};
    double horizon_y[types::MPC_HORIZON]{};
};
#pragma pack(pop)

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

    // Timeout 3 giây & Bật TCP_NODELAY cho cả Windows và Ubuntu
#ifdef _WIN32
    DWORD timeout = 3000;
    setsockopt(static_cast<SOCKET>(sock_), SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    setsockopt(static_cast<SOCKET>(sock_), SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
    BOOL flag_nodelay = TRUE;
    setsockopt(static_cast<SOCKET>(sock_), IPPROTO_TCP, TCP_NODELAY, (const char*)&flag_nodelay, sizeof(flag_nodelay));
    if (::connect(static_cast<SOCKET>(sock_), (sockaddr*)&server_addr, sizeof(server_addr)) != 0) {
        socket_close();
        return false;
    }
#else
    struct timeval tv{3, 0};
    setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    int flag_nodelay = 1;
    setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &flag_nodelay, sizeof(flag_nodelay));
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
                                          std::vector<perception::RawActorData>& out_actors,
                                          types::PerceptionSensorData* out_sensor) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    out_actors.clear();

    if (!connected_) {
        if (sim_fallback_) {
            state = fallback_state_;
            if (out_sensor) {
                out_sensor->map_road_min = config::STANDARD_ROAD_MIN_M;
                out_sensor->map_road_max = config::STANDARD_ROAD_MAX_M;
                out_sensor->map_is_yellow_solid = 1;
                out_sensor->is_junction = 0;
                out_sensor->has_cam_frame = 0;
            }
            return true;
        }
        return false;
    }

    auto read_single_packet = [&](std::vector<char>& out_buf, uint32_t& out_len) -> bool {
        char header[8];
#ifdef _WIN32
        int n = recv(static_cast<SOCKET>(sock_), header, 8, MSG_WAITALL);
#else
        int n = recv(sock_, header, 8, MSG_WAITALL);
#endif
        if (n != 8 || std::memcmp(header, "CARL", 4) != 0) {
            socket_close();
            return false;
        }
        std::memcpy(&out_len, header + 4, sizeof(uint32_t));
        out_buf.resize(out_len);
#ifdef _WIN32
        int recvd = recv(static_cast<SOCKET>(sock_), out_buf.data(), out_len, MSG_WAITALL);
#else
        int recvd = recv(sock_, out_buf.data(), out_len, MSG_WAITALL);
#endif
        if (recvd != static_cast<int>(out_len)) {
            socket_close();
            return false;
        }
        return true;
    };

    auto unpack_camera_if_present = [&](const std::vector<char>& pkt_buf, uint32_t pkt_len) {
        if (pkt_len < sizeof(types::VehicleStateData) + sizeof(BridgeMapCamHeader) + sizeof(uint32_t)) return;
        size_t off = sizeof(types::VehicleStateData);
        BridgeMapCamHeader mc{};
        std::memcpy(&mc, pkt_buf.data() + off, sizeof(BridgeMapCamHeader));
        off += sizeof(BridgeMapCamHeader);
        uint32_t n_act = 0;
        std::memcpy(&n_act, pkt_buf.data() + off, sizeof(uint32_t));
        off += sizeof(uint32_t) + static_cast<size_t>(n_act) * sizeof(perception::RawActorData);
        if (mc.has_cam_frame && off + types::CAM_SHM_BYTES <= pkt_len) {
            std::memcpy(cached_cam_rgb_.data(), pkt_buf.data() + off, types::CAM_SHM_BYTES);
            has_cached_cam_ = true;
        }
    };

    uint32_t payload_len = 0;
    std::vector<char> buf;
    if (!read_single_packet(buf, payload_len)) {
        return false;
    }
    unpack_camera_if_present(buf, payload_len);

    // Xả toàn bộ các gói tin cũ còn tồn đọng trong TCP kernel buffer (hỗ trợ cả Windows & Linux)
    while (true) {
#ifdef _WIN32
        u_long bytes_avail = 0;
        if (ioctlsocket(static_cast<SOCKET>(sock_), FIONREAD, &bytes_avail) != 0 || bytes_avail < 8) {
            break;
        }
        char peek_hdr[8];
        int np = recv(static_cast<SOCKET>(sock_), peek_hdr, 8, MSG_PEEK);
#else
        int bytes_avail = 0;
        if (ioctl(sock_, FIONREAD, &bytes_avail) < 0 || bytes_avail < 8) {
            break;
        }
        char peek_hdr[8];
        int np = recv(sock_, peek_hdr, 8, MSG_PEEK | MSG_DONTWAIT);
#endif
        if (np != 8 || std::memcmp(peek_hdr, "CARL", 4) != 0) {
            break;
        }
        uint32_t next_len = 0;
        std::memcpy(&next_len, peek_hdr + 4, sizeof(uint32_t));
        if (static_cast<uint32_t>(bytes_avail) < (8U + next_len)) {
            break;
        }
        if (!read_single_packet(buf, payload_len)) {
            return false;
        }
        unpack_camera_if_present(buf, payload_len);
    }

    // 1. Unpack VehicleStateData
    size_t offset = 0;
    if (payload_len < sizeof(types::VehicleStateData)) return false;
    std::memcpy(&state, buf.data(), sizeof(types::VehicleStateData));
    offset += sizeof(types::VehicleStateData);

    // 2. Unpack BridgeMapCamHeader (nếu gói tin từ Bridge có kèm dữ liệu OpenDRIVE + Camera)
    BridgeMapCamHeader mc_hdr{};
    if (offset + sizeof(BridgeMapCamHeader) + sizeof(uint32_t) <= payload_len) {
        std::memcpy(&mc_hdr, buf.data() + offset, sizeof(BridgeMapCamHeader));
        offset += sizeof(BridgeMapCamHeader);
    }

    // 3. Unpack Actors
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

    if (out_sensor) {
        out_sensor->map_road_min = mc_hdr.map_road_min;
        out_sensor->map_road_max = mc_hdr.map_road_max;
        out_sensor->map_is_yellow_solid = mc_hdr.map_is_yellow_solid;
        out_sensor->is_junction = mc_hdr.is_junction;
        out_sensor->cam_w = types::CAM_SHM_W;
        out_sensor->cam_h = types::CAM_SHM_H;
        out_sensor->has_cam_frame = has_cached_cam_ ? 1 : 0;
        if (has_cached_cam_) {
            out_sensor->cam_rgb = cached_cam_rgb_;
        }
    }

    fallback_state_ = state;
    return true;
}

bool CarlaBridgeClient::apply_actuator_control(double throttle, double brake, double steer) {
    types::ActuatorCommandData cmd{};
    cmd.throttle = throttle;
    cmd.brake = brake;
    cmd.steer = steer;
    return apply_actuator_control(cmd);
}

bool CarlaBridgeClient::apply_actuator_control(const types::ActuatorCommandData& cmd) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    double throttle = std::clamp(cmd.throttle, 0.0, 1.0);
    double brake = std::clamp(cmd.brake, 0.0, 1.0);
    double steer = std::clamp(cmd.steer, -1.0, 1.0);

    if (!connected_) {
        constexpr double dt_sim = 0.01;
        double a = (throttle > 0.0) ? (throttle * config::MAX_ACCELERATION_MS2) : (brake * config::MAX_BRAKING_MS2);
        fallback_state_.velocity_m_s = std::max(0.0, fallback_state_.velocity_m_s + a * dt_sim);
        fallback_state_.yaw_rad += (fallback_state_.velocity_m_s / config::WHEELBASE_L) * std::tan(steer * config::MAX_WHEEL_ANGLE_RAD) * dt_sim;
        fallback_state_.x += fallback_state_.velocity_m_s * std::cos(fallback_state_.yaw_rad) * dt_sim;
        fallback_state_.y += fallback_state_.velocity_m_s * std::sin(fallback_state_.yaw_rad) * dt_sim;
        fallback_state_.current_steer = steer;
        fallback_state_.acceleration_m_s2 = a;
        return true;
    }

    ActuatorTelemetryPkt pkt{};
    pkt.throttle = throttle;
    pkt.brake = brake;
    pkt.steer = steer;
    pkt.wheel_angle_rad = cmd.wheel_angle_rad;
    pkt.accel_cmd = cmd.acceleration_cmd;
    pkt.v_ref_m_s = cmd.v_ref_m_s;
    pkt.jerk_m_s3 = cmd.jerk_filtered_m_s3;
    pkt.road_min_m = cmd.road_min_m;
    pkt.road_max_m = cmd.road_max_m;
    pkt.s_ego_m = cmd.s_ego_m;
    pkt.s_total_m = cmd.s_total_m;
    pkt.d_yellow_m = cmd.d_yellow_line_m;
    pkt.detect_ms = cmd.detect_time_ms;
    pkt.current_idx = cmd.current_idx;
    pkt.total_idx = cmd.total_idx;
    pkt.obs_count = cmd.obs_count;
    pkt.active_obs_id = cmd.active_obs_id;
    pkt.bypass_side = cmd.bypass_side;
    pkt.is_emergency = cmd.is_emergency_stop ? 1 : 0;
    pkt.is_roadblock = cmd.is_roadblock ? 1 : 0;
    pkt.yellow_line_type = cmd.yellow_line_type;
    pkt.yellow_source = cmd.yellow_source;
    for (size_t k = 0; k < types::MPC_HORIZON; ++k) {
        pkt.horizon_x[k] = cmd.horizon_x[k];
        pkt.horizon_y[k] = cmd.horizon_y[k];
    }

#ifdef _WIN32
    int sent = send(static_cast<SOCKET>(sock_), (const char*)&pkt, sizeof(pkt), 0);
#else
    int sent = send(sock_, (const char*)&pkt, sizeof(pkt), 0);
#endif
    return (sent == sizeof(pkt));
}

} // namespace av::hal
