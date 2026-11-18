// ==============================================================================
// CARLA BRIDGE TCP CLIENT (HAL IMPLEMENTATION)
// Modern C++20 | Cross-platform High-Speed Socket Transport
// ==============================================================================

#pragma once

#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include "vehicle_hal.hpp"

namespace av::hal {

class CarlaBridgeClient : public IVehicleHal {
public:
    CarlaBridgeClient();
    ~CarlaBridgeClient() override;

    bool connect(const std::string& host, int port) override;
    void disconnect() override;
    bool is_connected() const override { return connected_; }

    bool read_vehicle_state(types::VehicleStateData& state,
                           std::vector<perception::RawActorData>& out_actors,
                           types::PerceptionSensorData* out_sensor = nullptr) override;

    bool apply_actuator_control(double throttle, double brake, double steer) override;
    bool apply_actuator_control(const types::ActuatorCommandData& cmd) override;

    void set_simulation_fallback(bool enable) { sim_fallback_ = enable; }
    void set_initial_state(const types::VehicleStateData& init_st) { fallback_state_ = init_st; }

private:
    void socket_close();

    std::string host_{"127.0.0.1"};
    int port_{20050};
    std::atomic<bool> connected_{false};
    bool sim_fallback_{true};
    std::mutex io_mutex_;

#ifdef _WIN32
    uintptr_t sock_{~0ULL}; // INVALID_SOCKET
#else
    int sock_{-1};
#endif

    // Fallback simulation state & cached camera frame
    types::VehicleStateData fallback_state_{};
    bool has_cached_cam_{false};
    std::array<uint8_t, types::CAM_SHM_BYTES> cached_cam_rgb_{};
};

} // namespace av::hal
