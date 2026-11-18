// ==============================================================================
// VEHICLE HARDWARE ABSTRACTION LAYER (HAL) INTERFACE
// Modern C++20 | Abstract Interface for CAN / CARLA API
// ==============================================================================

#pragma once

#include <vector>
#include <string>
#include "common/autosar_types.hpp"
#include "perception_core/obstacle_detector.hpp"

namespace av::hal {

class IVehicleHal {
public:
    virtual ~IVehicleHal() = default;

    virtual bool connect(const std::string& host, int port) = 0;
    virtual void disconnect() = 0;
    virtual bool is_connected() const = 0;

    virtual bool read_vehicle_state(types::VehicleStateData& state,
                                   std::vector<perception::RawActorData>& out_actors,
                                   types::PerceptionSensorData* out_sensor = nullptr) = 0;

    virtual bool apply_actuator_control(double throttle, double brake, double steer) = 0;
    virtual bool apply_actuator_control(const types::ActuatorCommandData& cmd) {
        return apply_actuator_control(cmd.throttle, cmd.brake, cmd.steer);
    }
};

} // namespace av::hal
