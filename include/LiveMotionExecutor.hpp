#pragma once

#include "BeltConveyorController.hpp"

#include "MarlinTransport.hpp"
#include "MotionController.hpp"

#include <map>
#include <memory>
#include <string>

namespace belt {

class LiveMotionExecutor {
public:
    /** Build one Marlin controller per unique configured serial port. */
    explicit LiveMotionExecutor(const BeltConveyorController& definitions);

    /** Execute only validated motion commands; continuous PATH streaming is unsupported. */
    void execute(const Command& command, std::ostream& output);
    /** Read an axis position after ensuring that its port has been homed. */
    double position(const std::string& beltName, motion::Axis axis, std::ostream& output);

private:
    struct Device {
        std::unique_ptr<motion::MotionController> controller;
        motion::MarlinTransport* transport;
        std::string homeCommand;
        // Homing is tracked per physical port because multiple belt names may share a device.
        bool homed{false};
    };

    void ensureConnectedAndHomed(const std::string& port, Device& device,
                                 std::ostream& output);
    Device& deviceFor(const std::string& beltName);

    const BeltConveyorController& definitions_;
    std::map<std::string, std::string> beltPorts_;
    std::map<std::string, Device> devices_;
};

}
