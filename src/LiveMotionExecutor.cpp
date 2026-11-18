#include "LiveMotionExecutor.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <ostream>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

namespace belt {
namespace {

char axisLetter(motion::Axis axis) {
    switch (axis) {
    case motion::Axis::X: return 'X';
    case motion::Axis::Y: return 'Y';
    case motion::Axis::Z: return 'Z';
    }
    throw std::invalid_argument("Invalid axis value");
}

double& coordinate(motion::Position& position, motion::Axis axis) {
    switch (axis) {
    case motion::Axis::X: return position.x;
    case motion::Axis::Y: return position.y;
    case motion::Axis::Z: return position.z;
    }
    throw std::invalid_argument("Invalid axis value");
}

} // namespace

LiveMotionExecutor::LiveMotionExecutor(const BeltConveyorController& definitions)
    : definitions_(definitions) {
    struct DeviceConfiguration {
        motion::Position maximumPosition{1.0, 1.0, 1.0};
        std::set<motion::Axis> axes;
        std::string homeCommand{"G28"};
    };
    std::map<std::string, DeviceConfiguration> configurations;

    for (const auto& entry : definitions_.belts()) {
        const Belt& belt = entry.second;
        beltPorts_.emplace(belt.name, belt.port);
        auto& configuration = configurations[belt.port];
        // Several logical belt names can share a port, but a physical axis must be defined once.
        for (const auto& axis : belt.axes) {
            if (!configuration.axes.insert(axis.first).second) {
                throw std::invalid_argument("Physical axis " +
                                            std::string(1, axisLetter(axis.first)) +
                                            " is configured more than once on port " + belt.port);
            }
            coordinate(configuration.maximumPosition, axis.first) =
                axis.second.maximumPositionMm;
            configuration.homeCommand += " ";
            configuration.homeCommand += axisLetter(axis.first);
        }
    }

    if (beltPorts_.empty()) {
        throw std::invalid_argument("No belts are configured for live operation");
    }
    for (const auto& entry : configurations) {
        if (entry.second.axes.empty()) {
            throw std::invalid_argument("No axes are configured for port " + entry.first);
        }
        auto transport = std::make_unique<motion::MarlinTransport>(entry.first);
        motion::MarlinTransport* transportView = transport.get();
        const SafetyLimits& limits = definitions_.safetyLimits();
        motion::MotionLimits motionLimits{
            entry.second.maximumPosition,
            limits.maximumDistancePerMoveMm,
            std::numeric_limits<double>::max(),
            limits.maximumDuration,
            limits.commandTimeout};
        auto controller = std::make_unique<motion::MotionController>(
            std::move(transport), motionLimits);
        devices_.emplace(entry.first,
                         Device{std::move(controller), transportView, entry.second.homeCommand,
                                false});
    }
}

void LiveMotionExecutor::execute(const Command& command, std::ostream& output) {
    if (const auto* path = std::get_if<PathCommand>(&command)) {
        throw std::runtime_error("PATH " + path->curveName +
                                 " is not available in live mode; the current MotionController "
                                 "API cannot stream a continuous curve safely");
    }
    if (const auto* wait = std::get_if<WaitCommand>(&command)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(wait->durationMs));
        output << "LIVE WAIT " << wait->durationMs << " ms complete\n";
        return;
    }
    if (const auto* coordinated = std::get_if<MultiAxisMoveCommand>(&command)) {
        Device& device = deviceFor(coordinated->beltName);
        const std::string& port = beltPorts_.at(coordinated->beltName);
        ensureConnectedAndHomed(port, device, output);
        motion::Position target = device.controller->position();
        for (const auto& axisTarget : coordinated->targetsMm) {
            coordinate(target, axisTarget.first) = axisTarget.second;
        }
        device.controller->moveTo(target, coordinated->feedrateMmPerMinute);
        output << "LIVE COORDINATED MOVE " << coordinated->beltName;
        for (const auto& axisTarget : coordinated->targetsMm) {
            output << " " << axisLetter(axisTarget.first) << axisTarget.second;
        }
        output << " feedrate=" << coordinated->feedrateMmPerMinute
               << " mm/min complete\n";
        return;
    }
    if (const auto* circle = std::get_if<CircleCommand>(&command)) {
        Device& device = deviceFor(circle->beltName);
        const std::string& port = beltPorts_.at(circle->beltName);
        ensureConnectedAndHomed(port, device, output);
        motion::Position start = device.controller->position();
        const double radius = circle->diameterMm / 2.0;
        start.x = circle->centerXmm + radius;
        if (circle->plane == CirclePlane::XY) {
            start.y = circle->centerSecondMm;
        } else {
            start.z = circle->centerSecondMm;
        }
        device.controller->moveTo(start, circle->feedrateMmPerMinute);
        if (circle->plane == CirclePlane::XY) {
            device.controller->moveArcXY(-radius, 0.0, circle->clockwise,
                                         circle->feedrateMmPerMinute);
        } else {
            device.controller->moveArcXZ(-radius, 0.0, circle->clockwise,
                                         circle->feedrateMmPerMinute);
        }
        output << "LIVE CIRCLE " << circle->beltName << " plane="
               << (circle->plane == CirclePlane::XY ? "XY" : "XZ")
               << " center=(X" << circle->centerXmm << ","
               << (circle->plane == CirclePlane::XY ? "Y" : "Z")
               << circle->centerSecondMm << ") diameter=" << circle->diameterMm
               << " mm direction=" << (circle->clockwise ? "CW" : "CCW")
               << " feedrate=" << circle->feedrateMmPerMinute << " mm/min complete\n";
        return;
    }
    const auto* move = std::get_if<MoveCommand>(&command);
    if (!move) {
        throw std::invalid_argument("Definitions cannot be executed as live commands");
    }

    Device& device = deviceFor(move->beltName);
    ensureConnectedAndHomed(beltPorts_.at(move->beltName), device, output);
    device.controller->moveTo(move->axis, move->targetPositionMm,
                              move->feedrateMmPerMinute);
    output << "LIVE MOVE " << move->beltName << " " << axisLetter(move->axis)
           << " target=" << move->targetPositionMm << " mm feedrate="
           << move->feedrateMmPerMinute << " mm/min complete\n";
}

double LiveMotionExecutor::position(const std::string& beltName,
                                    motion::Axis axis,
                                    std::ostream& output) {
    const auto port = beltPorts_.find(beltName);
    if (port == beltPorts_.end()) {
        throw std::runtime_error("Unknown belt: " + beltName);
    }
    Device& device = deviceFor(beltName);
    ensureConnectedAndHomed(port->second, device, output);
    motion::Position currentPosition = device.controller->position();
    return coordinate(currentPosition, axis);
}

void LiveMotionExecutor::ensureConnectedAndHomed(const std::string& port,
                                                 Device& device,
                                                 std::ostream& output) {
    if (device.homed) return;
    try {
        device.controller->connect();
        // Home only axes configured on this port; setCurrentPosition records that frame locally.
        device.transport->sendCommand(device.homeCommand,
                                      definitions_.safetyLimits().commandTimeout);
        device.controller->setCurrentPosition({0.0, 0.0, 0.0});
        device.homed = true;
        output << "LIVE HOMED " << port << " using " << device.homeCommand << '\n';
    } catch (...) {
        device.controller->disconnect();
        throw;
    }
}

LiveMotionExecutor::Device& LiveMotionExecutor::deviceFor(const std::string& beltName) {
    const auto belt = beltPorts_.find(beltName);
    if (belt == beltPorts_.end()) {
        throw std::runtime_error("Unknown belt: " + beltName);
    }
    const auto device = devices_.find(belt->second);
    if (device == devices_.end()) {
        throw std::runtime_error("No live device exists for belt " + beltName);
    }
    return device->second;
}

}
