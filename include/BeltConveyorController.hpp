#pragma once

#include "MotionTransport.hpp"

#include <array>
#include <cstddef>
#include <chrono>
#include <functional>
#include <iosfwd>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <variant>

namespace belt {

class BeltProgramHost;

struct BeltDeclaration {
    std::string name;
    std::string communication;
    std::string port;
};

struct AxisDeclaration {
    std::string beltName;
    motion::Axis axis;
    double maximumPositionMm;
};

struct RemoveDeclaration {
    std::string beltName;
};

struct SafetyLimitsDeclaration {
    double maximumDistancePerMoveMm;
    std::chrono::milliseconds maximumDuration;
    std::chrono::milliseconds commandTimeout;
};

struct MoveCommand {
    std::string beltName;
    motion::Axis axis;
    double targetPositionMm;
    double feedrateMmPerMinute;
};

struct MultiAxisMoveCommand {
    std::string beltName;
    std::map<motion::Axis, double> targetsMm;
    double feedrateMmPerMinute;
};

enum class CirclePlane {
    XY,
    XZ
};

struct CircleCommand {
    std::string beltName;
    CirclePlane plane;
    double centerXmm;
    double centerSecondMm;
    double diameterMm;
    bool clockwise;
    double feedrateMmPerMinute;
};

struct WaitCommand {
    unsigned long long durationMs;
};

struct PathCommand {
    std::string curveName;
    double feedrateMmPerMinute;
};

using Command =
    std::variant<BeltDeclaration,
                 AxisDeclaration,
                 RemoveDeclaration,
                 SafetyLimitsDeclaration,
                 MoveCommand,
                 MultiAxisMoveCommand,
                 CircleCommand,
                 WaitCommand,
                 PathCommand>;

struct AxisSettings {
    double maximumPositionMm;
};

inline constexpr double kDefaultFeedrateMmPerMinute = 10000.0;

struct Belt {
    std::string name;
    std::string communication;
    std::string port;
    std::map<motion::Axis, AxisSettings> axes;
};

struct Point2D {
    double x;
    double y;
};

struct BezierCurve {
    std::string name;
    std::string beltName;
    std::array<Point2D, 4> controlPoints;
};

struct SafetyLimits {
    double maximumDistancePerMoveMm{220.0};
    std::chrono::milliseconds maximumDuration{30000};
    std::chrono::milliseconds commandTimeout{35000};
};

/**
 * Parses belt definitions and motion commands, validates them against configured
 * axes/limits, and maintains a simulated position for dry-run programs.
 * Hardware access is injected separately through runProgram's live callbacks.
 */
class BeltConveyorController {
public:
    using LiveExecutor = std::function<void(const Command&, std::ostream&)>;
    using PositionProvider = std::function<double(const std::string&, motion::Axis)>;

    static std::optional<Command> parseLine(const std::string& line,
                                            const std::string& source,
                                            std::size_t lineNumber);

    /** Validate and apply one definition/motion command to this controller's state. */
    void apply(const Command& command, std::ostream& output);
    /** Load definitions transactionally: invalid input leaves the current state unchanged. */
    void loadDefinitions(std::istream& input, const std::string& source);
    void loadCurves(std::istream& input, const std::string& source);
    /**
     * Validate a complete EML/ASM program before execution. With no executor the
     * program is a dry-run; live position conditions require the supplied provider.
     */
    void runProgram(
        std::istream& input,
        const std::string& source,
        std::ostream& output,
        const LiveExecutor& liveExecutor = {},
        const PositionProvider& livePositionProvider = {});
    void saveDefinitions(std::ostream& output) const;

    const std::map<std::string, Belt>& belts() const noexcept;
    const SafetyLimits& safetyLimits() const noexcept;

private:
    friend class BeltProgramHost;

    void validateMove(const MoveCommand& command) const;
    void validateMove(const MultiAxisMoveCommand& command) const;
    void validateCircle(const CircleCommand& command) const;
    void applyDefinition(const BeltDeclaration& declaration, std::ostream& output);
    void applyDefinition(const AxisDeclaration& declaration, std::ostream& output);
    void applyDefinition(const RemoveDeclaration& declaration, std::ostream& output);
    void applyDefinition(const SafetyLimitsDeclaration& declaration, std::ostream& output);
    void executePath(const PathCommand& command, std::ostream& output);

    std::map<std::string, Belt> belts_;
    std::map<std::string, BezierCurve> curves_;
    std::map<std::pair<std::string, motion::Axis>, double> simulatedPositions_;
    SafetyLimits safetyLimits_;
};

}
