#include "BeltConveyorController.hpp"

#include "EasyMachineLanguage.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace belt {
namespace {

std::string location(const std::string& source, std::size_t lineNumber) {
    return source + ":" + std::to_string(lineNumber);
}

std::runtime_error parseError(const std::string& source,
                              std::size_t lineNumber,
                              const std::string& detail) {
    return std::runtime_error(location(source, lineNumber) + ": " + detail);
}

std::vector<std::string> tokenize(const std::string& line) {
    std::istringstream input(line);
    std::vector<std::string> tokens;
    std::string token;
    while (input >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

double parseNumber(const std::string& token,
                   const std::string& source,
                   std::size_t lineNumber,
                   const std::string& field) {
    std::size_t parsed = 0;
    double value = 0.0;
    try {
        value = std::stod(token, &parsed);
    } catch (const std::exception&) {
        throw parseError(source, lineNumber, "invalid " + field + ": " + token);
    }
    if (parsed != token.size() || !std::isfinite(value)) {
        throw parseError(source, lineNumber, "invalid " + field + ": " + token);
    }
    return value;
}

motion::Axis parseAxis(const std::string& token,
                       const std::string& source,
                       std::size_t lineNumber) {
    if (token == "X") return motion::Axis::X;
    if (token == "Y") return motion::Axis::Y;
    if (token == "Z") return motion::Axis::Z;
    throw parseError(source, lineNumber, "axis must be X, Y, or Z");
}

char axisLetter(motion::Axis axis) {
    switch (axis) {
    case motion::Axis::X: return 'X';
    case motion::Axis::Y: return 'Y';
    case motion::Axis::Z: return 'Z';
    }
    throw std::invalid_argument("Invalid axis value");
}

bool isDefinition(const Command& command) {
    return std::holds_alternative<BeltDeclaration>(command) ||
           std::holds_alternative<AxisDeclaration>(command) ||
           std::holds_alternative<RemoveDeclaration>(command) ||
           std::holds_alternative<SafetyLimitsDeclaration>(command);
}

std::chrono::milliseconds parseMilliseconds(const std::string& token,
                                            const std::string& source,
                                            std::size_t lineNumber,
                                            const std::string& field) {
    if (token.empty() ||
        !std::all_of(token.begin(), token.end(), [](unsigned char ch) {
            return std::isdigit(ch) != 0;
        })) {
        throw parseError(source, lineNumber, "invalid " + field + ": " + token);
    }
    std::size_t parsed = 0;
    unsigned long long value = 0;
    try {
        value = std::stoull(token, &parsed);
    } catch (const std::exception&) {
        throw parseError(source, lineNumber, field + " is out of range");
    }
    if (parsed != token.size() || value == 0 ||
        value > static_cast<unsigned long long>(
                    std::numeric_limits<std::chrono::milliseconds::rep>::max())) {
        throw parseError(source, lineNumber, field + " must be a positive millisecond value");
    }
    return std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(value));
}

Point2D bezierPoint(const std::array<Point2D, 4>& points, double t) {
    const double inverse = 1.0 - t;
    const double inverseSquared = inverse * inverse;
    const double tSquared = t * t;
    return {
        inverseSquared * inverse * points[0].x + 3.0 * inverseSquared * t * points[1].x +
            3.0 * inverse * tSquared * points[2].x + tSquared * t * points[3].x,
        inverseSquared * inverse * points[0].y + 3.0 * inverseSquared * t * points[1].y +
            3.0 * inverse * tSquared * points[2].y + tSquared * t * points[3].y};
}

double quinticProgress(double t) {
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

} // namespace

class BeltProgramHost final : public easyml::ProgramHost {
public:
    BeltProgramHost(BeltConveyorController state,
                    BeltConveyorController::LiveExecutor liveExecutor,
                    BeltConveyorController::PositionProvider positionProvider)
        : state_(std::move(state)),
          liveExecutor_(std::move(liveExecutor)),
          positionProvider_(std::move(positionProvider)) {}

    std::unique_ptr<easyml::ProgramHost> cloneForValidation() const override {
        return std::make_unique<BeltProgramHost>(*this);
    }

    void validateInstruction(const std::string& instruction,
                             const easyml::SourceLocation& source) override {
        // Expand each instruction against a copied simulated state; no callback reaches hardware here.
        try {
            const auto command = BeltConveyorController::parseLine(
                instruction, source.source, source.line);
            if (!command) return;
            if (isDefinition(*command)) {
                throw std::invalid_argument(
                    "definitions cannot appear inside FOR or IF blocks");
            }
            if (liveExecutor_ && std::holds_alternative<PathCommand>(*command)) {
                throw std::runtime_error(
                    "PATH is dry-run only; continuous hardware path execution is not supported");
            }
            if (liveExecutor_) {
                const auto validateLiveDistance = [&](double distance,
                                                      double feedrate) {
                    if (!std::isfinite(feedrate) || feedrate <= 0.0) {
                        throw std::invalid_argument(
                            "Feedrate must be finite and positive for belt motion");
                    }
                    if (distance > state_.safetyLimits_.maximumDistancePerMoveMm) {
                        throw std::out_of_range(
                            "Move distance exceeds the configured per-move limit");
                    }
                    if (distance > 0.0 &&
                        distance * 60000.0 / feedrate >
                            static_cast<double>(state_.safetyLimits_.maximumDuration.count())) {
                        throw std::out_of_range(
                            "Estimated move duration exceeds the configured maximum");
                    }
                };
                if (const auto* move = std::get_if<MoveCommand>(&*command)) {
                    const auto previous =
                        state_.simulatedPositions_.find({move->beltName, move->axis});
                    const double current = previous == state_.simulatedPositions_.end()
                                               ? 0.0
                                               : previous->second;
                    validateLiveDistance(std::abs(move->targetPositionMm - current),
                                         move->feedrateMmPerMinute);
                } else if (const auto* move =
                               std::get_if<MultiAxisMoveCommand>(&*command)) {
                    double distanceSquared = 0.0;
                    for (const auto& target : move->targetsMm) {
                        const auto previous =
                            state_.simulatedPositions_.find({move->beltName, target.first});
                        const double current = previous == state_.simulatedPositions_.end()
                                                   ? 0.0
                                                   : previous->second;
                        const double difference = target.second - current;
                        distanceSquared += difference * difference;
                    }
                    validateLiveDistance(std::sqrt(distanceSquared),
                                         move->feedrateMmPerMinute);
                } else if (const auto* circle = std::get_if<CircleCommand>(&*command)) {
                    const double startX = circle->centerXmm + circle->diameterMm / 2.0;
                    const motion::Axis secondAxis =
                        circle->plane == CirclePlane::XY ? motion::Axis::Y : motion::Axis::Z;
                    const auto previousX = state_.simulatedPositions_.find(
                        {circle->beltName, motion::Axis::X});
                    const auto previousSecond = state_.simulatedPositions_.find(
                        {circle->beltName, secondAxis});
                    const double currentX = previousX == state_.simulatedPositions_.end()
                                                ? 0.0
                                                : previousX->second;
                    const double currentSecond =
                        previousSecond == state_.simulatedPositions_.end()
                            ? 0.0
                            : previousSecond->second;
                    validateLiveDistance(std::hypot(startX - currentX,
                                                    circle->centerSecondMm - currentSecond),
                                         circle->feedrateMmPerMinute);
                }
            }
            std::ostringstream ignored;
            // apply() validates limits and advances the simulated coordinates for later commands.
            state_.apply(*command, ignored);
        } catch (const std::exception& error) {
            rethrowAtLocation(source, error);
        }
    }

    void executeInstruction(const std::string& instruction,
                            const easyml::SourceLocation& source,
                            std::ostream& output) override {
        try {
            const auto command = BeltConveyorController::parseLine(
                instruction, source.source, source.line);
            if (!command) return;
            if (liveExecutor_) liveExecutor_(*command, output);
            else state_.apply(*command, output);
        } catch (const std::exception& error) {
            rethrowAtLocation(source, error);
        }
    }

    void validateIdentifier(const std::string& identifier,
                            const easyml::SourceLocation& source) const override {
        try {
            if (identifier.find('.') == std::string::npos) {
                throw std::invalid_argument("unknown loop variable: " + identifier);
            }
            const auto [beltName, axis] = parsePositionIdentifier(identifier, source);
            const auto belt = state_.belts_.find(beltName);
            if (belt == state_.belts_.end()) {
                throw std::invalid_argument("unknown belt in IF condition: " + beltName);
            }
            if (belt->second.axes.find(axis) == belt->second.axes.end()) {
                throw std::invalid_argument("IF condition references an undefined axis");
            }
            if (liveExecutor_ && !positionProvider_) {
                throw std::invalid_argument(
                    "live position provider is required for position IF");
            }
        } catch (const std::exception& error) {
            rethrowAtLocation(source, error);
        }
    }

    double resolveIdentifier(const std::string& identifier,
                             const easyml::SourceLocation& source) override {
        const auto [beltName, axis] = parsePositionIdentifier(identifier, source);
        if (liveExecutor_) {
            try {
                return positionProvider_(beltName, axis);
            } catch (const std::exception& error) {
                rethrowAtLocation(source, error);
            }
        }
        const auto position = state_.simulatedPositions_.find({beltName, axis});
        if (position == state_.simulatedPositions_.end()) {
            throw parseError(source.source, source.line,
                             "position is unknown in IF condition for " + identifier);
        }
        return position->second;
    }

    void mergeValidatedBranches(const easyml::ProgramHost& trueBranch,
                                const easyml::ProgramHost& falseBranch) override {
        const auto* trueHost = dynamic_cast<const BeltProgramHost*>(&trueBranch);
        const auto* falseHost = dynamic_cast<const BeltProgramHost*>(&falseBranch);
        if (!trueHost || !falseHost) {
            throw std::logic_error("Cannot merge validation state from a different host");
        }
        // Retain only coordinates known to be identical whichever IF branch runs.
        std::map<std::pair<std::string, motion::Axis>, double> merged;
        for (const auto& position : trueHost->state_.simulatedPositions_) {
            const auto other = falseHost->state_.simulatedPositions_.find(position.first);
            if (other != falseHost->state_.simulatedPositions_.end() &&
                other->second == position.second) {
                merged.emplace(position.first, position.second);
            }
        }
        state_.simulatedPositions_ = std::move(merged);
    }

    void conditionEvaluated(const std::string& expression,
                            const easyml::SourceLocation&,
                            bool result,
                            std::ostream& output) override {
        output << (liveExecutor_ ? "LIVE IF " : "DRY-RUN IF ") << expression << " -> "
               << (result ? "TRUE" : "FALSE") << '\n';
    }

private:
    static std::pair<std::string, motion::Axis> parsePositionIdentifier(
        const std::string& identifier,
        const easyml::SourceLocation& source) {
        const auto dot = identifier.find_last_of('.');
        if (dot == std::string::npos || dot == 0 || dot + 2 != identifier.size()) {
            throw parseError(source.source, source.line,
                             "position identifier must have the form <belt>.<X|Y|Z>");
        }
        return {identifier.substr(0, dot),
                parseAxis(identifier.substr(dot + 1), source.source, source.line)};
    }

    static void rethrowAtLocation(const easyml::SourceLocation& source,
                                 const std::exception& error) {
        const std::string prefix = location(source.source, source.line) + ":";
        if (std::string(error.what()).compare(0, prefix.size(), prefix) == 0) throw;
        throw parseError(source.source, source.line, error.what());
    }

    BeltConveyorController state_;
    BeltConveyorController::LiveExecutor liveExecutor_;
    BeltConveyorController::PositionProvider positionProvider_;
};

std::optional<Command> BeltConveyorController::parseLine(const std::string& line,
                                                         const std::string& source,
                                                         std::size_t lineNumber) {
    std::string content = line;
    const auto comment = content.find_first_of(";#");
    if (comment != std::string::npos) {
        content.erase(comment);
    }
    const auto tokens = tokenize(content);
    if (tokens.empty()) {
        return std::nullopt;
    }

    if (tokens[0] == "BELT") {
        if (tokens.size() != 4) {
            throw parseError(source, lineNumber, "expected: BELT <name> <communication> <port>");
        }
        return BeltDeclaration{tokens[1], tokens[2], tokens[3]};
    }
    if (tokens[0] == "AXIS") {
        if (tokens.size() != 4) {
            throw parseError(source, lineNumber,
                             "expected: AXIS <name> <X|Y|Z> <maximum-position-mm>");
        }
        return AxisDeclaration{tokens[1],
                               parseAxis(tokens[2], source, lineNumber),
                               parseNumber(tokens[3], source, lineNumber, "maximum position")};
    }
    if (tokens[0] == "LIMITS") {
        if (tokens.size() != 4) {
            throw parseError(source, lineNumber,
                             "expected: LIMITS <maximum-move-mm> <maximum-duration-ms> "
                             "<command-timeout-ms>");
        }
        return SafetyLimitsDeclaration{
            parseNumber(tokens[1], source, lineNumber, "maximum move distance"),
            parseMilliseconds(tokens[2], source, lineNumber, "maximum duration"),
            parseMilliseconds(tokens[3], source, lineNumber, "command timeout")};
    }
    if (tokens[0] == "REMOVE") {
        if (tokens.size() != 2) {
            throw parseError(source, lineNumber, "expected: REMOVE <name>");
        }
        return RemoveDeclaration{tokens[1]};
    }
    if (tokens[0] == "WAIT") {
        if (tokens.size() != 2 || tokens[1].empty() ||
            !std::all_of(tokens[1].begin(), tokens[1].end(), [](unsigned char ch) {
                return std::isdigit(ch) != 0;
            })) {
            throw parseError(source, lineNumber, "expected: WAIT <milliseconds>");
        }
        std::size_t parsed = 0;
        unsigned long long duration = 0;
        try {
            duration = std::stoull(tokens[1], &parsed);
        } catch (const std::exception&) {
            throw parseError(source, lineNumber, "wait duration is out of range");
        }
        if (parsed != tokens[1].size() || duration == 0 ||
            duration > static_cast<unsigned long long>(
                           std::numeric_limits<std::chrono::milliseconds::rep>::max())) {
            throw parseError(source, lineNumber, "wait duration must be a positive integer");
        }
        return WaitCommand{duration};
    }
    if (tokens[0] == "PATH") {
        if (tokens.size() < 2 || tokens.size() > 3 ||
            (tokens.size() == 3 &&
             (tokens[2].size() < 2 || tokens[2][0] != 'F'))) {
            throw parseError(source, lineNumber,
                             "expected: PATH <curve-name> [F<feedrate-mm/min>]");
        }
        const double feedrateMmPerMinute =
            tokens.size() == 3
                ? parseNumber(tokens[2].substr(1), source, lineNumber, "feedrate")
                : kDefaultFeedrateMmPerMinute;
        return PathCommand{tokens[1], feedrateMmPerMinute};
    }

    if (tokens[0] == "CIRCLE") {
        if (tokens.size() < 5 || tokens.size() > 7) {
            throw parseError(source, lineNumber,
                             "expected: CIRCLE <belt> X<center-x> <Y|Z><center> "
                             "D<diameter> [CW|CCW] [F<feedrate-mm/min>]");
        }
        const auto parsePrefixedNumber = [&](const std::string& token, char prefix,
                                             const std::string& field) {
            if (token.size() < 2 || token[0] != prefix) {
                throw parseError(source, lineNumber, "expected " + std::string(1, prefix) +
                                                        " value for " + field);
            }
            return parseNumber(token.substr(1), source, lineNumber, field);
        };
        const CirclePlane plane = tokens[3].size() >= 2 && tokens[3][0] == 'Z'
                                      ? CirclePlane::XZ
                                      : CirclePlane::XY;
        const char secondAxis = plane == CirclePlane::XY ? 'Y' : 'Z';
        CircleCommand circle{tokens[1],
                             plane,
                             parsePrefixedNumber(tokens[2], 'X', "circle center X"),
                             parsePrefixedNumber(tokens[3], secondAxis,
                                                 plane == CirclePlane::XY
                                                     ? "circle center Y"
                                                     : "circle center Z"),
                             parsePrefixedNumber(tokens[4], 'D', "circle diameter"),
                             true,
                             kDefaultFeedrateMmPerMinute};
        bool directionProvided = false;
        bool feedrateProvided = false;
        for (std::size_t index = 5; index < tokens.size(); ++index) {
            if (tokens[index] == "CW" || tokens[index] == "CCW") {
                if (directionProvided) {
                    throw parseError(source, lineNumber, "circle direction is specified twice");
                }
                circle.clockwise = tokens[index] == "CW";
                directionProvided = true;
            } else if (tokens[index].size() >= 2 && tokens[index][0] == 'F') {
                if (feedrateProvided) {
                    throw parseError(source, lineNumber, "circle feedrate is specified twice");
                }
                circle.feedrateMmPerMinute =
                    parseNumber(tokens[index].substr(1), source, lineNumber, "feedrate");
                feedrateProvided = true;
            } else {
                throw parseError(source, lineNumber, "expected CW, CCW, or F<feedrate-mm/min>");
            }
        }
        return circle;
    }

    if (tokens.size() < 2) {
        throw parseError(source, lineNumber,
                         "expected: <belt> <X|Y|Z><position-mm> [<axis><position-mm>] "
                         "[F<feedrate-mm/min>]");
    }
    std::map<motion::Axis, double> targets;
    double feedrateMmPerMinute = kDefaultFeedrateMmPerMinute;
    bool feedrateProvided = false;
    for (std::size_t index = 1; index < tokens.size(); ++index) {
        if (tokens[index].size() >= 2 && tokens[index][0] == 'F') {
            if (feedrateProvided || index + 1 != tokens.size()) {
                throw parseError(source, lineNumber,
                                 "feedrate must be specified once at the end of a move");
            }
            feedrateMmPerMinute =
                parseNumber(tokens[index].substr(1), source, lineNumber, "feedrate");
            feedrateProvided = true;
            continue;
        }
        if (tokens[index].size() < 2) {
            throw parseError(source, lineNumber,
                             "expected axis coordinate <X|Y|Z><position-mm>");
        }
        const motion::Axis axis = parseAxis(tokens[index].substr(0, 1), source, lineNumber);
        const double target =
            parseNumber(tokens[index].substr(1), source, lineNumber, "target position");
        if (!targets.emplace(axis, target).second) {
            throw parseError(source, lineNumber, "axis is specified more than once");
        }
    }
    if (targets.size() == 1) {
        return MoveCommand{tokens[0], targets.begin()->first, targets.begin()->second,
                           feedrateMmPerMinute};
    }
    return MultiAxisMoveCommand{tokens[0], std::move(targets), feedrateMmPerMinute};
}

void BeltConveyorController::apply(const Command& command, std::ostream& output) {
    if (const auto* declaration = std::get_if<BeltDeclaration>(&command)) {
        applyDefinition(*declaration, output);
        return;
    }
    if (const auto* declaration = std::get_if<AxisDeclaration>(&command)) {
        applyDefinition(*declaration, output);
        return;
    }
    if (const auto* declaration = std::get_if<RemoveDeclaration>(&command)) {
        applyDefinition(*declaration, output);
        return;
    }
    if (const auto* declaration = std::get_if<SafetyLimitsDeclaration>(&command)) {
        applyDefinition(*declaration, output);
        return;
    }
    if (const auto* move = std::get_if<MoveCommand>(&command)) {
        validateMove(*move);
        const auto key = std::make_pair(move->beltName, move->axis);
        const auto previous = simulatedPositions_.find(key);
        output << std::setprecision(6) << "DRY-RUN MOVE " << move->beltName << " "
               << axisLetter(move->axis) << " target=" << move->targetPositionMm
               << " mm feedrate=" << move->feedrateMmPerMinute << " mm/min";
        if (previous == simulatedPositions_.end()) {
            output << " from=UNKNOWN";
        } else {
            output << " from=" << previous->second << " mm";
        }
        output << " profile=QUINTIC samples=101\n";
        for (int sample = 0; sample <= 100; ++sample) {
            const double t = static_cast<double>(sample) / 100.0;
            output << "  u=" << t << " progress=" << quinticProgress(t) << '\n';
        }
        simulatedPositions_[key] = move->targetPositionMm;
        return;
    }
    if (const auto* move = std::get_if<MultiAxisMoveCommand>(&command)) {
        validateMove(*move);
        output << std::setprecision(6) << "DRY-RUN COORDINATED MOVE " << move->beltName;
        for (const auto& target : move->targetsMm) {
            output << " " << axisLetter(target.first) << target.second;
        }
        output << " feedrate=" << move->feedrateMmPerMinute
               << " mm/min profile=QUINTIC samples=101\n";
        for (int sample = 0; sample <= 100; ++sample) {
            const double t = static_cast<double>(sample) / 100.0;
            output << "  u=" << t << " progress=" << quinticProgress(t) << '\n';
        }
        for (const auto& target : move->targetsMm) {
            simulatedPositions_[{move->beltName, target.first}] = target.second;
        }
        return;
    }
    if (const auto* circle = std::get_if<CircleCommand>(&command)) {
        validateCircle(*circle);
        const double radius = circle->diameterMm / 2.0;
        const double startX = circle->centerXmm + radius;
        const motion::Axis secondAxis =
            circle->plane == CirclePlane::XY ? motion::Axis::Y : motion::Axis::Z;
        const char secondAxisLetter =
            circle->plane == CirclePlane::XY ? 'Y' : 'Z';
        const char* planeName = circle->plane == CirclePlane::XY ? "XY" : "XZ";
        output << std::setprecision(6) << "DRY-RUN CIRCLE " << circle->beltName
               << " plane=" << planeName << " center=(X" << circle->centerXmm
               << "," << secondAxisLetter << circle->centerSecondMm
               << ") diameter=" << circle->diameterMm << " mm direction="
               << (circle->clockwise ? "CW" : "CCW")
               << " start=(X" << startX << "," << secondAxisLetter
               << circle->centerSecondMm << ")"
               << " feedrate=" << circle->feedrateMmPerMinute
               << " mm/min full-circle\n";
        simulatedPositions_[{circle->beltName, motion::Axis::X}] = startX;
        simulatedPositions_[{circle->beltName, secondAxis}] = circle->centerSecondMm;
        return;
    }
    if (const auto* wait = std::get_if<WaitCommand>(&command)) {
        if (wait->durationMs == 0) {
            throw std::invalid_argument("Wait duration must be positive");
        }
        output << "DRY-RUN WAIT " << wait->durationMs << " ms (not delayed)\n";
        return;
    }
    if (const auto* path = std::get_if<PathCommand>(&command)) {
        executePath(*path, output);
        return;
    }
    throw std::logic_error("Unhandled command type");
}

void BeltConveyorController::loadDefinitions(std::istream& input, const std::string& source) {
    BeltConveyorController candidate = *this;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        const auto command = parseLine(line, source, lineNumber);
        if (!command) continue;
        if (!isDefinition(*command)) {
            throw parseError(source, lineNumber,
                             "only LIMITS, BELT, AXIS, and REMOVE are allowed here");
        }
        std::ostringstream ignored;
        candidate.apply(*command, ignored);
    }
    if (input.bad()) {
        throw std::runtime_error("Error reading definitions from " + source);
    }
    *this = std::move(candidate);
}

void BeltConveyorController::loadCurves(std::istream& input, const std::string& source) {
    BeltConveyorController candidate = *this;
    std::optional<std::string> activeName;
    std::string activeBelt;
    std::array<Point2D, 4> controlPoints{};
    std::array<bool, 4> pointDefined{false, false, false, false};
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        std::string content = line;
        const auto comment = content.find_first_of(";#");
        if (comment != std::string::npos) content.erase(comment);
        const auto tokens = tokenize(content);
        if (tokens.empty()) continue;

        if (!activeName) {
            if (tokens.size() != 4 || tokens[0] != "CURVE" || tokens[2] != "BEZIER") {
                throw parseError(source, lineNumber,
                                 "expected: CURVE <name> BEZIER <belt-name>");
            }
            if (candidate.curves_.find(tokens[1]) != candidate.curves_.end()) {
                throw parseError(source, lineNumber, "duplicate curve name: " + tokens[1]);
            }
            activeName = tokens[1];
            activeBelt = tokens[3];
            pointDefined.fill(false);
            continue;
        }

        if (tokens.size() == 1 && tokens[0] == "END") {
            if (!std::all_of(pointDefined.begin(), pointDefined.end(), [](bool defined) {
                    return defined;
                })) {
                throw parseError(source, lineNumber, "Bezier curve requires P0, P1, P2, and P3");
            }
            candidate.curves_.emplace(
                *activeName, BezierCurve{*activeName, activeBelt, controlPoints});
            activeName.reset();
            activeBelt.clear();
            continue;
        }

        if (tokens.size() != 3 || tokens[0].size() != 2 || tokens[0][0] != 'P' ||
            tokens[0][1] < '0' || tokens[0][1] > '3' || tokens[1].size() < 2 ||
            tokens[2].size() < 2 || tokens[1][0] != 'X' || tokens[2][0] != 'Y') {
            throw parseError(source, lineNumber, "expected: P0..P3 X<x-mm> Y<y-mm> or END");
        }
        const auto pointIndex = static_cast<std::size_t>(tokens[0][1] - '0');
        if (pointDefined[pointIndex]) {
            throw parseError(source, lineNumber, "duplicate control point " + tokens[0]);
        }
        controlPoints[pointIndex] = {
            parseNumber(tokens[1].substr(1), source, lineNumber, "X coordinate"),
            parseNumber(tokens[2].substr(1), source, lineNumber, "Y coordinate")};
        pointDefined[pointIndex] = true;
    }
    if (input.bad()) {
        throw std::runtime_error("Error reading curves from " + source);
    }
    if (activeName) {
        throw parseError(source, lineNumber, "curve " + *activeName + " is missing END");
    }
    *this = std::move(candidate);
}

void BeltConveyorController::runProgram(std::istream& input,
                                        const std::string& source,
                                        std::ostream& output,
                                        const LiveExecutor& liveExecutor,
                                        const PositionProvider& livePositionProvider) {
    // Definitions establish the simulated machine before the executable section is checked.
    easyml::Program program = easyml::Program::parse(input, source);
    std::vector<std::pair<std::size_t, Command>> definitions;
    std::vector<easyml::Node> executableNodes;
    bool sequenceStarted = false;
    for (const easyml::Node& node : program.nodes()) {
        if (node.kind != easyml::Node::Kind::Instruction) {
            sequenceStarted = true;
            executableNodes.push_back(node);
            continue;
        }
        const auto command = parseLine(node.text, source, node.location.line);
        if (!command) continue;
        if (isDefinition(*command)) {
            if (sequenceStarted) {
                throw parseError(source, node.location.line,
                                 "definitions must precede motion commands");
            }
            definitions.emplace_back(node.location.line, *command);
        } else {
            sequenceStarted = true;
            executableNodes.push_back(node);
        }
    }

    BeltConveyorController initialState = *this;
    std::ostringstream definitionOutput;
    for (const auto& definition : definitions) {
        try {
            initialState.apply(definition.second, definitionOutput);
        } catch (const std::exception& error) {
            throw parseError(source, definition.first, error.what());
        }
    }

    easyml::Program executableProgram(source, std::move(executableNodes));
    BeltProgramHost validator(initialState, liveExecutor, livePositionProvider);
    // Check every expanded instruction and both conditional paths before live execution.
    executableProgram.validate(validator);

    BeltProgramHost executor(initialState, liveExecutor, livePositionProvider);
    if (liveExecutor) {
        executableProgram.execute(executor, output);
        return;
    }
    // Buffer dry-run output so invalid programs do not leave a misleading partial transcript.
    std::ostringstream bufferedOutput;
    bufferedOutput << definitionOutput.str();
    executableProgram.execute(executor, bufferedOutput);
    output << bufferedOutput.str();
}

void BeltConveyorController::saveDefinitions(std::ostream& output) const {
    output << "; BeltConveyorController limits (distance mm, durations ms)\n"
           << "LIMITS " << std::setprecision(std::numeric_limits<double>::max_digits10)
           << safetyLimits_.maximumDistancePerMoveMm << " "
           << safetyLimits_.maximumDuration.count() << " "
           << safetyLimits_.commandTimeout.count() << '\n'
           << "; MARLIN uses 115200 baud\n";
    for (const auto& entry : belts_) {
        const Belt& belt = entry.second;
        output << "BELT " << belt.name << " " << belt.communication << " " << belt.port << '\n';
        for (const auto& axis : belt.axes) {
            output << "AXIS " << belt.name << " " << axisLetter(axis.first) << " "
                   << axis.second.maximumPositionMm << '\n';
        }
    }
}

const std::map<std::string, Belt>& BeltConveyorController::belts() const noexcept {
    return belts_;
}

const SafetyLimits& BeltConveyorController::safetyLimits() const noexcept {
    return safetyLimits_;
}

void BeltConveyorController::executePath(const PathCommand& command, std::ostream& output) {
    const auto curve = curves_.find(command.curveName);
    if (curve == curves_.end()) {
        throw std::runtime_error("Unknown curve: " + command.curveName);
    }
    const auto belt = belts_.find(curve->second.beltName);
    if (belt == belts_.end()) {
        throw std::runtime_error("Curve refers to unknown belt: " + curve->second.beltName);
    }
    const auto xAxis = belt->second.axes.find(motion::Axis::X);
    const auto yAxis = belt->second.axes.find(motion::Axis::Y);
    if (xAxis == belt->second.axes.end() || yAxis == belt->second.axes.end()) {
        throw std::runtime_error("Bezier curve " + command.curveName +
                                 " requires X and Y axes on belt " + curve->second.beltName);
    }
    if (!std::isfinite(command.feedrateMmPerMinute) ||
        command.feedrateMmPerMinute <= 0.0) {
        throw std::out_of_range("Path feedrate must be finite and positive for curve " +
                                command.curveName);
    }
    for (const auto& point : curve->second.controlPoints) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0.0 ||
            point.x > xAxis->second.maximumPositionMm || point.y < 0.0 ||
            point.y > yAxis->second.maximumPositionMm) {
            throw std::out_of_range("Bezier control point is outside axis limits for curve " +
                                    command.curveName);
        }
    }

    const auto xKey = std::make_pair(curve->second.beltName, motion::Axis::X);
    const auto yKey = std::make_pair(curve->second.beltName, motion::Axis::Y);
    const auto currentX = simulatedPositions_.find(xKey);
    const auto currentY = simulatedPositions_.find(yKey);
    if ((currentX != simulatedPositions_.end() &&
         std::abs(currentX->second - curve->second.controlPoints[0].x) > 1e-9) ||
        (currentY != simulatedPositions_.end() &&
         std::abs(currentY->second - curve->second.controlPoints[0].y) > 1e-9)) {
        throw std::runtime_error("Current dry-run position does not match Bezier start point P0");
    }

    // Quintic time scaling eases velocity at each endpoint; samples are diagnostic only.
    std::vector<Point2D> samples;
    samples.reserve(101);
    for (int sample = 0; sample <= 100; ++sample) {
        const double t = static_cast<double>(sample) / 100.0;
        const double curveParameter = quinticProgress(t);
        const Point2D point = bezierPoint(curve->second.controlPoints, curveParameter);
        if (point.x < 0.0 || point.x > xAxis->second.maximumPositionMm || point.y < 0.0 ||
            point.y > yAxis->second.maximumPositionMm) {
            throw std::out_of_range("Bezier sample is outside axis limits for curve " +
                                    command.curveName);
        }
        samples.push_back(point);
    }
    output << std::setprecision(6) << "DRY-RUN PATH " << command.curveName << " BELT "
           << curve->second.beltName << " BEZIER feedrate=" << command.feedrateMmPerMinute
           << " mm/min profile=QUINTIC samples=" << samples.size() << '\n';
    for (std::size_t sample = 0; sample < samples.size(); ++sample) {
        const double t = static_cast<double>(sample) / 100.0;
        const double curveParameter = quinticProgress(t);
        output << "  u=" << t << " curve-t=" << curveParameter << " X=" << samples[sample].x
               << " Y=" << samples[sample].y << '\n';
    }
    simulatedPositions_[xKey] = curve->second.controlPoints[3].x;
    simulatedPositions_[yKey] = curve->second.controlPoints[3].y;
}

void BeltConveyorController::validateMove(const MoveCommand& command) const {
    const auto belt = belts_.find(command.beltName);
    if (belt == belts_.end()) {
        throw std::runtime_error("Unknown belt: " + command.beltName);
    }
    const auto axis = belt->second.axes.find(command.axis);
    if (axis == belt->second.axes.end()) {
        throw std::runtime_error("Axis " + std::string(1, axisLetter(command.axis)) +
                                 " is not defined for belt " + command.beltName);
    }
    if (!std::isfinite(command.targetPositionMm) || command.targetPositionMm < 0.0 ||
        command.targetPositionMm > axis->second.maximumPositionMm) {
        throw std::out_of_range("Target position is outside the configured limit for belt " +
                                command.beltName);
    }
    if (!std::isfinite(command.feedrateMmPerMinute) ||
        command.feedrateMmPerMinute <= 0.0) {
        throw std::out_of_range("Feedrate must be finite and positive for belt " +
                                command.beltName);
    }
}

void BeltConveyorController::validateMove(const MultiAxisMoveCommand& command) const {
    if (command.targetsMm.size() < 2) {
        throw std::invalid_argument("A coordinated move must specify at least two axes");
    }
    double knownDistanceSquared = 0.0;
    bool allStartingPositionsKnown = true;
    for (const auto& target : command.targetsMm) {
        validateMove(MoveCommand{command.beltName, target.first, target.second,
                                 command.feedrateMmPerMinute});
        const auto previous = simulatedPositions_.find({command.beltName, target.first});
        if (previous == simulatedPositions_.end()) {
            allStartingPositionsKnown = false;
        } else {
            const double distance = target.second - previous->second;
            knownDistanceSquared += distance * distance;
        }
    }
    if (allStartingPositionsKnown &&
        std::sqrt(knownDistanceSquared) > safetyLimits_.maximumDistancePerMoveMm) {
        throw std::out_of_range("Coordinated move exceeds the configured per-move distance");
    }
}

void BeltConveyorController::validateCircle(const CircleCommand& command) const {
    const auto belt = belts_.find(command.beltName);
    if (belt == belts_.end()) {
        throw std::runtime_error("Unknown belt: " + command.beltName);
    }
    const auto xAxis = belt->second.axes.find(motion::Axis::X);
    const motion::Axis secondAxis =
        command.plane == CirclePlane::XY ? motion::Axis::Y : motion::Axis::Z;
    const auto secondAxisSettings = belt->second.axes.find(secondAxis);
    if (xAxis == belt->second.axes.end() ||
        secondAxisSettings == belt->second.axes.end()) {
        throw std::runtime_error(std::string("CIRCLE in the ") +
                                 (command.plane == CirclePlane::XY ? "XY" : "XZ") +
                                 " plane requires X and " +
                                 (command.plane == CirclePlane::XY ? "Y" : "Z") +
                                 " axes on belt " + command.beltName);
    }
    if (!std::isfinite(command.centerXmm) || !std::isfinite(command.centerSecondMm) ||
        !std::isfinite(command.diameterMm) || command.diameterMm <= 0.0) {
        throw std::invalid_argument("Circle center must be finite and diameter must be positive");
    }
    if (!std::isfinite(command.feedrateMmPerMinute) ||
        command.feedrateMmPerMinute <= 0.0) {
        throw std::out_of_range("Feedrate must be finite and positive for circle");
    }
    const double radius = command.diameterMm / 2.0;
    if (command.centerXmm - radius < 0.0 ||
        command.centerXmm + radius > xAxis->second.maximumPositionMm ||
        command.centerSecondMm - radius < 0.0 ||
        command.centerSecondMm + radius > secondAxisSettings->second.maximumPositionMm) {
        throw std::out_of_range(std::string("Circle is outside the configured ") +
                                (command.plane == CirclePlane::XY ? "XY" : "XZ") +
                                " travel limits");
    }
    const double circumference = 2.0 * std::acos(-1.0) * radius;
    if (circumference > safetyLimits_.maximumDistancePerMoveMm) {
        throw std::out_of_range("Circle length exceeds the configured per-move distance");
    }
    const double estimatedDurationMs =
        circumference * 60000.0 / command.feedrateMmPerMinute;
    if (estimatedDurationMs > static_cast<double>(safetyLimits_.maximumDuration.count())) {
        throw std::out_of_range("Estimated circle duration exceeds the configured maximum");
    }
}

void BeltConveyorController::applyDefinition(const BeltDeclaration& declaration,
                                             std::ostream& output) {
    if (declaration.name.empty() || declaration.port.empty()) {
        throw std::invalid_argument("Belt name and port must not be empty");
    }
    if (declaration.communication != "MARLIN") {
        throw std::invalid_argument("Unsupported communication method: " +
                                    declaration.communication + " (supported: MARLIN)");
    }
    if (std::any_of(declaration.port.begin(), declaration.port.end(), [](unsigned char ch) {
            return std::isspace(ch) != 0;
        })) {
        throw std::invalid_argument("Port paths must not contain whitespace");
    }
    if (belts_.find(declaration.name) != belts_.end()) {
        throw std::invalid_argument("Belt is already registered: " + declaration.name);
    }
    belts_.emplace(declaration.name,
                   Belt{declaration.name, declaration.communication, declaration.port, {}});
    output << "REGISTERED " << declaration.name << " " << declaration.communication << " "
           << declaration.port << '\n';
}

void BeltConveyorController::applyDefinition(const AxisDeclaration& declaration,
                                             std::ostream& output) {
    const char letter = axisLetter(declaration.axis);
    const auto belt = belts_.find(declaration.beltName);
    if (belt == belts_.end()) {
        throw std::invalid_argument("Cannot assign axis to unknown belt: " + declaration.beltName);
    }
    if (!std::isfinite(declaration.maximumPositionMm) || declaration.maximumPositionMm <= 0.0) {
        throw std::invalid_argument("Axis maximum position must be finite and positive");
    }
    if (belt->second.axes.find(declaration.axis) != belt->second.axes.end()) {
        throw std::invalid_argument("Axis is already defined for belt " + declaration.beltName);
    }
    belt->second.axes.emplace(declaration.axis, AxisSettings{declaration.maximumPositionMm});
    output << "CONFIGURED " << declaration.beltName << " "
           << letter << " max-position="
           << declaration.maximumPositionMm << " mm\n";
}

void BeltConveyorController::applyDefinition(const RemoveDeclaration& declaration,
                                             std::ostream& output) {
    if (belts_.erase(declaration.beltName) == 0) {
        throw std::invalid_argument("Cannot remove unknown belt: " + declaration.beltName);
    }
    for (auto position = simulatedPositions_.begin(); position != simulatedPositions_.end();) {
        if (position->first.first == declaration.beltName) {
            position = simulatedPositions_.erase(position);
        } else {
            ++position;
        }
    }
    output << "REMOVED " << declaration.beltName << '\n';
}

void BeltConveyorController::applyDefinition(const SafetyLimitsDeclaration& declaration,
                                             std::ostream& output) {
    if (!std::isfinite(declaration.maximumDistancePerMoveMm) ||
        declaration.maximumDistancePerMoveMm <= 0.0 ||
        declaration.maximumDuration.count() <= 0 ||
        declaration.commandTimeout < declaration.maximumDuration) {
        throw std::invalid_argument(
            "Limits must be positive and command timeout must cover maximum duration");
    }
    safetyLimits_ = SafetyLimits{declaration.maximumDistancePerMoveMm,
                                 declaration.maximumDuration,
                                 declaration.commandTimeout};
    output << "LIMITS max-move=" << safetyLimits_.maximumDistancePerMoveMm
           << " mm max-duration=" << safetyLimits_.maximumDuration.count()
           << " ms timeout=" << safetyLimits_.commandTimeout.count() << " ms\n";
}

}
