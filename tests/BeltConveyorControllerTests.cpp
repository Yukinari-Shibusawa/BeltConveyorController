#include "BeltConveyorController.hpp"

#include <chrono>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Function>
void requireThrows(Function&& function, const std::string& expectedText) {
    try {
        function();
    } catch (const std::exception& error) {
        require(std::string(error.what()).find(expectedText) != std::string::npos,
                "exception did not contain expected text: " + expectedText);
        return;
    }
    throw std::runtime_error("expected an exception containing: " + expectedText);
}

void testParser() {
    require(!belt::BeltConveyorController::parseLine("; comment", "test", 1),
            "comments should be ignored");
    const auto move = belt::BeltConveyorController::parseLine("A X100", "test", 2);
    require(move && std::holds_alternative<belt::MoveCommand>(*move) &&
                std::get<belt::MoveCommand>(*move).feedrateMmPerMinute ==
                    belt::kDefaultFeedrateMmPerMinute,
            "move without F should use the default feedrate");
    const auto explicitFeedrate =
        belt::BeltConveyorController::parseLine("A X100 F600", "test", 3);
    require(explicitFeedrate &&
                std::get<belt::MoveCommand>(*explicitFeedrate).feedrateMmPerMinute == 600.0,
            "explicit procedure feedrate should remain in mm/min");
    const auto coordinated =
        belt::BeltConveyorController::parseLine("A X100 Y100 F600", "test", 4);
    require(coordinated &&
                std::holds_alternative<belt::MultiAxisMoveCommand>(*coordinated) &&
                std::get<belt::MultiAxisMoveCommand>(*coordinated).targetsMm.size() == 2,
            "multiple coordinates on one line should parse as a coordinated move");
    const auto circle =
        belt::BeltConveyorController::parseLine("CIRCLE A X100 Y100 D50 CCW F600", "test", 5);
    require(circle && std::holds_alternative<belt::CircleCommand>(*circle) &&
                !std::get<belt::CircleCommand>(*circle).clockwise,
            "circle geometry should parse center, diameter, direction, and feedrate");
    const auto xzCircle =
        belt::BeltConveyorController::parseLine("CIRCLE A X110 Z125 D40 CW F600", "test", 5);
    require(xzCircle && std::holds_alternative<belt::CircleCommand>(*xzCircle) &&
                std::get<belt::CircleCommand>(*xzCircle).plane == belt::CirclePlane::XZ,
            "XZ circle geometry should parse the Z center and select the XZ plane");
    const auto wait = belt::BeltConveyorController::parseLine("WAIT 250", "test", 6);
    require(wait && std::get<belt::WaitCommand>(*wait).durationMs == 250,
            "wait syntax should parse");
    const auto axis =
        belt::BeltConveyorController::parseLine("AXIS A X 220", "test", 7);
    require(axis && std::get<belt::AxisDeclaration>(*axis).maximumPositionMm == 220.0,
            "axis definitions should contain only a position limit");
    const auto path = belt::BeltConveyorController::parseLine("PATH curve", "test", 8);
    require(path && std::get<belt::PathCommand>(*path).feedrateMmPerMinute ==
                        belt::kDefaultFeedrateMmPerMinute,
            "path without F should use the default feedrate");
    belt::BeltConveyorController noSpeedLimitController;
    std::ostringstream noSpeedLimitOutput;
    noSpeedLimitController.apply(
        belt::BeltConveyorController::parseLine("BELT A MARLIN /dev/ttyUSB0", "test", 9)
            .value(),
        noSpeedLimitOutput);
    noSpeedLimitController.apply(*axis, noSpeedLimitOutput);
    const auto highFeedrate =
        belt::BeltConveyorController::parseLine("A X220 F600000", "test", 10);
    noSpeedLimitController.apply(*highFeedrate, noSpeedLimitOutput);
    requireThrows(
        [] { belt::BeltConveyorController::parseLine("A Xno F10", "program.asm", 11); },
        "program.asm:11");
}

void testCoordinatedAndCircleMotion() {
    belt::BeltConveyorController controller;
    std::istringstream program(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 200\n"
        "AXIS A Y 200\n"
        "A X100 Y100 F600\n"
        "FOR step = 1 TO 2\n"
        "A X{step} Y{step} F600\n"
        "END\n"
        "A X50 F600\n"
        "CIRCLE A X100 Y100 D50 F600\n");
    std::ostringstream output;
    controller.runProgram(program, "geometry.asm", output);
    const std::string result = output.str();
    require(result.find("DRY-RUN COORDINATED MOVE A X100 Y100 feedrate=600 mm/min") !=
                std::string::npos,
            "XY targets on one line should be dry-run as one coordinated move");
    require(result.find("DRY-RUN COORDINATED MOVE A X1 Y1") != std::string::npos &&
                result.find("DRY-RUN COORDINATED MOVE A X2 Y2") != std::string::npos,
            "loop variables should expand in coordinated multi-axis commands");
    require(result.find("DRY-RUN MOVE A X target=50 mm") != std::string::npos,
            "single-axis motion should remain supported after coordinated motion");
    require(result.find("DRY-RUN CIRCLE A plane=XY center=(X100,Y100) diameter=50 mm direction=CW") !=
                std::string::npos,
            "circle command should dry-run the requested geometry");
    require(result.find("start=(X125,Y100)") != std::string::npos,
            "circle should document its automatic start point on the circumference");

    std::istringstream xzCircleProgram(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 220\nAXIS A Z 250\n"
        "CIRCLE A X110 Z125 D40 CW F600\n");
    std::ostringstream xzCircleOutput;
    belt::BeltConveyorController xzCircleController;
    xzCircleController.runProgram(xzCircleProgram, "xz-circle.asm", xzCircleOutput);
    require(xzCircleOutput.str().find(
                "DRY-RUN CIRCLE A plane=XZ center=(X110,Z125) diameter=40 mm") !=
                std::string::npos,
            "XZ circle should dry-run with XZ plane and center coordinates");

    std::istringstream invalidCircle(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 100\nAXIS A Y 100\n"
        "CIRCLE A X90 Y90 D30\n");
    requireThrows([&] { controller.runProgram(invalidCircle, "outside-circle.asm", output); },
                  "outside the configured XY travel limits");

    std::istringstream duplicateAxis("A X10 X20\n");
    requireThrows([&] {
        belt::BeltConveyorController::parseLine(duplicateAxis.str(), "duplicate-axis.asm", 1);
    }, "specified more than once");
}

void testDefinitionsAndLimits() {
    belt::BeltConveyorController controller;
    std::ostringstream output;
    controller.apply(belt::BeltDeclaration{"A", "MARLIN", "/dev/ttyUSB0"}, output);
    controller.apply(belt::AxisDeclaration{"A", motion::Axis::X, 100.0}, output);
    controller.apply(belt::AxisDeclaration{"A", motion::Axis::Y, 80.0}, output);
    require(controller.belts().at("A").axes.size() == 2,
            "multiple axes should be allowed on an instance");
    requireThrows([&] {
        controller.apply(belt::MoveCommand{"A", motion::Axis::X, 101.0, 10.0}, output);
    }, "position");
    controller.apply(belt::MoveCommand{"A", motion::Axis::X, 50.0, 10000.0}, output);
    controller.apply(belt::MoveCommand{"A", motion::Axis::X, 100.0, 20.0}, output);
    requireThrows([&] {
        controller.apply(belt::BeltDeclaration{"A", "MARLIN", "/dev/ttyUSB1"}, output);
    }, "already registered");
}

void testDefinitionPersistence() {
    belt::BeltConveyorController original;
    std::istringstream definitions(
        "LIMITS 220 30000 35000\n"
        "BELT B MARLIN /dev/ttyUSB1\n"
        "AXIS B Z 300\n");
    original.loadDefinitions(definitions, "belts.conf");
    std::ostringstream saved;
    original.saveDefinitions(saved);

    belt::BeltConveyorController restored;
    std::istringstream savedInput(saved.str());
    restored.loadDefinitions(savedInput, "saved.conf");
    require(restored.belts().at("B").port == "/dev/ttyUSB1",
            "saved definitions should preserve ports");
    require(restored.belts().at("B").axes.at(motion::Axis::Z).maximumPositionMm == 300.0,
            "saved definitions should preserve the axis position limit");
    require(restored.safetyLimits().maximumDistancePerMoveMm == 220.0 &&
                restored.safetyLimits().maximumDuration == std::chrono::seconds(30) &&
                restored.safetyLimits().commandTimeout == std::chrono::seconds(35),
            "saved definitions should preserve live safety limits");
}

void testBezierProgram() {
    belt::BeltConveyorController controller;
    std::istringstream curves(
        "CURVE diagonal BEZIER A\n"
        "P0 X0 Y0\n"
        "P1 X0 Y10\n"
        "P2 X10 Y10\n"
        "P3 X10 Y0\n"
        "END\n");
    controller.loadCurves(curves, "curves.curve");

    std::istringstream program(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 100\n"
        "AXIS A Y 100\n"
        "PATH diagonal F10\n"
        "A X10 F10\n"
        "WAIT 100\n");
    std::ostringstream output;
    controller.runProgram(program, "sequence.asm", output);
    require(output.str().find("DRY-RUN PATH diagonal") != std::string::npos,
            "program should execute the named Bezier path");
    require(output.str().find("feedrate=10 mm/min") != std::string::npos,
            "path feedrate should be reported in mm/min");
    require(output.str().find("t=0.5 X=5 Y=7.5") != std::string::npos,
            "cubic Bezier midpoint should be sampled correctly");
    require(output.str().find("u=0.25 curve-t=0.103516") != std::string::npos,
            "path parameter should use the quintic progress profile");
    require(output.str().find("profile=QUINTIC samples=101") != std::string::npos,
            "single-axis moves should expose the quintic profile");
    require(output.str().find("DRY-RUN WAIT 100 ms") != std::string::npos,
            "wait should be reported without delaying");
}

void testForAndIfProgram() {
    belt::BeltConveyorController controller;
    std::istringstream program(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 100\n"
        "A X0\n"
        "FOR i = 1 TO 3\n"
        "A X{i} F600\n"
        "IF 2 <= i\n"
        "WAIT 1\n"
        "END\n"
        "END\n"
        "IF A.X <= 3.5\n"
        "A X10\n"
        "ELSE\n"
        "A X20\n"
        "END\n");
    std::ostringstream output;
    controller.runProgram(program, "control-flow.asm", output);
    const std::string result = output.str();
    require(result.find("target=1 mm") != std::string::npos &&
                result.find("target=2 mm") != std::string::npos &&
                result.find("target=3 mm") != std::string::npos,
            "FOR counter should substitute into motion coordinates");
    require(result.find("DRY-RUN IF A.X <= 3.5 -> TRUE") != std::string::npos &&
                result.find("target=10 mm") != std::string::npos &&
                result.find("target=20 mm") == std::string::npos,
            "position IF should execute the matching dry-run branch");
    require(result.find("DRY-RUN WAIT 1 ms") != std::string::npos,
            "IF should evaluate loop counter variables");

    belt::BeltConveyorController steppedController;
    std::istringstream steppedProgram(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 500\n"
        "FOR i = 100 TO 300 STEP 50\n"
        "A X{i} F600\n"
        "END\n");
    std::ostringstream steppedOutput;
    steppedController.runProgram(steppedProgram, "stepped-loop.asm", steppedOutput);
    const auto steppedResult = steppedOutput.str();
    for (const int target : {100, 150, 200, 250, 300}) {
        require(steppedResult.find("target=" + std::to_string(target) + " mm") !=
                    std::string::npos,
                "FOR STEP should include each value at the requested increment");
    }
    require(steppedResult.find("target=125 mm") == std::string::npos,
            "FOR STEP should not use the default one-unit increment");

    std::istringstream outOfScope(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 100\n"
        "IF i == 1\n"
        "WAIT 1\n"
        "END\n");
    requireThrows([&] { controller.runProgram(outOfScope, "scope.asm", output); },
                  "unknown loop variable: i");

    std::istringstream unknownPosition(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 100\n"
        "IF A.X <= 10\n"
        "WAIT 1\n"
        "END\n");
    std::ostringstream unknownPositionOutput;
    requireThrows(
        [&] { controller.runProgram(unknownPosition, "unknown-position.asm",
                                    unknownPositionOutput); },
        "position is unknown in IF condition");
    require(unknownPositionOutput.str().empty(),
            "unknown dry-run position must fail without partial output");

    std::istringstream oversizedLoop(
        "FOR i = 1 TO 10001\n"
        "WAIT 1\n"
        "END\n");
    requireThrows([&] { controller.runProgram(oversizedLoop, "large-loop.asm", output); },
                  "10000-iteration limit");
}

void testLiveIfUsesProvidedPositionAndValidatesBothBranches() {
    belt::BeltConveyorController controller;
    std::istringstream definitions(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 100\n");
    controller.loadDefinitions(definitions, "belts.conf");

    std::istringstream validProgram(
        "IF A.X <= 10\n"
        "A X20\n"
        "ELSE\n"
        "A X30\n"
        "END\n");
    std::vector<double> executedTargets;
    std::ostringstream output;
    controller.runProgram(
        validProgram, "live-if.asm", output,
        [&executedTargets](const belt::Command& command, std::ostream&) {
            if (const auto* move = std::get_if<belt::MoveCommand>(&command)) {
                executedTargets.push_back(move->targetPositionMm);
            }
        },
        [](const std::string&, motion::Axis) { return 5.0; });
    require(executedTargets == std::vector<double>{20.0},
            "live IF should use the position provider and execute only the selected branch");
    require(output.str().find("LIVE IF A.X <= 10 -> TRUE") != std::string::npos,
            "live branch decision should be reported");

    std::istringstream falseBranchProgram(
        "IF A.X <= 10\n"
        "A X20\n"
        "ELSE\n"
        "A X30\n"
        "END\n");
    executedTargets.clear();
    controller.runProgram(
        falseBranchProgram, "live-if-else.asm", output,
        [&executedTargets](const belt::Command& command, std::ostream&) {
            if (const auto* move = std::get_if<belt::MoveCommand>(&command)) {
                executedTargets.push_back(move->targetPositionMm);
            }
        },
        [](const std::string&, motion::Axis) { return 15.0; });
    require(executedTargets == std::vector<double>{30.0},
            "live IF should execute ELSE when the reported position does not match");

    std::istringstream invalidProgram(
        "IF A.X <= 10\n"
        "A X20\n"
        "ELSE\n"
        "A X101\n"
        "END\n");
    executedTargets.clear();
    requireThrows(
        [&] {
            controller.runProgram(
                invalidProgram, "invalid-live-if.asm", output,
                [&executedTargets](const belt::Command& command, std::ostream&) {
                    if (const auto* move = std::get_if<belt::MoveCommand>(&command)) {
                        executedTargets.push_back(move->targetPositionMm);
                    }
                },
                [](const std::string&, motion::Axis) { return 5.0; });
        },
        "invalid-live-if.asm:4");
    require(executedTargets.empty(),
            "both branches must be validated before any live motion is executed");
}

void testProgramFailureIsTransactional() {
    belt::BeltConveyorController controller;
    std::istringstream invalidProgram(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 20\n"
        "A X30 F2\n");
    std::ostringstream output;
    requireThrows([&] { controller.runProgram(invalidProgram, "bad.asm", output); },
                  "bad.asm:3");
    require(controller.belts().empty(), "failed program must not leave partial registrations");
    require(output.str().empty(), "failed program must not emit success-shaped output");
}

void testRemovalAndCurveValidation() {
    belt::BeltConveyorController controller;
    std::istringstream definitions(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 100\n"
        "REMOVE A\n");
    controller.loadDefinitions(definitions, "belts.conf");
    require(controller.belts().empty(), "REMOVE should unregister an instance");

    std::istringstream incompleteCurve("CURVE broken BEZIER A\nP0 X0 Y0\nEND\n");
    requireThrows([&] { controller.loadCurves(incompleteCurve, "curves.curve"); },
                  "requires P0, P1, P2, and P3");
}

void testLiveExecutorIsCalledOnlyAfterValidation() {
    belt::BeltConveyorController controller;
    std::istringstream definitions(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 100\n");
    controller.loadDefinitions(definitions, "belts.conf");
    std::istringstream program("A X10 F600\nWAIT 1\n");
    std::ostringstream output;
    std::size_t executed = 0;
    controller.runProgram(
        program, "live.asm", output,
        [&executed](const belt::Command&, std::ostream& out) {
            ++executed;
            out << "mock execution\n";
        });
    require(executed == 2, "live executor should receive validated motion commands");
    require(output.str().find("mock execution") != std::string::npos,
            "live executor output should be forwarded");
}

void testLiveCurveRejectedBeforeExecution() {
    belt::BeltConveyorController controller;
    std::istringstream curves(
        "CURVE line BEZIER A\n"
        "P0 X0 Y0\nP1 X1 Y1\nP2 X2 Y2\nP3 X3 Y3\nEND\n");
    controller.loadCurves(curves, "curves.curve");
    std::istringstream program(
        "BELT A MARLIN /dev/ttyUSB0\n"
        "AXIS A X 10\nAXIS A Y 10\n"
        "PATH line F1\n");
    std::ostringstream output;
    std::size_t executed = 0;
    requireThrows(
        [&] {
            controller.runProgram(
                program, "curve-live.asm", output,
                [&executed](const belt::Command&, std::ostream&) { ++executed; });
        },
        "PATH is dry-run only");
    require(executed == 0, "live curve rejection must happen before any live command");
}

} // namespace

int main() {
    try {
        testParser();
        testCoordinatedAndCircleMotion();
        testDefinitionsAndLimits();
        testDefinitionPersistence();
        testBezierProgram();
        testForAndIfProgram();
        testLiveIfUsesProvidedPositionAndValidatesBothBranches();
        testProgramFailureIsTransactional();
        testRemovalAndCurveValidation();
        testLiveExecutorIsCalledOnlyAfterValidation();
        testLiveCurveRejectedBeforeExecution();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
