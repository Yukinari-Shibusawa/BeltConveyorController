#include "BeltConveyorController.hpp"
#include "LiveMotionExecutor.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace {

struct Options {
    std::string configPath{"belts.conf"};
    std::string curvesPath;
    std::string runPath;
    bool live{false};
};

void printHelp(const char* executable) {
    std::cout << "Usage:\n"
    << "  " << executable << " [--config FILE] [--curves FILE] [--live]\n"
    << "  " << executable
    << " --run FILE [--config FILE] [--curves FILE] [--live]\n\n"
    << "ASM files support FOR/IF blocks; see docs/USAGE.md for the syntax.\n"
    << "Default mode is dry-run. --live may home and move real hardware.\n";
}

Options parseOptions(int argc, char* argv[]) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];
        if (option == "--help" || option == "-h") {
            printHelp(argv[0]);
            std::exit(0);
        }
        if (option == "--live") {
            options.live = true;
            continue;
        }
        if (index + 1 >= argc) {
            throw std::invalid_argument("Missing value for " + option);
        }
        const std::string value = argv[++index];
        if (value.empty()) {
            throw std::invalid_argument("Value must not be empty for " + option);
        }
        if (option == "--config") {
            options.configPath = value;
        } else if (option == "--curves") {
            options.curvesPath = value;
        } else if (option == "--run") {
            options.runPath = value;
        } else {
            throw std::invalid_argument("Unknown option: " + option);
        }
    }
    return options;
}

bool confirmLiveMode() {
    std::cout << "\nLIVE MODE: configured Marlin devices may home and move. "
                 "Verify the selected ports, travel limits, and emergency stop.\n"
                 "Continue? [y/N]: "
              << std::flush;
    std::string confirmation;
    return std::getline(std::cin, confirmation) &&
           (confirmation == "y" || confirmation == "Y");
}

void loadDefinitions(belt::BeltConveyorController& controller, const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open belt definitions file: " + path);
    }
    controller.loadDefinitions(input, path);
}

void loadCurvesIfRequested(belt::BeltConveyorController& controller,
                           const std::string& path) {
    if (path.empty()) return;
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open curve file: " + path);
    }
    controller.loadCurves(input, path);
}

void saveDefinitionsAtomically(const belt::BeltConveyorController& controller,
                               const std::string& path) {
    const std::filesystem::path destination(path);
    const std::filesystem::path temporary = destination.string() + ".tmp";
    {
        std::ofstream output(temporary, std::ios::trunc);
        if (!output) {
            throw std::runtime_error("Cannot write definitions to " + temporary.string());
        }
        controller.saveDefinitions(output);
        output.flush();
        if (!output) {
            throw std::runtime_error("Failed while writing definitions to " + temporary.string());
        }
        output.close();
        if (!output) {
            throw std::runtime_error("Failed while closing definitions file " +
                                     temporary.string());
        }
    }

    std::error_code error;
    std::filesystem::rename(temporary, destination, error);
    if (error) {
        std::error_code cleanupError;
        std::filesystem::remove(temporary, cleanupError);
        throw std::runtime_error("Cannot replace definitions file " + path + ": " +
                                 error.message());
    }
}

bool isDefinitionCommand(const belt::Command& command) {
    return std::holds_alternative<belt::BeltDeclaration>(command) ||
           std::holds_alternative<belt::AxisDeclaration>(command) ||
           std::holds_alternative<belt::RemoveDeclaration>(command) ||
           std::holds_alternative<belt::SafetyLimitsDeclaration>(command);
}

void printList(const belt::BeltConveyorController& controller) {
    if (controller.belts().empty()) {
        std::cout << "No belts registered.\n";
        return;
    }
    for (const auto& entry : controller.belts()) {
        const auto& device = entry.second;
        std::cout << device.name << " " << device.communication << " " << device.port
                  << " (115200 baud)\n";
        for (const auto& axis : device.axes) {
            char letter = '?';
            switch (axis.first) {
            case motion::Axis::X: letter = 'X'; break;
            case motion::Axis::Y: letter = 'Y'; break;
            case motion::Axis::Z: letter = 'Z'; break;
            }
            std::cout << "  " << letter << " max-position=" << axis.second.maximumPositionMm
                      << " mm\n";
        }
    }
}

void printReplHelp() {
    std::cout << "Commands:\n"
              << "  BELT <name> MARLIN <port>\n"
    << "  AXIS <name> <X|Y|Z> <max-position-mm>\n"
    << "  LIMITS <max-move-mm> <max-duration-ms> <timeout-ms>\n"
    << "  REMOVE <name>\n"
              << "  <name> <X|Y|Z><target-mm> [F<feedrate-mm/min>]\n"
              << "  PATH <curve-name> [F<feedrate-mm/min>]\n"
              << "  WAIT <milliseconds> | LIST | HELP | QUIT\n"
              << "FOR/IF blocks are available in ASM files, not in the one-line REPL.\n"
              << "Definition edits are saved immediately. Default is dry-run; --live is required "
                 "for hardware. Default feedrate is F10000 mm/min.\n";
}

int runInteractive(const Options& options) {
    belt::BeltConveyorController controller;
    std::error_code fileError;
    const bool configExists = std::filesystem::exists(options.configPath, fileError);
    if (fileError) {
        throw std::runtime_error("Cannot inspect definitions file " + options.configPath + ": " +
                                 fileError.message());
    }
    if (configExists) {
        loadDefinitions(controller, options.configPath);
    } else if (options.live) {
        throw std::runtime_error("Live mode requires a belt definitions file: " +
                                 options.configPath);
    }
    loadCurvesIfRequested(controller, options.curvesPath);
    if (options.live && !confirmLiveMode()) {
        std::cout << "Live mode cancelled.\n";
        return 0;
    }
    auto liveExecutor = options.live
                            ? std::make_unique<belt::LiveMotionExecutor>(controller)
                            : nullptr;

    std::cout << (options.live ? "BeltConveyorController LIVE MODE.\n"
                               : "BeltConveyorController dry-run (no hardware access).\n")
              << "Type HELP for commands.\n";
    std::string line;
    std::size_t lineNumber = 0;
    while (std::cout << "> " && std::getline(std::cin, line)) {
        ++lineNumber;
        if (line == "QUIT" || line == "quit" || line == "exit") break;
        if (line == "HELP" || line == "help") {
            printReplHelp();
            continue;
        }
        if (line == "LIST" || line == "list") {
            printList(controller);
            continue;
        }
        try {
            const auto command =
                belt::BeltConveyorController::parseLine(line, "<stdin>", lineNumber);
            if (!command) continue;
            if (isDefinitionCommand(*command)) {
                belt::BeltConveyorController candidate = controller;
                std::ostringstream response;
                candidate.apply(*command, response);
                saveDefinitionsAtomically(candidate, options.configPath);
                controller = std::move(candidate);
                if (options.live) {
                    liveExecutor = std::make_unique<belt::LiveMotionExecutor>(controller);
                }
                std::cout << response.str();
            } else if (options.live) {
                belt::BeltConveyorController candidate = controller;
                std::ostringstream validation;
                candidate.apply(*command, validation);
                liveExecutor->execute(*command, std::cout);
                controller = std::move(candidate);
            } else {
                controller.apply(*command, std::cout);
            }
        } catch (const std::exception& exception) {
            std::cerr << exception.what() << '\n';
        }
    }
    return 0;
}

int runFile(const Options& options) {
    std::ifstream input(options.runPath);
    if (!input) {
        throw std::runtime_error("Cannot open program file: " + options.runPath);
    }
    belt::BeltConveyorController controller;
    loadDefinitions(controller, options.configPath);
    loadCurvesIfRequested(controller, options.curvesPath);
    if (options.live && !confirmLiveMode()) {
        std::cout << "Live mode cancelled.\n";
        return 0;
    }
    if (options.live) {
        // Supply live execution and position callbacks; runProgram validates before invoking them.
        belt::LiveMotionExecutor liveExecutor(controller);
        controller.runProgram(input, options.runPath, std::cout,
                              [&liveExecutor](const belt::Command& command, std::ostream& output) {
                                  liveExecutor.execute(command, output);
                              },
                              [&liveExecutor](const std::string& beltName,
                                             motion::Axis axis) {
                                  return liveExecutor.position(beltName, axis, std::cout);
                              });
    } else {
        controller.runProgram(input, options.runPath, std::cout);
    }
    return 0;
}

} // namespace

int main(int argc, char* argv[]) {
    try {
        const Options options = parseOptions(argc, argv);
        if (!options.runPath.empty()) {
            return runFile(options);
        }
        return runInteractive(options);
    } catch (const std::exception& exception) {
        std::cerr << "Error: " << exception.what() << '\n';
        return 1;
    }
}
