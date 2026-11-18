# BeltConveyorController Development Guidelines

## Purpose

This project describes and dry-runs motion sequences for multiple belt
conveyors; explicit `--live` mode can execute single-axis motion through the
neighboring MarlinController project. Bezier PATH remains dry-run only.

## Development approach

- Inspect existing code and clarify requirements before broad changes.
- Keep parsing, motion planning, CLI, and tests separated by responsibility.
- Keep generic FOR/IF parsing and interpretation in the sibling
  `../EasyMachineLanguage/` project.
  Domain-specific instruction parsing and device behavior belong in a
  `easyml::ProgramHost` implementation in the consuming project.
- Prefer RAII, clear ownership, and explicit error handling.
- Keep external dependencies to a minimum.
- Update the README and relevant documentation when syntax, units, or behavior
  change.
- Add or update tests for behavior changes.

## Build and test

- Use CMake and a C++17-capable compiler.
- Build with `make build` and run tests with `make test`.
- Report tests that were not run; do not imply they passed.
- Do not add generated build output to source control.

## Safety

- Dry-run is the default. Live motion requires `--live` and the interactive
  `y` confirmation. Never perform hardware operations during build or tests.
- Keep device protocol details behind `motion::MotionController`'s transport abstraction.
- Use millimetres and mm/min throughout the application and MarlinController
  API. Marlin G5 uses mm/s, so convert only at that protocol boundary if it is
  implemented.
- Validate coordinates against configured travel and moves against `LIMITS`;
  validate feedrates as positive mm/min values. There is no configured
  software feedrate ceiling.
- Never run serial or physical motion actions as part of building or testing.
