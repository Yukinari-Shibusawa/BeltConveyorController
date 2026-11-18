cmake_minimum_required(VERSION 3.16)

project(BeltConveyorController LANGUAGES CXX)

set(MARLIN_CONTROLLER_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../Marlin/MarlinController"
    CACHE PATH "Path to the local MarlinController project")
set(EASY_MACHINE_LANGUAGE_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../EasyMachineLanguage"
    CACHE PATH "Path to the local EasyMachineLanguage project")

if(NOT EXISTS "${MARLIN_CONTROLLER_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "MarlinController was not found at '${MARLIN_CONTROLLER_SOURCE_DIR}'. "
        "Set MARLIN_CONTROLLER_SOURCE_DIR to its local project directory.")
endif()

if(NOT EXISTS "${EASY_MACHINE_LANGUAGE_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "EasyMachineLanguage was not found at '${EASY_MACHINE_LANGUAGE_SOURCE_DIR}'. "
        "Set EASY_MACHINE_LANGUAGE_SOURCE_DIR to its project directory.")
endif()

add_subdirectory(
    "${MARLIN_CONTROLLER_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/marlincontroller"
)
add_subdirectory(
    "${EASY_MACHINE_LANGUAGE_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/easymachinelanguage"
)

add_library(beltcontroller_core
    src/BeltConveyorController.cpp
    src/LiveMotionExecutor.cpp
)
add_library(beltcontroller::core ALIAS beltcontroller_core)

target_include_directories(beltcontroller_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(beltcontroller_core PUBLIC cxx_std_17)
target_link_libraries(beltcontroller_core
    PUBLIC marlincontroller
    PRIVATE EasyMachineLanguage::easy_machine_language
)

add_executable(belt_conveyor_controller src/main.cpp)
target_link_libraries(belt_conveyor_controller PRIVATE beltcontroller::core)

include(CTest)
if(BUILD_TESTING)
    add_executable(beltcontroller_tests tests/BeltConveyorControllerTests.cpp)
    target_link_libraries(beltcontroller_tests PRIVATE beltcontroller::core)
    add_test(NAME beltcontroller_tests COMMAND beltcontroller_tests)
endif()

include(GNUInstallDirs)
install(TARGETS beltcontroller_core belt_conveyor_controller
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
