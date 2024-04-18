#include "user.hpp"
#include "threads.hpp"

#include <cstdlib>

/*
 * This program sends a simple CAN message between two testboards.
 * Load the program onto two testboards and connect them together. 
 * Pressing the user button will toggle the OK LED on the other testboard.
 * Long pressing the user button will toggle the error LED on the other testboard.
 */

extern "C" void CPP_UserSetup(void);
extern "C" CAN_HandleTypeDef hcan1;
extern "C" CAN_HandleTypeDef hcan2;


// Initialize User button
Button user_button = Button(BTN1_GPIO_Port, BTN1_Pin, 50, GPIO_PIN_SET, false);

// Initialize CAN frames and devices
CANDevice candev1 = CANDevice(&hcan1);
CANDevice candev2 = CANDevice(&hcan2);

// Initialize command map
static void HeadlightsCommand();
static void LeftTurnCommand();
static void RightTurnCommand();
static void HazardsCommand();
static void BrakeCommand();
static void StrobeCommand();
const etl::unordered_map<etl::string<TOKEN_MAX_SIZE>, void(*)(), 6> command_map = {
    {"headlights", HeadlightsCommand},
    {"leftturn", LeftTurnCommand},
    {"rightturn", RightTurnCommand},
    {"hazards", HazardsCommand},
    {"brake", BrakeCommand},
    {"strobe", StrobeCommand}
};


void CPP_UserSetup(void) {
    // Test that timer priorities are configured correctly
    HAL_Delay(1);

    // Setup button callbacks
    user_button.RegisterNormalPressCallback(ButtonSingleCallback);
    user_button.RegisterLongPressCallback(ButtonLongCallback);
    //user_button.RegisterDoublePressCallback(DoubleCallback);

    // Add devices to CANController
    CANController::AddDevice(&candev1);
    CANController::AddDevice(&candev2);

    // Add message to rx map, otherwise CANController will ignore it
    // Simultaneously register callback with rx message
    CANController::AddRxMessage(&IoTestFrame::Instance(), IoMsgCallback);
    CANController::AddRxMessage(&MitsubaFrame0::Instance(), MitsubaMsgCallback);

    // Set filter to accept all messages
    CANController::AddFilterAll();

    // Start CAN peripherals
    CANController::Start();

    // Initialize virtual COM port
    VCOM_Init();

    // Initialize threads
    ThreadsInit();
}

/* Command helper functions */
void HeadlightsCommand() {
    int value = std::stoi(strtok(NULL, " "));

    DriverControlsFrame1::SetHeadlight(static_cast<bool>(value));
    CANController::Send(&DriverControlsFrame1::Instance());

    VCOM_Transmit("Set headlights ", 15);
    VCOM_Transmit(value ? "on\n" : "off\n", value ? 3 : 4);
}

void LeftTurnCommand() {
    int value = std::stoi(strtok(NULL, " "));

    DriverControlsFrame1::SetLeftTurn(static_cast<bool>(value));
    CANController::Send(&DriverControlsFrame1::Instance());

    VCOM_Transmit("Set left turn ", 14);
    VCOM_Transmit(value ? "on\n" : "off\n", value ? 3 : 4);
}

void RightTurnCommand() {
    int value = std::stoi(strtok(NULL, " "));

    DriverControlsFrame1::SetRightTurn(static_cast<bool>(value));
    CANController::Send(&DriverControlsFrame1::Instance());

    VCOM_Transmit("Set right turn ", 15);
    VCOM_Transmit(value ? "on\n" : "off\n", value ? 3 : 4);
}

void HazardsCommand() {
    int value = std::stoi(strtok(NULL, " "));

    DriverControlsFrame1::SetHazards(static_cast<bool>(value));
    CANController::Send(&DriverControlsFrame1::Instance());

    VCOM_Transmit("Set hazards ", 12);
    VCOM_Transmit(value ? "on\n" : "off\n", value ? 3 : 4);
}

void BrakeCommand() {
    int value = std::stoi(strtok(NULL, " "));

    DriverControlsFrame0::SetBrakeEnable(static_cast<bool>(value));
    CANController::Send(&DriverControlsFrame0::Instance());

    VCOM_Transmit("Set brake lights ", 16);
    VCOM_Transmit(value ? "on\n" : "off\n", value ? 3 : 4);
}

void StrobeCommand() {

    CANController::Send(&DriverControlsFrame1::Instance());
}