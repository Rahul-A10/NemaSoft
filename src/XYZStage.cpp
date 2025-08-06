#include "XYZStage.h"
#include "utils.h"

// Global variable definition
GlobalVars globle_vars;

// Global instance definition (similar to your xyz_object)
XYZStage xyz_object("COM5");

// Private helper method to get serial handle
// This method opens the serial port and sets the parameters
HANDLE XYZStage::getSerial() {
    HANDLE hSerial = CreateFileA(
        port.c_str(),
        GENERIC_READ | GENERIC_WRITE,  // Added READ permission
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (hSerial == INVALID_HANDLE_VALUE) {
        LOG_CRITICAL("Error opening serial port " << port);
        return INVALID_HANDLE_VALUE;
    }

    // Set parameters
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        LOG_CRITICAL("Failed to get current serial parameters!");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        LOG_CRITICAL("Could not set serial port parameters!");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    // Set timeouts for reading
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 2000;  // 2 second timeout
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 2000;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        LOG_CRITICAL("Could not set serial port timeouts!");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    return hSerial;
}

// Helper method to parse position from COM5 response
void XYZStage::parsePositionResponse(const std::string& response) {
    // Find the backtick character
    size_t backtickPos = response.find('`');

    if (backtickPos == std::string::npos) {
        LOG_INFO("No backtick found in response, cannot parse position");
        return;
    }

    // Extract substring after backtick
    std::string positionPart = response.substr(backtickPos + 1);

    // Find the first three comma-separated numbers
    std::vector<int> positions;
    std::stringstream ss(positionPart);
    std::string token;

    // Extract up to 3 numbers
    int count = 0;
    while (std::getline(ss, token, ',') && count < 3) {
        try {
            // Remove any non-digit characters except minus sign
            std::string cleanToken;
            for (char c : token) {
                if (std::isdigit(c) || c == '-') {
                    cleanToken += c;
                }
            }

            if (!cleanToken.empty()) {
                int value = std::stoi(cleanToken);
                positions.push_back(value);
                count++;
            }
        }
        catch (const std::exception& e) {
            LOG_INFO("Error parsing position token: " << token << " - " << e.what());
            break;
        }
    }

    // Assign to global variables if we got all 3 values
    if (positions.size() >= 3) {
        globle_vars.current_x = positions[0]/scale.x;
        globle_vars.current_y = positions[1]/scale.y;
        globle_vars.current_z = positions[2]/scale.z;

        LOG_INFO("Position updated from COM5 - X: " << globle_vars.current_x
            << ", Y: " << globle_vars.current_y
            << ", Z: " << globle_vars.current_z);
    }
    else {
        LOG_INFO("Could not extract 3 position values from response. Found " << positions.size() << " values.");
    }
}

// Helper method to read response from COM port
std::string XYZStage::readResponse(HANDLE hSerial, int maxWaitMs) {
    std::string response = "";
    char buffer[512] = { 0 };
    DWORD bytesRead = 0;

    // Wait a bit for device to respond
    Sleep(100);

    // Try to read multiple times in case data comes in chunks
    int attempts = maxWaitMs / 100;  // Number of 100ms attempts

    for (int i = 0; i < attempts; i++) {
        if (ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL)) {
            if (bytesRead > 0) {
                buffer[bytesRead] = '\0';  // Null terminate
                response += std::string(buffer);

                // Check if we have a complete response (ends with \r\n or similar)
                if (response.find('\r') != std::string::npos ||
                    response.find('\n') != std::string::npos) {
                    break;
                }
            }
        }

        // If no data yet, wait a bit more
        if (response.empty()) {
            Sleep(100);
        }
    }

    return response;
}

// Private helper method for actual movement
XYZStage::Position XYZStage::_move(double x, double y, double z, double vx, double vy, double vz, char direction) {

    HANDLE hSerial = getSerial();
    if (hSerial == INVALID_HANDLE_VALUE) {
        LOG_CRITICAL("MOVE FAILED - Returning old position");
        return position;
    }

    int sign = (direction == 'D') ? -1 : 1;

    LOG_INFO("trying to move FROM: x=" << globle_vars.current_x << ", y=" << globle_vars.current_y << ", z=" << globle_vars.current_z);

    // Update global variables
    globle_vars.current_x += (x * sign);
    globle_vars.current_y += (y * sign);
    globle_vars.current_z += (z * sign);

    LOG_INFO("TO: x=" << globle_vars.current_x << ", y=" << globle_vars.current_y << ", z=" << globle_vars.current_z);

    // Convert to controller units
    int x_units = static_cast<int>(x * scale.x);
    int y_units = static_cast<int>(y * scale.y);
    int z_units = static_cast<int>(z * scale.z);
    int vx_units = static_cast<int>(vx * scale.x);
    int vy_units = static_cast<int>(vy * scale.y);
    int vz_units = static_cast<int>(vz * scale.z);

    // Update position
    position.x += x_units * sign;
    position.y += y_units * sign;
    position.z += z_units * sign;

    // Create command string based on zero values
    std::string cmd;

    if (x_units == 0 && y_units == 0 && z_units == 0) {
        cmd = "0";
    }
    else if (x_units == 0 && y_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V,,%d%c,,%dR\r\n", vz_units, direction, z_units);
        cmd = buffer;
    }
    else if (x_units == 0 && z_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V,%d%c,%dR\r\n", vy_units, direction, y_units);
        cmd = buffer;
    }
    else if (y_units == 0 && z_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V%d%c%dR\r\n", vx_units, direction, x_units);
        cmd = buffer;
    }
    else if (x_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V,%d,%d%c,%d,%dR\r\n", vy_units, vz_units, direction, y_units, z_units);
        cmd = buffer;
    }
    else if (y_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V%d,,%d%c%d,,%dR\r\n", vx_units, vz_units, direction, x_units, z_units);
        cmd = buffer;
    }
    else if (z_units == 0) {
        char buffer[256];
        sprintf_s(buffer, "/1V%d,%d%c%d,%dR\r\n", vx_units, vy_units, direction, x_units, y_units);
        cmd = buffer;
    }
    else {
        char buffer[256];
        sprintf_s(buffer, "/1V%d,%d,%d%c%d,%d,%dR\r\n", vx_units, vy_units, vz_units, direction, x_units, y_units, z_units);
        cmd = buffer;
    }

    // Send command and read response
    if (hSerial != INVALID_HANDLE_VALUE) {
        DWORD bytesWritten;
        if (!WriteFile(hSerial, cmd.c_str(), static_cast<DWORD>(cmd.length()), &bytesWritten, NULL)) {
            LOG_CRITICAL("Failed to write to serial port!");
        }
        else {
            LOG_INFO("Move command SENT: " << cmd);

            // Calculate and wait for movement to complete
            if (x_units != 0 || y_units != 0 || z_units != 0) {
                double sleep_time = 0.0;
                double temp_time = 0.0;

                if (vx_units > 1) {
                    temp_time = std::abs(static_cast<double>(x_units) / (vx_units - 1));
                    if (temp_time > sleep_time) sleep_time = temp_time;
                }
                if (vy_units > 1) {
                    temp_time = std::abs(static_cast<double>(y_units) / (vy_units - 1));
                    if (temp_time > sleep_time) sleep_time = temp_time;
                }
                if (vz_units > 1) {
                    temp_time = std::abs(static_cast<double>(z_units) / (vz_units - 1));
                    if (temp_time > sleep_time) sleep_time = temp_time;
                }

                sleep_time += 0.5; // Add 0.5 seconds buffer

                // Sleep the main thread for the duration of the movement
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
            }

            // Send position query command after move completes
            char buffer2[256];
            sprintf_s(buffer2, "/1?aA\r\n");
            std::string cmd2 = buffer2;

            if (!WriteFile(hSerial, cmd2.c_str(), static_cast<DWORD>(cmd2.length()), &bytesWritten, NULL)) {
                LOG_CRITICAL("Failed to write position query command!");
            }
            else {
                LOG_INFO("Position query command SENT: " << cmd2);

                // Read response from position query (cmd2)
                std::string response = readResponse(hSerial, 2000);  // Wait up to 2 seconds

                if (!response.empty()) {
                    // Extract clean response - get 3 numbers after backtick
                    std::string cleanResponse;
                    size_t backtickPos = response.find('`');

                    if (backtickPos != std::string::npos) {
                        // Extract substring after backtick
                        std::string positionPart = response.substr(backtickPos + 1);

                        // Extract first 3 comma-separated numbers
                        std::stringstream ss(positionPart);
                        std::string token;
                        int count = 0;

                        while (std::getline(ss, token, ',') && count < 3) {
                            // Clean token to keep only digits and minus sign
                            std::string cleanToken;
                            for (char c : token) {
                                if (std::isdigit(c) || c == '-') {
                                    cleanToken += c;
                                }
                            }

                            if (!cleanToken.empty()) {
                                if (count > 0) cleanResponse += ",";
                                cleanResponse += cleanToken;
                                count++;
                            }
                        }

                        if (count == 3) {
                            LOG_INFO("COM5 Response: " << cleanResponse);
                            // Parse and extract position values
                            parsePositionResponse(response);
                            // Also output to Visual Studio Debug window
                            std::string debugMsg = "COM5 Response: " + cleanResponse + "\n";
                            OutputDebugStringA(debugMsg.c_str());
                        }
                        else {
                            LOG_INFO("No response");
                            OutputDebugStringA("No response\n");
                        }
                    }
                    else {
                        LOG_INFO("No response");
                        OutputDebugStringA("No response\n");
                    }
                }
                else {
                    LOG_INFO("No response");
                    OutputDebugStringA("No response\n");
                }
            }
        }

        CloseHandle(hSerial);
    }

    return position;
}

// Constructor
XYZStage::XYZStage(const std::string& portName) : port(portName) {
    LOG_INFO("XYZStage initialized to: x=" << position.x << ", y=" << position.y << ", z=" << position.z);
}

// Public move method
void XYZStage::move(double dx, double dy, double dz, double velocity_x, double velocity_y, double velocity_z) {
    char direction = (dx >= 0 && dy >= 0 && dz >= 0) ? 'P' : 'D';
    _move(std::abs(dx), std::abs(dy), std::abs(dz), velocity_x, velocity_y, velocity_z, direction);
}

// Getter for current position
XYZStage::Position XYZStage::getPosition() const {
    return position;
}

// Getter for port
std::string XYZStage::getPort() const {
    return port;
}

// Setter for port
void XYZStage::setPort(const std::string& newPort) {
    port = newPort;
}