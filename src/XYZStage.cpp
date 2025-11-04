#include "XYZStage.h"
#include "utils.h"
#include "mainwindow.h"
#include "logger.h"
// Global variable definition
GlobalVars globle_vars;

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
        //log("Error opening serial port " + QString::fromStdString(port), "CRITICAL");
        return INVALID_HANDLE_VALUE;
    }

    // Set parameters
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        //log("Failed to get current serial parameters!", "CRITICAL");
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        //log("Could not set serial port parameters!", "CRITICAL");
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
        //log("Could not set serial port timeouts!", "CRITICAL");
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
        log("No backtick found in response, cannot parse position", "INFO");
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
            log(QString("Error parsing position token: %1 - %2").arg(QString::fromStdString(token)).arg(e.what()), "INFO");
            break;
        }
    }

    // Assign to global variables if we got all 3 values
    if (positions.size() >= 3) {
        globle_vars.current_x = positions[0] / scale.x;
        globle_vars.current_y = positions[1] / scale.y;
        globle_vars.current_z = positions[2] / scale.z;

        log(QString("Position updated from COM5 - X: %1, Y: %2, Z: %3")
            .arg(globle_vars.current_x)
            .arg(globle_vars.current_y)
            .arg(globle_vars.current_z), "INFO");
    }
    else {
        log(QString("Could not extract 3 position values from response. Found %1 values.").arg(positions.size()), "INFO");
    }
}

// Helper method to read response from COM port
std::string XYZStage::readResponse(HANDLE hSerial, int maxWaitMs) {
    std::string response = "";
    char buffer[256] = { 0 };
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
XYZStage::Position XYZStage::_home() {
    HANDLE hSerial = getSerial();
    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        log("MOVE FAILED - Returning old position", "CRITICAL");
        return position;
    }

    else {
        //DWORD bytesWritten;
        char buffer[256];
        sprintf_s(buffer, "/1aM1f1aM2f1aM3f1R\r\n");
        std::string cmd = buffer;
        log("Home command SENT: " + QString::fromStdString(cmd), "INFO");
        std::this_thread::sleep_for(std::chrono::seconds(10));
        sprintf_s(buffer, "/1aM1N1Z10000R\r\n");
        cmd = buffer;
        log("Home command SENT: " + QString::fromStdString(cmd), "INFO");
        std::this_thread::sleep_for(std::chrono::seconds(10));
        sprintf_s(buffer, "/1aM2N1Z10000R\r\n");
        cmd = buffer;
        log("Home command SENT: " + QString::fromStdString(cmd), "INFO");
        std::this_thread::sleep_for(std::chrono::seconds(10));
        sprintf_s(buffer, "/1aM3N1Z10000R\r\n");
        cmd = buffer;
        log("Home command SENT: " + QString::fromStdString(cmd), "INFO");
        std::this_thread::sleep_for(std::chrono::seconds(10));
        sprintf_s(buffer, "/1z0,0,0R\r\n");
        cmd = buffer;

        log("Home command SENT: " + QString::fromStdString(cmd), "INFO");
        // Wait for homing to complete - this is a guess, adjust as needed
        std::this_thread::sleep_for(std::chrono::seconds(10));
        // Send position query command after homing completes
    //getPosition();




    }

}
XYZStage::Position XYZStage::_move(double x, double y, double z, double vx, double vy, double vz, char direction) {

    HANDLE hSerial = getSerial();
    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        log("MOVE FAILED - Returning old position", "CRITICAL");
        globle_vars.current_x = globle_vars.current_x + x;
        globle_vars.current_y = globle_vars.current_y + y;
        globle_vars.current_z = globle_vars.current_z + z;

        return position;
    }


    int sign = (direction == 'D') ? -1 : 1;
    //direction = 'A'; // 'A' for absolute movement


    log(QString("trying to move FROM: x=%1, y=%2, z=%3")
        .arg(globle_vars.current_x)
        .arg(globle_vars.current_y)
        .arg(globle_vars.current_z), "INFO");

    // Update global variables
    int boundx = globle_vars.current_x + (x * sign);
    int boundy = globle_vars.current_y + (y * sign);
    int boundz = globle_vars.current_z + (z * sign);


    if (boundx < globle_vars.min_x || boundy < globle_vars.min_y || boundz < globle_vars.min_z) {
        log("MOVE FAILED - Negative position out of bounds", "CRITICAL");
        return position;
    }
    else if (boundx > globle_vars.max_x || boundy > globle_vars.max_y || boundz > globle_vars.max_z) {
        log("MOVE FAILED - Position out of bounds", "CRITICAL");
        return position;
    }

    //appendLog(QString("TO: x=%1, y=%2, z=%3")
    //    .arg(globle_vars.current_x)
    //    .arg(globle_vars.current_y)
    //    .arg(globle_vars.current_z), "INFO");

    // Convert to controller units
    int x_units = static_cast<int>(x * scale.x);
    int y_units = static_cast<int>(y * scale.y);
    int z_units = static_cast<int>(z * scale.z);
    int vx_units = static_cast<int>(vx * scale.x);
    int vy_units = static_cast<int>(vy * scale.y);
    int vz_units = static_cast<int>(vz * scale.z);

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
    if (m_serialHandle != INVALID_HANDLE_VALUE) {
        DWORD bytesWritten;
        if (!WriteFile(m_serialHandle, cmd.c_str(), static_cast<DWORD>(cmd.length()), &bytesWritten, NULL)) {
            log("Failed to write to serial port!", "CRITICAL");
        }
        else {
            log("Move command SENT: " + QString::fromStdString(cmd), "INFO");

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
                    sleep_time += 0.5;
                }

                sleep_time += 0.5; // Add 0.5 seconds buffer

                // Sleep the main thread for the duration of the movement
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
            }

            // Send position query command after move completes
            getPosition();

        }

        //CloseHandle(hSerial);
    }

    return position;
}
XYZStage::Position XYZStage::getPosition() {
    //std::lock_guard<std::mutex> lock(m_syncMutex); // Ensure thread safety

    DWORD bytesWritten;

    char buffer2[256];
    sprintf_s(buffer2, "/1?aA\r\n");
    //sprintf_s(buffer2, "/1s0R\r\n");
    std::string cmd2 = buffer2;

    if (!WriteFile(m_serialHandle, cmd2.c_str(), static_cast<DWORD>(cmd2.length()), &bytesWritten, NULL)) {
        log("Failed to write position query command!", "CRITICAL");
    }
    else {
        log("Position query command SENT: " + QString::fromStdString(cmd2), "INFO");

        // Read response from position query (cmd2)
        std::string response = readResponse(m_serialHandle, 2000);  // Wait up to 2 seconds

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
                    log("COM5 Response: " + QString::fromStdString(cleanResponse), "INFO");
                    // Parse and extract position values
                    parsePositionResponse(response);
                    // Also output to Visual Studio Debug window
                    std::string debugMsg = "COM5 Response: " + cleanResponse + "\n";
                    OutputDebugStringA(debugMsg.c_str());

                }
                else {
                    log("No response", "INFO");
                    OutputDebugStringA("No response\n");
                }
            }
            else {
                log("No response", "INFO");
                OutputDebugStringA("No response\n");
            }
        }
        else {
            log("No response", "INFO");
            OutputDebugStringA("No response\n");
        }
    }
    return position;
}

XYZStage::XYZStage(const std::string& portName)
    : port(portName), m_stopWorker(false) {
    log(QString("XYZStage initialized to: x=%1, y=%2, z=%3")
        .arg(position.x)
        .arg(position.y)
        .arg(position.z), "INFO");
    m_serialHandle = getSerial();
    if (m_serialHandle != INVALID_HANDLE_VALUE) {
        log("Querying initial position from stage...", "INFO");

        // ?? Avoid deadlock by using a temporary call without locking
        {
            // Make sure no other thread can contend here because
            // the worker thread isn't started yet.
            getPosition();
        }
    }
    else {
        log("Serial connection failed, cannot query initial position.", "CRITICAL");
    }
    // Start the worker thread upon construction
    m_workerThread = std::thread(&XYZStage::worker, this);
}


XYZStage::~XYZStage() {
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_stopWorker = true;
    }
    log("Stopping XYZStage worker thread...", "INFO");
    // Notify the condition variable to wake the thread up if it's waiting
    m_condition.notify_one();

    // Wait for the thread to finish its work and exit
    if (m_workerThread.joinable()) {
        m_workerThread.join();
    }
}


void XYZStage::move(double dx, double dy, double dz, double velocity_x, double velocity_y, double velocity_z) {
    {
        // Acquire lock to safely add to the queue
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_commandQueue.push({ dx, dy, dz, velocity_x, velocity_y, velocity_z });
        log(QString("Queued move command: dx=%1, dy=%2, dz=%3 and notifying the worker")
            .arg(dx)
            .arg(dy)
            .arg(dz), "INFO");
    }
    // Notify the worker thread that a new command is available
    m_condition.notify_one();
}

void XYZStage::home() {
    {
        //std::lock_guard<std::mutex> lock(m_queueMutex);
        _home();
        log("Queued home command and notifying the worker", "INFO");
    }
    //m_condition.notify_one();
}

// This function runs in a separate thread, processing commands from the queue.
void XYZStage::worker() {
    while (true) {
        MoveCommand currentCommand;

        {
            // Acquire a unique lock to wait on the condition variable
            std::unique_lock<std::mutex> lock(m_queueMutex);

            // Wait until the queue is not empty OR the stop signal is received
            m_condition.wait(lock, [this] {
                return !m_commandQueue.empty() || m_stopWorker;
                });

            // If woken up to stop and the queue is empty, exit the thread
            if (m_stopWorker && m_commandQueue.empty()) {
                return;
            }

            // Get the next command from the queue
            currentCommand = m_commandQueue.front();
            m_commandQueue.pop();
            log(QString("Dequeued move command: dx=%1, dy=%2, dz=%3")
                .arg(currentCommand.dx)
                .arg(currentCommand.dy)
                .arg(currentCommand.dz), "INFO");
        } // The lock is automatically released here

        // --- Execute the move ---
        // The logic to determine direction is moved from 'move' to here
        char direction = (currentCommand.dx >= 0 && currentCommand.dy >= 0 && currentCommand.dz >= 0) ? 'P' : 'D';
        _move(std::abs(currentCommand.dx),
            std::abs(currentCommand.dy),
            std::abs(currentCommand.dz),
            currentCommand.vx,
            currentCommand.vy,
            currentCommand.vz,
            direction);

        // Check if a blocking call is waiting and notify it
        if (m_isWaitingForMoveCompletion.load()) {
            {
                std::lock_guard<std::mutex> lock(m_syncMutex);
                m_isWaitingForMoveCompletion = false; // Reset the flag
            }
            m_syncCondition.notify_one(); // Wake up the move_and_wait function
        }
    }
}


void XYZStage::move_and_wait(double dx, double dy, double dz, double velocity_x, double velocity_y, double velocity_z) {
    {
        std::unique_lock<std::mutex> lock(m_syncMutex);

        // Set the flag indicating that we will wait
        m_isWaitingForMoveCompletion = true;

        // Queue the command using the normal non-blocking method
        move(dx, dy, dz, velocity_x, velocity_y, velocity_z);

        // Now, wait until the worker thread signals completion
        log("move_and_wait: Waiting for move to complete...", "INFO");
        m_syncCondition.wait(lock, [this] { return !m_isWaitingForMoveCompletion.load(); });
        log("move_and_wait: Move completed. Proceeding.", "INFO");
    }
}