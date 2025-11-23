#include "XYZStage.h"
#include "logger.h"

// Global variable definition
GlobalVars globle_vars;

// Constructor
XYZStage::XYZStage(const std::string& portName)
    : port(portName)
    , m_stopWorker(false)
    , m_isWaitingForMoveCompletion(false)
    , m_serialHandle(INVALID_HANDLE_VALUE)
{
    log(QString("XYZStage initializing on port: %1").arg(QString::fromStdString(port)), "INFO");

    // Open serial connection
    m_serialHandle = getSerial();

    if (m_serialHandle != INVALID_HANDLE_VALUE) {
        log("Serial connection established", "INFO");

        // Query initial position
        getPosition();
    }
    else {
        log("Serial connection failed - running in simulation mode", "WARNING");
    }

    // Start the worker thread
    log("Starting worker thread...", "INFO");
    m_workerThread = std::thread(&XYZStage::worker, this);
    log("Worker thread started successfully", "INFO");
}

// Destructor
XYZStage::~XYZStage() {
    log("Shutting down XYZStage...", "INFO");

    // Signal worker thread to stop
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_stopWorker = true;
    }

    // Wake up worker thread
    m_condition.notify_one();

    // Wait for worker thread to finish
    if (m_workerThread.joinable()) {
        log("Waiting for worker thread to finish...", "INFO");
        m_workerThread.join();
        log("Worker thread stopped", "INFO");
    }

    // Close serial connection
    if (m_serialHandle != INVALID_HANDLE_VALUE) {
        CloseHandle(m_serialHandle);
        m_serialHandle = INVALID_HANDLE_VALUE;
    }

    log("XYZStage shutdown complete", "INFO");
}

// Worker thread function
void XYZStage::worker() {
    log("Worker thread running", "INFO");

    while (true) {
        try {
            MoveCommand currentCommand;

            {
                std::unique_lock<std::mutex> lock(m_queueMutex);

                // Wait for command or stop signal
                m_condition.wait(lock, [this] {
                    return !m_commandQueue.empty() || m_stopWorker;
                    });

                // Exit if stopping and queue is empty
                if (m_stopWorker && m_commandQueue.empty()) {
                    log("Worker thread exiting", "INFO");
                    return;
                }

                // Get command from queue
                currentCommand = m_commandQueue.front();
                m_commandQueue.pop();

                log(QString("Worker dequeued command: dx=%1, dy=%2, dz=%3")
                    .arg(currentCommand.dx)
                    .arg(currentCommand.dy)
                    .arg(currentCommand.dz), "INFO");
            }

            // Execute the move (outside the lock)
            char direction = (currentCommand.dx >= 0 && currentCommand.dy >= 0 && currentCommand.dz >= 0) ? 'P' : 'D';

            // Wrap low level move in try/catch so worker thread doesn't terminate on exceptions
            try {
                _move(std::abs(currentCommand.dx),
                    std::abs(currentCommand.dy),
                    std::abs(currentCommand.dz),
                    currentCommand.vx,
                    currentCommand.vy,
                    currentCommand.vz,
                    direction);
            } catch (const std::exception& e) {
                log(QString("Exception in _move(): %1").arg(e.what()), "CRITICAL");
            } catch (...) {
                log("Unknown exception in _move()", "CRITICAL");
            }

            // Notify if blocking call is waiting
            if (m_isWaitingForMoveCompletion.load()) {
                {
                    std::lock_guard<std::mutex> lock(m_syncMutex);
                    m_isWaitingForMoveCompletion = false;
                }
                m_syncCondition.notify_one();
            }
        }
        catch (const std::exception& e) {
            log(QString("Worker thread caught exception: %1").arg(e.what()), "CRITICAL");
            // small sleep to avoid busy-loop in case of persistent failure
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        catch (...) {
            log("Worker thread caught unknown exception", "CRITICAL");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
    }
}

// Public move method (non-blocking, queues command)
void XYZStage::move(double dx, double dy, double dz, double velocity_x, double velocity_y, double velocity_z) {
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_commandQueue.push({ dx, dy, dz, velocity_x, velocity_y, velocity_z });

        log(QString("Queued move: dx=%1, dy=%2, dz=%3")
            .arg(dx).arg(dy).arg(dz), "INFO");
    }

    // Notify worker thread
    m_condition.notify_one();
}

// Blocking move method (waits for completion)
void XYZStage::move_and_wait(double dx, double dy, double dz, double velocity_x, double velocity_y, double velocity_z) {
    std::unique_lock<std::mutex> lock(m_syncMutex);

    // Set waiting flag
    m_isWaitingForMoveCompletion = true;

    // Queue the command
    move(dx, dy, dz, velocity_x, velocity_y, velocity_z);

    // Wait for completion with timeout to avoid indefinite hangs
    log("move_and_wait: Waiting for completion...", "INFO");
    bool completed = m_syncCondition.wait_for(lock, std::chrono::seconds(20), [this] {
        return !m_isWaitingForMoveCompletion.load();
    });

    if (!completed) {
        log("move_and_wait: Timeout waiting for move completion", "WARNING");
        // Clear the waiting flag to avoid deadlocks
        m_isWaitingForMoveCompletion = false;
    } else {
        log("move_and_wait: Move complete", "INFO");
    }
}

// Home command
void XYZStage::home() {
    log("Home command called", "INFO");
    _home();
}

// Core movement function
XYZStage::Position XYZStage::_move(double x, double y, double z, double vx, double vy, double vz, char direction) {

    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        log("SIMULATION MODE - No actual movement", "WARNING");

        // Update simulated position
        int sign = (direction == 'D') ? -1 : 1;
        // atomic<double> may not support fetch_add on all STL implementations (MSVC), use CAS loop
        {
            double oldVal = globle_vars.current_x.load();
            double newVal;
            do {
                newVal = oldVal + (x * sign);
            } while (!globle_vars.current_x.compare_exchange_weak(oldVal, newVal));
        }
        {
            double oldVal = globle_vars.current_y.load();
            double newVal;
            do {
                newVal = oldVal + (y * sign);
            } while (!globle_vars.current_y.compare_exchange_weak(oldVal, newVal));
        }
        {
            double oldVal = globle_vars.current_z.load();
            double newVal;
            do {
                newVal = oldVal + (z * sign);
            } while (!globle_vars.current_z.compare_exchange_weak(oldVal, newVal));
        }

        log(QString("Simulated position: X=%1, Y=%2, Z=%3")
            .arg(globle_vars.current_x.load())
            .arg(globle_vars.current_y.load())
            .arg(globle_vars.current_z.load()), "INFO");

        // Simulate movement time
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        return position;
    }

    int sign = (direction == 'D') ? -1 : 1;

    log(QString("Moving FROM: X=%1, Y=%2, Z=%3")
        .arg(globle_vars.current_x.load())
        .arg(globle_vars.current_y.load())
        .arg(globle_vars.current_z.load()), "INFO");

    // Read current position atomically into locals
    double currentX = globle_vars.current_x.load();
    double currentY = globle_vars.current_y.load();
    double currentZ = globle_vars.current_z.load();

    // Calculate target position
    int targetX = static_cast<int>(currentX + (x * sign));
    int targetY = static_cast<int>(currentY + (y * sign));
    int targetZ = static_cast<int>(currentZ + (z * sign));

    // Check bounds
    if (targetX < globle_vars.min_x || targetY < globle_vars.min_y || targetZ < globle_vars.min_z) {
        log("MOVE FAILED - Position below minimum bounds", "CRITICAL");
        return position;
    }

    if (targetX > globle_vars.max_x || targetY > globle_vars.max_y || targetZ > globle_vars.max_z) {
        log("MOVE FAILED - Position exceeds maximum bounds", "CRITICAL");
        return position;
    }

    // Convert to controller units
    int x_units = static_cast<int>(x * scale.x);
    int y_units = static_cast<int>(y * scale.y);
    int z_units = static_cast<int>(z * scale.z);
    int vx_units = static_cast<int>(vx * scale.x);
    int vy_units = static_cast<int>(vy * scale.y);
    int vz_units = static_cast<int>(vz * scale.z);

    // Build command string
    std::string cmd;

    if (x_units == 0 && y_units == 0 && z_units == 0) {
        log("No movement requested (all zeros)", "INFO");
        return position;
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

    // Send command
    DWORD bytesWritten;
    if (!WriteFile(m_serialHandle, cmd.c_str(), static_cast<DWORD>(cmd.length()), &bytesWritten, NULL)) {
        log("Failed to write to serial port!", "CRITICAL");
        return position;
    }

    log(QString("Command sent: %1").arg(QString::fromStdString(cmd).trimmed()), "INFO");

    // Calculate wait time
    double sleep_time = 3.0;

    if (vx_units > 1) {
        double time = std::abs(static_cast<double>(x_units) / vx_units);
        if (time > sleep_time) sleep_time = time;
    }
    if (vy_units > 1) {
        double time = std::abs(static_cast<double>(y_units) / vy_units);
        if (time > sleep_time) sleep_time = time;
    }
    if (vz_units > 1) {
        double time = std::abs(static_cast<double>(z_units) / vz_units);
        if (time > sleep_time) sleep_time = time;
    }

    sleep_time += 0.5; // Buffer time

    log(QString("Waiting %1 seconds for movement...").arg(sleep_time, 0, 'f', 2), "INFO");

    // Wait for movement
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

    // Query new position
    getPosition();

    return position;
}

// Home function
XYZStage::Position XYZStage::_home() {
    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        log("Cannot home - no serial connection", "CRITICAL");
        return position;
    }

    log("Starting homing sequence...", "INFO");

    // Send homing commands
    const char* commands[] = {
        "/1aM1f1aM2f1aM3f1R\r\n",
        "/1aM1N1Z10000R\r\n",
        "/1aM2N1Z10000R\r\n",
        "/1aM3N1Z10000R\r\n",
        "/1z0,0,0R\r\n"
    };

    for (int i = 0; i < 5; i++) {
        DWORD bytesWritten;
        if (!WriteFile(m_serialHandle, commands[i], strlen(commands[i]), &bytesWritten, NULL)) {
            log(QString("Failed to send home command %1").arg(i + 1), "CRITICAL");
            return position;
        }

        log(QString("Home command %1 sent").arg(i + 1), "INFO");
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    log("Homing complete", "INFO");

    // Reset position (store atomically)
    globle_vars.current_x.store(0.0);
    globle_vars.current_y.store(0.0);
    globle_vars.current_z.store(0.0);

    position.x = 0;
    position.y = 0;
    position.z = 0;

    return position;
}

// Get position
XYZStage::Position XYZStage::getPosition() {
    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        return position;
    }

    char buffer[256];
    sprintf_s(buffer, "/1?aA\r\n");

    DWORD bytesWritten;
    if (!WriteFile(m_serialHandle, buffer, strlen(buffer), &bytesWritten, NULL)) {
        log("Failed to send position query", "WARNING");
        return position;
    }

    // Read response
    std::string response = readResponse(m_serialHandle, 2000);

    if (!response.empty()) {
        parsePositionResponse(response);
    }

    return position;
}

// Parse position response
void XYZStage::parsePositionResponse(const std::string& response) {
    size_t backtickPos = response.find('`');

    if (backtickPos == std::string::npos) {
        return;
    }

    std::string positionPart = response.substr(backtickPos + 1);
    std::vector<int> positions;
    std::stringstream ss(positionPart);
    std::string token;

    int count = 0;
    while (std::getline(ss, token, ',') && count < 3) {
        try {
            std::string cleanToken;
            for (char c : token) {
                if (std::isdigit(c) || c == '-') {
                    cleanToken += c;
                }
            }

            if (!cleanToken.empty()) {
                positions.push_back(std::stoi(cleanToken));
                count++;
            }
        }
        catch (...) {
            break;
        }
    }

    if (positions.size() >= 3) {
        globle_vars.current_x.store(static_cast<double>(positions[0]) / scale.x);
        globle_vars.current_y.store(static_cast<double>(positions[1]) / scale.y);
        globle_vars.current_z.store(static_cast<double>(positions[2]) / scale.z);

        position.x = globle_vars.current_x.load();
        position.y = globle_vars.current_y.load();
        position.z = globle_vars.current_z.load();

        log(QString("Position: X=%1, Y=%2, Z=%3")
            .arg(globle_vars.current_x.load())
            .arg(globle_vars.current_y.load())
            .arg(globle_vars.current_z.load()), "DEBUG");
    }
}

// Get serial handle
HANDLE XYZStage::getSerial() {
    HANDLE hSerial = CreateFileA(
        port.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (hSerial == INVALID_HANDLE_VALUE) {
        return INVALID_HANDLE_VALUE;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 2000;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 2000;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        CloseHandle(hSerial);
        return INVALID_HANDLE_VALUE;
    }

    return hSerial;
}

// Read serial response
std::string XYZStage::readResponse(HANDLE hSerial, int maxWaitMs) {
    std::string response;
    char buffer[256] = { 0 };
    DWORD bytesRead = 0;

    Sleep(100);

    int attempts = maxWaitMs / 100;

    for (int i = 0; i < attempts; i++) {
        if (ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL)) {
            if (bytesRead > 0) {
                buffer[bytesRead] = '\0';
                response += std::string(buffer);

                if (response.find('\r') != std::string::npos ||
                    response.find('\n') != std::string::npos) {
                    break;
                }
            }
        }

        if (response.empty()) {
            Sleep(100);
        }
    }

    return response;
}