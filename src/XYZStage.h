#pragma once
#include <windows.h>
#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>

#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

// Global variables structure (similar to your globle_vars)
struct GlobalVars {
    double current_x = 0.0;
    double current_y = 0.0;
    double current_z = 0.0;
};

// Global instance declaration
extern GlobalVars globle_vars;

class XYZStage {
private:
    void parsePositionResponse(const std::string& response);

    struct Position {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct MoveCommand {
        double dx;
        double dy;
        double dz;
        double vx;
        double vy;
        double vz;
    };

    struct Scale {
        double x = 88/ 1000.0;
        double y = 88/ 1000.0;
        double z = 1260.0 / 1000.0;
    };

	HANDLE m_serialHandle;
    Position position;
    std::string port;
    Scale scale;
    std::thread m_workerThread;
    std::queue<MoveCommand> m_commandQueue;
    std::mutex m_queueMutex;
    std::condition_variable m_condition;
    std::atomic<bool> m_stopWorker;

    std::mutex m_syncMutex;
    std::condition_variable m_syncCondition;
    std::atomic<bool> m_isWaitingForMoveCompletion{ false };

    void worker();

    // Private helper method to get serial handle
    HANDLE getSerial();

    std::string readResponse(HANDLE hSerial, int maxWaitMs = 1000);

    // Private helper method for actual movement
    Position _move(double x, double y, double z, double vx, double vy, double vz, char direction);

public:
    XYZStage(const std::string& portName = "COM5");
    ~XYZStage();

    // Public move method
    void move(double dx, double dy, double dz, double velocity_x = 10000, double velocity_y = 10000, double velocity_z = 10000);

    // Worker Blocking move method that waits for movement to complete
    void move_and_wait(double dx, double dy, double dz, double velocity_x = 10000, double velocity_y = 10000, double velocity_z = 10000);

    // Getter for current position
    XYZStage::Position getPosition() const { return position; }

	// currently unused
    // Getter for port
    std::string getPort() const { return port; }

    // Setter for port
    void setPort(const std::string& newPort) { port = newPort; }

    HANDLE getSerialHandle() { return m_serialHandle; }
};
