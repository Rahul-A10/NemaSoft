#pragma once
#include <windows.h>
#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio> // For sprintf_s

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

    struct Scale {
        double x = 88/ 1000.0;
        double y = 88/ 1000.0;
        double z = 1260.0 / 1000.0;
    };

    Position position;
    std::string port;
    Scale scale;

    // Private helper method to get serial handle
    HANDLE getSerial();

    std::string readResponse(HANDLE hSerial, int maxWaitMs = 1000);

    // Private helper method for actual movement
    Position _move(double x, double y, double z, double vx, double vy, double vz, char direction);

public:
    // Constructor
    XYZStage(const std::string& portName = "COM5");

    // Public move method
    void move(double dx, double dy, double dz, double velocity_x = 10000, double velocity_y = 10000, double velocity_z = 10000);

    // Getter for current position
    Position getPosition() const;


    // where are below methods used?
    // Getter for port
    std::string getPort() const;

    // Setter for port
    void setPort(const std::string& newPort);
};

// Global instance declaration (similar to your xyz_object)
extern XYZStage xyz_object;