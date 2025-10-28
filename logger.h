#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QString>
#include <QMutex>
#include <QQueue>
#include <QThread>
#include <QDateTime>

// Singleton Logger that can be accessed from anywhere
class Logger : public QObject {
    Q_OBJECT

public:
    // Get the singleton instance
    static Logger& instance() {
        static Logger instance;
        return instance;
    }

    // Delete copy constructor and assignment operator
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // Thread-safe logging methods - can be called from ANY thread
    static void info(const QString& message) {
        instance().log(message, "INFO");
    }

    static void warning(const QString& message) {
        instance().log(message, "WARNING");
    }

    static void error(const QString& message) {
        instance().log(message, "ERROR");
    }

    static void critical(const QString& message) {
        instance().log(message, "CRITICAL");
    }

    static void debug(const QString& message) {
        instance().log(message, "DEBUG");
    }

signals:
    // Signal emitted when a new log message is available
    void logMessage(const QString& timestamp, const QString& level, const QString& message);

private:
    Logger() = default;
    ~Logger() = default;

    void log(const QString& message, const QString& level) {
        QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
        emit logMessage(timestamp, level, message);
    }
};

#endif // LOGGER_H