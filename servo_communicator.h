//
// Created by stand on 16.04.2023.
//
#pragma once

#include <iostream>
#include "BS_thread_pool.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>

#define LOG_ERROR(...) std::cout << __VA_ARGS__ << std::endl

class ServoCommunicator {
    enum Operation {
        READ = 0x01,
        WRITE = 0x02,
    };

    enum MessageGroup {
        ENABLE_ELEVATION = 0x11,
        ENABLE_AZIMUTH = 0x12,
        ELEVATION = 0x19,
        AZIMUTH = 0x1A,
    };

    enum MessageElement {
        ENABLE = 0x00,
        ACCELERATION = 0x00,
        DECELERATION = 0x01,
        ANGLE = 0x04,
        REVOL = 0x05,
        SPEED = 0x07,
        MODE = 0x09,
        BITS_PER_REVOL = 0x0C,
    };

    struct AzimuthElevation {
        double azimuth;
        double elevation;
    };

public:

    explicit ServoCommunicator(BS::thread_pool &threadPool);

    [[nodiscard]] bool isInitialized() const { return isInitialized_; };

    [[nodiscard]] bool isReady() const { return isReady_; };

    [[nodiscard]] bool servosEnabled() const { return servosEnabled_; }

    void enableServos(bool enable, BS::thread_pool &threadPool);

    void resetErrors(BS::thread_pool &threadPool);

    void setSpeed(int32_t speed, BS::thread_pool &threadPool);

    void setAcceleration(int32_t acceleration, BS::thread_pool &threadPool);

    void setDeceleration(int32_t deceleration, BS::thread_pool &threadPool);

    void setPose(int32_t azimuth, int32_t elevation, BS::thread_pool &threadPool);

private:

    [[nodiscard]] bool checkReadiness() const;

    void setMode(BS::thread_pool &threadPool);

    void sendMessage(const std::vector<unsigned char> &message);

    bool waitForResponse();

    template<typename IntType>
    [[nodiscard]] inline static std::vector<uint8_t> serializeLEInt(const IntType &value) {
        std::vector<uint8_t> data{};
        for (size_t nBytes = 0; nBytes <= (sizeof(IntType) - 1); ++nBytes) {
            data.push_back((value >> nBytes * 8) & 0xFF);
        }

        return data;
    }

    static AzimuthElevation quaternionToAzimuthElevation(double x, double y, double z, double w);

    bool isInitialized_ = false;
    bool isReady_ = false;
    bool servosEnabled_ = false;

    int socket_ = -1;
    sockaddr_in myAddr_{}, destAddr_{};
};