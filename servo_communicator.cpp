//
// Created by stand on 16.04.2023.
//
#include <cstring>
#include <bitset>
#include <sstream>
#include "servo_communicator.h"

constexpr int RESPONSE_TIMEOUT_US = 50000;
constexpr std::string_view SERVO_IP = "192.168.1.200";
constexpr int SERVO_PORT = 502;

constexpr int RESPONSE_MIN_BYTES = 6;

constexpr unsigned char IDENTIFIER_1 = 0x47;
constexpr unsigned char IDENTIFIER_2 = 0x54;


ServoCommunicator::ServoCommunicator(BS::thread_pool &threadPool) : socket_(socket(AF_INET, SOCK_DGRAM, 0)) {

    if (socket_ < 0) {
        LOG_ERROR("socket creation failed");
        return;
    }

    memset(&myAddr_, 0, sizeof(myAddr_));
    myAddr_.sin_family = AF_INET;
    myAddr_.sin_addr.s_addr = INADDR_ANY;
    myAddr_.sin_port = htons(8552);

    if (bind(socket_, (sockaddr *) &myAddr_, sizeof(myAddr_)) < 0) {
        LOG_ERROR("bind socket failed");
    }

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = RESPONSE_TIMEOUT_US;

    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        LOG_ERROR("setsockopt failed");
    }

    memset(&destAddr_, 0, sizeof(destAddr_));
    destAddr_.sin_family = AF_INET;
    destAddr_.sin_addr.s_addr = inet_addr(SERVO_IP.data());
    destAddr_.sin_port = htons(SERVO_PORT);

    setMode(threadPool);
}

void ServoCommunicator::enableServos(bool enable, BS::thread_pool &threadPool) {
    if (!checkReadiness()) {
        return;
    }

    threadPool.push_task([this, enable]() {
        unsigned char const en = enable ? 0x01 : 0x00;

        std::vector<unsigned char> const enableAzimuthBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::ENABLE_AZIMUTH,
                                                                MessageElement::ENABLE, en};

        std::vector<unsigned char> const enableElevationBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::ENABLE_ELEVATION,
                                                                  MessageElement::ENABLE, en};

        while (true) {
            sendMessage(enableAzimuthBuffer);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(enableElevationBuffer);

            if (waitForResponse()) {
                break;
            }
        }

        servosEnabled_ = enable;
        if (enable) {
            LOG_ERROR("Servos enabled!");
        } else {
            LOG_ERROR("Servos disabled!");
        }
    });
}

void ServoCommunicator::resetErrors(BS::thread_pool &threadPool) {
    if (!checkReadiness()) {
        return;
    }

    threadPool.push_task([this]() {
        std::vector<unsigned char> const resetErrorsAzimuthBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::ENABLE_AZIMUTH,
                                                                     MessageElement::ENABLE, 0x08};

        std::vector<unsigned char> const resetErrorsElevationBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::ENABLE_ELEVATION,
                                                                       MessageElement::ENABLE, 0x08};

        while (true) {
            sendMessage(resetErrorsAzimuthBuffer);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(resetErrorsElevationBuffer);

            if (waitForResponse()) {
                break;
            }
        }
    });
}


void ServoCommunicator::setSpeed(int32_t speed, BS::thread_pool &threadPool) {
    if (!checkReadiness()) {
        return;
    }

    threadPool.push_task([this, speed]() {
        auto speedBytes = serializeLEInt(speed);

        std::vector<unsigned char> const azimuthSpeedBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::AZIMUTH,
                                                               MessageElement::SPEED, speedBytes[0], speedBytes[1], speedBytes[2],
                                                               speedBytes[3]};

        std::vector<unsigned char> const elevationSpeedBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::ELEVATION,
                                                                 MessageElement::SPEED, speedBytes[0], speedBytes[1], speedBytes[2],
                                                                 speedBytes[3]};

        while (true) {
            sendMessage(azimuthSpeedBuffer);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(elevationSpeedBuffer);

            if (waitForResponse()) {
                break;
            }
        }

    });
}

void ServoCommunicator::setAcceleration(int32_t acceleration, BS::thread_pool &threadPool) {
    if (!checkReadiness()) {
        return;
    }

    threadPool.push_task([this, acceleration]() {
        auto accBytes = serializeLEInt(acceleration);

        std::vector<unsigned char> const azimuthAccBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::AZIMUTH,
                                                             MessageElement::ACCELERATION, accBytes[0], accBytes[1], accBytes[2],
                                                             accBytes[3]};

        std::vector<unsigned char> const elevationAccBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::ELEVATION,
                                                               MessageElement::ACCELERATION, accBytes[0], accBytes[1], accBytes[2],
                                                               accBytes[3]};

        while (true) {
            sendMessage(azimuthAccBuffer);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(elevationAccBuffer);

            if (waitForResponse()) {
                break;
            }
        }

    });
}

void ServoCommunicator::setDeceleration(int32_t deceleration, BS::thread_pool &threadPool) {
    if (!checkReadiness()) {
        return;
    }

    threadPool.push_task([this, deceleration]() {
        auto decBytes = serializeLEInt(deceleration);

        std::vector<unsigned char> const azimuthDecBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::AZIMUTH,
                                                             MessageElement::ACCELERATION, decBytes[0], decBytes[1], decBytes[2],
                                                             decBytes[3]};

        std::vector<unsigned char> const elevationDecBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::ELEVATION,
                                                               MessageElement::ACCELERATION, decBytes[0], decBytes[1], decBytes[2],
                                                               decBytes[3]};

        while (true) {
            sendMessage(azimuthDecBuffer);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(elevationDecBuffer);

            if (waitForResponse()) {
                break;
            }
        }

    });
}

void ServoCommunicator::setPose(int32_t azimuth, int32_t elevation, BS::thread_pool &threadPool) {
    if (!checkReadiness()) {
        return;
    }

    threadPool.push_task([this, azimuth, elevation]() {
        auto azimuthBytes = serializeLEInt(azimuth);
        auto elevationBytes = serializeLEInt(elevation);

        std::vector<unsigned char> const azimuthAngleBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::AZIMUTH,
                                                               MessageElement::ANGLE, azimuthBytes[0], azimuthBytes[1], azimuthBytes[2],
                                                               azimuthBytes[3]};

        std::vector<unsigned char> const elevationAngleBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::ELEVATION,
                                                                 MessageElement::ANGLE, elevationBytes[0], elevationBytes[1], elevationBytes[2],
                                                                 elevationBytes[3]};

        while (true) {
            sendMessage(azimuthAngleBuffer);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(elevationAngleBuffer);

            if (waitForResponse()) {
                break;
            }
        }

    });
}

bool ServoCommunicator::checkReadiness() const {
    if (!isInitialized()) {
        LOG_ERROR("ServoCommunicator is not yet initialized!");
        return false;
    }

    if (!isReady()) {
        //LOG_ERROR("ServoCommunicator is not currently ready to communicate!");
        return false;
    }

    return true;
}

void ServoCommunicator::setMode(BS::thread_pool &threadPool) {
    threadPool.push_task([this]() {
        std::vector<unsigned char> const azimuthModeBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::AZIMUTH,
                                                              MessageElement::MODE, 0x01};
        std::vector<unsigned char> const elevationModeBuffer = {IDENTIFIER_1, IDENTIFIER_2, Operation::WRITE, MessageGroup::ELEVATION,
                                                                MessageElement::MODE, 0x01};

        while (true) {
            sendMessage(azimuthModeBuffer);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(elevationModeBuffer);

            if (waitForResponse()) {
                break;
            }
        }

        isInitialized_ = true;
        isReady_ = true;

        LOG_ERROR("ServoCommunicator correctly initialized, mode set to position on both axes");
    });
}

void ServoCommunicator::sendMessage(const std::vector<unsigned char> &message) {
    if (sendto(socket_, message.data(), message.size(), 0, (sockaddr *) &destAddr_, sizeof(destAddr_)) < 0) {
        LOG_ERROR("failed to send message");
    }
    isReady_ = false;
}

bool ServoCommunicator::waitForResponse() {
    std::array<char, 1024> buffer{};
    struct sockaddr_in src_addr{};
    socklen_t addrlen = sizeof(src_addr);

    size_t nbytes = recvfrom(socket_, buffer.data(), sizeof(buffer), 0, (sockaddr *) &src_addr, &addrlen);
    if (nbytes <= 0) {
        if (errno == EWOULDBLOCK) {
            LOG_ERROR("Timeout reached. No data received.");
        } else {
            LOG_ERROR("recvfrom failed");
        }
        return false;
    }

    std::string response;
    for (int i = 0; i < nbytes; i++) {
        response += buffer[i];
    }

    isReady_ = true;
    if (response.length() == 0) {
        LOG_ERROR("No response received! Check if the servo is connected properly");
    } else if (response.length() < RESPONSE_MIN_BYTES) {
        LOG_ERROR("Malformed packet received!");
    } else {
        std::cout << "Received response: " << response.c_str() << std::endl;
        return response.c_str()[5] == '\0'; // Fifth byte should be 0
    }

    return false;
}

ServoCommunicator::AzimuthElevation ServoCommunicator::quaternionToAzimuthElevation(double x, double y, double z, double w) {
    double const sinr_cosp = 2 * (w * x + y * z);
    double const cosr_cosp = 1 - 2 * (x * x + y * y);
    double const elevation = std::atan2(sinr_cosp, cosr_cosp);

    double const sinp = 2 * (w * y - z * x);
    double azimuth = std::asin(sinp);
    if (std::abs(sinp) >= 1) {
        azimuth = std::copysign(M_PI / 2, sinp);
    }

    double const siny_cosp = 2 * (w * z + x * y);
    double const cosy_cosp = 1 - 2 * (y * y + z * z);
    double const roll = std::atan2(siny_cosp, cosy_cosp);

    return AzimuthElevation{azimuth, elevation};

}
