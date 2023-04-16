//
// Created by stand on 16.04.2023.
//

#include <cstring>
#include <bitset>
#include <sstream>
#include "servo_communicator.h"

#define RESPONSE_TIMEOUT_US 50000
#define SERVO_IP "192.168.1.129"
#define SERVO_PORT 5801

#define IDENTIFIER_1 0x47
#define IDENTIFIER_2 0x54
#define READ_FLAG 0x01
#define WRITE_FLAG 0x02


ServoCommunicator::ServoCommunicator(BS::thread_pool &threadPool) {
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) {
        LOG_ERROR("socket creation failed");
        return;
    }

    memset(&myAddr_, 0, sizeof(myAddr_));
    myAddr_.sin_family = AF_INET;
    myAddr_.sin_addr.s_addr = INADDR_ANY;
    myAddr_.sin_port = htons(5801);

    if (bind(socket_, (struct sockaddr *) &myAddr_, sizeof(myAddr_)) < 0) {
        LOG_ERROR("bind socket failed");
    }

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = RESPONSE_TIMEOUT_US;

    if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, (const char *) &timeout, sizeof(timeout)) < 0) {
        LOG_ERROR("setsockopt failed");
    }

    memset(&destAddr_, 0, sizeof(destAddr_));
    destAddr_.sin_family = AF_INET;
    destAddr_.sin_addr.s_addr = inet_addr(SERVO_IP);
    destAddr_.sin_port = htons(SERVO_PORT);

    setMode(threadPool);
}

void ServoCommunicator::enableServos(bool enable, BS::thread_pool &threadPool) {
    if (!checkReadiness()) {
        return;
    }

    threadPool.push_task([&, enable]() {
        unsigned char en;
        if (enable) en = 0x01;
        else en = 0x00;

        unsigned char enableAzimuthBuffer[6] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::AZIMUTH,
                                                MessageElement::ENABLE, en};

        unsigned char enableElevationBuffer[6] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::ELEVATION,
                                                  MessageElement::ENABLE, en};

        while (true) {
            sendMessage(enableAzimuthBuffer, 6);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(enableElevationBuffer, 6);

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


void ServoCommunicator::setSpeed(int32_t speed, BS::thread_pool &threadPool) {
    if (!checkReadiness()) {
        return;
    }

    threadPool.push_task([&, speed]() {
        auto speedBytes = intToByteArray(speed);

        unsigned char azimuthSpeedBuffer[9] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::AZIMUTH,
                                               MessageElement::SPEED, speedBytes[0], speedBytes[1], speedBytes[2],
                                               speedBytes[3]};

        unsigned char elevationSpeedBuffer[9] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::ELEVATION,
                                                 MessageElement::SPEED, speedBytes[0], speedBytes[1], speedBytes[2],
                                                 speedBytes[3]};

        while (true) {
            sendMessage(azimuthSpeedBuffer, 9);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(elevationSpeedBuffer, 9);

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

    threadPool.push_task([&, acceleration]() {
        auto accBytes = intToByteArray(acceleration);

        unsigned char azimuthAccBuffer[9] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::AZIMUTH,
                                             MessageElement::ACCELERATION, accBytes[0], accBytes[1], accBytes[2],
                                             accBytes[3]};

        unsigned char elevationAccBuffer[9] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::ELEVATION,
                                               MessageElement::ACCELERATION, accBytes[0], accBytes[1], accBytes[2],
                                               accBytes[3]};

        while (true) {
            sendMessage(azimuthAccBuffer, 9);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(elevationAccBuffer, 9);

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

    threadPool.push_task([&, deceleration]() {
        auto decBytes = intToByteArray(deceleration);

        unsigned char azimuthDecBuffer[9] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::AZIMUTH,
                                             MessageElement::ACCELERATION, decBytes[0], decBytes[1], decBytes[2],
                                             decBytes[3]};

        unsigned char elevationDecBuffer[9] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::ELEVATION,
                                               MessageElement::ACCELERATION, decBytes[0], decBytes[1], decBytes[2],
                                               decBytes[3]};

        while (true) {
            sendMessage(azimuthDecBuffer, 9);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(elevationDecBuffer, 9);

            if (waitForResponse()) {
                break;
            }
        }

    });
}

void ServoCommunicator::setPose(float azimuth, float elevation, BS::thread_pool &threadPool) {
    if (!checkReadiness()) {
        return;
    }

    //TODO:
    /*
    threadPool.push_task([&, azimuth, elevation]() {
        auto speedBytes = intToByteArray(azimuth);

        unsigned char azimuthSpeedBuffer[9] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::AZIMUTH,
                                               MessageElement::SPEED, speedBytes[0], speedBytes[1], speedBytes[2],
                                               speedBytes[3]};

        unsigned char elevationSpeedBuffer[9] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::ELEVATION,
                                                 MessageElement::SPEED, speedBytes[0], speedBytes[1], speedBytes[2],
                                                 speedBytes[3]};

        while (true) {
            sendMessage(azimuthSpeedBuffer, 9);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(elevationSpeedBuffer, 9);

            if (waitForResponse()) {
                break;
            }
        }

    });
     */
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
    threadPool.push_task([&]() {
        unsigned char azimuthModeBuffer[6] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::AZIMUTH,
                                              MessageElement::MODE, 0x01};
        unsigned char elevationModeBuffer[6] = {IDENTIFIER_1, IDENTIFIER_2, WRITE_FLAG, MessageGroup::AZIMUTH,
                                                MessageElement::MODE, 0x01};

        while (true) {
            sendMessage(azimuthModeBuffer, 6);

            if (waitForResponse()) {
                break;
            }
        }

        while (true) {
            sendMessage(elevationModeBuffer, 6);

            if (waitForResponse()) {
                break;
            }
        }

        isInitialized_ = true;
        isReady_ = true;

        LOG_ERROR("ServoCommunicator correctly initialized, mode set to position on both axes");
    });
}

int ServoCommunicator::getBitsPerRevol() {
    unsigned char azimuthBitsPerRevolBuffer[9] = {IDENTIFIER_1, IDENTIFIER_2, READ_FLAG, MessageGroup::AZIMUTH,
                                                  MessageElement::BITS_PER_REVOL};

    unsigned char elevationBitsPerRevolBuffer[9] = {IDENTIFIER_1, IDENTIFIER_2, READ_FLAG, MessageGroup::ELEVATION,
                                                    MessageElement::BITS_PER_REVOL};

    char azimuthBitsPerRevol[4];
    char elevationBitsPerRevol[4];

    while (true) {
        sendMessage(azimuthBitsPerRevolBuffer, 6);

        if (waitForResponse(azimuthBitsPerRevol)) {
            break;
        }
    }

    while (true) {
        sendMessage(elevationBitsPerRevolBuffer, 6);

        if (waitForResponse(elevationBitsPerRevol)) {
            break;
        }
    }

    printf("Azimuth bits: %c%c%c%c\n", azimuthBitsPerRevol[0], azimuthBitsPerRevol[1], azimuthBitsPerRevol[2],
           azimuthBitsPerRevol[3]);
    printf("Elevation bits: %c%c%c%c\n", elevationBitsPerRevol[0], elevationBitsPerRevol[1], elevationBitsPerRevol[2],
           elevationBitsPerRevol[3]);
}

void ServoCommunicator::sendMessage(const unsigned char *message, uint8_t msgLength) {
    if (sendto(socket_, message, msgLength, 0, reinterpret_cast<sockaddr *>(&destAddr_),
               sizeof(destAddr_)) < 0) {
        LOG_ERROR("failed to send message");
    }
    isReady_ = false;
}

bool ServoCommunicator::waitForResponse(char *retBytes) {
    std::array<char, 1024> buffer{};
    struct sockaddr_in src_addr{};
    socklen_t addrlen = sizeof(src_addr);

    size_t nbytes = recvfrom(socket_, buffer.data(), sizeof(buffer), 0, (struct sockaddr *) &src_addr, &addrlen);
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
    } else if (response.length() < 6) {
        LOG_ERROR("Malformed packet received!");
    } else {
        printf("Received response: %s\n", response.c_str());
        bool isOk = response.c_str()[5] == '0'; // Fifth byte should be 0
        if (response.length() > 6 && retBytes != nullptr) {
            LOG_ERROR("Copying received bytes");
            retBytes[0] = buffer[6];
            retBytes[1] = buffer[7];
            retBytes[2] = buffer[8];
            retBytes[3] = buffer[9];
        }
        return isOk;
    }

    return false;
}

std::array<unsigned char, 4> ServoCommunicator::intToByteArray(int32_t paramInt) {
    std::array<unsigned char, 4> bytes{};

    bytes[0] = (paramInt >> 24) & 0xFF;
    bytes[1] = (paramInt >> 16) & 0xFF;
    bytes[2] = (paramInt >> 8) & 0xFF;
    bytes[3] = paramInt & 0xFF;

    return bytes;
}