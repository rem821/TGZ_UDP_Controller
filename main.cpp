#include "servo_communicator.h"
#include <chrono>
#include <thread>

int main() {

    BS::thread_pool threadPool;
    auto com = ServoCommunicator(threadPool);

    while (!com.isInitialized());

    com.getBitsPerRevol();
    com.enableServos(true, threadPool);


    //com.setSpeed(1000000, threadPool);
    //com.setAcceleration(100, threadPool);
    //com.setDeceleration(100, threadPool);

    //com.setPose(0.0f, 0.0f, threadPool);


    return 0;
}
