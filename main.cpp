#include "servo_communicator.h"
#include "thread"
#include "chrono"

int main() {

    BS::thread_pool threadPool;
    auto com = ServoCommunicator(threadPool);

    while (!com.isInitialized());

    com.enableServos(true, threadPool);
    while(!com.isReady());
    com.setSpeed(10000000, threadPool);

    int it = 0;
    int32_t heading = -2000000000;
    while (true) {
        heading += 100000000;
        if (heading > 2000000000) {
            heading = -2000000000;
            if(it == 9) break;
            it ++;
        }
        com.setPose(heading, 0, threadPool);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        while(!com.isReady());
    }

    //com.setAcceleration(100, threadPool);
    //com.setDeceleration(100, threadPool);
    //com.setSpeed(1000000, threadPool);

    //com.enableServos(true, threadPool);

    //com.setPose(0.0f, 0.0f, threadPool);


    return 0;
}
