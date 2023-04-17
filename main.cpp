#include "servo_communicator.h"
#include "thread"
#include "chrono"

int main() {

    BS::thread_pool threadPool;
    auto com = ServoCommunicator(threadPool);

    while (!com.isInitialized());

    com.enableServos(true, threadPool);

    int heading = 0;
    while (true) {
        heading += 1000000;
        if (heading > 4000000000) {
            heading = 0;
        }
        com.setPose(heading, 0, threadPool);
        while(!com.isReady());
    }
    //com.setAcceleration(100, threadPool);
    //com.setDeceleration(100, threadPool);
    //com.setSpeed(1000000, threadPool);

    //com.enableServos(true, threadPool);

    //com.setPose(0.0f, 0.0f, threadPool);


    return 0;
}
