/*! @file simulator_message.hpp
 *  @brief Messages sent to/from the development simulator
 *
 *  These messsages contain all data that is exchanged between the robot program
 * and the simulator using shared memory.   This is basically everything except
 * for debugging logs, which are handled by LCM instead
 */

#ifndef PROJECT_SIMULATORTOROBOTMESSAGE_H
#define PROJECT_SIMULATORTOROBOTMESSAGE_H

#include "shared_memory.hpp"

struct SimulatorToRos {
    double data[120];
};

struct RosToSimulator {
    double data[120];
};

struct SharedData {
    SimulatorToRos sim2ros;
    RosToSimulator ros2sim;
};

template class SharedMemoryObject<SharedData>;
#endif  // PROJECT_SIMULATORTOROBOTMESSAGE_H
