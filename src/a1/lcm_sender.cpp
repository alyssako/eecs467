#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <common/timestamp.h>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"


int main(){
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    maebot_pose_t my_data;
    my_data.utime = utime_now();

    lcm.publish("MAEBOT_POSE", &my_data);

    std::cout << "sent" << std::endl;

    return 0;
}
