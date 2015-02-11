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
#include "lcmtypes/maebot_occupancy_grid_t.hpp"

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"


class LCMHandler
{
    public:
        LCMHandler(){}
        ~LCMHandler(){}

        void handleMessage1(const lcm::ReceiveBuffer *rbuf,
                           const std::string& channel,
                           const maebot_laser_scan_t *msg)
        {
            std::cout << "Laser: " << msg->utime << std::endl;
        }

        void handleMessage2(const lcm::ReceiveBuffer *rbuf,
                           const std::string& channel,
                           const maebot_motor_feedback_t *msg)
        {
            std::cout << "Feedback: " << msg->utime << std::endl;
        }
};

int main(){
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    LCMHandler handler;

    lcm.subscribe("MAEBOT_LASER_SCAN",
                          &LCMHandler::handleMessage1,
                          &handler);
    lcm.subscribe("MAEBOT_MOTOR_FEEDBACK",
                          &LCMHandler::handleMessage2,
                          &handler);

    while(lcm.handle() == 0);
    return 0;
}
