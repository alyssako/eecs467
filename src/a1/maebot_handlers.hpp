#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"

class MaebotPoseHandler
{
    public:
        ~MaebotPoseHandler(){}

        void handleMessage(const lcm::ReceiveBuffer *rbuf,
                           const std::string& channel,
                           const maebot_pose_t *msg)
        {
        }
};

class MaebotLaserScanHandler
{
    public:
        ~MaebotLaserScanHandler(){}

        void handleMessage(const lcm::ReceiveBuffer *rbuf,
                           const std::string& channel,
                           const maebot_laser_scan_t *msg)
        {
        }
};
