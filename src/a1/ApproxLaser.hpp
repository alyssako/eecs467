#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <vector>
#include <deque>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"

#include "MovingLaser.hpp"

/* class to figure out approximately where laser scan originated from */
class ApproxLaser
{
    private:
        /* keep track of past five poses */
        deque<maebot_pose_t> poses;
    public:
        /* find the two points the scan originated between */
        LaserScanApprox findPts(maebot_laser_scan_t& scan);

        /* add pose to deque (and delete oldest pose */
        bool addPose(maebot_pose_t newPose);

        /* check and see if the poses came through in the correct order */
        bool checkOrder(maebot_pose_t newPose);
};
