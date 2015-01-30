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

#include "ApproxLaser.hpp"
#include "occupancy_grid.hpp"
#include "occupancy_grid_utils.hpp"

class OccupancyGridMapper
{
    private:
        std::queue<maebot_laser_scan_t> laser_scans;
        pthread_mutex_t laser_scans_mutex;
        std::queue<maebot_pose_t> poses;
        pthread_mutex_t poses mutex;
        
        LaserScanApprox approx;
        MovingLaser moving_laser;
        OccupancyGrid occupancy_grid;
    public:
        void calculateLaserOrigins();
        void updateGrid();
        
        void addLaserScan(maebot_laser_scan_t input_scan);
        void addPose(maebot_pose_t input_pose);
};
