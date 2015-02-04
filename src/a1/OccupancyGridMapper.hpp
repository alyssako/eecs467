#ifndef OCCUPANCY_GRID_MAPPER_HPP
#define OCCUPANCY_GRID_MAPPER_HPP

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <vector>
#include <queue>
#include <iostream>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"
#include "lcmtypes/maebot_occupancy_grid_t.hpp"

#include "ApproxLaser.hpp"
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"

class OccupancyGridMapper
{
    private:
        std::queue<maebot_laser_scan_t> laser_scans_;
        pthread_mutex_t laser_scans_mutex_;
        
        std::queue<maebot_pose_t> poses_;
        pthread_mutex_t poses_mutex_;
        
        ApproxLaser approx_laser_;
        MovingLaser moving_laser_;
        eecs467::OccupancyGrid occupancy_grid_;
        lcm::LCM *lcm;
    public:
        OccupancyGridMapper(maebot_occupancy_grid_t lcm_occupancy_grid);
        ~OccupancyGridMapper();

        void setLCM(lcm::LCM *lcm_t);
        void calculateLaserOrigins();
        void updateGrid();
        
        void addLaserScan(maebot_laser_scan_t input_scan);
        void addPose(maebot_pose_t input_pose);
        ApproxLaser& getApproxLaser();
        MovingLaser& getMovingLaser();
        eecs467::OccupancyGrid& getOccupancyGrid();
};

#endif
