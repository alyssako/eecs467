#ifndef MAEBOT_HANDLERS_HPP
#define MAEBOT_HANDLERS_HPP

#include <stdio.h>
#include <iostream>
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

#include "OccupancyGridMapper.hpp"

class MaebotPoseHandler
{
    private:
        ApproxLaser *approx_laser;

    public:
        MaebotPoseHandler(ApproxLaser *approx_laser_t){
            approx_laser = approx_laser_t;
        }
        
        ~MaebotPoseHandler(){}

        void handleMessage(const lcm::ReceiveBuffer *rbuf,
                           const std::string& channel,
                           const maebot_pose_t *msg)
        {   
            //std::cout << "Got pose" << std::endl;
            if(approx_laser->addPose(*msg))
                std::cout << "Pose added" << std::endl;
        }
};

class MaebotLaserScanHandler
{
    private:
        eecs467::OccupancyGrid* occupancy_grid;

    public:
        MaebotLaserScanHandler(eecs467::OccupancyGrid* occupancy_grid_t){
            occupancy_grid = occupancy_grid_t;
        }

        ~MaebotLaserScanHandler(){}

        void handleMessage(const lcm::ReceiveBuffer *rbuf,
                           const std::string& channel,
                           const maebot_laser_scan_t *msg)
        {
        }
};

#endif
