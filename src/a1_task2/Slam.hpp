#ifndef SLAM_HPP
#define SLAM_HPP

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

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"

#include "ApproxLaser.hpp"
#include "OccupancyGridMapper.hpp"
#include "Particles.hpp"

#include "math/point.hpp"
#include "MagicNumbers.hpp"

class Slam
{
    private:
        Particles particles_;
        OccupancyGridMapper grid_mapper_;

        std::queue<maebot_motor_feedback_t> motor_feedbacks_;
        pthread_mutex_t motor_feedbacks_mutex_;

        lcm::LCM *lcm;

    public:
        void setLCM(lcm::LCM *lcm_t) : lcm(lcm_t) { }
        void addMotorFeedback(maebot_motor_feedback_t input_feedback);
        
        bool motorFeedbacksEmpty();
        void lockMotorFeedbacksMutex();
        void unlockMotorFeedbacksMutex();
};

#endif
