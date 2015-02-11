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

// (0.032 * 3.14)/480.0
#define TICKS_TO_DIST 0.00020933

#define POSES_SIZE 20

class Slam
{
    private:
        Particles particles_;
        OccupancyGridMapper *grid_mapper_;

        std::deque<maebot_pose_t> poses_;
        pthread_mutex_t poses_mutex_;

        pthread_mutex_t slam_mutex_;
        pthread_cond_t cv_;
        bool scan_received_;

        pthread_mutex_t grid_mutex_;
        pthread_cond_t grid_cv_;
        bool grid_initialized_;

        lcm::LCM *lcm;

        float left_prev_dist;
        float right_prev_dist;
        maebot_pose_t origin;

    public:
        Slam() : grid_mapper_(gm) {
            slam_muex
        }
        ~Slam() {}

        void setLCM(lcm::LCM *lcm_t) : lcm(lcm_t) { }
        void addMotorFeedback(maebot_motor_feedback_t input_feedback);
        
        bool posesEmpty();
        void lockPosesMutex();
        void unlockPosesMutex();
        maebot_pose_t getDelta(float left_ticks, float right_ticks);
        void publish();
};

#endif
