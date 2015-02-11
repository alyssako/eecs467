#ifndef PARTICLES_HPP
#define PARTICLES_HPP

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

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

#include "math/point.hpp"
#include "math/gsl_util_rand.h"

struct Particle
{
    float x;
    float y;
    float theta;
    float probability;
}; 

#define NUM_PARTICLES 1000

class Particles
{
    private:
        std::vector<Particle> particles_;
        Particle most_likely_;
    public:
        Particles();
        ~Particles();
        
        void moveRandom(float mean_x, float mean_y, float mean_theta, float stddev_x, float stddev_y, float stddev_theta);
        void moveRandomSingle(double delta_s, double alpha, double theta_alpha, int index);

        void calculateProbability(occupancy_grid_t *grid, maebot_laser_scan_t *scan);
        void calculateProbabilitySingle(occupancy_grid_t *grid, maebot_laser_scan_t *scan, int index);
        void rotateParticle(double theta, int index);
        void moveParticle(double s, int index);

        void normalizeProbabilities();
        void exponentiate();
        void subtractProbabilities(float max);
        float findLargestProbability();
        float sumProbabilities();
        void divideProbabilities();
        
        void resample();
        
        MovingLaser::LaserScan Particles::getLaserScan(maebot_pose_t *poseA, maebot_scan_t *scanB, vector<maebot_pose_t> poses, MovingLaser *moving_laser)
        vector<maebot_pose_t> findLeftRightPoses(int64_t time, vector<maebot_pose_t> poses)
};


#endif
