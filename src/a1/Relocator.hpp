#ifndef RELOCATOR_HPP
#define RELOCATOR_HPP

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

#include "math/point.hpp"
//#include "MagicNumbers.hpp"

typedef struct position_t partical_t;
struct position_t {
    double x;
    double y;
    double theta;

    position_t() : x(0.), y(0.), theta(0.) {}
};

class Relocator
{
    private:
        maebot_motor_feedback_t previous_feedback;

    public:
        partical_t particals[PARTICAL_NUM];

        Relocator(){}
        ~Relocator(){}
        
        void move(maebot_motor_feedback_t feedback);// use both feedback to get deltas and move all 1000 particals 
        void resample(double* probabilities);
};

#endif
