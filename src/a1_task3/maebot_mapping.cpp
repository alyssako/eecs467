#include <stdio.h>
#include <fenv.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <math.h>
#include <string>
#include <time.h>

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/matd.h"
#include "math/math_util.h"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "maebot_handlers.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "OccupancyGridMapper.hpp"
#include "Slam.hpp"
#include "MagicNumbers.hpp"


#define MAX_REVERSE_SPEED 0.125f
#define MAX_FORWARD_SPEED 0.2f

#define dmax(A,B) A < B ? B : A
#define dmin(A,B) A < B ? A : B

typedef struct state state_t;
struct state
{
    int running;

    double joy_bounds;
    double last_click[3];

    maebot_motor_command_t cmd;
    pthread_mutex_t cmd_mutex;
    pthread_t cmd_thread;

    //pthread_t lcm_thread;
    pthread_t render_thread;
    pthread_mutex_t render_mutex;
    pthread_mutex_t layer_mutex;

    pthread_t update_map_thread;

    getopt_t *gopt;
    char *url;
    image_source_t *isrc;
    int fidx;

    lcm::LCM *lcm;
    pthread_mutex_t lcm_mutex;

    OccupancyGridMapper *grid_mapper;
    Slam *slam;
};

static int verbose = 0;

static state_t *global_state;

static void rotateTowards(double dir, state_t *state)
{
    dir = (dir > 0 ? 1 : (dir < 0 ? -1 : 0));

    //PUBLISH TO LCM!
    state->cmd.motor_left_speed = -dir*MAX_FORWARD_SPEED;
    state->cmd.motor_right_speed = dir*MAX_FORWARD_SPEED;
    state->lcm->publish("MAEBOT_MOTOR_COMMAND", &(state->cmd));
}

static void moveForward(state_t *state)
{
    //PUBLISH TO LCM!
    state->cmd.motor_left_speed = MAX_FORWARD_SPEED;
    state->cmd.motor_right_speed = MAX_FORWARD_SPEED;
    state->lcm->publish("MAEBOT_MOTOR_COMMAND", &(state->cmd));
}

static void moveTowardsPoint(maebot_pose_t a, int nextCell, state_t *state)
{
    if(state->slam->bfs_result.size() <= 0)
        return;

    double theta = eecs467::angle_diff(atan2(state->grid_mapper->toY(nextCell) - a.y, state->grid_mapper->toX(nextCell) - a.x), a.theta);
    if(abs(theta) > THETA_VARIANCE_MAX)
    {
        std::cout << "rotating towards: " << theta << std::endl;
        rotateTowards(theta, state);
    }
    else
    {
        std::cout << "moving forward" << std::endl;
        moveForward(state);
        auto d2 = (a.x - state->grid_mapper->toX(nextCell)) * (a.x - state->grid_mapper->toX(nextCell)) + (a.y - state->grid_mapper->toY(nextCell)) * (a.y - state->grid_mapper->toY(nextCell));
        if(d2 <= WAYPOINT_RADIUS * WAYPOINT_RADIUS)
        { 
            pthread_mutex_lock(&state->slam->path_mutex_);
            state->slam->bfs_result.pop_back();
            pthread_mutex_unlock(&state->slam->path_mutex_);
            std::cout << "REACHED WAYPOINT. bfs_result.size() = " << state->slam->bfs_result.size() << std::endl;
        }
    }
}

// This thread continuously publishes command messages to the maebot
static void* send_cmds(void *data)
{
    state_t *state = (state_t *) data;
    uint32_t Hz = 20;

    while (state->running) {
        pthread_mutex_lock(&state->cmd_mutex);
        pthread_mutex_lock(&state->slam->path_mutex_);
        if(!state->slam->bfs_result.empty())
        {
            int nextCell = state->slam->bfs_result.back();
            pthread_mutex_unlock(&state->slam->path_mutex_);

            moveTowardsPoint(state->slam->mostProbableParticle(), nextCell, state);
            pthread_mutex_unlock(&state->cmd_mutex);

            usleep(1000000/Hz);
        }
        else
        {
            pthread_mutex_unlock(&state->slam->path_mutex_);
            pthread_mutex_unlock(&state->cmd_mutex);
        }
    }

    return NULL;
}

static void* update_map(void *data)
{
    state_t *state = (state_t*) data;

    state->slam->lockSlamMutex();
    while(!state->slam->scanReceived())
    {
        state->slam->wait();
    }
    state->slam->unlockSlamMutex();
    state->slam->pushFirstScan();
    LaserScan updated_scan = state->grid_mapper->calculateLaserOrigins();
    if(!updated_scan.valid)
    {
        std::cout << "initial scan not working" << std::endl;
        exit(1);
    }

    state->grid_mapper->updateGrid(updated_scan);
    state->grid_mapper->publishOccupancyGrid(updated_scan.end_pose);

    while(state->running)
    {
        state->slam->lockSlamMutex();
        while(!state->slam->scanReceived())
        {
            state->slam->wait();
        }
        state->slam->unlockSlamMutex();
        maebot_laser_scan_t next_scan = state->slam->updateParticles();
        //std::cout << "received scan" << std::endl;
        
        state->grid_mapper->addLaserScan(next_scan);
        //std::cout << "added laser scan" << std::endl;
        
        LaserScan updated_scan = state->grid_mapper->calculateLaserOrigins();
        //std::cout << "updated scan" << std::endl;
        if(!updated_scan.valid) exit(1);

        pthread_mutex_lock(&state->slam->path_mutex_);
        std::vector<int> retval = state->grid_mapper->updateGrid(updated_scan);
        state->slam->bfs_result.clear();
        state->slam->bfs_result = retval;
        pthread_mutex_unlock(&state->slam->path_mutex_);
        //std::cout << "update grid" << std::endl;
        state->grid_mapper->publishOccupancyGrid(updated_scan.end_pose);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    // === State initialization ============================
    state_t *state = new state_t;
    global_state = state;
    state->gopt = getopt_create();
    state->last_click[0] = 0;
    state->last_click[1] = 0;
    state->last_click[2] = 0;
    state->joy_bounds = 10.0;

    state->running = 1;
    state->lcm = new lcm::LCM;
    pthread_mutex_init(&state->layer_mutex, NULL);
    pthread_mutex_init(&state->cmd_mutex, NULL);
    pthread_mutex_init(&state->lcm_mutex, NULL);
    pthread_mutex_init(&state->render_mutex, NULL);

    //feenableexcept(FE_DIVBYZERO| FE_INVALID|FE_OVERFLOW); 
    feenableexcept(FE_ALL_EXCEPT & ~FE_INEXACT & ~FE_UNDERFLOW);

    static OccupancyGridMapper ogm(state->lcm);
    state->grid_mapper = &ogm;

    static Slam s(&ogm, state->lcm);
    state->slam = &s;
    // === End =============================================

    // Clean up on Ctrl+C
    //signal(SIGINT, handler);

    getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(state->gopt, 'v', "verbose", 0, "Show extra debugging info");
    getopt_add_int(state->gopt, 'l', "limitKBs", "-1", "Remote display bandwith limit in KBs. < 0: unlimited.");
    getopt_add_int(state->gopt, 'p', "port", "15151", "Vx display port");

    if (!getopt_parse(state->gopt, argc, argv, 0) ||
            getopt_get_bool(state->gopt, "help"))
    {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    // Set up display
    verbose = getopt_get_bool(state->gopt, "verbose");

    // LCM subscriptions
    MaebotLCMHandler lcm_handler(state->grid_mapper, state->slam);

    state->lcm->subscribe("MAEBOT_LASER_SCAN",
            &MaebotLCMHandler::handleLaserScan,
            &lcm_handler);

    state->lcm->subscribe("MAEBOT_MOTOR_FEEDBACK",
            &MaebotLCMHandler::handleMotorFeedback,
            &lcm_handler);

    /*state->lcm->subscribe("MAEBOT_POSE_BEST",
            &MaebotLCMHandler::handlePose,
            &lcm_handler);*/

    std::cout << "listening" << std::endl;

    // Spin up threads
    pthread_create(&state->cmd_thread, NULL, send_cmds, (void*)state);
    pthread_create(&state->update_map_thread, NULL, update_map, state);

    // Loop forever
    while(state->lcm->handle() == 0);

    pthread_join (state->update_map_thread, NULL);
    pthread_join (state->cmd_thread, NULL);

    return 0;
}
