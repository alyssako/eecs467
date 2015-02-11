#include "Slam.hpp"

Slam::Slam(OccupancyGridMapper *gm) :
    grid_mapper_(gm),
    scan_received_(false),
    grid_initialized_(false)
{
    pthread_mutex_init(&poses_mutex_, NULL);
    pthread_mutex_init(&slam_mutex_, NULL);
    pthread_cond_init(&cv_, NULL);
    pthread_mutex_init(&grid_mutex_, NULL);
    pthread_cond_init(&grid_cv_, NULL);

    left_prev_dist = -1;
    right_prev_dist = -1;
    origin.x = 0;
    origin.y = 0;
    origin.theta = 0;
    poses_.push_front(origin);
}
void Slam::addMotorFeedback(maebot_motor_feedback_t input_feedback)
{
    if(left_prev_dist == -1 || right_prev_dist == -1)
    {
        left_prev_dist = input_feedback.encoder_left_ticks * TICKS_TO_DIST;
        right_prev_dist = input_feedback.encoder_right_ticks * TICKS_TO_DIST;
        return;
    }
    pthread_mutex_lock(&delta_poses_mutex_);
    addDelta(input_feedback.encoder_left_ticks, input_feedback.encoder_right_ticks);
    if(poses_.size() > POSES_SIZE)
    {
        poses_.pop_back();
    }
    pthread_mutex_unlock(&delta_poses_mutex_);
}

bool Slam::posesEmpty()
{
    return delta_poses_.empty();
}

void Slam::lockPosesMutex()
{
    pthread_mutex_lock(&delta_poses_mutex_);
}

void Slam::unlockPosesMutex()
{
    pthread_mutex_unlock(&delta_poses_mutex_);
}

maebot_pose_t Slam::addDelta(float left_ticks, float right_ticks)
{

    //update the distance traveled since last timestep
    double cur_dist = right_ticks * TICKS_TO_DIST;
    double right_step = cur_dist - right_prev_dist;
    right_prev_dist = cur_dist;

    cur_dist = left_ticks * TICKS_TO_DIST;
    double left_step = cur_dist - left_prev_dist;
    left_prev_dist = cur_dist;

    //odometry
    double s_ = (right_step + left_step)/2;
    double delta_theta = (right_step - left_step)/0.08;
    double alpha = delta_theta/2;
    maebot_pose_t delta;
    delta.x = cos(origin.theta + alpha) * s_ + origin.x;
    delta.y = sin(origin.theta + alpha) * s_ + origin.y;
    delta.theta = theta = delta_theta + origin.theta;

    maebot_pose_t m = poses_.front();
    m.x += delta.x;
    m.y += delta.y;
    m.theta = eecs467::wrap_to_2pi(m.theta + delta.theta);

    poses_.push_front(m);
}

void Slam::publish()
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        maebot_pose_t pose = particles_.toPose(i);
        lcm->publish("MAEBOT_PARTICLE_GUI", &pose);
    }
    std::cout << "sent\n";
}
