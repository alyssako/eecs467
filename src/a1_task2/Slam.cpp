#include "Slam.hpp"

Slam::Slam(OccupancyGridMapper *gm) :
    grid_mapper_(gm),
    scan_received_(false)
{
    pthread_mutex_init(&poses_mutex_, NULL);
    pthread_mutex_init(&slam_mutex_, NULL);
    pthread_cond_init(&cv_, NULL);
    pthread_mutex_init(&scans_mutex_, NULL);

    left_prev_dist = -1;
    right_prev_dist = -1;
    origin.x = prev_pose.x = 0;
    origin.y = prev_pose.y = 0;
    origin.theta = prev_pose.theta = 0;
    poses_.push_front(origin);
}

Slam::~Slam()
{
}

void Slam::setLCM(lcm::LCM *lcm_t)
{
    lcm = lcm_t;
}

bool Slam::posesEmpty()
{
    return poses_.empty();
}

void Slam::addMotorFeedback(maebot_motor_feedback_t input_feedback)
{
    if(left_prev_dist == -1 || right_prev_dist == -1)
    {
        left_prev_dist = input_feedback.encoder_left_ticks * TICKS_TO_DIST;
        right_prev_dist = input_feedback.encoder_right_ticks * TICKS_TO_DIST;
        return;
    }
    pthread_mutex_lock(&poses_mutex_);
    addPose(input_feedback.encoder_left_ticks, input_feedback.encoder_right_ticks);
    if(poses_.size() > POSES_SIZE)
    {
        poses_.pop_back();
    }
    pthread_mutex_unlock(&poses_mutex_);
}

void Slam::addScan(maebot_laser_scan_t laser_scan)
{
    pthread_mutex_lock(&scans_mutex_);
    scans_.push(laser_scan);
    pthread_mutex_unlock(&scans_mutex_);
}

void Slam::addPose(float left_ticks, float right_ticks)
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
    delta.theta = delta_theta + origin.theta;

    maebot_pose_t m = poses_.front();
    m.x += delta.x;
    m.y += delta.y;
    m.theta = eecs467::wrap_to_2pi(m.theta + delta.theta);

    poses_.push_front(m);
}

void Slam::updateParticles()
{
    pthread_mutex_lock(&scans_mutex_);
    maebot_laser_scan_t nextScan = scans_.front();
    scans_.pop();
    pthread_mutex_unlock(&scans_mutex_);

    pthread_mutex_lock(&poses_mutex_);
    LaserScanRange lsr = particles_.getLaserScan(&prev_pose, &nextScan, poses_);
    pthread_mutex_unlock(&poses_mutex_);

    float delta_x = lsr.end_pose.x - prev_pose.x;
    float delta_y = lsr.end_pose.y - prev_pose.y;
    float delta_theta = eecs467::wrap_to_2pi(eecs467::angle_diff(lsr.end_pose.theta, prev_pose.theta));
    prev_pose.x = lsr.end_pose.x;
    prev_pose.y = lsr.end_pose.y;
    prev_pose.theta = lsr.end_pose.theta;
    particles_.updateParticles(delta_x, delta_y, delta_theta, &grid_mapper_->getOccupancyGrid(), &lsr);
}

void Slam::publish()
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        maebot_pose_t pose = particles_.toPose(i);
        lcm->publish("MAEBOT_PARTICLE_GUI", &pose);
    }
    maebot_pose_t mostProbable = particles_.mostProbable();
    lcm->publish("MAEBOT_POSE_GUI", &mostProbable);
    std::cout << "sent\n";
}

bool Slam::scanReceived()
{
    return scan_received_;
}

void Slam::lockSlamMutex()
{
    pthread_mutex_lock(&slam_mutex_);
}

void Slam::unlockSlamMutex()
{
    pthread_mutex_unlock(&slam_mutex_);
}

void Slam::signal()
{
    pthread_cond_signal(&cv_);
}

void Slam::wait()
{
    pthread_cond_wait(&cv_, &slam_mutex_);
}
