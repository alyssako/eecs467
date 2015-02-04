#include "OccupancyGridMapper.hpp"

OccupancyGridMapper::OccupancyGridMapper() : 
    occupancy_grid_(10, 10, 0.05)
{
}

OccupancyGridMapper::~OccupancyGridMapper()
{
}

void OccupancyGridMapper::setLCM(lcm::LCM *lcm_t){
    lcm = lcm_t;
}

void OccupancyGridMapper::calculateLaserOrigins()
{
}

void OccupancyGridMapper::updateGrid()
{
}

//ApproxLaser& OccupancyGridMapper::getApproxLaser()
//{
//    return approx_laser_;
//}

//MovingLaser& OccupancyGridMapper::getMovingLaser() {
//    return moving_laser_;
//}

eecs467::OccupancyGrid& OccupancyGridMapper::getOccupancyGrid() {
    return occupancy_grid_;
}

void OccupancyGridMapper::addLaserScan(maebot_laser_scan_t input_scan)
{
//    pthread_mutex_lock(&laser_scans_mutex_);
//    laser_scans_.push(input_scan);
//    pthread_mutex_unlock(&laser_scans_mutex_);
}

void OccupancyGridMapper::addPose(maebot_pose_t input_pose)
{
//    pthread_mutex_lock(&poses_mutex_);
//    poses_.push(input_pose);
//    pthread_mutex_unlock(&poses_mutex_);
}
