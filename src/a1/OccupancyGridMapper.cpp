#include "OccupancyGridMapper.hpp"


OccupancyGridMapper::OccupancyGridMapper(maebot_occupancy_grid_t lcm_occupancy_grid)
{
    occupancy_grid_.fromLCM(lcm_occupancy_grid);
}

OccupancyGridMapper::~OccupancyGridMapper()
{
}

void OccupancyGridMapper::calculateLaserOrigins()
{
}

void OccupancyGridMapper::updateGrid()
{
}

void OccupancyGridMapper::addLaserScan(maebot_laser_scan_t input_scan)
{
    pthread_mutex_lock(&laser_scans_mutex_);
    laser_scans_.push(input_scan);
    pthread_mutex_unlock(&laser_scans_mutex_);
}

void OccupancyGridMapper::addPose(maebot_pose_t input_pose)
{
    pthread_mutex_lock(&poses_mutex_);
    poses_.push(input_pose);
    pthread_mutex_unlock(&poses_mutex_);
}
