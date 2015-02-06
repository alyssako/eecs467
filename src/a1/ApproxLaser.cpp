#include "ApproxLaser.hpp"
#include <cassert>
#include <iostream>

LaserScanRange ApproxLaser::findPts(const maebot_laser_scan_t *scan)
{
    if(poses_.empty()){
        std::cout << "waiting for initial pose" << std::endl;
        //lasers_.push(*scan_t);
        exit(1); // if we called this without adding a pose first, we dun fucked up
    }

    //maebot_laser_scan_t *scan;
    //lasers_.push(*scan_t);
    //scan = &lasers_.front();

    // scrap the first lidar scan because we don't have two poses to interpolate from
    maebot_pose_t start, end;
    LaserScanRange retval = {false, start, end, *scan};
    if(poses_.size() != 1){
        bool found = false;
        std::deque<maebot_pose_t>::iterator iter;
        for(iter = poses_.begin(); iter != poses_.end(); ++iter)
        {
            // the end time (scan->utime) of the lidar scan should be equal to the
            // utime of the pose
            if(iter->utime == scan->utime)
            {
                end = *iter;
                found = true;
                break;
            }
        }
        // assert that range of poses is correct
        assert(found);
        assert((iter-1) >= poses_.begin());

        start = *(iter-1);
        retval.valid = true;
        retval.start_pose = start;
        retval.end_pose = end;
    }

    //lasers_.pop();
    return retval;
}

void ApproxLaser::addPose(const maebot_pose_t* newPose)
{
    if(poses_.size() == 5)
    {
        poses_.pop_back();
    }
    
    poses_.push_front(*newPose);
}
