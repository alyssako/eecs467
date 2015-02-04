#include "ApproxLaser.hpp"
#include <cassert>
#include <iostream>

bool ApproxLaser::findPts(const maebot_laser_scan_t *scan)
{
    maebot_pose_t start, end;
    bool found = false;
    std::deque<maebot_pose_t>::iterator it;
    for(it = poses.begin(); it != poses.end(); ++it)
    {
        if(it->utime > scan->times[0] && (it+1)->utime < scan->times[0])
        {
            end = *it;
            found = true;
            break;
        }
    }
    // assert that range of poses is correct
    assert(found);
    assert((it+1) != poses.end());

    start = *(it+1);
    LaserScanApprox retval = {start, end, *scan};
    return found;
}

bool ApproxLaser::addPose(const maebot_pose_t* newPose)
{
    if(poses.size() == 5)
    {
        poses.pop_back();
    }
    
    if(checkOrder(newPose))
    {
        poses.push_front(*newPose);
        return true;
    }

    std::deque<maebot_pose_t>::iterator it;
    for(it = poses.begin(); it != poses.end(); ++it)
    {
        if(it->utime < newPose->utime)
            break;
    }
    
    if(it == poses.end())
        return false;

    poses.insert(it, *newPose);
    return true;
}

bool ApproxLaser::checkOrder(maebot_pose_t *newPose)
{
    if(!poses.empty())
        return newPose->utime > poses.front().utime;

    return true;
}
