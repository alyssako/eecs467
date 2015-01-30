#include "ApproxLaser.hpp"

LaserScanApprox ApproxLaser::findPts(maebot_laser_scan_t& scan)
{
    maebot_pose_t start, end;
    bool found = false;
    deque<maebot_pose_t>::iterator it;
    for(it = poses.begin(); it != poses.end(); ++it)
    {
        if(*it < LaserScan.scan[0] && *(it+1) > LaserScan.scan[0])
        {
            start = *it;
            found = true;
            break;
        }
    }
    // assert that range of poses is correct
    assert(found);
    assert((it+1) != poses.end());

    end = *(it+1);
    LaserScanApprox retval = {start, end, LaserScan};
    return retval;
}

bool ApproxLaser::addPose(maebot_pose_t newPose)
{
    if(poses.size() == 5)
    {
        poses.pop_back();
    }
    if(checkOrder(newPose))
    {
        poses.push_front(newPose);
        return true;
    }
    return false;
}

bool ApproxLaser::checkOrder(maebot_pose_t newPose)
{
    return newPose.utime < poses.front().utime;
}
