#include "ApproxLaser.hpp"

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
