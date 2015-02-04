#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <vector>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"

#include "MovingLaser.hpp"
//Creates and returns a LaserScan and fills its origins vector.
LaserScan MovingLaser::findOrigin(LaserScanApprox approx_scan)
{
	LaserScan ls;
	
	for(unsigned int i = 0; i < approx_scan.scan.times.size(); i++)
		ls.origins.push_back(findOriginSingle(approx_scan.scan.times[i],
			approx_scan.start_pose, approx_scan.end_pose));
	
	ls.scan = approx_scan.scan;
	
	return ls;
}

//Returns a new pose corresponding to the interpolated pose between a and b at time t.
maebot_pose_t MovingLaser::findOriginSingle(int64_t t, maebot_pose_t a, maebot_pose_t b)
{
	if(t < a.utime || t > b.utime)
		std::cout << "Out of range! t: " << t << ", a: " << a.utime << ", b: " << b.utime << std::endl;

	float percent = (t - a.utime) / (b.utime - a.utime);
	maebot_pose_t n;
	n.x = (b.x - a.x) * percent + a.x;
	n.y = (b.y - a.y) * percent + a.y;
	n.theta = angle_diff(a.theta, b.theta) + a.theta;
	n.utime= t;
	return n;
}

//Returns the difference b-a (radians) in the range [-PI, PI)
float MovingLaser::angle_diff(float a, float b)
{
	float d = b - a;
	while(d >= 3.1415926) d -= 3.1415926;
	while(d < -3.1415926) d += 3.1415926;
	return d;
}
