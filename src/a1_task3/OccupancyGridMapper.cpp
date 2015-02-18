#include "OccupancyGridMapper.hpp"
#include <iostream>
#include <cassert>

OccupancyGridMapper::OccupancyGridMapper(lcm::LCM *lcm_t) :
	occupancy_grid_(10, 10, 0.05),
	occupancy_grid_expanded_(10, 10, 0.05),
    lcm(lcm_t)
{
    pthread_mutex_init(&mapper_mutex_, NULL);
    pthread_cond_init(&cv_, NULL);
    end_points_.reserve(300);
}

/*OccupancyGridMapper::OccupancyGridMapper(int height, int width, double cellSize) :
    occupancy_grid_(width, height, cellSize)
{
    pthread_mutex_init(&mapper_mutex_, NULL);
    pthread_cond_init(&cv_, NULL);
}*/

OccupancyGridMapper::~OccupancyGridMapper()
{
}

void OccupancyGridMapper::setLogOddsMapper(int x, int y, double logOdds)
{
    occupancy_grid_(x, y) = logOdds;
}

void OccupancyGridMapper::expandOccupancyGrid()
{
    int w = occupancy_grid_.widthInCells();
    int h = occupancy_grid_.heightInCells();
    std::unordered_set<int> s;
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            auto v = occupancy_grid_(x, y);
            occupancy_grid_expanded_(x, y) = v;
            if(v > 0)
            {
                for(int y0 = y - EXPAND_THICKNESS; y0 <= y+EXPAND_THICKNESS; y0++)
                {
                    for(int x0 = x - EXPAND_THICKNESS; x0 <= x + EXPAND_THICKNESS; x0++)
                    {
                        s.insert(y0*w + x0);
                    }
                }
            }
        }
    }

    for(const int &i: s)
    {
        occupancy_grid_expanded_.setLogOdds(i%w, i/w, 127);
    }
}

LaserScan OccupancyGridMapper::calculateLaserOrigins()
{
    maebot_pose_t pose = poses_.front();
    poses_.pop();
    maebot_laser_scan_t laser_scan = laser_scans_.front();
    laser_scans_.pop();

    if(poses_.empty())
    {
        approx_laser_.addPose(&pose);
        LaserScanRange lsr(true, pose, pose, laser_scan);

        LaserScan ls = moving_laser_.findOrigin(lsr);

        return ls;
    }
    else
    {
        assert(laser_scan.utime == pose.utime);

        if(!approx_laser_.containsPoses())
        {
            approx_laser_.addPose(&pose);
            LaserScan ls;
            ls.valid = false;
            return ls;
        }

        approx_laser_.addPose(&pose);
        LaserScanRange lsr = approx_laser_.findPts(&laser_scan);
        if(lsr.valid == false)
        {
            LaserScan ls;
            ls.valid = false;
            return ls;
        }

        LaserScan ls = moving_laser_.findOrigin(lsr);

        return ls;
    }
}

void OccupancyGridMapper::updateGrid(LaserScan scan)
{
    for(unsigned int i = 0; i < scan.origins.size(); ++i)
    {
        double a = scan.origins[i].theta - scan.scan.thetas[i];
        double d = scan.scan.ranges[i];
        double e_x = scan.origins[i].x + d * cos(a);
        double e_y = scan.origins[i].y + d * sin(a);
        maebot_pose_t ep;
        ep.x = e_x;
        ep.y = e_y;
        ep.utime = scan.scan.utime;
        end_points_[i] = ep;
        drawLineMeters(scan.origins[i].x, scan.origins[i].y, e_x, e_y, -2, 1);
    }
    expandOccupancyGrid();
}

/*
	s_x, s_y :	start position of the line (meters)
	e_x, e_y : end position of the line (meters)
	inc		 : amount to travel along the line per loop (meters)
	value 	 : value to set for all cells along the line
*/
void OccupancyGridMapper::drawLineMeters(double s_x, double s_y, double e_x, double e_y, double inc, eecs467::CellOdds value, eecs467::CellOdds value_end)
{
	double a = atan2(e_y - s_y, e_x - s_x);
    double dist = sqrt((e_x - s_x)*(e_x - s_x)+(e_y - s_y)*(e_y - s_y));
    if(dist == 0)
    {
        return;
    }
	int n = ceil(dist / inc);
    inc = dist/n;
	double i_x = cos(a) * inc,
		   i_y = sin(a) * inc,
		   p_x = s_x,
		   p_y = s_y;

    eecs467::Point<double> last_cell(e_x, e_y);
    last_cell = eecs467::global_position_to_grid_cell(last_cell, occupancy_grid_);
		   
	for(int i = 0; i <= n; i++)
	{
        int v;
        eecs467::Point<double> p(p_x, p_y);
		auto p_origin = eecs467::global_position_to_grid_cell(p, occupancy_grid_);
        if(p_origin == last_cell)
        {
            v = (int)occupancy_grid_.logOdds(p_origin.x, p_origin.y) + (int)value_end;
        }
        else
        {
            v = (int)occupancy_grid_.logOdds(p_origin.x, p_origin.y) + (int)value;
        }
        v = v > 127 ? 127 : (v < -127 ? -127 : v);
        occupancy_grid_.setLogOdds(p_origin.x, p_origin.y, v);
			
		p_x += i_x;
		p_y += i_y;
	}
}
//Calls with a default increment of 1 cell
void OccupancyGridMapper::drawLineMeters(double s_x, double s_y, double e_x, double e_y, eecs467::CellOdds value, eecs467::CellOdds value_end)
{
	drawLineMeters(s_x, s_y, e_x, e_y, occupancy_grid_.metersPerCell(), value, value_end);
}

void OccupancyGridMapper::publishOccupancyGrid(maebot_pose_t pose)
{
    //maebot_occupancy_grid_t data = occupancy_grid_expanded_.toLCM();
    maebot_occupancy_grid_t data = occupancy_grid_.toLCM();
    lcm->publish("OCCUPANCY_GRID_GUI", &data);
    int64_t time = end_points_[0].utime;
    int i = 0;
    while(end_points_[i].utime == time)
    {
        lcm->publish("MAEBOT_LASER_ENDPOINTS", &end_points_[i]);
        i++;
    }
    lcm->publish("MAEBOT_POSE_BEST", &pose);
    //std::cout << "sent grid\n";
}

ApproxLaser OccupancyGridMapper::getApproxLaser()
{
    return approx_laser_;
}

MovingLaser OccupancyGridMapper::getMovingLaser() {
    return moving_laser_;
}

eecs467::OccupancyGrid& OccupancyGridMapper::getOccupancyGrid() {
    return occupancy_grid_;
}

bool OccupancyGridMapper::laserScansEmpty()
{
    return laser_scans_.empty();
}

bool OccupancyGridMapper::posesEmpty()
{
    return poses_.empty();
}

void OccupancyGridMapper::lockMapperMutex()
{
    pthread_mutex_lock(&mapper_mutex_);
}

void OccupancyGridMapper::unlockMapperMutex()
{
    pthread_mutex_unlock(&mapper_mutex_);
}

void OccupancyGridMapper::wait()
{
    pthread_cond_wait(&cv_, &mapper_mutex_);
}

void OccupancyGridMapper::signal()
{
    pthread_cond_signal(&cv_);
}

void OccupancyGridMapper::addLaserScan(maebot_laser_scan_t input_scan)
{
    laser_scans_.push(input_scan);
}

void OccupancyGridMapper::addPose(maebot_pose_t input_pose)
{
    poses_.push(input_pose);
}

double OccupancyGridMapper::metersPerCellMapper()
{
    return occupancy_grid_.metersPerCell();
}
