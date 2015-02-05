#include "OccupancyGridMapper.hpp"

OccupancyGridMapper::OccupancyGridMapper(maebot_occupancy_grid_t lcm_occupancy_grid) 
{
    occupancy_grid_.fromLCM(lcm_occupancy_grid);
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

/*
	s_x, s_y :	start position of the line (meters)
	e_x, e_y : end position of the line (meters)
	inc		 : amount to travel along the line per loop (meters)
	value 	 : value to set for all cells along the line
*/
void OccupancyGridMapper::drawLineMeters(double s_x, double s_y, double e_x, double e_y, const double inc, eecs467::CellOdds value)
{
	drawLineCells((int)(s_x / occupancy_grid_.metersPerCell()),
				  (int)(s_y / occupancy_grid_.metersPerCell()),
				  (int)(e_x / occupancy_grid_.metersPerCell()),
				  (int)(e_y / occupancy_grid_.metersPerCell()),
				  inc / occupancy_grid_.metersPerCell(), value);
}
//Calls with a default increment of 1 cell
void OccupancyGridMapper::drawLineMeters(double s_x, double s_y, double e_x, double e_y, eecs467::CellOdds value)
{
	drawLineCells((int)(s_x / occupancy_grid_.metersPerCell()),
				  (int)(s_y / occupancy_grid_.metersPerCell()),
				  (int)(e_x / occupancy_grid_.metersPerCell()),
				  (int)(e_y / occupancy_grid_.metersPerCell()),
				  1, value);
}
/*
	s_x, s_y :	start position of the line (cell coords, not meters)
	e_x, e_y : end position of the line (cell coords, not meters)
	inc		 : amount to travel along the line per loop (cell coords, not meters)
	value 	 : value to set for all cells along the line
*/
void OccupancyGridMapper::drawLineCells(int s_x, int s_y, int e_x, int e_y, const double inc, eecs467::CellOdds value)

{
	double a = atan2(e_y - s_y, e_x - s_x);
	int n = ceil(sqrt((e_x - s_x)*(e_x - s_x)+(e_y - s_y)*(e_y - s_y)) / inc);
	double i_x = cos(a) * inc,
		   i_y = sin(a) * inc,
		   p_x = s_x,
		   p_y = s_y;
		   
	for(int i = 0; i < n; i++)
	{
		int v = (int)occupancy_grid_.logOdds(p_x, p_y) + (int)value;
		v = v > INT8_MAX ? INT8_MAX : (v < INT8_MIN ? INT8_MIN : v);
		occupancy_grid_.setLogOdds(p_x, p_y, v);
			
		p_x += i_x;
		p_y += i_y;
	}
}
//Calls with a default increment of 1 cell
void OccupancyGridMapper::drawLineCells(int s_x, int s_y, int e_x, int e_y, eecs467::CellOdds value)
{
	drawLineCells(s_x, s_y, e_x, e_y, 1, value);
}

ApproxLaser& OccupancyGridMapper::getApproxLaser()
{
    return approx_laser_;
}

MovingLaser& OccupancyGridMapper::getMovingLaser() {
    return moving_laser_;
}

eecs467::OccupancyGrid& OccupancyGridMapper::getOccupancyGrid() {
    return occupancy_grid_;
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
