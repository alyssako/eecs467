#include <deque>
#include <math.h>
#include <math/point.hpp>
#include "particle_data.hpp"
#include <math/gsl_util_rand.h>
#include <mapping/occupancy_grid_utils.hpp>
#include <iostream>
#include <string>


particle_data::particle_data(){
    number = 0;
}

particle_data::particle_data(const particle_data& other):
	number(other.number),pose(other.pose),
    weight(other.weight){}

particle_data::particle_data(int numb, maebot_pose_t starting_loc){
	number = numb;

	for(int i=0; i < number ; ++i){
		pose.push_back(starting_loc);
        double temp = 1/numb;
		weight.push_back(temp);
	}
}

maebot_pose_t particle_data::get_pose(int index){
	return pose[index];
}

float particle_data::get_weight(int index){
	return weight[index];
}

int particle_data::get_size(){
	return number;
}


void particle_data::translate(maebot_pose_delta_t deltas){
	float dx = deltas.x;
    float dy = deltas.y;
    float dt = deltas.theta;

    gslu_rand_seed();
	gsl_rng * obj = gslu_rand_rng_alloc ();

	for(int i=0; i< number; ++i){
		float devx= gslu_rand_gaussian( obj , dx,0.05*dx);
		float devy= gslu_rand_gaussian( obj , dy,0.05*dy);
		float devt= gslu_rand_gaussian( obj , dt,0.05*dt);

		pose[i].x += devx;
		pose[i].y += devy;
		pose[i].theta += devt;
	}

}
void particle_data::calc_weight(eecs467::OccupancyGrid &grid, const maebot_laser_scan_t &lasers)
{
    double max_weight = 0.0;

    for (int n = 0; n < this->number; n++)
    {        
        double calc_weight = 0.0;            
        int num_rays = lasers.num_ranges;

        int ccc = 0;

        for (int r = 0; r < num_rays; r++)
        {
            //RMC - maybe remove this?
            if (lasers.intensities[r] == 0) {continue;}

            //std::cout << "ray_" << ccc << " is: ";

            float theta = lasers.thetas[r];
            float range = lasers.ranges[r];
            float mae_x = this->pose[n].x;
            float mae_y = this->pose[n].y;
            
            float cos = cosf(theta);
            float sin = sinf(theta);

            bool check_term_cell = true;

            //sample along laser path
            for (float sample_m = 0.0; sample_m < (range - 0.5); sample_m += 0.05)
            {
                float dx = sample_m * cos;
                float dy = sample_m * sin;
                
                eecs467::Point<int> sample_cell = eecs467::global_position_to_grid_cell(eecs467::Point<float>(mae_x + dx, mae_y + dy), grid);
                
                if (grid.isCellInGrid(sample_cell.y, sample_cell.x))
                {
                    if (grid.logOdds(sample_cell.y, sample_cell.x) > 0) //laser longer than expected (expected termination in this cell)
                    {
                        //std::cout << "BAD\n";
                        calc_weight -= 12.0; //BAD
                        check_term_cell = false;
                        break;
                    }
                } 
            }
            
            if (check_term_cell)
            {
                //sample_cell = laser termination cell
                eecs467::Point<int> sample_cell = eecs467::global_position_to_grid_cell(eecs467::Point<float>(mae_x + (range*cos), mae_y + (range*sin)), grid);
                
                if (grid.isCellInGrid(sample_cell.y, sample_cell.x))
                {
                    if (grid.logOdds(sample_cell.y, sample_cell.x) > 0) //laser is expected dist
                    {
                        //std::cout << "GOOD\n";
                        calc_weight -= 4.0; //GOOD
                    }
                    else //laser shorter than expected
                    {
                        //std::cout << "OK\n";
                        calc_weight -= 8.0; //OK
                    }
                }
            }
        
        //std::string garbage = "";
        //std::cin >> garbage;
        }
        
        //std::cout << "weight of this particle: " <<  calc_weight << "\n";
        this->weight[n] = calc_weight;
        max_weight = std::max(calc_weight, max_weight);
    }
    
    //shift by max, exponentiate, sum all weight
    double sum = 0.0;
    for (int s = 0; s < this->number; s++)
    {
        this->weight[s] += max_weight;
        this->weight[s] = pow(10.0, this->weight[s]);
        sum += this->weight[s];
    }

    //normalize into [0,1]
    for (int s = 0; s < this->number; s++)
    {
        this->weight[s] /= sum;
    }
}


maebot_pose_t particle_data::get_best(){
	int highest_index=0;
	for(int i=0; i< number; ++i){
		if(weight[i] > highest_index){
			highest_index = weight[i];
		}
	}

	return pose[highest_index];
}

float* particle_data::get_particle_coords(){
    float pts[3*number];
    for(int i=0;i<pose.size();++i){
        pts[3*i+0] = pose[i].x;
        pts[3*i+1] = pose[i].y;
        pts[3*i+2] = 0;
    }
    return pts;
}