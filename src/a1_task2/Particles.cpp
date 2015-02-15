#include "Particles.hpp"

#include <cassert>

Particles::Particles()
{
    // construct random environment
    r = gslu_rand_rng_alloc();

    // construct individual particles
    Particle particle = {0, 0, 0, 0, 0.001};
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        particles_.push_back(particle);
    }
    most_likely_ = particles_[0];
}

Particles::~Particles()
{
}

maebot_pose_t Particles::toPose(int index)
{
    maebot_pose_t retval;
    retval.x = particles_[index].x;
    //std::cout << "particles x: " << particles_[index].x << std::endl;
    retval.y = particles_[index].y;
    retval.theta = particles_[index].theta;
    retval.utime = particles_[index].utime;

    return retval;
}

maebot_pose_t Particles::mostProbable()
{
    maebot_pose_t retval;
    retval.x = most_likely_.x;
    retval.y = most_likely_.y;
    retval.theta = most_likely_.theta;
    retval.utime = most_likely_.utime;
    return retval;
}

void Particles::updateParticles(float delta_x, float delta_y, float delta_theta, eecs467::OccupancyGrid *grid, LaserScanRange scan)
{
    //std::cout << "deltas: (" << delta_x << ", " << delta_y << ", " << delta_theta << ")\n";
    moveRandom(grid, scan, delta_x, delta_y, delta_theta);
    //std::cout << "here 4" << std::endl;
    normalizeProbabilities();
    //std::cout << "here 5" << std::endl;
    resample();
    //std::cout << "here 6" << std::endl;
    most_likely_ = particles_[findLargestProbability()];
}

void Particles::moveRandom(eecs467::OccupancyGrid *grid, LaserScanRange lsr, float delta_x, float delta_y, float delta_theta)
{
    double delta_s = sqrt((delta_x * delta_x) + (delta_y * delta_y));
    double alpha = eecs467::wrap_to_2pi(eecs467::angle_diff(eecs467::wrap_to_2pi(atan2(delta_y, delta_x)), eecs467::wrap_to_2pi(delta_theta)));
    double theta_alpha = eecs467::wrap_to_2pi(eecs467::angle_diff(eecs467::wrap_to_2pi(alpha), eecs467::wrap_to_2pi(delta_theta)));

    //std::cout << "deltas after: (" << alpha << ", " << delta_s << ", " << theta_alpha << ")\n";

    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        moveRandomSingle(grid, lsr, delta_s, alpha, theta_alpha, i);
    }
}

void Particles::moveRandomSingle(eecs467::OccupancyGrid *grid, LaserScanRange laser_scan_range, double delta_s, double alpha, double theta_alpha, int index)
{
    /*std::cout << "OLD Delta S:   " << delta_s << std::endl
              << "OLD Delta A:   " << alpha << std::endl 
              << "OLD Delta T-A: " << theta_alpha << std::endl;*/
    double new_delta_s = gslu_rand_gaussian(r, delta_s, 0.1*delta_s);
    double new_delta_alpha = gslu_rand_gaussian(r, alpha, 0.1*alpha);
    double new_delta_theta_alpha = gslu_rand_gaussian(r, theta_alpha, 0.1*theta_alpha);
    /*std::cout << "New Delta S:   " << new_delta_s << std::endl
              << "New Delta A:   " << new_delta_alpha << std::endl 
              << "New Delta T-A: " << new_delta_theta_alpha << std::endl;*/

    laser_scan_range.start_pose = toPose(index);
    /*std::cout << "start pose: " << laser_scan_range.start_pose.x << ", "
                                << laser_scan_range.start_pose.y << ", "
                                << laser_scan_range.start_pose.theta << std::endl;*/

    rotateParticle(new_delta_alpha, index);
    //std::cout << "heah2" << std::endl;
    moveParticle(new_delta_s, index);
    //std::cout << "heah3" << std::endl;
    rotateParticle(new_delta_theta_alpha, index);
    //std::cout << "heah4" << std::endl;
    particles_[index].utime = laser_scan_range.end_pose.utime;

    laser_scan_range.end_pose = toPose(index);
    /*std::cout << "end pose: " << laser_scan_range.end_pose.x << ", "
                              << laser_scan_range.end_pose.y << ", "
                              << laser_scan_range.end_pose.theta << std::endl;
    std::cout << "heah5" << std::endl;*/

    // calculate probability of new particle
    calculateProbabilitySingle(grid, laser_scan_range, index);
}

void Particles::rotateParticle(double theta, int index)
{
    particles_[index].theta = eecs467::wrap_to_2pi(particles_[index].theta + theta);
}

void Particles::moveParticle(double s, int index)
{
    particles_[index].x += s * cos(particles_[index].theta);
    particles_[index].y += s * sin(particles_[index].theta);
}

/*void Particles::calculateProbability(occupancy_grid_t *grid, maebot_laser_scan_t *scan)
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        calculateProbabilitySingle(grid, scan, i);
    }
}*/

void Particles::calculateProbabilitySingle(eecs467::OccupancyGrid *grid, LaserScanRange scan, int index)
{
    MovingLaser ml;
    LaserScan ls = ml.findOrigin(scan);
    //std::cout << "ls.origins.size(): " << ls.origins.size() << std::endl;
    if(ls.origins.size() != ls.scan.ranges.size()) { std::cout << "uh oh\n"; exit(1); }
    for(uint i = 0; i < ls.origins.size(); i++)
    {
        auto a = ls.scan.thetas[i] - ls.origins[i].theta;
        auto x = ls.origins[i].x + ls.scan.ranges[i] * cos(a);
        auto y = ls.origins[i].y + ls.scan.ranges[i] * sin(a);
        auto v = grid->logOdds(x, y);
        if(v < 0)
        {
            particles_[index].probability -= 6;
        }
        else if(v > 0)
        {
            particles_[index].probability -= 12;
        }
        else
        {
            particles_[index].probability -= 2;
        }
    }
}

void Particles::normalizeProbabilities()
{
    int index_max = findLargestProbability();
    subtractProbabilities(particles_[index_max].probability);
    exponentiate();
    float sum = sumProbabilities();
    if(sum == 0) { std::cout << "WHAT THE FUCK IS THIS SHIT" << std::endl; exit(1); }
    divideProbabilities(sum);
}

void Particles::exponentiate()
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        particles_[i].probability = exp(particles_[i].probability);
    }
}

void Particles::subtractProbabilities(float max)
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        particles_[i].probability -= max;
    }

    return;
}

int Particles::findLargestProbability()
{
    int index = 0;
    for(int i = 1; i < NUM_PARTICLES; i++)
    {
        if(particles_[i].probability > particles_[index].probability)
            index = i;
    }
    return index;
}

float Particles::sumProbabilities()
{
    float sum = 0;
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        sum += particles_[i].probability;
    }
    return sum;
}

void Particles::divideProbabilities(float sum)
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        particles_[i].probability /= sum;
    }
}

void Particles::resample()
{
    // write own functor for comparing probs
    comp comparator;
    std::sort(particles_.begin(), particles_.end(), comparator);
    double increment = 1.0/NUM_PARTICLES;
    std::vector<Particle> resampled;
    resampled.resize(NUM_PARTICLES);
    for(double i = 0, index = 0, newvecindex = 0; i < 1; i+= increment, newvecindex++)
    {
        bool found = false;
        for(int j = index; j < NUM_PARTICLES; j++)
        {
            if(particles_[j].probability > i)
            {
                index = (j == 0 ? 0:j-1);
                found = true;
                break;
            }
        }
        if(!found) { index = NUM_PARTICLES - 1; }
        resampled[newvecindex] = particles_[index];
    }
    particles_ = resampled;
}

/* find position of laser scan and positions of individual laser beams within scan */

LaserScanRange Particles::getLaserScan(maebot_pose_t poseA, maebot_laser_scan_t scanB, std::deque<maebot_pose_t>& poses)
{
    std::vector<maebot_pose_t> mB = findLeftRightPoses(scanB.utime, poses);
    if(mB.size() == 1)
    {
        LaserScanRange lsr(true, poseA, poseA, scanB);
        return lsr;
    }
    MovingLaser moving_laser;
    
    // find the pose of the last laser scan
    //std::cout << "mB[0]: " << mB[0].utime << "\nmB[1]: " << mB[1].utime << "\nutime: " << scanB->utime << std::endl;
    maebot_pose_t poseB = moving_laser.findOriginSingle(scanB.utime, mB[0], mB[1]);
    LaserScanRange lsr(true, poseA, poseB, scanB);
    return lsr;
}

std::vector<maebot_pose_t> Particles::findLeftRightPoses(int64_t time, std::deque<maebot_pose_t>& poses)
{
    //std::cout << "poses size: " << poses.size() << std::endl;
    //std::cout << "poses[0]: " << (poses.end()-1)->utime << ", " << (poses.end()-1)->x << ", " << (poses.end()-1)->y << ", " << (poses.end()-1)->theta << ")" << std::endl;
    //std::cout << "poses[1]: " << (poses.end()-2)->utime << std::endl;
    //std::cout << "time:     " << time << std::endl;
    std::vector<maebot_pose_t> m;
    assert(!poses.empty());
    for(std::deque<maebot_pose_t>::iterator it = poses.end()-1; it != poses.begin()-1; it--)
    {
 //       assert(it->utime);
        assert(time);
        if(it->utime > time)
        {
            if(it+1 >= poses.end())
                m.push_back(*it);
            else
                m.push_back(*(it+1));
            m.push_back(*it);
            return m;
        }
    }

    if(poses.size() == 1)
    {
        m.push_back(*(poses.begin()));
    }
    
    return m;
}
