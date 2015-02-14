#include "Particles.hpp"

Particles::Particles()
{
    // construct random environment
    const gsl_rng_type *T;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    r = gsl_rng_alloc(T);

    // construct individual particles
    Particle particle = {0, 0, 0, 0.001};
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
    retval.y = particles_[index].y;
    retval.theta = particles_[index].theta;

    return retval;
}

void Particles::updateParticles(float delta_x, float delta_y, float delta_theta, eecs467::OccupancyGrid *grid, MovingLaser::LaserScanRange *scan)
{
    moveRandom(grid, scan, delta_x, delta_y, delta_theta);
    normalizeProbabilities();
    resample();
    most_likely_ = particles_[findLargestProbability()];
}

void Particles::moveRandom(eecs467::OccupancyGrid *grid, MovingLaser::LaserScanRange *lsr, float delta_x, float delta_y, float delta_theta)
{
    double delta_s = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    double alpha = eecs467::wrap_to_2pi(eecs467::angle_diff(eecs467::wrap_to_2pi(atan2(delta_y, delta_x)), eecs467::wrap_to_2pi(delta_theta)));
    double theta_alpha = eecs467::wrap_to_2pi(eecs467::angle_diff(eecs467::wrap_to_2pi(alpha), eecs467::wrap_to_2pi(delta_theta)));

    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        moveRandomSingle(grid, *lsr, delta_s, alpha, theta_alpha, i, r);
    }
}

void Particles::moveRandomSingle(eecs467::OccupancyGrid *grid, MovingLaser::LaserScanRange laser_scan_range, double delta_s, double alpha, double theta_alpha, int index)
{
    double new_delta_s = gslu_rnd_gaussian(r, delta_s, 0.1*delta_s);
    double new_delta_alpha = gslu_rnd_gaussian(r, alpha, 0.1*alpha);
    double new_delta_theta_alpha = gslu_rnd_gaussian(r, theta_alpha, 0.1*theta_alpha);

    laser_scan_range.start_pose = toPose(index);

    rotateParticle(new_delta_alpha, index);
    moveParticle(new_delta_s, index);
    rotateParticle(new_delta_theta_alpha, index);

    laser_scan_range.end_pose = toPose(index);

    // calculate probability of new particle
    calculateProbabilitySingle(grid, &laser_scan_range, index);
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

void Particles::calculateProbabilitySingle(eecs467::OccupancyGrid *grid, MovingLaser::LaserScanRange *scan, int index)
{
    MovingLaser ml;
    MovingLaser::LaserScan ls = ml.findOrigin(*scan);
    for(int i = 0; i < ls.origins.size(); i++)
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
    index_max = findLargestProbability();
    subtractProbabilities(particles_[index_max].probability);
    exponentiate();
    float sum = sumProbabilities();
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
        particles_[i] -= max;
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
    double increment = 1f/NUM_PARTICLES;
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

MovingLaser::LaserScanRange Particles::getLaserScan(maebot_pose_t *poseA, maebot_scan_t *scanB, std::deque<maebot_pose_t>& poses)
{
    std::vector<maebot_pose_t> mB = findLeftRightPoses(scanB->utime, poses);
    if(mB.size() < 2)
        std::cout << "This isn't working either...\n";
    MovingLaser moving_laser;
    
    // find the pose of the last laser scan
    maebot_pose_t poseB = moving_laser->findOriginSingle(scanB->utime, mB[0], mB[1]);
    
    MovingLaser::LaserScanRange lsr(true, *poseA, poseB, *scanB);
    return lsr;
}

std::vector<maebot_pose_t> Particles::findLeftRightPoses(int64_t time, std::deque<maebot_pose_t>& poses)
{
    std::vector<maebot_pose_t> m;
    for(std::deque<maebot_pose_t>::iterator it = poses.begin(); it != poses.end(); it++)
    {
        if(poses[i].utime > time)
        {
            if(i-1 < 0)
                m.push_back(*it);
            else
                m.push_back(*(it-1));
            m.push_back(*it);
            return m;
        }
    }
    
    if(poses.size() >= 2)
    {
        m.push_back(*(poses.end()-1));
        m.push_back(*(poses.end()-2));
    }
    else if(poses.size() == 1)
    {
        m.push_back(*(poses.end()-1));
        m.push_back(*(poses.end()-1));
    }
    
    return m;
}
