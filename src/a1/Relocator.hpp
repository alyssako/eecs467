#ifndef RELOCATOR_HPP
#define RELOCATOR_HPP


typedef struct position_t partical_t;
struct position_t {
    double x;
    double y;
    double theta;

    position_t() : x(0.), y(0.), theta(0.) {}
};

class Relocator
{
    private:
        maebot_motor_feedback_t previous_feedback;

    public:
        partical_t particals[PARTICAL_NUM];

        Relocator(){}
        ~Relocator(){}
        
        void move(maebot_motor_feedback_t feedback);// use both feedback to get deltas and move all 1000 particals 
        double * align();
};

#endif
