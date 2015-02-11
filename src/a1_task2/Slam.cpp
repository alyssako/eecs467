#include "Slam.hpp"

Slam::Slam()
{
    pthread_mutex_init(&motor_feedbacks_mutex_, NULL);
}
void Slam::addMotorFeedback(maebot_motor_feedback_t input_feedback)
{
    motor_feedbacks_.push(input_feedback);
}

bool Slam::motorFeedbacksEmpty()
{
    return motor_feedbacks_.empty();
}

void Slam::lockMotorFeedbacksMutex()
{
    pthread_mutex_lock(&motor_feedbacks_mutex_);
}

void Slam::unlockMotorFeedbacksMutex()
{
    pthread_mutex_unlock(&motor_feedbacks_mutex_);
}
