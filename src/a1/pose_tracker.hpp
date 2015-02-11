#ifndef POSE_TRACKER_HPP
#define POSE_TRACKER_HPP

#include <deque>
#include <lcmtypes/maebot_motor_feedback_t.hpp>
#include <lcmtypes/maebot_pose_t.hpp>
#include "action_model.hpp"

typedef maebot_pose_t maebot_pose_delta_t;

class pose_tracker
{
	public:
	    std::deque<maebot_pose_t> poses;

	    pose_tracker();
	    void push_msg(const maebot_motor_feedback_t *msg, action_model & model);
	    maebot_pose_delta_t calc_deltas(int64_t t);
	    int64_t recent_pose_time();

	private:
	    std::deque<const maebot_motor_feedback_t*> odo_msgs;
	    maebot_pose_t last_calc_pose;
};

#endif
