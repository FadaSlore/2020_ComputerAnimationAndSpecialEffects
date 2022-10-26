#include "kinematics_time_warper.h"
#include <utility>
#include "boost/numeric/conversion/cast.hpp"
#include "math_utils.h"

namespace kinematics {

// public func.

TimeWarper::TimeWarper()
    :original_motion_sequence_(new math::SpatialTemporalVector6d_t),
    hard_constraint_coll_(new TimeWarpHardConstraintColl_t),
    time_step_(double{0.0}),
    min_time_step_(double{0.0}),
    max_time_step_(double{0.0})
{
}

TimeWarper::~TimeWarper()
{
}

double TimeWarper::time_step() const
{
    return time_step_;
}

double TimeWarper::min_time_step() const
{
    return min_time_step_;
}

double TimeWarper::max_time_step() const
{
    return max_time_step_;
}

void TimeWarper::Configure(
        const math::SpatialTemporalVector6d_t &original_motion_sequence,
        const double time_step,
        const double min_time_step,
        const double max_time_step
        )
{
    *original_motion_sequence_ = original_motion_sequence;
    time_step_ = time_step;
    min_time_step_ = min_time_step;
    max_time_step_ = max_time_step;
}

math::SpatialTemporalVector6d_t TimeWarper::ComputeWarpedMotion(
        const TimeWarpHardConstraintColl_t &hard_constraint_coll
        )
{
    // TO DO
    *hard_constraint_coll_ = hard_constraint_coll;

	math::SpatialTemporalVector6d_t output = *original_motion_sequence_;
	/*
	const kinematics::TimeWarpHardConstraintColl_t time_warp_hard_constraint_coll =
    {
        kinematics::TimeWarpHardConstraint_t(int32_t{0}, double{0.0}),
        kinematics::TimeWarpHardConstraint_t(
                param_->value<int32_t>("time_warp.desired_catch_frame_idx"),
                param_->value<double>("time_warp.drop_ball_catch_frame_idx")
                * acclaim::Motion::time_step()
                ),
        kinematics::TimeWarpHardConstraint_t(
                (fk_solver_coll_->at(0)->motion()->frame_num() - 1),
                boost::numeric_cast<double>((fk_solver_coll_->at(0)->motion()->frame_num() - 1))
                * acclaim::Motion::time_step()
                ),
    };

    warped_motion_sequence_.reset(new math::SpatialTemporalVector6d_t);
    *warped_motion_sequence_ = time_warper_->ComputeWarpedMotion(
            time_warp_hard_constraint_coll
            );
	*/

	int32_t desired_frame = (*hard_constraint_coll_)[1].frame_idx;
	int32_t catched_frame = (*hard_constraint_coll_)[1].play_second / time_step_;
	//printf("desired_frame: %d, catched_frame: %d\n", desired_frame, catched_frame);
	int32_t joint_num = original_motion_sequence_->spatial_size();
	int32_t frame_num = original_motion_sequence_->temporal_size();
	
	double ratio, point;
	int32_t frame_1, frame_2;
	math::Quaternion_t quaternion_1, quaternion_2, quaternion_slerp;
	math::Vector6d_t vector6d_1, vector6d_2;
	math::Vector3d_t radian_1, radian_2;
	math::Vector3d_t angular_vector, linear_vector;

	for (int32_t i = 0; i < joint_num; i++)
	{
		for (int32_t j = 1; j < frame_num; j++)
		{
			ratio = (double)catched_frame / (double)desired_frame;
			point = ratio * j;
			if (j > desired_frame)
			{
				ratio = (double)(frame_num - catched_frame) / (double)(frame_num - desired_frame);
				point = ratio * (j - desired_frame) + catched_frame;
			}

			frame_1 = floor(point);
			frame_2 = ceil(point);
			ratio = point - (double)frame_1;

			/*
			if (frame_1 >= frame_num)
				printf("outrange\n");
			*/

			if (frame_2 >= frame_num)
				frame_2 = frame_num - 1;

			//i: spatial, j: temporal
			vector6d_1 = original_motion_sequence_->element(i, frame_1);
			vector6d_2 = original_motion_sequence_->element(i, frame_2);
			radian_1 = math::ToRadian(vector6d_1.angular_vector());
			radian_2 = math::ToRadian(vector6d_1.angular_vector());
			quaternion_1 = math::ComputeQuaternionXyz(radian_1[0], radian_1[1], radian_1[2]);
			quaternion_2 = math::ComputeQuaternionXyz(radian_2[0], radian_2[1], radian_2[2]);
			quaternion_slerp = math::Slerp(quaternion_1, quaternion_2, ratio);

			angular_vector = math::ToDegree(math::ComputeEulerAngleXyz(quaternion_slerp.toRotationMatrix()));
			linear_vector = vector6d_1.linear_vector() * (1 - ratio) + vector6d_2.linear_vector() * ratio;
			
			output.set_element(i, j, math::Vector6d_t(angular_vector, linear_vector));
		}
	}
	return output;
    //return *original_motion_sequence_;
}
// protected func.

// private func.

} // namespace kinematics {
