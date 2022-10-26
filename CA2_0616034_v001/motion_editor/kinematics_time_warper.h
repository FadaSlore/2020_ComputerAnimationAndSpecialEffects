#ifndef _KINEMATICS_TIME_WARPER_H_
#define _KINEMATICS_TIME_WARPER_H_

#include "kinematics_def.h"
#include <memory>
#include <utility>
#include "math_type.h"
#include "kinematics_time_warp_hard_constraint.h"

namespace kinematics {

class TimeWarper final
{

public:

    TimeWarper();
    TimeWarper(const TimeWarper &) = delete;
    virtual ~TimeWarper();
    TimeWarper &operator=(const TimeWarper &) = delete;

    double time_step() const;
    double min_time_step() const;
    double max_time_step() const;

    void Configure(
            const math::SpatialTemporalVector6d_t &original_motion_sequence,
            const double time_step,
            const double min_time_step,
            const double max_time_step
            );

    math::SpatialTemporalVector6d_t ComputeWarpedMotion(
            const TimeWarpHardConstraintColl_t &hard_constraint_coll
            );

protected:

private:

    std::unique_ptr<math::SpatialTemporalVector6d_t> original_motion_sequence_;
    std::unique_ptr<TimeWarpHardConstraintColl_t> hard_constraint_coll_;

    double time_step_;
    double min_time_step_;
    double max_time_step_;
};

} // namespace kinematics {

#endif // #ifndef _KINEMATICS_TIME_WARPER_H_
