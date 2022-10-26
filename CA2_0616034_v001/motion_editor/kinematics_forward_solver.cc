#include "kinematics_forward_solver.h"
#include <algorithm>
#include "math_utils.h"
#include "acclaim_skeleton.h"
#include "acclaim_motion.h"
#include "helper_forward_kinematics.h"
#include "kinematics_artic_idx.h"
#include "kinematics_pose.h"

namespace kinematics {

// public func.

ForwardSolver::ForwardSolver()
    :skeleton_(nullptr),
    motion_(nullptr),
    artic_path_(new ArticIdxColl_t),
    helper_fk_(new helper::ForwardKinematics)
{
}

ForwardSolver::~ForwardSolver()
{
}

std::shared_ptr<acclaim::Skeleton> ForwardSolver::skeleton() const
{
    return skeleton_;
}

std::shared_ptr<acclaim::Motion> ForwardSolver::motion() const
{
    return motion_;
}

void ForwardSolver::set_skeleton(const std::shared_ptr<acclaim::Skeleton> &skeleton)
{
    skeleton_ = skeleton;
    helper_fk_->set_skeleton(skeleton_);
}

void ForwardSolver::set_motion(const std::shared_ptr<acclaim::Motion> &motion)
{
    motion_ = motion;
    helper_fk_->set_motion(motion_);
}

void ForwardSolver::ConstructArticPath()
{
    int bone_num = skeleton_->bone_num();
    std::vector<ArticIdx*> ptr;
    for (int32_t i = 0; i < bone_num; i++)
        ptr.push_back(new ArticIdx(i));
    
    for (int32_t i = 1; i < bone_num; i++)
    {
        switch (i)
        {
        case 1: break;
        case 6: break;
        case 11: break;
        case 17: break;
        case 23: break;
        case 24: break;
        case 30: break;
        default: ptr[i]->set_parent_idx(i - 1); break;
        }
    }
    ptr[1]->set_parent_idx(0);
    ptr[6]->set_parent_idx(0);
    ptr[11]->set_parent_idx(0);
    ptr[17]->set_parent_idx(13);
    ptr[23]->set_parent_idx(20);
    ptr[24]->set_parent_idx(13);
    ptr[30]->set_parent_idx(27);

    for (int32_t i = 0; i < bone_num; i++)
        artic_path_->push_back(*ptr[i]);
   
    //helper_fk_->ConstructArticPath();
}

PoseColl_t ForwardSolver::ComputeSkeletonPose(const int32_t frame_idx)
{
    return this->ComputeSkeletonPose(motion_->joint_spatial_pos(frame_idx));
}

PoseColl_t ForwardSolver::ComputeSkeletonPose(const math::Vector6dColl_t &joint_spatial_pos)
{
    // TO DO
    
    kinematics::PoseColl_t output;

    math::Vector3d_t root_position = joint_spatial_pos[0].linear_vector();
    kinematics::Pose root_pose;
    root_pose.set_start_pos(root_position);
    root_pose.set_end_pos(root_position);
    output.push_back(root_pose);

    int32_t bone_num = skeleton_->bone_num();
    int32_t bone_id;
    kinematics::Pose last_pose, next_pose;

    math::Vector3d_t radian_vector;
    math::Quaternion_t quaternion_asf, quaternion_angulr;
    math::Quaternion_t quaternion_rotation, quaternion_vector;
    
    std::vector<int32_t> index;
    
    for (int32_t i = 1; i < bone_num; i++)
    {
        last_pose = output[artic_path_->at(i).parent_idx().value()];
        next_pose.set_start_pos(last_pose.end_pos());

        bone_id = i;
        index.clear();
        while (bone_id != 0)
        {
            index.push_back(bone_id);
            bone_id = artic_path_->at(bone_id).parent_idx().value();
        }
        index.push_back(bone_id);

        quaternion_rotation.setIdentity();
        for (int32_t i = index.size() - 1; i >= 0; i--)
        {
            radian_vector = math::ToRadian(joint_spatial_pos[index[i]].angular_vector());
            //R_asf: 
            //Rotation matrix from the local coordinate of this bone to the local coordinate system of its parent. 
            //This matrix is computed from the axis information above, and stored in its transposed version to match OpenGL¡¦s convention.
            quaternion_asf = math::Quaternion_t(math::ToRotMat(skeleton()->bone_ptr(index[i])->rot_parent_current).transpose());
            quaternion_angulr = math::ComputeQuaternionXyz(radian_vector.x(), radian_vector.y(), radian_vector.z());

            quaternion_rotation *= quaternion_asf * quaternion_angulr;
        }

        quaternion_vector.w() = 0;
        quaternion_vector.vec() = skeleton_->bone_ptr(i)->dir * skeleton()->bone_ptr(i)->length; //unit vector
        quaternion_vector = quaternion_rotation * quaternion_vector * quaternion_rotation.inverse();

        next_pose.set_end_pos(quaternion_vector.vec() + last_pose.end_pos());
        output.push_back(next_pose);
    }

    return output;
    //return helper_fk_->ComputeSkeletonPose(joint_spatial_pos);
}

// protected func.

// private func.

} // namespace kinematics {
