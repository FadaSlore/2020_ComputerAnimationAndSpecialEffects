#include "kinematics_inverse_jacobian_ik_solver.h"
#include <limits>
#include <queue>
#include <stack>
#include "console_log.h"
#include "math_utils.h"
#include "math_linear_system_solver.h"
#include "acclaim_skeleton.h"
#include "acclaim_motion.h"
#include "kinematics_forward_solver.h"
#include "kinematics_pose.h"

namespace kinematics {

InverseJacobianIkSolver::InverseJacobianIkSolver()
    :skeleton_(nullptr),
    fk_solver_(new ForwardSolver),
    step_(double{0.0}),
    distance_epsilon_(double{0.0}),
    max_iteration_num_(0),
    linear_system_solver_(nullptr)
{
}

InverseJacobianIkSolver::~InverseJacobianIkSolver()
{
}

void InverseJacobianIkSolver::Configure(
        const std::shared_ptr<acclaim::Skeleton> &skeleton,
        const std::shared_ptr<math::LinearSystemSolver> &linear_system_solver,
        const double step,
        const double distance_epsilon,
        const int32_t max_iteration_num
        )
{
    skeleton_ = skeleton;
    fk_solver_->set_skeleton(skeleton_);
    fk_solver_->ConstructArticPath();

    linear_system_solver_ = linear_system_solver;

    step_ = step;
    distance_epsilon_ = distance_epsilon;
    max_iteration_num_ = max_iteration_num;
}

math::Vector6dColl_t InverseJacobianIkSolver::Solve(
    const math::Vector3d_t& target_pos,
    const int32_t start_bone_idx,
    const int32_t end_bone_idx,
    const math::Vector6dColl_t& original_whole_body_joint_pos6d
)
{//TO DO
    /* Configuration
    skeleton
        std::shared_ptr<acclaim::Skeleton>
        the loaded skeleton, applied ot forward kinematics
    linear_system_solver
        std::shared_ptr<math::LinearSystemSolver>
        solver of angular velocity, applied to solve inverse Jacobian
    step
        double
        linearization step t
        PPT p.43
    distance_epsilon
        double
        the desired tolerance of the distance between the target position & end-effector
    max_iteration_num
        int32_t
        the maximum allowable iterations to solve IK

    fk_solver
        std::unique_ptr<ForwardSolver>
            std::shared_ptr<acclaim::Skeleton> skeleton_;
            std::shared_ptr<acclaim::Motion> motion_;
            std::unique_ptr<ArticIdxColl_t> artic_path_;
            std::unique_ptr<helper::ForwardKinematics> helper_fk_;
        function:
            void set_skeleton(const std::shared_ptr<acclaim::Skeleton> &skeleton);
            void set_motion(const std::shared_ptr<acclaim::Motion> &motion);
            void ConstructArticPath();
            PoseColl_t ComputeSkeletonPose(const int32_t frame_idx);
            PoseColl_t ComputeSkeletonPose(const math::Vector6dColl_t &joint_spaital_pos);
    */

    /* input parameters
    target_pos
        The target position of end-effector
    start_bone_idx/end_bone_idx
        Specify the bones used to reach target position
        Default value is rhumerus/rhand
        Start and end index used in IK process
    original_whole_body_joint_pos6d
        Motion data of the reference pose
    */

    //Use pseudoinverse Method

     /*
     math::VectorNd_t PseudoinverseSolver::Solve(
        const math::MatrixN_t &coef_mat,
        const math::VectorNd_t &desired_vector
        ) const
     Input:
        coef_mat
            J(theta)
        desired_vector
            V
     Output:
        angular velocity = J^(-1)*V
    */




    // original_whole_body_joint_pos6d is const
    math::Vector6dColl_t new_whole_body_joint_pos6d = original_whole_body_joint_pos6d;

    // Jacobian: 3*N, N: total number of DOFs
    // **const** Bone *Skeleton::bone_ptr(const int32_t bone_idx) const
    int32_t N = 0, index = start_bone_idx;

    //vector for index_save
    std::vector<int32_t> index_save;
    /*
    std::queue<int32_t> child_index;
    std::stack<int32_t> sibling_index;
    child_index.push(index);
    while (!child_index.empty() || !sibling_index.empty())
    {
        if (child_index.empty())
        {
            index = sibling_index.top();
            sibling_index.pop();
            child_index.push(index);
        }
        else
        {
            index = child_index.front();
            child_index.pop();
            index_save.push_back(index);
            if (index == end_bone_idx)
                break;

            if (skeleton_->bone_ptr(index)->child != nullptr)
                child_index.push(skeleton_->bone_ptr(index)->child->idx);
            if (skeleton_->bone_ptr(index)->sibling != nullptr)
                sibling_index.push(skeleton_->bone_ptr(index)->sibling->idx);

        }
    }
    */
    index = end_bone_idx;
    while (skeleton_->bone_ptr(index)->parent != nullptr)
    {
        index_save.push_back(index);
        if (index == start_bone_idx)
            break;
        index = skeleton_->bone_ptr(index)->parent->idx;
    }
    /*
    printf("\n\nvector:\n");
    for (int32_t i = 0; i < index_save.size(); i++)
        printf("%d ", index_save.at(i));
    printf("\n");
    system("pause");
    */
    
    for (int32_t i = 0; i < index_save.size(); i++)
        N += skeleton_->bone_ptr(index_save.at(i))->dof;
    math::MatrixN_t Jacobian(3, N);


    kinematics::PoseColl_t pose;

    // update iteration
    math::Vector3d_t last_end_position;
    math::Vector3d_t end_to_target;

    // caculate Jacobian matrix: 
    //  partial P / partial theta_i = a_i cross ( p - r_i )
    math::Vector3d_t a_i, r_i_to_p;
    math::RotMat3d_t rotation_i;
    math::Vector3d_t column_1(1, 0, 0), column_2(0, 1, 0), column_3(0, 0, 1);

    // update whole body joint ( new_whole_body_joint_pos6d )
    math::VectorNd_t angular_velocity;
    math::VectorNd_t temp;

    for (int32_t i = 0; i < max_iteration_num_; i++)
    {

        pose = fk_solver_->ComputeSkeletonPose(new_whole_body_joint_pos6d);
        last_end_position = pose[end_bone_idx].end_pos();

        // distance_epsilon_
        end_to_target = target_pos - last_end_position;
        if (sqrt(end_to_target.x() * end_to_target.x() + end_to_target.y() * end_to_target.y() + end_to_target.z() * end_to_target.z()) < distance_epsilon_)
            break;
        //printf("%f\n", distance_epsilon_);

        // Jacobian: 
        //  partial P / partial theta_i = a_i cross ( p - r_i )
        index = 0;
        for (int32_t k = 0; k < index_save.size(); k++)
        {
            r_i_to_p = target_pos - pose[index_save.at(k)].start_pos();
            rotation_i = pose[index_save.at(k)].rotation();

            // 2-DOFs or 3-DOFs joint -> 2 or 3 1-DOF joints
            if (skeleton_->bone_ptr(index_save.at(k))->dofx != 0)
            {
                a_i = (rotation_i * column_1).normalized();
                Jacobian.col(index++) = a_i.cross(r_i_to_p);
            }
            if (skeleton_->bone_ptr(index_save.at(k))->dofy != 0)
            {
                a_i = (rotation_i * column_2).normalized();
                Jacobian.col(index++) = a_i.cross(r_i_to_p);
            }
            if (skeleton_->bone_ptr(index_save.at(k))->dofz != 0)
            {
                a_i = (rotation_i * column_3).normalized();
                Jacobian.col(index++) = a_i.cross(r_i_to_p);
            }
        }

        /* theta_{k+1} = theta_k + delta_t * (angular velocity)
        angular velocity:
            math::VectorNd_t PseudoinverseSolver::Solve(J, V)
        */
        angular_velocity = linear_system_solver_->Solve(Jacobian, target_pos - last_end_position);
        index = 0;
        for (int32_t k = 0; k < index_save.size(); k++)
        {
            //temp = original_whole_body_joint_pos6d[index].angular_vector();
            temp = new_whole_body_joint_pos6d[index_save.at(k)].angular_vector();
            if (skeleton_->bone_ptr(index_save.at(k))->dofx != 0)
                temp.x() += step_ * angular_velocity[index++];
            if (skeleton_->bone_ptr(index_save.at(k))->dofy != 0)
                temp.y() += step_ * angular_velocity[index++];
            if (skeleton_->bone_ptr(index_save.at(k))->dofz != 0)
                temp.z() += step_ * angular_velocity[index++];

            new_whole_body_joint_pos6d[index_save.at(k)].set_angular_vector(temp);
        }
    }

    return new_whole_body_joint_pos6d;
    //return original_whole_body_joint_pos6d;
}

} // namespace kinematics {

