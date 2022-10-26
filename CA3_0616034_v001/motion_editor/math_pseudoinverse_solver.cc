#include "math_pseudoinverse_solver.h"

namespace math {

// public func.

const char *PseudoinverseSolver::tag()
{
    return "PseudoinverseSolver";
}

PseudoinverseSolver::PseudoinverseSolver()
    :LinearSystemSolver()
{
}

PseudoinverseSolver::~PseudoinverseSolver()
{
}

std::string PseudoinverseSolver::id() const
{
    return std::string(PseudoinverseSolver::tag());
}

math::VectorNd_t PseudoinverseSolver::Solve(
        const math::MatrixN_t &coef_mat,
        const math::VectorNd_t &desired_vector
        ) const
{//TO DO

    /* PPT p.43
    Input:
        coef_mat
            J(theta)
        desired_vector
            V
    Output:
        angular velocity = J^(-1)*V
    */
    
    /* Jacobian may not be invertible
    Pseudo Inverse of the Jacobian
        Jt = J's transpose matrix
    Output:
        angular velocity = (Jt*J)^(-1)*Jt*V
        angular velocity = Jt*(J*Jt)^(-1)*V

        Jt*J: N*N
        J*Jt: 3*3 (better)

    */

    //return math::VectorNd_t((coef_mat.transpose() * coef_mat).inverse() * coef_mat.transpose() * desired_vector);
    return math::VectorNd_t(coef_mat.transpose() * (coef_mat * coef_mat.transpose()).inverse() * desired_vector);
}

// protected func.

// private func.

} // namespace math {
