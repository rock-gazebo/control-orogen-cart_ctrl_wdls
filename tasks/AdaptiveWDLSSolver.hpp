/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CART_CTRL_WDLS_CARTCTRLADPATIVEWDLS_HPP
#define CART_CTRL_WDLS_CARTCTRLADPATIVEWDLS_HPP

#include "cart_ctrl_wdls/AdaptiveWDLSSolverBase.hpp"
#include <kdl/chainjnttojacsolver.hpp>

namespace cart_ctrl_wdls {
class AdaptiveWDLSSolver : public AdaptiveWDLSSolverBase
{
    friend class AdaptiveWDLSSolverBase;
protected:
    KDL::Jacobian jacobian_;
    KDL::ChainJntToJacSolver* jac_solver_;
    double max_manipulability_;
    double lambda_, lambda_max_;

    //Helpers
    Eigen::Matrix<double,6,Eigen::Dynamic> eigen_jac;
    Eigen::Matrix<double,Eigen::Dynamic,6> eigen_jac_transp;
    Eigen::Matrix<double,6,6> prod;

    double manipulability(const KDL::JntArray& joint_position);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    AdaptiveWDLSSolver(std::string const& name = "cartesian_control::AdaptiveWDLSSolver");
    AdaptiveWDLSSolver(std::string const& name, RTT::ExecutionEngine* engine);
    ~AdaptiveWDLSSolver(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){AdaptiveWDLSSolverBase::errorHook();}
    void stopHook(){AdaptiveWDLSSolverBase::stopHook();}
    void cleanupHook();
};
}

#endif

