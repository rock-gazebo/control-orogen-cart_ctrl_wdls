/* Generated from orogen/lib/orogen/templates/WDLSSolvers/WDLSSolver.hpp */

#ifndef WDLS_SOLVER_HPP
#define WDLS_SOLVER_HPP

#include "cart_ctrl_wdls/WDLSSolverBase.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <base/commands/Joints.hpp>

namespace cart_ctrl_wdls {

class WDLSSolver : public WDLSSolverBase
{
    friend class WDLSSolverBase;
protected:
    uint no_joints_;

    //Solvers
    KDL::ChainIkSolverVel_wdls *vel_wdls_solver_;
    KDL::ChainFkSolverPos_recursive* pos_fk_solver_;
    KDL::ChainFkSolverVel_recursive* vel_fk_solver_;

    //Model
    KDL::Chain chain_;

    //Port Data
    base::samples::Joints joint_status_from_port_;
    base::samples::RigidBodyState desired_twist_from_port_;
    base::samples::RigidBodyState cartesian_status_to_port_;
    base::commands::Joints solver_output_to_port_;

    KDL::JntArrayVel joint_status_kdl_;
    KDL::Twist desired_twist_;
    KDL::Frame pose_kdl_;
    KDL::FrameVel frame_vel_kdl_;
    KDL::JntArray solver_output_kdl_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    WDLSSolver(std::string const& name = "cartesian_control::WDLSSolver");
    WDLSSolver(std::string const& name, RTT::ExecutionEngine* engine);
    ~WDLSSolver(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){ WDLSSolverBase::errorHook();}
    void stopHook();
    void cleanupHook();
};
}

#endif


