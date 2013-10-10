/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AdaptiveWDLSSolver.hpp"

using namespace KDL;
using namespace cart_ctrl_wdls;

AdaptiveWDLSSolver::AdaptiveWDLSSolver(std::string const& name)
    : AdaptiveWDLSSolverBase(name),
      jac_solver_(0),
      max_manipulability_(0){
}

AdaptiveWDLSSolver::AdaptiveWDLSSolver(std::string const& name, RTT::ExecutionEngine* engine)
    : AdaptiveWDLSSolverBase(name, engine),
      jac_solver_(0),
      max_manipulability_(0){
}

bool AdaptiveWDLSSolver::configureHook(){

    if (! AdaptiveWDLSSolverBase::configureHook())
        return false;

    jac_solver_ = new KDL::ChainJntToJacSolver(chain_);
    jacobian_.resize(no_joints_);
    lambda_max_ = _lambda.get();

    return true;
}

bool AdaptiveWDLSSolver::startHook(){
    if (! AdaptiveWDLSSolverBase::startHook())
        return false;
    return true;
}

void AdaptiveWDLSSolver::updateHook(){

    double m = manipulability(joint_status_kdl_.q);
    _manipulability.write(m);

    //Update maximum manipulability if a higher value is found in workspace
    if(m > max_manipulability_)
        max_manipulability_ = m;

    lambda_ = lambda_max_ * ( 1 - m / max_manipulability_) * ( 1 - m / max_manipulability_ );
    vel_wdls_solver_->setLambda(lambda_);
    _cur_lambda.write(lambda_);

    //Important: Update Hook of base class has to come after Manipulability computation
    AdaptiveWDLSSolverBase::updateHook();
}

void AdaptiveWDLSSolver::cleanupHook(){
    AdaptiveWDLSSolverBase::cleanupHook();
    if(jac_solver_)
        delete jac_solver_;
}

double AdaptiveWDLSSolver::manipulability(const KDL::JntArray& joint_position){
    jac_solver_->JntToJac(joint_position, jacobian_);
    eigen_jac = jacobian_.data;
    eigen_jac_transp = eigen_jac.transpose();
    prod = eigen_jac * eigen_jac_transp;
    return sqrt( prod.determinant());

}
