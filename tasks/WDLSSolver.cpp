/* Generated from orogen/lib/orogen/templates/WDLSSolvers/WDLSSolver.cpp */

#include <kdl/frames_io.hpp>
#include "WDLSSolver.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <base/logging.h>
#include <kdl_conversions/KDLConversions.hpp>

using namespace std;
using namespace cart_ctrl_wdls;

WDLSSolver::WDLSSolver(std::string const& name)
    : WDLSSolverBase(name),
      vel_wdls_solver_(0),
      pos_fk_solver_(0),
      vel_fk_solver_(0){
}

WDLSSolver::WDLSSolver(std::string const& name, RTT::ExecutionEngine* engine)
    : WDLSSolverBase(name, engine),
      vel_wdls_solver_(0),
      pos_fk_solver_(0),
      vel_fk_solver_(0){
}

bool WDLSSolver::configureHook(){

    if (! WDLSSolverBase::configureHook())
        return false;

    KDL::Tree tree;
    if(!kdl_parser::treeFromFile(_urdf_file.get(), tree)){
        LOG_ERROR("Unable to parse URDF file %s", _urdf_file.get().c_str());
        return false;
    }

    if(!tree.getChain(_root.get(), _tip.get(), chain_)){
        LOG_ERROR("Unable to retrieve kinematic chain between %s and %s from tree", _root.get().c_str(), _tip.get().c_str());
        return false;
    }

    //Init controller output and status vector
    no_joints_ = chain_.getNrOfJoints();

    solver_output_to_port_.resize(no_joints_);
    joint_status_kdl_.resize(no_joints_);
    solver_output_kdl_.resize(no_joints_);

    //Extract joint names from kinematic chain and store them in ctrl_output
    int joint_idx = 0;
    for(uint i = 0; i < chain_.getNrOfSegments(); i++){
        int type = chain_.getSegment(i).getJoint().getType();
        if( type != KDL::Joint::None){
            solver_output_to_port_.elements[joint_idx].speed = 0; //controller shall send velocity output; init with 0
            solver_output_to_port_.names[joint_idx] = chain_.getSegment(i).getJoint().getName();
            joint_idx++;
        }
    }

    //Init solvers
    pos_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
    vel_wdls_solver_ = new KDL::ChainIkSolverVel_wdls(chain_, _epsilon.get());
    vel_fk_solver_ = new KDL::ChainFkSolverVel_recursive(chain_);
    vel_wdls_solver_->setLambda(_lambda.get());

    if(_weights_ts.get().size() != 6)
        vel_wdls_solver_->setWeightTS(Eigen::VectorXd::Constant(6,1).asDiagonal());
    else
        vel_wdls_solver_->setWeightTS(_weights_ts.get().asDiagonal());

    if(_weights_js.get().size() != no_joints_)
        vel_wdls_solver_->setWeightJS(Eigen::VectorXd::Constant(no_joints_,1).asDiagonal());
    else
        vel_wdls_solver_->setWeightJS(_weights_js.get().asDiagonal());


    LOG_DEBUG_S<<"Initialized Cartesian Controller with following parameters: \n"<<endl;
    LOG_DEBUG_S<<"Root: "<<_root.get()<<endl;
    LOG_DEBUG_S<<"Tip: "<<_tip.get()<<endl;
    LOG_DEBUG_S<<"No of joints: "<<no_joints_<<endl;
    LOG_DEBUG_S<<"Joint names: "; for(uint i = 0; i < solver_output_to_port_.names.size(); i++) cout<<solver_output_to_port_.names[i]<<" "; cout<<endl;
    LOG_DEBUG_S<<"Task Space weights: "; for(uint i = 0; i < _weights_ts.get().size(); i++) cout<<_weights_ts.get()(i)<<" "; cout<<endl;
    LOG_DEBUG_S<<"Joint Space weights: "; for(uint i = 0; i < _weights_js.get().size(); i++) cout<<_weights_js.get()(i)<<" "; cout<<endl;

    return true;
}

bool WDLSSolver::startHook()
{
    joint_status_from_port_.clear();
    desired_twist_from_port_.invalidate();
    return WDLSSolverBase::startHook();
}

void WDLSSolver::updateHook(){
    WDLSSolverBase::updateHook();

    while(_joint_status.read(joint_status_from_port_) == RTT::NewData){
        for(uint i = 0; i < no_joints_; i++){
            double pos = joint_status_from_port_.getElementByName(solver_output_to_port_.names[i]).position;
            std::string name = solver_output_to_port_.names[i];
            if(base::isNaN(pos) || base::isInfinity(pos)){
                LOG_WARN("Received invalid joint sample for joint %s", name.c_str());
            }

            joint_status_kdl_.q(i) = joint_status_from_port_.getElementByName(solver_output_to_port_.names[i]).position;
            joint_status_kdl_.qdot(i) = joint_status_from_port_.getElementByName(solver_output_to_port_.names[i]).speed;
        }

        //Compute and write debug data
        pos_fk_solver_->JntToCart(joint_status_kdl_.q, pose_kdl_);
        vel_fk_solver_->JntToCart(joint_status_kdl_, frame_vel_kdl_);
        kdl_conversions::KDL2RigidBodyState(pose_kdl_, frame_vel_kdl_.deriv(), cartesian_status_to_port_);
        _cartesian_status.write(cartesian_status_to_port_);
    }

    while(_desired_twist.read(desired_twist_from_port_) == RTT::NewData){

        kdl_conversions::RigidBodyState2KDL(desired_twist_from_port_, desired_twist_);

        if(vel_wdls_solver_->CartToJnt(joint_status_kdl_.q, desired_twist_, solver_output_kdl_) < 0){
            LOG_ERROR("IK Computation failed");
            return;
        }

        for(uint i = 0; i < no_joints_; i++)
            solver_output_to_port_.elements[i].speed  = solver_output_kdl_(i);
        solver_output_to_port_.time = base::Time::now();
        _solver_output.write(solver_output_to_port_);

        LOG_DEBUG_S<<"Status: "<<endl;
        LOG_DEBUG_S<<"Position: "; for(uint i = 0; i < joint_status_from_port_.size(); i++) cout<<joint_status_from_port_.elements[i].position<<" "; cout<<endl;
        LOG_DEBUG_S<<"Velocity: "; for(uint i = 0; i < joint_status_from_port_.size(); i++) cout<<joint_status_from_port_.elements[i].speed<<" "; cout<<endl;
        LOG_DEBUG_S<<"Cur Pose: "<<pose_kdl_<<endl;
        LOG_DEBUG_S<<"Solver Input: "<<desired_twist_<<endl;
        LOG_DEBUG_S<<"Solver Output: "<<solver_output_kdl_.data<<endl;
        LOG_DEBUG_S<<"..................................................."<<endl<<endl;
    }
}

void WDLSSolver::cleanupHook(){
    WDLSSolverBase::cleanupHook();
    delete pos_fk_solver_;
    delete vel_fk_solver_;
    delete vel_wdls_solver_;
}

void WDLSSolver::stopHook()
{
    WDLSSolverBase::stopHook();
}
