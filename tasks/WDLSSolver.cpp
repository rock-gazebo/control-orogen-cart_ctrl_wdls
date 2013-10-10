/* Generated from orogen/lib/orogen/templates/WDLSSolvers/WDLSSolver.cpp */

#include <kdl/frames_io.hpp>
#include "WDLSSolver.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <base/logging.h>
#include <kdl_conversions/KDLConversions.hpp>

//#define DEBUG

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


#ifdef DEBUG
    cout<<"Initialized Cartesian Controller with following parameters: "<<endl<<endl;
    cout<<"Root: "<<_root.get()<<endl;
    cout<<"Tip: "<<_tip.get()<<endl;
    cout<<"No of joints: "<<no_joints_<<endl;
    cout<<"Joint names: "; for(uint i = 0; i < solver_output_to_port_.names.size(); i++) cout<<solver_output_to_port_.names[i]<<" "; cout<<endl;
    cout<<"Task Space weights: "; for(uint i = 0; i < _weights_ts.get().size(); i++) cout<<_weights_ts.get()(i)<<" "; cout<<endl;
    cout<<"Joint Space weights: "; for(uint i = 0; i < _weights_js.get().size(); i++) cout<<_weights_js.get()(i)<<" "; cout<<endl;
#endif

    return true;
}

void WDLSSolver::updateHook(){
    WDLSSolverBase::updateHook();

    if(_joint_status.read(joint_status_from_port_) != RTT::NoData){
        for(uint i = 0; i < no_joints_; i++){
            joint_status_kdl_.q(i) = joint_status_from_port_.getElementByName(solver_output_to_port_.names[i]).position;
            joint_status_kdl_.qdot(i) = joint_status_from_port_.getElementByName(solver_output_to_port_.names[i]).speed;
        }

        //Compute and write debug data
        pos_fk_solver_->JntToCart(joint_status_kdl_.q, pose_kdl_);
        vel_fk_solver_->JntToCart(joint_status_kdl_, frame_vel_kdl_);
        kdl_conversions::KDL2RigidBodyState(pose_kdl_, frame_vel_kdl_.deriv(), cartesian_status_to_port_);
        _cartesian_status.write(cartesian_status_to_port_);

        _desired_twist.read(desired_twist_from_port_);
        kdl_conversions::RigidBodyState2KDL(desired_twist_from_port_, desired_twist_);

        if(vel_wdls_solver_->CartToJnt(joint_status_kdl_.q, desired_twist_, solver_output_kdl_) < 0){
            LOG_ERROR("IK Computation failed");
            return;
        }

        for(uint i = 0; i < no_joints_; i++)
            solver_output_to_port_.elements[i].speed  = solver_output_kdl_(i);
        _solver_output.write(solver_output_to_port_);


#ifdef DEBUG
        cout<<"Status: "<<endl;
        cout<<"Position: "; for(uint i = 0; i < joint_status_from_port_.size(); i++) cout<<joint_status_from_port_.elements[i].position<<" "; cout<<endl;
        cout<<"Velocity: "; for(uint i = 0; i < joint_status_from_port_.size(); i++) cout<<joint_status_from_port_.elements[i].speed<<" "; cout<<endl;
        cout<<"Cur Pose: "<<pose_kdl_<<endl;
        cout<<"Solver Input: "<<desired_twist_<<endl;
        cout<<"Solver Output: "<<solver_output_kdl_.data<<endl;
        cout<<"..................................................."<<endl<<endl;
#endif

    }
}

void WDLSSolver::cleanupHook(){
    WDLSSolverBase::cleanupHook();
    if(pos_fk_solver_)
        delete pos_fk_solver_;
    if(vel_fk_solver_)
        delete vel_fk_solver_;
    if(vel_wdls_solver_)
        delete vel_wdls_solver_;
}
