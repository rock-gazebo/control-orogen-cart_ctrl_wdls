/* Generated from orogen/lib/orogen/templates/CartCtrls/CartCtrl.cpp */

#include <kdl/frames_io.hpp>
#include "CartCtrl.hpp"
#include <base/logging.h>
#include <kdl_conversions/KDLConversions.hpp>

//#define DEBUG

using namespace std;
using namespace cart_ctrl_wdls;

CartCtrl::CartCtrl(std::string const& name)
    : CartCtrlBase(name){
}

CartCtrl::CartCtrl(std::string const& name, RTT::ExecutionEngine* engine)
    : CartCtrlBase(name, engine){
}

bool CartCtrl::configureHook(){

    if (! CartCtrlBase::configureHook())
        return false;

    std::vector<double>p = _p_gain.get();
    p = _p_gain.get();
    if(p.size() != 6){
        LOG_WARN("P Gain is not set! Or malformatted. Will use ones.");
        p_gain_ = Eigen::VectorXd::Ones(6);
    }
    else{
        p_gain_ = Eigen::VectorXd::Ones(6);
        for(uint i=0; i<p.size(); i++){
            p_gain_(i) = p[i];
        }
        LOG_INFO_S << "P Gain is set to " << p_gain_ << endl;
    }

    max_ctrl_out_ = _max_ctrl_out.get();
    if(!max_ctrl_out_.hasValidVelocity())
        max_ctrl_out_.velocity = Eigen::VectorXd::Constant(3,1e10);
    if(!max_ctrl_out_.hasValidAngularVelocity())
        max_ctrl_out_.angular_velocity = Eigen::VectorXd::Constant(3,1e10);

    LOG_DEBUG_S<<"P gain: "; for(uint i = 0; i < p_gain_.size(); i++) cout<<p_gain_(i)<<" "; cout<<endl;
    LOG_DEBUG_S<<"Max Ctrl Output: "; for(uint i = 0; i < 3; i++) cout<<max_ctrl_out_.velocity(i)<<" ";
    for(uint i = 0; i < 3; i++)
        LOG_DEBUG_S<<max_ctrl_out_.angular_velocity(i)<<" ";

    return true;
}

bool CartCtrl::startHook(){
    if(!CartCtrlBase::startHook())
        return false;
    ctrl_out_ = KDL::Twist::Zero();
    command_from_port_ = base::samples::RigidBodyState();
    cartesian_status_from_port_ = base::samples::RigidBodyState();
    des_pose_kdl_.Identity();
    pose_kdl_.Identity();
    ctrl_error_to_port_.invalidate();
    return true;
}

void CartCtrl::updateHook(){
    CartCtrlBase::updateHook();

    base::Time timestamp = base::Time::now();

    while(_command.read(command_from_port_) == RTT::NewData){
        LOG_DEBUG("Received new command pos: %f, %f, %f", command_from_port_.position.x(), command_from_port_.position.y(), command_from_port_.position.z());
        state(FOLLOWING);
    }

    if(state() == FOLLOWING){
        while(_cartesian_status.read(cartesian_status_from_port_) == RTT::NewData){
            kdl_conversions::RigidBodyState2KDL(command_from_port_, des_pose_kdl_);
            kdl_conversions::RigidBodyState2KDL(cartesian_status_from_port_, pose_kdl_);

            ///////// Control Law: q = J^(-1) * (Kp*(X_des-X)), |q| <= max /////////////////////

            ctrl_error_ = KDL::diff(pose_kdl_, des_pose_kdl_);

            kdl_conversions::KDL2RigidBodyState(ctrl_error_, ctrl_error_to_port_);

            ctrl_error_to_port_.time = timestamp;
            _ctrl_error.write(ctrl_error_to_port_);

            for(uint i = 0; i < 6; i++ )
                ctrl_out_(i) = ctrl_error_(i) * p_gain_(i);

            //If one value exceeds max_ctrl_output, scale the joint values so that all joints reach the target at the same time
            double scale = 1;
            for (int i = 0; i < 3; i++){
                if(fabs(ctrl_out_(i)) > max_ctrl_out_.velocity(i)){
                    if(fabs(ctrl_out_(i) / max_ctrl_out_.velocity(i)) > scale)
                        scale = fabs(ctrl_out_(i)) / max_ctrl_out_.velocity(i);
                }
                if(fabs(ctrl_out_(i+3)) > max_ctrl_out_.angular_velocity(i)){
                    if(fabs(ctrl_out_(i+3) / max_ctrl_out_.angular_velocity(i)) > scale)
                        scale = fabs(ctrl_out_(i+3)) / max_ctrl_out_.angular_velocity(i);
                }
            }
            for(uint i = 0; i < 6; i++)
                ctrl_out_(i) = ctrl_out_(i) / scale;


            //////////////////////////////////////////////////////////////////////////////////////



            kdl_conversions::KDL2RigidBodyState(ctrl_out_, ctrl_out_to_port_);
            ctrl_out_to_port_.time = timestamp;
            _ctrl_out.write(ctrl_out_to_port_);

            LOG_DEBUG_S<<"Cur Pose: "<<pose_kdl_<<endl;
            LOG_DEBUG_S<<"Desired Pose: "<<des_pose_kdl_<<endl;
            LOG_DEBUG_S<<"Ctrl Error: "<<ctrl_error_<<endl;
            LOG_DEBUG_S<<"Ctrl Output: "<<ctrl_out_<<endl;
            LOG_DEBUG_S<<"...................................................\n"<<endl;
        }
    }
}

void CartCtrl::stopHook(){
    CartCtrlBase::stopHook();
}
