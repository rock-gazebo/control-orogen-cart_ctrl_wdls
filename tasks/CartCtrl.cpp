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

    p_gain_ = _p_gain.get();
    if(p_gain_.size() != 6)
        p_gain_ = Eigen::VectorXd::Ones(6);

    max_ctrl_out_ = _max_ctrl_out.get();
    if(!max_ctrl_out_.hasValidVelocity())
        max_ctrl_out_.velocity = Eigen::VectorXd::Constant(3,1e10);
    if(!max_ctrl_out_.hasValidAngularVelocity())
        max_ctrl_out_.angular_velocity = Eigen::VectorXd::Constant(3,1e10);

#ifdef DEBUG
    cout<<"P gain: "; for(uint i = 0; i < p_gain_.size(); i++) cout<<p_gain_(i)<<" "; cout<<endl;
    cout<<"Max Ctrl Output: "; for(uint i = 0; i < 3; i++) cout<<max_ctrl_out_.velocity(i)<<" ";
    for(uint i = 0; i < 3; i++) cout<<max_ctrl_out_.angular_velocity(i)<<" "; cout<<endl;
#endif

    return true;
}

bool CartCtrl::startHook(){
    if(!CartCtrlBase::startHook())
        return false;
    ctrl_out_ = KDL::Twist::Zero();
    return true;
}

void CartCtrl::updateHook(){
    CartCtrlBase::updateHook();

    if(_command.read(command_from_port_) != RTT::NoData &&
       _cartesian_status.read(cartesian_status_from_port_) != RTT::NoData){
        kdl_conversions::RigidBodyState2KDL(command_from_port_, des_pose_kdl_);
        kdl_conversions::RigidBodyState2KDL(cartesian_status_from_port_, pose_kdl_);

        ///////// Control Law: q = J^(-1) * (Kp*(X_des-X)), |q| <= max /////////////////////

        ctrl_error_ = KDL::diff(pose_kdl_, des_pose_kdl_);

        kdl_conversions::KDL2RigidBodyState(ctrl_error_, ctrl_error_to_port_);
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
    }

    kdl_conversions::KDL2RigidBodyState(ctrl_out_, ctrl_out_to_port_);
    _ctrl_out.write(ctrl_out_to_port_);

#ifdef DEBUG
        cout<<"Cur Pose: "<<pose_kdl_<<endl;
        cout<<"Desired Pose: "<<des_pose_kdl_<<endl;
        cout<<"Ctrl Error: "<<ctrl_error_<<endl;
        cout<<"Ctrl Output: "<<ctrl_out_<<endl;
        cout<<"..................................................."<<endl<<endl;
#endif
}
