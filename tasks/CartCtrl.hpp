/* Generated from orogen/lib/orogen/templates/CartCtrls/CartCtrl.hpp */

#ifndef CART_CTRL_HPP
#define CART_CTRL_HPP

#include "cart_ctrl_wdls/CartCtrlBase.hpp"
#include <kdl/frames.hpp>

namespace cart_ctrl_wdls {

class CartCtrl : public CartCtrlBase
{
    friend class CartCtrlBase;
protected:
    //Properties
    base::samples::RigidBodyState max_ctrl_out_;
    Eigen::VectorXd p_gain_;

    //Port Data
    base::samples::RigidBodyState command_from_port_, cartesian_status_from_port_;
    base::samples::RigidBodyState ctrl_error_to_port_, ctrl_out_to_port_;
    KDL::Twist ctrl_out_, ctrl_error_;
    KDL::Frame pose_kdl_, des_pose_kdl_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CartCtrl(std::string const& name = "cartesian_control::CartCtrl");
    CartCtrl(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartCtrl(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){CartCtrlBase::errorHook();}
    void stopHook(){CartCtrlBase::stopHook();}
    void cleanupHook(){CartCtrlBase::cleanupHook();}
};
}

#endif


