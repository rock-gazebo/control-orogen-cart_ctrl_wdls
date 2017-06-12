#ifndef CART_CTRL_WDLS_UTILITIES_HPP
#define CART_CTRL_WDLS_UTILITIES_HPP

#include <kdl_parser/RobotModelFormat.hpp>
#include <base/JointLimits.hpp>

namespace cart_ctrl_wdls{
    namespace utilities{
        std::pair<std::string, kdl_parser::ROBOT_MODEL_FORMAT> getRobotModelString(
                std::string const& urdf_model,
                std::string const& robot_model,
                kdl_parser::ROBOT_MODEL_FORMAT robot_model_format);
        base::JointLimits getJointLimitsFromURDF(std::string const& xml);
        base::JointLimits getJointLimitsFromSDF(std::string const& xml);
        base::JointLimits getRobotModelJointLimits(
                std::string const& robot_model,
                kdl_parser::ROBOT_MODEL_FORMAT robot_model_format);
    }
}

#endif
