/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CART_CTRL_WDLS_TOPOSCONVERTER_TASK_HPP
#define CART_CTRL_WDLS_TOPOSCONVERTER_TASK_HPP

#include "cart_ctrl_wdls/ToPosConverterBase.hpp"
#include <base/commands/Joints.hpp>
#include <base/JointLimits.hpp>

namespace cart_ctrl_wdls {

class ToPosConverter : public ToPosConverterBase
{
    friend class ToPosConverterBase;
protected:

    base::commands::Joints command_in_, command_out_, status_, prev_command_out_;
    double override_output_speed_;
    bool write_speed_;
    base::Time timestamp_, prev_timestamp_;
    base::JointLimits limits_;
    double position_scale_;
public:
    ToPosConverter(std::string const& name = "cart_ctrl_wdls::ToPosConverter", TaskCore::TaskState initial_state = Stopped);
    ToPosConverter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);
    ~ToPosConverter(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){ToPosConverterBase::errorHook();}
    void stopHook(){ToPosConverterBase::stopHook();}
    void cleanupHook(){ToPosConverterBase::cleanupHook();}
};
}

#endif

