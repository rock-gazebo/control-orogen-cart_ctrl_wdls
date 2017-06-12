/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ToPosConverter.hpp"
#include <base-logging/Logging.hpp>
#include "Utilities.hpp"

using namespace cart_ctrl_wdls;
using namespace std;

ToPosConverter::ToPosConverter(std::string const& name, TaskCore::TaskState initial_state)
    : ToPosConverterBase(name, initial_state){
}

ToPosConverter::ToPosConverter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ToPosConverterBase(name, engine, initial_state){
}

bool ToPosConverter::configureHook()
{
    if (!ToPosConverterBase::configureHook())
        return false;

    override_output_speed_ = _override_output_speed.get();
    write_speed_ = _write_speed.get();
    position_scale_ = _position_scale.get();

    pair<string, kdl_parser::ROBOT_MODEL_FORMAT> robot_model =
        utilities::getRobotModelString(_urdf_file, _robot_model, _robot_model_format);
    limits_ = utilities::getRobotModelJointLimits(robot_model.first, robot_model.second);
    return true;
}

bool ToPosConverter::startHook()
{
    if (!ToPosConverterBase::startHook())
        return false;

    prev_timestamp_ = base::Time::now();
    status_.clear();
    command_in_.clear();
    command_out_.clear();
    return true;
}

void ToPosConverter::updateHook(){
    ToPosConverterBase::updateHook();
    timestamp_ = base::Time::now();

    while(_command.read(command_in_) == RTT::NewData)
    {
        if(_joint_status.read(status_) == RTT::NoData){
            LOG_WARN("No sample on joint state port");
            return;
        }
        while(_joint_status.read(status_) == RTT::NewData){
            continue;
        }

        if(command_out_.empty()){
            command_out_.resize(command_in_.size());
            command_out_.names = command_in_.names;
        }

        double diff = (timestamp_ - prev_timestamp_).toSeconds();

        for(uint i = 0; i < command_in_.size(); i++){
            std::string const& joint_name = command_in_.names[i];
            base::JointState const& cmd_in  = command_in_.elements[i];
            base::JointState& cmd_out = command_out_.elements[i];

            size_t idx;
            try{
                idx = status_.mapNameToIndex(joint_name);
            }
            catch (std::exception e){
                LOG_ERROR("Joint %s is in input command but not in joint status", joint_name.c_str());
                throw std::invalid_argument("Invalid joint state");
            }

            double prev_pos;
            if(_use_position_cmd_as_current.value() && !prev_command_out_.empty()) //Use previous command as current position
                prev_pos = prev_command_out_[i].position;
            else //Use real position from joint state
                prev_pos = status_[idx].position;

            double new_pos = prev_pos + cmd_in.speed * diff * position_scale_;

            base::JointLimitRange range = limits_[joint_name];
            double min_position = range.min.position;
            double max_position = range.max.position;

            if (!base::isUnset(min_position)) {
                if(new_pos < min_position){
                    LOG_INFO("Truncated joint %s to lower limit: %f", joint_name.c_str(), min_position);
                    new_pos = min_position;
                }
            }
            if (!base::isUnset(max_position)) {
                if(new_pos > max_position){
                    LOG_INFO("Truncated joint %s to upper limit: %f", joint_name.c_str(), max_position);
                    new_pos = max_position;
                }
            }

            cmd_out.position = new_pos;
            
            if(write_speed_){
                if(!base::isUnset(override_output_speed_)){
                    LOG_DEBUG("Overriding speed for joint %s with %f", joint_name.c_str(), override_output_speed_);
                    cmd_out.speed = override_output_speed_;
                }
                else{
                    LOG_DEBUG("Setting speed for joint %s to %f", joint_name.c_str(), cmd_in.speed);
                    cmd_out.speed = cmd_in.speed;
                }
            }
        }

        command_out_.time = timestamp_;
        _command_out.write(command_out_);
        prev_command_out_ = command_out_;
    }
    prev_timestamp_ = timestamp_;
}


