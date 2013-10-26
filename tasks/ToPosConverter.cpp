/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ToPosConverter.hpp"
#include <base/logging.h>

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
    bool ok = ToPosConverterBase::configureHook();
    override_output_speed_ = _override_output_speed.get();
    write_speed_ = _write_speed.get();
    return ok;
}

bool ToPosConverter::startHook()
{
    bool ok = ToPosConverterBase::startHook();
    prev_timestamp_ = base::Time::now();
    status_.clear();
    command_in_.clear();
    command_out_.clear();
    return ok;
}

void ToPosConverter::updateHook(){
    ToPosConverterBase::updateHook();
    timestamp_ = base::Time::now();
    if(_joint_status.read(status_) == RTT::NoData){
        LOG_WARN("No sample on port");
        return;
    }
    while(_joint_status.read(status_) == RTT::NewData){
        continue;
    }

    while(_command.read(command_in_) == RTT::NewData)
    {
        LOG_DEBUG("Got Command to convert");
        if(command_out_.empty()){
            command_out_.resize(status_.size());
            command_out_.names = status_.names;
            for(uint i = 0; i < command_in_.size(); i++)
                command_out_.elements[i].position = status_.elements[i].position;
        }

        double diff = (timestamp_ - prev_timestamp_).toSeconds();
        for(uint i = 0; i < command_in_.size(); i++){
            command_out_.elements[i].position += command_in_.elements[i].speed * diff;
            if(write_speed_){
                if(!base::isUnset(override_output_speed_)){
                    LOG_DEBUG("Overriding speed for joint %s with %f", command_out_.names[i].c_str(), override_output_speed_);
                    command_out_.elements[i].speed = override_output_speed_;
                }
                else{
                    LOG_DEBUG("Setting speed for joint %s to %f", command_out_.names[i].c_str(), command_in_.elements[i].speed);
                    command_out_.elements[i].speed = command_in_.elements[i].speed;
                }
            }
        }

        command_out_.time = timestamp_;
        _command_out.write(command_out_);

        LOG_DEBUG_S << "Difftime: " <<diff<<endl;
        LOG_DEBUG_S <<"In: "; for(uint i = 0; i < command_in_.size(); i++) cout<<command_in_.elements[i].speed<<" "; cout<<endl;
        LOG_DEBUG_S <<"Out: "; for(uint i = 0; i < command_in_.size(); i++) cout<<command_out_.elements[i].position<<" "; cout<<endl;
    }
    prev_timestamp_ = timestamp_;
}


