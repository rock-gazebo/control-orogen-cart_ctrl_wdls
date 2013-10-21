/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ToPosConverter.hpp"
#include <base/logging.h>
//#define DEBUG

using namespace cart_ctrl_wdls;
using namespace std;

ToPosConverter::ToPosConverter(std::string const& name, TaskCore::TaskState initial_state)
    : ToPosConverterBase(name, initial_state){
}

ToPosConverter::ToPosConverter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ToPosConverterBase(name, engine, initial_state){
}

void ToPosConverter::updateHook(){
    ToPosConverterBase::updateHook();
    if(timestamp_.isNull())
        timestamp_ = base::Time::now();
#ifdef DEBUG
    cout<<"ToPosConverter::updateHook(): "<<endl;
#endif
    if(_command.read(command_in_) == RTT::NewData){

        if(command_out_.empty()){
            if(_joint_status.read(status_) == RTT::NoData)
                return;

            command_out_.resize(status_.size());
            command_out_.names = status_.names;
            for(uint i = 0; i < command_in_.size(); i++)
                command_out_.elements[i].position = status_.elements[i].position;
        }

        double diff = (base::Time::now() - timestamp_).toSeconds();
        for(uint i = 0; i < command_in_.size(); i++)
            command_out_.elements[i].position += command_in_.elements[i].speed * diff;

        command_out_.time = timestamp_;
        _command_out.write(command_out_);

#ifdef DEBUG
        cout<<"Difftime: "<<diff<<endl;
        cout<<"In: "; for(uint i = 0; i < command_in_.size(); i++) cout<<command_in_.elements[i].speed<<" "; cout<<endl;
        cout<<"Out: "; for(uint i = 0; i < command_in_.size(); i++) cout<<command_out_.elements[i].position<<" "; cout<<endl;
#endif
    }
    timestamp_ = base::Time::now();
}
