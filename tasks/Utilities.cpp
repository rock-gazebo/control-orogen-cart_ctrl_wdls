#include "Utilities.hpp"
#include <urdf_parser/urdf_parser.h>
#include <sdf/sdf.hh>

using namespace std;
using namespace cart_ctrl_wdls;
using namespace kdl_parser;

pair<string, ROBOT_MODEL_FORMAT> utilities::getRobotModelString(string const& urdf_model, string const& robot_model, ROBOT_MODEL_FORMAT robot_model_format)
{
    if (!urdf_model.empty())
        return getRobotModelString(urdf_model, ROBOT_MODEL_URDF);
    else
        return getRobotModelString(robot_model, robot_model_format);
}

base::JointLimits utilities::getJointLimitsFromURDF(string const& xml)
{
    urdf::ModelInterfaceSharedPtr urdf_model =
        urdf::parseURDF(xml);
    base::JointLimits limits;
    for (auto it : urdf_model->joints_) {
        shared_ptr<urdf::Joint> joint = it.second;

        if(joint->limits && joint->type != urdf::Joint::FIXED && !joint->mimic){
            base::JointLimitRange range;
            if (joint->type == urdf::Joint::CONTINUOUS) {
                range.max.position = M_PI;
                range.min.position = -M_PI;
            } else {
                range.max.position = joint->limits->upper;
                range.min.position = joint->limits->lower;
            }
            range.max.speed = joint->limits->velocity;
            range.min.speed = -joint->limits->velocity;
            range.max.effort = joint->limits->effort;
            range.min.effort = -joint->limits->effort;
            limits.names.push_back(it.first);
            limits.elements.push_back(range);
        }
    }
    return limits;
}

base::JointLimits utilities::getJointLimitsFromSDF(string const& xml)
{
    sdf::SDFPtr sdf(new sdf::SDF);

    if (!sdf::init(sdf))
        throw std::logic_error("unable to initialize sdf.");

    if (!sdf::readString(xml, sdf))
        throw std::logic_error("unable to interpret the given string as valid SDF");

    sdf::ElementPtr sdf_model = sdf->Root()->GetElement("model");
    if (!sdf_model)
        throw std::logic_error("the given SDF model is not a model (no toplevel model tag)");

    std::string model_name = sdf_model->Get<std::string>("name");
    base::JointLimits limits;

    sdf::ElementPtr jointElem = sdf_model->GetElement("joint");
    while (jointElem){
        std::string joint_name = model_name + "::" + jointElem->Get<std::string>("name");
        std::string joint_type = jointElem->Get<std::string>("type");

        base::JointLimitRange range;

        if (jointElem->HasElement("axis")){
            sdf::ElementPtr axisElem = jointElem->GetElement("axis");

            if (axisElem->HasElement("limit")){
                sdf::ElementPtr limitElem = axisElem->GetElement("limit");

                if (limitElem->HasElement("lower")){
                    range.min.position = limitElem->Get<double>("lower");
                }

                if (limitElem->HasElement("upper")){
                    range.max.position = limitElem->Get<double>("upper");
                }

                if (limitElem->HasElement("effort")){
                    double effort  = limitElem->Get<double>("effort");
                    range.min.effort = -effort;
                    range.max.effort = effort;
                }

                if (limitElem->HasElement("velocity")){
                    double speed  = limitElem->Get<double>("velocity");
                    range.min.speed = -speed;
                    range.max.speed = speed;
                }
            }

        }

        limits.names.push_back(joint_name);
        limits.elements.push_back(range);
        jointElem = jointElem->GetNextElement("joint");
    }
    return limits;
}

base::JointLimits utilities::getRobotModelJointLimits(
        std::string const& robot_model,
        kdl_parser::ROBOT_MODEL_FORMAT robot_model_format)
{
    if (robot_model_format == ROBOT_MODEL_SDF)
        return getJointLimitsFromSDF(robot_model);
    else if (robot_model_format == ROBOT_MODEL_URDF)
        return getJointLimitsFromURDF(robot_model);
    else
        throw std::invalid_argument("cannot use AUTO as format in getRobotModelJointLimits");
}

