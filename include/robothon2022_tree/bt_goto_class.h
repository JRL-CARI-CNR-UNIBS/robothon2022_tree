#ifndef BT_GOTO_CLASS_H
#define BT_GOTO_CLASS_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <manipulation_msgs/GoToAction.h>
#include <actionlib/client/simple_action_client.h>


class GoToActionNode : public BT::SyncActionNode
{
public:
    GoToActionNode(const std::string& name);

    std::vector<std::string> skillNames(const std::string& action_skill_name);
    BT::NodeStatus tick() override;

    template<typename T> bool getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value);

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_exec_clnt_;
    std::shared_ptr<actionlib::SimpleActionClient<manipulation_msgs::GoToAction>> go_to_action_;
    std::string param_ns_ = "exec_params";
};

template<typename T>
inline bool GoToActionNode::getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+skill_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
        ROS_ERROR("%s not set", param_str.c_str());
        return false;
    }
    return true;
}


#endif // BT_GOTO_CLASS_H
