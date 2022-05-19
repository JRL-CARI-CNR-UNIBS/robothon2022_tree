#include <robothon2022_tree/bt_goto_class.h>

GoToActionNode::GoToActionNode(const std::string &name) : BT::SyncActionNode(name, {})
{
    go_to_action_ = std::make_shared<actionlib::SimpleActionClient<manipulation_msgs::GoToAction>>("/go_to_location_server/manipulator/go_to", true);
    ROS_INFO("Wait for /go_to_location_server/manipulator/go_to");
    go_to_action_->waitForServer();
    ROS_INFO("Go_to_action_ ready");
}

std::vector<std::string> GoToActionNode::skillNames(const std::string& action_skill_name)
{
    std::size_t found = action_skill_name.find("/");
    std::vector<std::string> names;
    if (found==std::string::npos)
    {
        ROS_WARN("The name of action don't respect the standard");
        return names;
    }
    std::string action_name = action_skill_name;
    action_name.erase(action_name.begin()+found, action_name.end());
    std::string skill_name = action_skill_name;
    skill_name.erase(skill_name.begin(), skill_name.begin()+found+1);

    ROS_INFO("action_skill_name: %s",action_skill_name.c_str());
    ROS_INFO("action_name: %s",action_name.c_str());
    ROS_INFO("skill_name: %s",skill_name.c_str());

    names.push_back(action_name);
    names.push_back(skill_name);
    return names;
}


BT::NodeStatus GoToActionNode::tick()
{
    std::vector<std::string> names = skillNames(name());
    ROS_INFO("action_name: %s",names.at(0).c_str());
    ROS_INFO("skill_name: %s",names.at(1).c_str());
    if (names.empty())
    {
        ROS_WARN("The name of action don't respect the standard");
        return BT::NodeStatus::FAILURE;
    }
    manipulation_msgs::GoToGoal go_to_msg;
    manipulation_msgs::GoToResult go_to_result;
    if ( !getParam( names.at(0), names.at(1), "location_name", go_to_msg.location_names) )
    {
        ROS_ERROR("No location name");
        return BT::NodeStatus::FAILURE;
    }
    if ( !getParam( names.at(0), names.at(1), "property_exec_id", go_to_msg.property_exec_id) )
        go_to_msg.property_exec_id = "";
    if ( !getParam( names.at(0), names.at(1), "tool_id", go_to_msg.tool_id) )
        go_to_msg.tool_id = "";
    if ( !getParam( names.at(0), names.at(1), "to_loc_ctrl_id", go_to_msg.to_loc_ctrl_id) )
        go_to_msg.to_loc_ctrl_id = "trajectory_tracking";
    if ( !getParam( names.at(0), names.at(1), "job_exec_name", go_to_msg.job_exec_name) )
        go_to_msg.job_exec_name = "";

    ROS_INFO("Location_name: %s"   , go_to_msg.location_names.at(0).c_str());
    ROS_INFO("property_exec_id: %s", go_to_msg.property_exec_id.c_str());
    ROS_INFO("tool_id: %s"         , go_to_msg.tool_id.c_str());
    ROS_INFO("to_loc_ctrl_id: %s"  , go_to_msg.to_loc_ctrl_id.c_str());
    ROS_INFO("job_exec_name: %s"   , go_to_msg.job_exec_name.c_str());
    ROS_INFO("Ready to execute goto");

    go_to_action_->sendGoalAndWait(go_to_msg);
    ROS_INFO("Finisc movement");
    go_to_result = *go_to_action_->getResult();

    switch (go_to_result.result)
    {
    case manipulation_msgs::GoToResult::Success :
        ROS_INFO("Goto result: Success");
        break;
    case manipulation_msgs::GoToResult::SceneError :
        ROS_ERROR("Goto result: SceneError");
        break;
    case manipulation_msgs::GoToResult::NoAvailableTrajectories :
        ROS_ERROR("Goto result: NoAvailableTrajectories");
        break;
    case manipulation_msgs::GoToResult::TrajectoryError :
        ROS_ERROR("Goto result: TrajectoryError");
        break;
    case manipulation_msgs::GoToResult::NotInitialized :
        ROS_ERROR("Goto result: NotInitialized");
        break;
    case manipulation_msgs::GoToResult::ToolError :
        ROS_ERROR("Goto result: ToolError");
        break;
    case manipulation_msgs::GoToResult::ControllerError :
        ROS_ERROR("Goto result: ControllerError");
        break;
    }

    if (go_to_result.result < 0)
    {
        ROS_ERROR("[Unable to go to -> location name = %s", go_to_msg.location_names.at(0).c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}
