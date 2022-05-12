#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <skills_executer/bt_skills_classes.h>
#include <robothon2022_tree/bt_goto_class.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tree_test");
    ros::NodeHandle nh("tree_test_server");

    ROS_INFO("Tree test start");

    BT::BehaviorTreeFactory factory;

    ROS_INFO("Factory created");

    factory.registerNodeType<SkillActionNode>("SkillActionNode");
    ROS_INFO("SkillActionNode registered");

    factory.registerNodeType<GoToActionNode>("GoToActionNode");
    ROS_INFO("GoToActionNode registered");

    BT::Tree tree = factory.createTreeFromFile("/home/gauss/projects/planning_ws/src/robothon2022_tree/tree/robothon2022_tree.xml");

    ROS_INFO("Tree created");

    tree.tickRoot();

    ROS_INFO("Tree finish");
}
