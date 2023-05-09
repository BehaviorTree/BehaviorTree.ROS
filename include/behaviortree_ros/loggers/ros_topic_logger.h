#pragma once

#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <ros/ros.h>
#include <behaviortree_ros/StatusChangeLog.h>
#include <behaviortree_ros/StatusChange.h>


namespace BT
{

class RosTopicLogger : public StatusChangeLogger
{
    static std::atomic<bool> ref_count;

  public:
    RosTopicLogger(TreeNode* root_node, ros::NodeHandle nh, const std::string topic_name = "behavior_tree_log");


    ~RosTopicLogger() override;

    virtual void callback(Duration timestamp,
                          const TreeNode& node,
                          NodeStatus prev_status,
                          NodeStatus status) override;

    virtual void flush() override;

private:
    ros::NodeHandle nh_;
    std::string topic_name_;
    ros::Publisher status_change_pub_;
    std::vector<behaviortree_ros::StatusChange> event_log_;
};

}   // end namespace