#include "behaviortree_ros/loggers/ros_topic_logger.h"

namespace BT
{
std::atomic<bool> RosTopicLogger::ref_count(false);

RosTopicLogger::RosTopicLogger(TreeNode* root_node, ros::NodeHandle nh, const std::string topic_name) :
    StatusChangeLogger(root_node),
    nh_(nh),
    topic_name_(topic_name)
{
    bool expected = false;
    if (!ref_count.compare_exchange_strong(expected, true))
    {
        throw std::logic_error("Only a single instance of RosTopicLogger shall be created");
    }
    status_change_pub_ = nh_.advertise<behaviortree_ros::StatusChangeLog>(topic_name_, 5);

}



RosTopicLogger::~RosTopicLogger()
{
    ref_count.store(false);
}

void RosTopicLogger::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                             NodeStatus status)
{

    behaviortree_ros::StatusChange event;

    // BT timestamps are a duration since the epoch. Need to convert to a time_point
    // before converting to a msg.
    
    uint32_t sec = std::chrono::duration_cast<std::chrono::seconds>(timestamp).count();
    auto remainder = timestamp - std::chrono::duration_cast<std::chrono::seconds>(timestamp);
    uint32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(remainder).count() ;
    event.timestamp = ros::Time(sec, nsec);
    event.uid = node.UID();
    event.name = node.name();
    event.prev_status.value = static_cast<int8_t> (prev_status);
    event.status.value =  static_cast<int8_t> (status);
    event_log_.push_back(std::move(event));
    
}

void RosTopicLogger::flush()
{
    if (!event_log_.empty()){
        behaviortree_ros::StatusChangeLog log_msg;
        log_msg.state_changes = std::move(event_log_);
        status_change_pub_.publish(log_msg);
        event_log_.clear();
    }
}

}   // end namespace
