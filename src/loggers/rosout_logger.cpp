#include "behaviortree_ros/loggers/rosout_logger.h"

namespace BT
{
std::atomic<bool> RosoutLogger::ref_count(false);

RosoutLogger::RosoutLogger(TreeNode* root_node, ros::console::Level verbosity_level) :
    StatusChangeLogger(root_node),
    _level(verbosity_level)
{
    bool expected = false;
    if (!ref_count.compare_exchange_strong(expected, true))
    {
        throw std::logic_error("Only a single instance of RosoutLogger shall be created");
    }
}

ros::console::Level RosoutLogger::getLevel() const
{
    return _level;
}

void RosoutLogger::setLevel(ros::console::Level level)
{
    if( level != ros::console::Level::Debug &&
        level != ros::console::Level::Info )
    {
        throw std::invalid_argument("RosoutLogger::setLevel acepts only Debug or Info");
    }
    _level = level;
}

RosoutLogger::~RosoutLogger()
{
    ref_count.store(false);
}

void RosoutLogger::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                             NodeStatus status)
{
    constexpr const char* whitespaces = "                         ";
    const size_t ws_count = strlen(whitespaces)-1;

    const auto& node_name = node.name();

    switch( _level )
    {
    case ros::console::Level::Debug :
        ROS_DEBUG("[%s%s]: %s -> %s",  node_name.c_str(),
                  &whitespaces[std::min(ws_count, node_name.size())],
                  toStr(prev_status, true).c_str(),
                  toStr(status, true).c_str());
        break;

    case ros::console::Level::Info :
        ROS_INFO("[%s%s]: %s -> %s",  node_name.c_str(),
                  &whitespaces[std::min(ws_count, node_name.size())],
                  toStr(prev_status, true).c_str(),
                  toStr(status, true).c_srt());
        break;
    }
}

void RosoutLogger::flush()
{

}

}   // end namespace
