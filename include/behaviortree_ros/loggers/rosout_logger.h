#ifndef BT_ROSOUT_LOGGER_H
#define BT_ROSOUT_LOGGER_H

#include <behaviortree_cpp_v3/loggers/abstract_logger.h>
#include <ros/console.h>

namespace BT
{

class RosoutLogger : public StatusChangeLogger
{
    static std::atomic<bool> ref_count;

  public:
    RosoutLogger(TreeNode* root_node,
                 ros::console::Level verbosity_level = ros::console::Level::Info);

    ros::console::Level getLevel() const;

    // Accepts only Info and Debug
    void setLevel(ros::console::Level level);

    ~RosoutLogger() override;

    virtual void callback(Duration timestamp,
                          const TreeNode& node,
                          NodeStatus prev_status,
                          NodeStatus status) override;

    virtual void flush() override;

private:
    ros::console::Level _level;
};

}   // end namespace

#endif   //BT_ROSOUT_LOGGER_H
