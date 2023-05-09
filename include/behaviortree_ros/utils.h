#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/control_node.h>
#include <behaviortree_ros/BehaviorTree.h>
#include <behaviortree_ros/NodeStatus.h>
namespace BT
{
    static behaviortree_ros::TreeNode convertToRosMsg(const TreeNode *node, int parent_id )
    {
        behaviortree_ros::TreeNode msg;
        
        msg.uid = node->UID();
        msg.instance_name = node->name();
        msg.registration_name = node->registrationName();
        msg.type = static_cast<int8_t>(node->type());
        msg.parent_uid = parent_id;
        msg.status.value = static_cast<int8_t>(node->status());
        
        
        return msg;
    }
    static void addNodesToMsg(const TreeNode *node, behaviortree_ros::BehaviorTree& msg, int parent_id )
    {

        msg.nodes.push_back(convertToRosMsg(node, parent_id));
        auto &current_msg = msg.nodes.back();
        

        if (auto control = dynamic_cast<const BT::ControlNode*>(node))
        {
            for (const auto& child : control->children())
            {
            addNodesToMsg(child, msg, node->UID());
            current_msg.children_uid.push_back(child->UID());
            }
        }
        else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(node))
        {
            if (decorator->child())
            {
            addNodesToMsg(decorator->child(), msg, node->UID());
            current_msg.children_uid.push_back(decorator->child()->UID());
            }
        }
    }

    static behaviortree_ros::BehaviorTree convertToRosMsg(const Tree& tree)
    {
        behaviortree_ros::BehaviorTree msg;
        msg.nodes.reserve(tree.nodes.size());
        msg.root_uid = tree.rootNode()->UID();
        
        addNodesToMsg(tree.rootNode(), msg, msg.root_uid);
        return msg;
    }

    

} // namespace BT