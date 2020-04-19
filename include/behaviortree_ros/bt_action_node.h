// Copyright (c) 2019 Samsung Research America
// Copyright (c) 2020 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREE_BT_ACTION_NODE_HPP_
#define BEHAVIOR_TREE_BT_ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

namespace BT
{

/** Helper Node to call an actionlib::SimpleActionClient<>
 * inside a BT::ActionNode.
 *
 * Note that the user must implement the methods:
 *
 *  - sendGoal
 *  - onResult
 *  - onFailedRequest
 *  - halt (optionally)
 *
 */
template<class ActionT>
class RosActionNode : public BT::ActionNodeBase
{
protected:

  RosActionNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
  BT::ActionNodeBase(name, conf), node_(nh)
  {
    const std::string server_name = getInput<std::string>("server_name").value();
    action_client_ = std::make_shared<ActionClientType>( node_, server_name, true );
  }

public:

  using BaseClass  = RosActionNode<ActionT>;
  using ActionClientType = actionlib::SimpleActionClient<ActionT>;
  using ActionType = ActionT;
  using GoalType   = typename ActionT::_action_goal_type::_goal_type;
  using ResultType = typename ActionT::_action_result_type::_result_type;

  RosActionNode() = delete;

  virtual ~RosActionNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosAction<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("server_name", "name of the Action Server"),
      InputPort<unsigned>("timeout", 500, "timeout to connect (milliseconds)")
      };
  }

  /// Method called when the Action makes a transition from IDLE to RUNNING.
  /// If it return false, the entire action is immediately aborted, it returns
  /// FAILURE and no request is sent to the server.
  virtual bool sendGoal(GoalType& goal) = 0;

  /// Method (to be implemented by the user) to receive the reply.
  /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
  virtual NodeStatus onResult( const ResultType& res) = 0;

  enum FailureCause{
    MISSING_SERVER = 0,
    ABORTED_BY_SERVER = 1,
    REJECTED_BY_SERVER = 2
  };

  /// Called when a service call failed. Can be overriden by the user.
  virtual NodeStatus onFailedRequest(FailureCause failure)
  {
    return NodeStatus::FAILURE;
  }

  /// If you override this method, you MUST call this implementation invoking:
  ///
  ///    BaseClass::halt()
  ///
  virtual void halt() override
  {
    if( status() == NodeStatus::RUNNING )
    {
      action_client_->cancelGoal();
    }
    setStatus(NodeStatus::IDLE);
  }

protected:

  std::shared_ptr<ActionClientType> action_client_;

  ros::NodeHandle& node_;

  BT::NodeStatus tick() override
  {
    unsigned msec = getInput<unsigned>("timeout").value();
    ros::Duration timeout(static_cast<double>(msec) * 1e-3);

    bool connected = action_client_->waitForServer(timeout);
    if( !connected ){
      return onFailedRequest(MISSING_SERVER);
    }

    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      GoalType goal;
      bool valid_goal = sendGoal(goal);
      if( !valid_goal )
      {
        return NodeStatus::FAILURE;
      }
      action_client_->sendGoal(goal);
    }

    // RUNNING
    auto action_state = action_client_->getState();

    // Please refer to these states

    if( action_state == actionlib::SimpleClientGoalState::PENDING ||
        action_state == actionlib::SimpleClientGoalState::ACTIVE )
    {
      return NodeStatus::RUNNING;
    }
    else if( action_state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return onResult( *action_client_->getResult() );
    }
    else if( action_state == actionlib::SimpleClientGoalState::ABORTED)
    {
      return onFailedRequest( ABORTED_BY_SERVER );
    }
    else if( action_state == actionlib::SimpleClientGoalState::REJECTED)
    {
      return onFailedRequest( REJECTED_BY_SERVER );
    }
    else
    {
      // FIXME: is there any other valid state we should consider?
      throw std::logic_error("Unexpected state in RosActionNode::tick()");
    }
  }
};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosAction(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = RosActionNode< typename DerivedT::ActionType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );
  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_ACTION_NODE_HPP_
