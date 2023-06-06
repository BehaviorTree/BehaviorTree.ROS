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

#ifndef BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
#define BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <ros/service_client.h>

namespace BT
{

/**
 * Base Action to implement a ROS Service
 */
template<class ServiceT>
class RosAsyncServiceNode : public BT::ActionNodeBase
{
protected:

  RosAsyncServiceNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
   BT::ActionNodeBase(name, conf), node_(nh) { 
   }

  RosAsyncServiceNode(ros::NodeHandle& nh,const std::string& service_name, ros::Duration timeout, const std::string& name, const BT::NodeConfiguration & conf):
   BT::ActionNodeBase(name, conf), node_(nh) { 
   }

public:

  using BaseClass    = RosAsyncServiceNode<ServiceT>;
  using ServiceType  = ServiceT;
  using RequestType  = typename ServiceT::Request;
  using ResponseType = typename ServiceT::Response;

  RosAsyncServiceNode() = delete;

  virtual ~RosAsyncServiceNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosAction<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("service_name", "name of the ROS service"),
      InputPort<unsigned>("timeout", 100, "timeout to connect to server (milliseconds)")
      };
  }

  /// User must implement this method.
  virtual void sendRequest(RequestType& request) = 0;

  /// Method (to be implemented by the user) to receive the reply.
  /// User can decide which NodeStatus it will return (SUCCESS or FAILURE).
  virtual NodeStatus onResponse( const ResponseType& rep) = 0;

  enum FailureCause{
    MISSING_SERVER = 0,
    FAILED_CALL = 1
  };

  /// Called when a service call failed. Can be overriden by the user.
  virtual NodeStatus onFailedRequest(FailureCause failure)
  {
    return NodeStatus::FAILURE;
  }

protected:

  ros::ServiceClient service_client_;
  std::string service_name_;

  RequestType request_;
  ResponseType reply_;



  // The node that will be used for any ROS operations
  ros::NodeHandle& node_;

  ros::Duration timeout_;

  std::future< std::pair<bool,bool> > future_;

  std::pair<bool, bool> call_service(RequestType &request, ros::Duration timeout){
    bool connected = service_client_.waitForExistence(timeout);
    if( !connected ){
      return std::make_pair(false, false);
    }
    bool received = service_client_.call( request, reply_ );
    return std::make_pair(true, received);

  }

  BT::NodeStatus tick() override
  {
    if( !service_client_.isValid() ){
      
      service_client_ = node_.serviceClient<ServiceT>( service_name_ );
    }


    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      sendRequest(request_);

      future_ = std::async(std::launch::async, &RosAsyncServiceNode::call_service, this, std::ref(request_), timeout_);
      
    }

    if( future_.valid() ){
      if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
      }
      auto result = future_.get();
      if( !result.first ){
        return onFailedRequest(MISSING_SERVER);
      }
      if( !result.second ){
        return onFailedRequest(FAILED_CALL);
      }
      return onResponse(reply_);
    }
    throw BT::LogicError("future is not valid, this should never happen");
    return BT::NodeStatus::RUNNING;
  }

  //halt just resets the future so that result will be ignored, and next tick will start a new future
  virtual void halt() override{
    future_ = std::future< std::pair<bool,bool> >();
  }

};


/// Method to register the service into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosAsyncService(BT::BehaviorTreeFactory& factory,
                     const std::string& registration_ID,
                     ros::NodeHandle& node_handle, std::string service_name, ros::Duration timeout)
{
  NodeBuilder builder = [&node_handle, &service_name , &timeout](const std::string& name, const NodeConfiguration& config) {
    return std::make_unique<DerivedT>(node_handle,  service_name, timeout, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  //const auto& basic_ports = RosAsyncServiceNode< typename DerivedT::ServiceType>::providedPorts();
  //manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_SERVICE_NODE_HPP_
