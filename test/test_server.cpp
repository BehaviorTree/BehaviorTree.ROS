#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behaviortree_ros/AddTwoInts.h>
#include <behaviortree_ros/FibonacciAction.h>

bool Add(behaviortree_ros::AddTwoInts::Request  &req,
         behaviortree_ros::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%d, y=%d", req.a, req.b);
  ROS_INFO("sending back response: [%d]", res.sum);
  return true;
}


// Code inspeired by
// http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
class FibonacciServer
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<behaviortree_ros::FibonacciAction> server_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  behaviortree_ros::FibonacciFeedback feedback_;
  behaviortree_ros::FibonacciResult result_;

  int call_number_;

public:

  FibonacciServer(std::string name) :
  server_(nh_, name, boost::bind(&FibonacciServer::executeCB, this, _1), false),
  action_name_(name)
  {
    server_.start();
    call_number_ = 0;
  }

  ~FibonacciServer(void)
  {
  }

  void executeCB(const behaviortree_ros::FibonacciGoalConstPtr &goal)
  {
    // calculate the result
    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);
    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i",
             action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
             
    for(int i=1; i<=goal->order; i++)
    {
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      server_.publishFeedback(feedback_);
    }

    bool preempted = false;

    // simulate a long period of time.
    // check that preempt has not been requested by the client
    int required_time = 500 ;// 0.5 sec;

    if( call_number_ % 2 == 0)
    {
      required_time = 100;
    }


    // check periodically for preemption
    while ( required_time > 0 )
    {
      if (server_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        server_.setPreempted();
        preempted = true;
        break;
      }
      ros::Duration take_break(0.010);
      take_break.sleep();
      required_time -= 10;
    }

    if(!preempted)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      server_.setSucceeded(result_);
    }
    else{
      ROS_WARN("%s: Preempted", action_name_.c_str());
    }
  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", Add);
  ROS_INFO("Ready to add two ints.");

  FibonacciServer fibonacci("fibonacci");
  ros::spin();

  return 0;
}
