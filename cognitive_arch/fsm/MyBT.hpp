// NOLINT (legal/copyright) 

#ifndef MYBT_H_
#define MYBT_H_

#include <string>

#include "std_msgs/msg/string.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace cascade_hfsm
{
class MyBT : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  MyBT();
  virtual ~MyBT();

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  virtual void State2_code_iterative() {}
  virtual void State2_code_once() {}
  virtual void Initial_code_iterative() {}
  virtual void Initial_code_once() {}
  virtual void State1_code_iterative() {}
  virtual void State1_code_once() {}

  virtual bool State1_2_State2() {return false;}
  virtual bool Initial_2_State1() {return false;}
  virtual bool State2_2_Initial() {return false;}


  void tick();

protected:
  rclcpp::Time state_ts_;

private:
  void deactivateAllDeps();
  void State2_activateDeps();
  void Initial_activateDeps();
  void State1_activateDeps();


  static const int STATE2 = 0;
  static const int INITIAL = 1;
  static const int STATE1 = 2;


  int state_;

  std::string myBaseId_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

}  // namespace cascade_hfsm

#endif  // MYBT_H_
