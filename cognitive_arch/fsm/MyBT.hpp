// NOLINT (legal/copyright) 

#ifndef MYBT_H_
#define MYBT_H_

#include <string>

#include "std_msgs/msg/string.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

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

  virtual void Cocina_code_iterative() {}
  virtual void Cocina_code_once() {}
  virtual void FINAL_code_iterative() {}
  virtual void FINAL_code_once() {}
  virtual void H2_code_iterative() {}
  virtual void H2_code_once() {}
  virtual void Initial_code_iterative() {}
  virtual void Initial_code_once() {}
  virtual void B1_code_iterative() {}
  virtual void B1_code_once() {}
  virtual void H1_code_iterative() {}
  virtual void H1_code_once() {}
  virtual void B2_code_iterative() {}
  virtual void B2_code_once() {}

  virtual bool Cocina_2_B1() {return false;}
  virtual bool B1_2_H1() {return false;}
  virtual bool Initial_2_Cocina() {return false;}
  virtual bool B2_2_H2() {return false;}
  virtual bool H1_2_B2() {return false;}
  virtual bool H2_2_FINAL() {return false;}


  void tick();

protected:
  rclcpp::Time state_ts_;

private:
  void deactivateAllDeps();
  void Cocina_activateDeps();
  void FINAL_activateDeps();
  void H2_activateDeps();
  void Initial_activateDeps();
  void B1_activateDeps();
  void H1_activateDeps();
  void B2_activateDeps();


  static const int COCINA = 0;
  static const int FINAL = 1;
  static const int H2 = 2;
  static const int INITIAL = 3;
  static const int B1 = 4;
  static const int H1 = 5;
  static const int B2 = 6;


  int state_;

  std::string myBaseId_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
};

}  // namespace cascade_hfsm

#endif  // MYBT_H_
