// NOLINT (legal/copyright)

#include "MyBT.hpp"

namespace cascade_hfsm
{
MyBT::MyBT()
: CascadeLifecycleNode("MyBT"), state_(INITIAL), myBaseId_("MyBT")
{
  declare_parameter("frequency");

  state_ts_ = now();
  state_pub_ = create_publisher<std_msgs::msg::String>("/" + myBaseId_ + "/state", 1);
}

MyBT::~MyBT()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyBT::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  deactivateAllDeps();

  state_ = INITIAL;
  state_ts_ = now();

  Initial_activateDeps();
  Initial_code_once();


  double frequency = 5.0;
  get_parameter_or<double>("frequency", frequency, 5.0);

  loop_timer_ = create_wall_timer(
    std::chrono::duration<double, std::ratio<1>>(1.0 / frequency),
    std::bind(&MyBT::tick, this));

  state_pub_->on_activate();
  return CascadeLifecycleNode::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyBT::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  loop_timer_ = nullptr;

  return CascadeLifecycleNode::on_deactivate(previous_state);
}

void MyBT::tick()
{
  std_msgs::msg::String msg;

  switch (state_) {
    case STATE2:
      State2_code_iterative();

      msg.data = "State2";
      state_pub_->publish(msg);

      if (State2_2_Initial()) {
        deactivateAllDeps();

        state_ = INITIAL;
        state_ts_ = now();

        Initial_activateDeps();
        Initial_code_once();
      }
      break;
    case INITIAL:
      Initial_code_iterative();

      msg.data = "Initial";
      state_pub_->publish(msg);

      if (Initial_2_State1()) {
        deactivateAllDeps();

        state_ = STATE1;
        state_ts_ = now();

        State1_activateDeps();
        State1_code_once();
      }
      break;
    case STATE1:
      State1_code_iterative();

      msg.data = "State1";
      state_pub_->publish(msg);

      if (State1_2_State2()) {
        deactivateAllDeps();

        state_ = STATE2;
        state_ts_ = now();

        State2_activateDeps();
        State2_code_once();
      }
      break;
  }
}

void
MyBT::deactivateAllDeps()
{
}

void
MyBT::State2_activateDeps()
{
}
void
MyBT::Initial_activateDeps()
{
}
void
MyBT::State1_activateDeps()
{
}


}  // namespace cascade_hfsm
