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
    case COCINA:
      Cocina_code_iterative();

      msg.data = "Cocina";
      state_pub_->publish(msg);

      if (Cocina_2_B1()) {
        deactivateAllDeps();

        state_ = B1;
        state_ts_ = now();

        B1_activateDeps();
        B1_code_once();
      }
      break;
    case FINAL:
      FINAL_code_iterative();

      msg.data = "FINAL";
      state_pub_->publish(msg);

      break;
    case H2:
      H2_code_iterative();

      msg.data = "H2";
      state_pub_->publish(msg);

      if (H2_2_FINAL()) {
        deactivateAllDeps();

        state_ = FINAL;
        state_ts_ = now();

        FINAL_activateDeps();
        FINAL_code_once();
      }
      break;
    case INITIAL:
      Initial_code_iterative();

      msg.data = "Initial";
      state_pub_->publish(msg);

      if (Initial_2_Cocina()) {
        deactivateAllDeps();

        state_ = COCINA;
        state_ts_ = now();

        Cocina_activateDeps();
        Cocina_code_once();
      }
      break;
    case B1:
      B1_code_iterative();

      msg.data = "B1";
      state_pub_->publish(msg);

      if (B1_2_H1()) {
        deactivateAllDeps();

        state_ = H1;
        state_ts_ = now();

        H1_activateDeps();
        H1_code_once();
      }
      break;
    case H1:
      H1_code_iterative();

      msg.data = "H1";
      state_pub_->publish(msg);

      if (H1_2_B2()) {
        deactivateAllDeps();

        state_ = B2;
        state_ts_ = now();

        B2_activateDeps();
        B2_code_once();
      }
      break;
    case B2:
      B2_code_iterative();

      msg.data = "B2";
      state_pub_->publish(msg);

      if (B2_2_H2()) {
        deactivateAllDeps();

        state_ = H2;
        state_ts_ = now();

        H2_activateDeps();
        H2_code_once();
      }
      break;
  }
}

void
MyBT::deactivateAllDeps()
{
}

void
MyBT::Cocina_activateDeps()
{
}
void
MyBT::FINAL_activateDeps()
{
}
void
MyBT::H2_activateDeps()
{
}
void
MyBT::Initial_activateDeps()
{
}
void
MyBT::B1_activateDeps()
{
}
void
MyBT::H1_activateDeps()
{
}
void
MyBT::B2_activateDeps()
{
}


}  // namespace cascade_hfsm
