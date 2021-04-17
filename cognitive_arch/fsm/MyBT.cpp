// NOLINT (legal/copyright)

#include "MyBT.hpp"

namespace cascade_hfsm
{
MyBT::MyBT()
: CascadeLifecycleNode("MyBT"), state_(INITIAL), myBaseId_("MyBT")
{
  declare_parameter("frequency");
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
  executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());

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

void define_problem() {
    problem_expert_->addInstance(plansys2::Instance{"r", "robot"});

    problem_expert_->addInstance(plansys2::Instance{"salon", "room"});
    problem_expert_->addInstance(plansys2::Instance{"cocina", "room"});
    problem_expert_->addInstance(plansys2::Instance{"h1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"h2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"b1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"b2", "room"});

    problem_expert_->addInstance(plansys2::Instance{"pasillo", "corridor"});

    problem_expert_->addInstance(plansys2::Instance{"o1", "object"});

    problem_expert_->addInstance(plansys2::Instance{"z1", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"z2", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"z3", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"z4", "zone"});
    problem_expert_->addInstance(plansys2::Instance{"z5", "zone"});


    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_available r)"));
    problem_expert_->addPredicate(plansys2::Predicate("(not_robot_at_zone r)"));
    problem_expert_->addPredicate(plansys2::Predicate("(free r)"));

    problem_expert_->addPredicate(plansys2::Predicate("(object_at o1 z1)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected salon pasillo)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected pasillo salon)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected h1 pasillo)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected pasillo h1)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected b1 pasillo)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected pasillo b1)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected b2 pasillo)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected pasillo b2)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected salon cocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected cocina salon)"));

    problem_expert_->addPredicate(plansys2::Predicate("(zone_at z1 salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_at z2 salon)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_at z3 cocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_at z4 cocina)"));
    problem_expert_->addPredicate(plansys2::Predicate("(zone_at z5 h1)"));
}

void MyBT::tick()
{
  std_msgs::msg::String msg;
  
  define_problem();
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

        problem_expert_->setGoal(
        plansys2::Goal(
          "(and(object_at o1 h1))"));
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
        problem_expert_->setGoal(
        plansys2::Goal(
          "(and(robot_at r kitchen))"));
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
        problem_expert_->setGoal(
        plansys2::Goal(
          "(and(object_at o1 kitchen))"));
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
