// Copyright 2021 El Grupo Del Flow
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

#include "MyBT.hpp"

namespace cascade_hfsm
{
MyBT::MyBT()
: CascadeLifecycleNode("MyBT"), state_(INITIAL), myBaseId_("MyBT") 
{
  declare_parameter("frequency");
}

MyBT::~MyBT()
{
}


void MyBT::init()
{
  std::cout << "uwu 1" << std::endl;
  auto aux_node = rclcpp::Node::make_shared("aux");
  std::cout << "uwu 2" << std::endl;

  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(aux_node);
  executor_client_ = std::make_shared<plansys2::ExecutorClient>(aux_node);

  state_ts_ = now();
  std::cout << "uwu 3" << std::endl;
  state_pub_ = create_publisher<std_msgs::msg::String>("/" + myBaseId_ + "/state", 1);
  std::cout << "uwu 4" << std::endl;


  problem_expert_->addInstance(plansys2::Instance{"r", "robot"});
  std::cout << "uwu 5" << std::endl;

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
    case INITIAL:
         Initial_code_iterative();
      std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
      // Set the goal for next state, and execute plan
      problem_expert_->setGoal(
        plansys2::Goal(
          "(and(explored cocina))"));

      if (executor_client_->start_plan_execution()) {
        state_ = STATE1;
      }
      break;

    case STATE1:
      {
        State1_code_iterative();
        std::cout << "BBBBBBBBBBBBBBBBBBBBBB" << std::endl;
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
          std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;

        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            std::cout << "Successful finished " << std::endl;


            // Set the goal for next state, and execute plan
            problem_expert_->setGoal(plansys2::Goal("(and(explored h1))"));

            if (executor_client_->start_plan_execution()) {
              state_ = STATE2;
            }
          } else {
            for (const auto & action_feedback : feedback.action_execution_status) {
              if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                  action_feedback.message_status << std::endl;
              }
            }
            executor_client_->start_plan_execution();  // replan and execute
          }
        }
      }
      break;

    case STATE2:
      {
        State2_code_iterative();
        std::cout << "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCC" << std::endl;
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
          std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;

        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            std::cout << "Successful finished " << std::endl;


            // Set the goal for next state, and execute plan
            problem_expert_->setGoal(plansys2::Goal("(and(robot_at r h1))"));

            if (executor_client_->start_plan_execution()) {
              state_ = STATE1;
            }
          } else {
            for (const auto & action_feedback : feedback.action_execution_status) {
              if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                  action_feedback.message_status << std::endl;
              }
            }
            executor_client_->start_plan_execution();  // replan and execute
          }
        }
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




