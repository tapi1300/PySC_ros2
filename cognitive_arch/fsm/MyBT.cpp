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

  problem_expert_->addPredicate(plansys2::Predicate("(connected h2 pasillo)"));
  problem_expert_->addPredicate(plansys2::Predicate("(connected pasillo h2)"));

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
      Initial_code_once();
      Initial_2_Cocina();
      break;

    case COCINA:
      {
        Cocina_code_iterative();

        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            Cocina_2_B1();
          } else {
            Cocina_replan();
          }
        }
      }
      break;

    case B1:
      {
        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          B1_code_iterative();
          if (executor_client_->getResult().value().success) {
            B1_2_H1();
          } else {
            B1_replan();
          }
        }
      }
      break;

    case H1:
      {
        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          H1_code_iterative();
          if (executor_client_->getResult().value().success) {
            H1_2_B2();
          } else {
            H1_replan();
          }
        }
      }
      break;

    case B2:
      {
        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          B2_code_iterative();
          if (executor_client_->getResult().value().success) {
            B2_2_H2();
          } else {
            B2_replan();
          }
        }
      }
      break;

    case H2:
      {
        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          H2_code_iterative();
          if (executor_client_->getResult().value().success) {
            H2_2_FINAL();
          } else {
            H2_replan();
          }
        }
      }
      break;

    case FINAL:
      {
        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          FINAL_code_iterative();
          if (executor_client_->getResult().value().success) {
            return;
          } else {
            FINAL_replan();
          }
        }
      }
      break;
  }

  
}

/***INITIAL***/
void
MyBT::Initial_code_once()
{
  std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
  // Set the goal for next state, and execute plan
  problem_expert_->setGoal(
    plansys2::Goal(
      "(and(explored cocina))"));
}

bool
MyBT::Initial_2_Cocina()
{
  if (executor_client_->start_plan_execution()) {
    state_ = COCINA;
  }
  return true;
}



/***COCINA***/
void
MyBT::Cocina_code_iterative()
{
  std::cout << "BBBBB11111" << std::endl;
  auto feedback = executor_client_->getFeedBack();

  for (const auto & action_feedback : feedback.action_execution_status) {
    std::cout << "[" << action_feedback.action << " " <<
      action_feedback.completion * 100.0 << "%]";
  }
  std::cout << std::endl;
}

bool
MyBT::Cocina_2_B1()
{
  std::cout << "BBBBB22222222222" << std::endl;
  std::cout << "Successful finished " << std::endl;

  // Set the goal for next state, and execute plan
  problem_expert_->setGoal(plansys2::Goal("(and(explored b1))"));

  if (executor_client_->start_plan_execution()) {
    state_ = B1;
  }
  return true;
}

void
MyBT::Cocina_replan()
{
  std::cout << "BBBBB44444444444" << std::endl;
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      std::cout << "[" << action_feedback.action << "] finished with error: " <<
        action_feedback.message_status << std::endl;
    }
  }
  executor_client_->start_plan_execution();  // replan and execute
}


/***B1***/
void
MyBT::B1_code_iterative()
{
  std::cout << "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCC" << std::endl;
  auto feedback = executor_client_->getFeedBack();

  for (const auto & action_feedback : feedback.action_execution_status) {
    std::cout << "[" << action_feedback.action << " " <<
      action_feedback.completion * 100.0 << "%]";
  }
  std::cout << std::endl;
}

bool
MyBT::B1_2_H1()
{
  std::cout << "Successful finished " << std::endl;

  // Set the goal for next state, and execute plan
  problem_expert_->setGoal(plansys2::Goal("(and(explored h1))"));

  if (executor_client_->start_plan_execution()) {
    state_ = H1;
  }
  return true;
}

void
MyBT::B1_replan()
{
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      std::cout << "[" << action_feedback.action << "] finished with error: " <<
        action_feedback.message_status << std::endl;
    }
  }
  executor_client_->start_plan_execution();  // replan and execute
}


/***H1***/
void
MyBT::H1_code_iterative()
{
  std::cout << "DDDDDDDDDDDDDDDDDDDDDDD" << std::endl;
  auto feedback = executor_client_->getFeedBack();

  for (const auto & action_feedback : feedback.action_execution_status) {
    std::cout << "[" << action_feedback.action << " " <<
      action_feedback.completion * 100.0 << "%]";
  }
  std::cout << std::endl;
}

bool
MyBT::H1_2_B2()
{
  std::cout << "Successful finished " << std::endl;

  // Set the goal for next state, and execute plan
  problem_expert_->setGoal(plansys2::Goal("(and(explored b2))"));

  if (executor_client_->start_plan_execution()) {
    state_ = B2;
  }
  return true;
}

void
MyBT::H1_replan()
{
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      std::cout << "[" << action_feedback.action << "] finished with error: " <<
        action_feedback.message_status << std::endl;
    }
  }
  executor_client_->start_plan_execution();  // replan and execute
}


/***B2***/
void
MyBT::B2_code_iterative()
{
  std::cout << "EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE" << std::endl;
  auto feedback = executor_client_->getFeedBack();

  for (const auto & action_feedback : feedback.action_execution_status) {
    std::cout << "[" << action_feedback.action << " " <<
      action_feedback.completion * 100.0 << "%]";
  }
  std::cout << std::endl;
}

bool
MyBT::B2_2_H2()
{
  std::cout << "Successful finished " << std::endl;

  // Set the goal for next state, and execute plan
  problem_expert_->setGoal(plansys2::Goal("(and(explored h2))"));

  if (executor_client_->start_plan_execution()) {
    state_ = H2;
  }
  return true;
}

void
MyBT::B2_replan()
{
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      std::cout << "[" << action_feedback.action << "] finished with error: " <<
        action_feedback.message_status << std::endl;
    }
  }
  executor_client_->start_plan_execution();  // replan and execute
}


/***H2***/
void
MyBT::H2_code_iterative()
{
  std::cout << "FFFFFFFFFFFFFFFFFFFFFFFFFFFFF" << std::endl;
  auto feedback = executor_client_->getFeedBack();

  for (const auto & action_feedback : feedback.action_execution_status) {
    std::cout << "[" << action_feedback.action << " " <<
      action_feedback.completion * 100.0 << "%]";
  }
  std::cout << std::endl;
}

bool
MyBT::H2_2_FINAL()
{
  std::cout << "Successful finished " << std::endl;

  // Set the goal for next state, and execute plan
  problem_expert_->setGoal(plansys2::Goal("(and(robot_at r salon))"));

  if (executor_client_->start_plan_execution()) {
    state_ = FINAL;
  }
  return true;
}

void
MyBT::H2_replan()
{
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      std::cout << "[" << action_feedback.action << "] finished with error: " <<
        action_feedback.message_status << std::endl;
    }
  }
  executor_client_->start_plan_execution();  // replan and execute
}


/***FINAL***/
void
MyBT::FINAL_code_iterative()
{
  std::cout << "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG" << std::endl;
  auto feedback = executor_client_->getFeedBack();

  for (const auto & action_feedback : feedback.action_execution_status) {
    std::cout << "[" << action_feedback.action << " " <<
      action_feedback.completion * 100.0 << "%]";
  }
  std::cout << std::endl;
}

void
MyBT::FINAL_replan()
{
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
      std::cout << "[" << action_feedback.action << "] finished with error: " <<
        action_feedback.message_status << std::endl;
    }
  }
  executor_client_->start_plan_execution();  // replan and execute
}


}  // namespace cascade_hfsm




