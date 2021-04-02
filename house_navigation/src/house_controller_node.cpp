// Copyright 2019 El grupo del Flow
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

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class HouseNav : public rclcpp::Node
{
public:
  HouseNav()
  : rclcpp::Node("house_controller")
  {
  }

  void init()
  {
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(shared_from_this());
    executor_client_ = std::make_shared<plansys2::ExecutorClient>(shared_from_this());

    init_knowledge();

    if (!executor_client_->start_plan_execution()) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }
  }

  void init_knowledge()
  {
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

    problem_expert_->addPredicate(plansys2::Predicate("(object_at o1 h2)"));

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


    problem_expert_->setGoal(
      plansys2::Goal(
        "(and(robot_at r pasillo))"));
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
  }

private:
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HouseNav>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
