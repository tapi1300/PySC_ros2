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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"




#include "MyBT.hpp"

class Test : public cascade_hfsm::MyBT
{
public:
    Test()
    : MyBT()
    {
    }


    /***INITIAL***/
    virtual void Initial_code_once()
    {
        auto aux_node = rclcpp::Node::make_shared("aux");

        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(aux_node);
        executor_client_ = std::make_shared<plansys2::ExecutorClient>(aux_node);

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

        problem_expert_->addPredicate(plansys2::Predicate("(connected h2 pasillo)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected pasillo h2)"));

        problem_expert_->addPredicate(plansys2::Predicate("(connected salon cocina)"));
        problem_expert_->addPredicate(plansys2::Predicate("(connected cocina salon)"));

        problem_expert_->addPredicate(plansys2::Predicate("(zone_at z1 salon)"));
        problem_expert_->addPredicate(plansys2::Predicate("(zone_at z2 salon)"));
        problem_expert_->addPredicate(plansys2::Predicate("(zone_at z3 cocina)"));
        problem_expert_->addPredicate(plansys2::Predicate("(zone_at z4 cocina)"));
        problem_expert_->addPredicate(plansys2::Predicate("(zone_at z5 h1)"));


        std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
        // // Set the goal for next state, and execute plan
        // problem_expert_->setGoal(
        //     plansys2::Goal(
        //     "(and(explored cocina))"));
    }

    virtual void Initial_code_iterative() {return;}

    virtual bool Initial_2_Cocina()
    {
        return true;
    }


    /***COCINA***/
    virtual void Cocina_code_once()
    {
        // Set the goal for next state, and execute plan
        std::cout << "B parte 1" << std::endl;
        problem_expert_->setGoal(
            plansys2::Goal(
            "(and(explored cocina))"));
        executor_client_->start_plan_execution();
    }

    virtual void Cocina_code_iterative()
    {
        std::cout << "BBBBBBBBBBB" << std::endl;
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;
    }

    virtual bool Cocina_2_B1()
    {
        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            return true;
          } else {
            auto feedback = executor_client_->getFeedBack();
            for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
            }
            std::cout << "22222222222222222" << std::endl;
            executor_client_->start_plan_execution();  // replan and execute
          }
        }
        return false;
    }


    /***B1***/
    virtual void B1_code_once()
    {
        // Set the goal for next state, and execute plan
        problem_expert_->setGoal(
            plansys2::Goal(
            "(and(explored b1))"));
        executor_client_->start_plan_execution();
    }

    virtual void B1_code_iterative()
    {
        std::cout << "CCCCCCCCCCCCCC" << std::endl;
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;
    }

    virtual bool B1_2_B1()
    {
        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            return true;
          } else {
            auto feedback = executor_client_->getFeedBack();
            for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
            }
            executor_client_->start_plan_execution();  // replan and execute
          }
        }
        return false;
    }


    /***H1***/
    virtual void H1_code_once()
    {
        // Set the goal for next state, and execute plan
        problem_expert_->setGoal(
            plansys2::Goal(
            "(and(explored h1))"));
        executor_client_->start_plan_execution();
    }

    virtual void H1_code_iterative()
    {
        std::cout << "DDDDDDDDDDDDD" << std::endl;
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;
    }

    virtual bool H1_2_B2()
    {
        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            return true;
          } else {
            auto feedback = executor_client_->getFeedBack();
            for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
            }
            executor_client_->start_plan_execution();  // replan and execute
          }
        }
        return false;
    }


    /***B2***/
    virtual void B2_code_once()
    {
        // Set the goal for next state, and execute plan
        problem_expert_->setGoal(
            plansys2::Goal(
            "(and(explored b2))"));
        executor_client_->start_plan_execution();
    }

    virtual void B2_code_iterative()
    {
        std::cout << "EEEEEEEEEEEE" << std::endl;
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;
    }

    virtual bool B2_2_H2()
    {
        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            return true;
          } else {
            auto feedback = executor_client_->getFeedBack();
            for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
            }
            executor_client_->start_plan_execution();  // replan and execute
          }
        }
        return false;
    }


    /***H2***/
    virtual void H2_code_once()
    {
        // Set the goal for next state, and execute plan
        problem_expert_->setGoal(
            plansys2::Goal(
            "(and(explored h2))"));
        executor_client_->start_plan_execution();
    }

    virtual void H2_code_iterative()
    {
        std::cout << "FFFFFFFFFFFFFFFFFF" << std::endl;
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;
    }

    virtual bool H2_2_FINAL()
    {
        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            return true;
          } else {
            auto feedback = executor_client_->getFeedBack();
            for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
            }
            executor_client_->start_plan_execution();  // replan and execute
          }
        }
        return false;
    }


    /***FINAL***/
    virtual void FINAL_code_once()
    {
        // Set the goal for next state, and execute plan
        problem_expert_->setGoal(
            plansys2::Goal(
            "(and(robot_at r salon))"));
        executor_client_->start_plan_execution();
    }

    virtual void FINAL_code_iterative()
    {
        std::cout << "GGGGGGGGGGGGGGGGGG" << std::endl;
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;


        if (executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (!executor_client_->getResult().value().success) {
            auto feedback = executor_client_->getFeedBack();
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
    private:
        std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
        std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Test>();
    
    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node->get_node_base_interface());
    std::cout << "perro bajate warro 1" << std::endl;
    // node->init();
    std::cout << "perro bajate warro 2" << std::endl;
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.spin_some();
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
