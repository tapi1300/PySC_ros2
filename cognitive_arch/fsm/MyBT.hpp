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

#ifndef MYBT_H_
#define MYBT_H_

#include <string>
#include <memory>

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

  virtual void Initial_code_iterative() {}
  virtual void Initial_code_once();
  virtual void Cocina_code_iterative();
  virtual void Cocina_code_once() {}
  virtual void Cocina_replan();
  virtual void B1_code_iterative();
  virtual void B1_code_once() {}
  virtual void B1_replan();
  virtual void H1_code_iterative();
  virtual void H1_code_once() {}
  virtual void H1_replan();

  virtual bool Cocina_2_B1();
  virtual bool Initial_2_Cocina() {return false;}
  virtual bool B1_2_H1();
  virtual bool H1_2_Cocina();
  
  void init();
  void tick();
  bool get_result(){return executor_client_->getResult().value().success;}

protected:
  rclcpp::Time state_ts_;

private:
  void deactivateAllDeps() {}
  void B1_activateDeps() {}
  void Initial_activateDeps() {}
  void Cocina_activateDeps() {}
  void H1_activateDeps() {}


  static const int B1 = 0;
  static const int INITIAL = 1;
  static const int COCINA = 2;
  static const int H1 = 3;


  int state_;

  std::string myBaseId_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

}  // namespace cascade_hfsm

#endif  // MYBT_H_
