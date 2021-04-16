#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "MyBT.hpp"

class Test : public cascade_hfsm::MyBT
{
public:
    Test()
    : MyBT()
    {
    }
    virtual bool State1_2_State2()
    {
        return (now()-state_ts_).seconds() > 1.0;
    }
    virtual bool Initial_2_State1()
    {
        return (now()-state_ts_).seconds() > 1.0;
    }
    virtual bool State2_2_Initial()
    {
        return (now()-state_ts_).seconds() > 1.0;
    }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Test>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    executor.spin_some();
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
