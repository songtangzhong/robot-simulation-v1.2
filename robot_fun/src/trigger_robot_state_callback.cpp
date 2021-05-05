#include <robot_fun/robot_fun.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<robot_fun::RobotFun> node = 
        std::make_shared<robot_fun::RobotFun>("trigger_robot_state_callback");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
