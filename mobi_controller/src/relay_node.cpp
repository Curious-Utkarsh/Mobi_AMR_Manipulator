#include <rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class RelayNode : public rclcpp::Node
{
public:
    RelayNode() : Node("relay_node")
    {
        //pub_ = create_publisher<geometry_msgs::msg::Twist>("/mobi_controller/cmd_vel_stamped", 10); //Not required to publish here, cause twist mux in launch directly publishes to cmd_vel_unstamped since we are not uisng stamped is disabled in controllers.yaml, so publishing directly to topic unstamped.
        sub_ = create_subscription<geometry_msgs::msg::Twist>("/mobi_controller/cmd_vel_unstamped", 10, std::bind(&RelayNode::subCallback, this, _1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

    void subCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        pub_->publish(*msg);
    }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelayNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
