#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::placeholders;

class TestNode : public rclcpp::Node
{
public:
    TestNode(const std::string& node_name) : Node(node_name)
    {
        sub_ = create_generic_subscription(
            "test_topic_10", 
            "std_msgs/msg/String", 
            10, 
            std::bind(&TestNode::callback, this, _1));
    }

    void callback(std::shared_ptr<const rclcpp::SerializedMessage> message)
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", message->size());
    }

private:
    rclcpp::GenericSubscription::SharedPtr sub_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_topic");
    

    node->create_generic_subscription(
        "test_topic_10", 
        "std_msgs/msg/String", 
        10, 
        [](std::shared_ptr<const rclcpp::SerializedMessage> message) {
            RCLCPP_INFO(rclcpp::get_logger("test_topic"), "Received: '%s'", message->size());
        });
    std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs;
    for (size_t i = 0; i < 120; i++)
    {
        auto topic_name = "test_topic_" + std::to_string(i);
        pubs.push_back(std::move(node->create_publisher<std_msgs::msg::String>(topic_name, 10)));
    }
    
    std::vector<TestNode::SharedPtr> nodes;
    for (size_t i = 0; i < 30; i++)
    {
        std::string node_name = "test_node_" + std::to_string(i);
        auto node = std::make_shared<TestNode>(node_name);
        nodes.emplace_back(node);
    }
    
    rclcpp::spin(node);
    return 0;
}
