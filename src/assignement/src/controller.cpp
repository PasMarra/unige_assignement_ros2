#include "rclcpp/rclcpp.hpp"
#include "assignement_communication/srv/velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "assignement_communication/srv/threshold.hpp"

class ControllerServiceNode : public rclcpp::Node{
    public:
        ControllerServiceNode() : Node("ctrl_service_node"){

            vel_service_ = this->create_service<assignement_communication::srv::Velocity>(
                "ctrl_srv",
                std::bind(&ControllerServiceNode::vel_service_cb, this, std::placeholders::_1, std::placeholders::_2)
            );

            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            th_service_ = this->create_service<assignement_communication::srv::Threshold>(
                "threshold_srv",
                std::bind(&ControllerServiceNode::th_service_cb, this, std::placeholders::_1, std::placeholders::_2)
            );

            RCLCPP_INFO(this->get_logger(), "ControllerServiceNode ready.");
        }
    private:
        void vel_service_cb(
            const std::shared_ptr<assignement_communication::srv::Velocity::Request> request,
            std::shared_ptr<assignement_communication::srv::Velocity::Response> response)
        {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = request->linear.x;
            msg.linear.y = request->linear.y;
            msg.linear.z = request->linear.z;
            msg.angular.x = request->angular.x;
            msg.angular.y = request->angular.y;
            msg.angular.z = request->angular.z;

            publisher_->publish(msg);

            response->reply = true;
            RCLCPP_INFO(this->get_logger(), "Received linear x=%.2f, y=%.2f, z=%.2f - angular x=%.2f, y=%.2f, z=%.2f; Responding with reply=%s",
                        request->linear.x, request->linear.y, request->linear.z, request->angular.x, request->angular.y, request->angular.z, response->reply ? "true" : "false");
        }

        void th_service_cb(
            const std::shared_ptr<assignement_communication::srv::Threshold::Request> request,
            std::shared_ptr<assignement_communication::srv::Threshold::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Received threshold value: %.2f", request->threshold);
            threshold_ = request->threshold;
            response->reply = true;
        }

    rclcpp::Service<assignement_communication::srv::Velocity>::SharedPtr vel_service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    rclcpp::Service<assignement_communication::srv::Threshold>::SharedPtr th_service_;
    double threshold_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerServiceNode>());
    rclcpp::shutdown();
    return 0;
}