#include "rclcpp/rclcpp.hpp"
#include "assignement_communication/srv/velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "assignement_communication/srv/threshold.hpp"
#include "assignement_communication/srv/avg_velocity.hpp"
#include <Eigen/Dense>

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

            avgvel_service_ = this->create_service<assignement_communication::srv::AvgVelocity>(
                "avg_velocity_srv",
                std::bind(&ControllerServiceNode::avgvel_service_cb, this, std::placeholders::_2)
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

        avg_lin_vel_.push_back(Eigen::Vector3d(request->linear.x, request->linear.y, request->linear.z));
        avg_ang_vel_.push_back(Eigen::Vector3d(request->angular.x, request->angular.y, request->angular.z));
        if (avg_lin_vel_.size() > 5) {
            avg_lin_vel_.pop_front();
            avg_ang_vel_.pop_front();
        }

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

        void avgvel_service_cb(std::shared_ptr<assignement_communication::srv::AvgVelocity::Response> response)
        {
            Eigen::Vector3d avg_lin = Eigen::Vector3d::Zero();
            Eigen::Vector3d avg_ang = Eigen::Vector3d::Zero();
            
            if (avg_lin_vel_.size() > 0 && avg_ang_vel_.size() > 0) {
                for (const auto& v : avg_lin_vel_) {
                    avg_lin += v;
                }
                for (const auto& v : avg_ang_vel_) {
                    avg_ang += v;
                }
                avg_lin /= avg_lin_vel_.size();
                avg_ang /= avg_ang_vel_.size();
            }

            response->linear.x = avg_lin.x();
            response->linear.y = avg_lin.y();
            response->linear.z = avg_lin.z();
            response->angular.x = avg_ang.x();
            response->angular.y = avg_ang.y();
            response->angular.z = avg_ang.z();

            RCLCPP_INFO(this->get_logger(), "Responding with average linear x=%.2f, y=%.2f, z=%.2f - average angular x=%.2f, y=%.2f, z=%.2f",
                        response->linear.x, response->linear.y, response->linear.z, response->angular.x, response->angular.y, response->angular.z);
        }

    rclcpp::Service<assignement_communication::srv::Velocity>::SharedPtr vel_service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    rclcpp::Service<assignement_communication::srv::Threshold>::SharedPtr th_service_;
    double threshold_;

    rclcpp::Service<assignement_communication::srv::AvgVelocity>::SharedPtr avgvel_service_;
    std::list<Eigen::Vector3d> avg_lin_vel_, avg_ang_vel_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerServiceNode>());
    rclcpp::shutdown();
    return 0;
}