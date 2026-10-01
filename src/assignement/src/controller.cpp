#include "rclcpp/rclcpp.hpp"
#include "assignement_communication/srv/velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "assignement_communication/srv/threshold.hpp"
#include "assignement_communication/srv/avg_velocity.hpp"
#include <Eigen/Dense>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "assignement_communication/msg/my_message.hpp"

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

            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(&ControllerServiceNode::scan_cb, this, std::placeholders::_1)
            );

            timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ControllerServiceNode::timer_cb, this));

            my_message_pub_ = this->create_publisher<assignement_communication::msg::MyMessage>("my_message_topic", 10);

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

        void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            last_scan_ = *msg;
        }

        void timer_cb(){
            
            if (last_scan_.ranges.empty()) {
                RCLCPP_WARN(this->get_logger(), "No laser scan data received yet.");
                return;
            }
            
            float min_distance = std::numeric_limits<float>::infinity();
            int min_index = -1;
            for (size_t i = 0; i < last_scan_.ranges.size(); ++i) {
                last_scan_.ranges[i] = std::clamp(last_scan_.ranges[i], last_scan_.range_min, last_scan_.range_max);
                if (last_scan_.ranges[i] < min_distance) {
                    min_distance = last_scan_.ranges[i];
                    min_index = i;
                }
            }

            // From ros2 interface show sensor_msgs/msg/LaserScan
            // the first ray in the scan.
            //
            // in frame frame_id, angles are measured around
            // the positive Z axis (counterclockwise, if Z is up)
            // with zero angle being forward along the x axis

            // FRONT = +- 45 deg
            // LEFT = 45 deg to 135 deg
            // RIGHT = -45 deg to -135 deg

            float obstacle_angle = (180.0/M_PI) * (last_scan_.angle_min + min_index * last_scan_.angle_increment);
            std::string direction;
            if(-45 < obstacle_angle && obstacle_angle < 45) {
                direction = "front";
            } else if(45 <= obstacle_angle && obstacle_angle < 135) {
                direction = "left";
            } else if(-135 <= obstacle_angle && obstacle_angle < -45) {
                direction = "right";
            } else {
                direction ;
            }

            my_message_pub_->publish(assignement_communication::msg::MyMessage().set__distance(min_distance).set__direction(direction).set__threshold(threshold_));
        }

    rclcpp::Service<assignement_communication::srv::Velocity>::SharedPtr vel_service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    rclcpp::Service<assignement_communication::srv::Threshold>::SharedPtr th_service_;
    double threshold_;

    rclcpp::Service<assignement_communication::srv::AvgVelocity>::SharedPtr avgvel_service_;
    std::list<Eigen::Vector3d> avg_lin_vel_, avg_ang_vel_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    sensor_msgs::msg::LaserScan last_scan_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<assignement_communication::msg::MyMessage>::SharedPtr my_message_pub_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerServiceNode>());
    rclcpp::shutdown();
    return 0;
}