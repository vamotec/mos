#include "rclcpp/rclcpp.hpp"
#include <grpcpp/grpcpp.h>

// These headers are generated from the .proto file
#include "robot_controller.v1.grpc.pb.h"
#include "robot_controller.v1.pb.h"

// --- NEW: Include the message type for trajectory commands ---
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <memory>
#include <vector>

// Logic for your gRPC service.
class RobotControllerServiceImpl final : public mos::robot::v1::RobotController::Service {
public:
    explicit RobotControllerServiceImpl(rclcpp::Node::SharedPtr node)
        : node_(node) {
        // --- NEW: Create the publisher in the constructor ---
        publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_target_command", 10);
        RCLCPP_INFO(node_->get_logger(), "Created publisher for /joint_target_command");
    }

    grpc::Status MoveToJointTarget(grpc::ServerContext* context, 
                                   const mos::robot::v1::MoveToJointTargetRequest* request, 
                                   mos::robot::v1::CommandResponse* response) override {
        RCLCPP_INFO(node_->get_logger(), "Received MoveToJointTarget request.");

        // --- NEW: Create and publish the ROS2 message ---
        auto trajectory_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
        
        // TODO: These should probably be configurable or passed in the request
        trajectory_msg->joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.assign(request->joint_positions().begin(), request->joint_positions().end());
        // Set a time for the trajectory point to be reached
        point.time_from_start = rclcpp::Duration(2, 0); // 2 seconds

        trajectory_msg->points.push_back(point);

        RCLCPP_INFO(node_->get_logger(), "Publishing joint target command.");
        publisher_->publish(std::move(trajectory_msg));
        
        response->set_success(true);
        response->set_message("Joint target command published successfully.");
        // --- END NEW ---

        return grpc::Status::OK;
    }

    grpc::Status GetJointState(grpc::ServerContext* context, 
                               const mos::robot::v1::GetJointStateRequest* request, 
                               mos::robot::v1::GetJointStateResponse* response) override {
        RCLCPP_INFO(node_->get_logger(), "Received GetJointState request.");
        // TODO: Implement ROS2 logic to subscribe to /joint_states topic
        // and populate the response.
        for (int i = 0; i < 6; ++i) {
            response->add_position(0.0); // Placeholder data
        }
        return grpc::Status::OK;
    }

private:
    rclcpp::Node::SharedPtr node_;
    // --- NEW: Add publisher as a member variable ---
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
};

void RunServer(rclcpp::Node::SharedPtr node) {
    RobotControllerServiceImpl service(node);

    grpc::ServerBuilder builder;

    // Listen on a TCP port for development convenience
    std::string tcp_address("0.0.0.0:50051");
    builder.AddListeningPort(tcp_address, grpc::InsecureServerCredentials());
    RCLCPP_INFO(node->get_logger(), "gRPC Server listening on TCP: %s", tcp_address.c_str());

    // Listen on a UDS for production
    std::string uds_address("unix:///ros2_ws/src/run/mos-ros2.sock");
    builder.AddListeningPort(uds_address, grpc::InsecureServerCredentials());
    RCLCPP_INFO(node->get_logger(), "gRPC Server listening on UDS: %s", uds_address.c_str());

    builder.RegisterService(&service);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    RCLCPP_INFO(node->get_logger(), "gRPC server started. Waiting for requests...");

    server->Wait();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mos_ros2_grpc_server");

    RCLCPP_INFO(node->get_logger(), "Starting gRPC server thread...");
    std::thread server_thread(RunServer, node);

    RCLCPP_INFO(node->get_logger(), "Spinning ROS2 node...");
    rclcpp::spin(node);

    server_thread.join();
    rclcpp::shutdown();
    return 0;
}