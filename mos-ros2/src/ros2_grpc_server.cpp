#include "rclcpp/rclcpp.hpp"
#include <grpcpp/grpcpp.h>

// These headers are generated from the .proto file
#include "robot_controller.v1.grpc.pb.h"
#include "robot_controller.v1.pb.h"

#include <memory>

// Logic for your gRPC service.
class RobotControllerServiceImpl final : public mos::robot::v1::RobotController::Service {
public:
    explicit RobotControllerServiceImpl(rclcpp::Node::SharedPtr node)
        : node_(node) {}

    grpc::Status MoveToJointTarget(grpc::ServerContext* context, 
                                   const mos::robot::v1::MoveToJointTargetRequest* request, 
                                   mos::robot::v1::CommandResponse* response) override {
        RCLCPP_INFO(node_->get_logger(), "Received MoveToJointTarget request.");
        // TODO: Implement ROS2 logic to publish joint targets
        // For example, create a publisher and send the `request->joint_positions()` data.
        response->set_success(true);
        response->set_message("Command received, but not yet implemented.");
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

    // ... TODO: Implement other RPC methods from the proto file ...

private:
    rclcpp::Node::SharedPtr node_;
};

void RunServer(rclcpp::Node::SharedPtr node) {
    std::string server_address("0.0.0.0:50051");
    RobotControllerServiceImpl service(node);

    grpc::ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);

    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
    RCLCPP_INFO(node->get_logger(), "gRPC Server listening on %s", server_address.c_str());

    server->Wait();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mos_ros2_grpc_server");

    RCLCPP_INFO(node->get_logger(), "Starting gRPC server thread...");
    std::thread server_thread(RunServer, node);

    RCLCPP_INFO(node->get_logger(), "Spinning ROS2 node...");
    rclcpp::spin(node);

    // This part is not strictly necessary if server runs indefinitely, but good for clean shutdown
    server_thread.join();
    rclcpp::shutdown();
    return 0;
}
