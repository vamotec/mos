import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from pymycobot.mycobot import MyCobot
import time
import threading
import os

# --- NEW: Import the mock class ---
from mock_mycobot import MockMyCobot

class MyCobotDriver(Node):
    def __init__(self):
        super().__init__('mycobot_driver')

        # --- NEW: Simulation mode switch ---
        use_sim = os.environ.get('USE_SIM_ROBOT', 'false').lower() == 'true'

        if use_sim:
            self.get_logger().info("Using MockMyCobot for simulation.")
            # The port is fake, but we pass it to match the signature
            self.mc = MockMyCobot("sim_port", 115200)
        else:
            # --- Original hardware connection logic ---
            # Your myCobot serial port, needs to be changed according to the actual situation
            # Windows: "COM3"
            # Linux: "/dev/ttyUSB0"
            # MacOS: "/dev/tty.SLAB_USBtoUART"
            port = "/dev/ttyUSB0"
            self.get_logger().info(f"Attempting to connect to real myCobot on port: '{port}'...")
            try:
                self.mc = MyCobot(port, 115200)
            except Exception as e:
                self.get_logger().error(f"Failed to connect to myCobot: {e}")
                self.destroy_node()
                rclpy.shutdown()
                raise e

        if not self.mc.is_controller_connected():
            self.get_logger().error("myCobot controller not connected! Please check the port or simulation setup.")
            self.destroy_node()
            rclpy.shutdown()
            raise ConnectionError("myCobot controller not connected")

        self.get_logger().info("myCobot interface initialized successfully!")

        # --- Publisher (no change) ---
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # --- Subscriber (no change) ---
        self.joint_command_subscriber = self.create_subscription(
            JointTrajectory,
            'joint_target_command',
            self.command_callback,
            10)

        # --- Publishing thread (no change) ---
        self.publish_thread = threading.Thread(target=self.publish_states_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def publish_states_loop(self):
        """Loop to read and publish the robot's real joint states"""
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            try:
                angles_rad = [self.mc.degrees_to_radians(deg) for deg in self.mc.get_angles()]

                msg = JointState()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
                msg.position = [float(p) for p in angles_rad]

                self.joint_state_publisher.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Could not get angles from myCobot: {e}")
            rate.sleep()

    def command_callback(self, msg: JointTrajectory):
        """Receive trajectory commands and control the robot"""
        if not msg.points:
            self.get_logger().warn("Received an empty trajectory command.")
            return

        positions = msg.points[0].positions
        if len(positions) == 6:
            self.get_logger().info(f"Received command, moving joints to: {[round(p, 2) for p in positions]}")
            angles_deg = [self.mc.radians_to_degrees(rad) for rad in positions]
            self.mc.send_angles(angles_deg, 80)  # Set speed to 80
        else:
            self.get_logger().warn(f"Received command with incorrect number of joints: {len(positions)}")

def main(args=None):
    rclpy.init(args=args)
    try:
        driver_node = MyCobotDriver()
        rclpy.spin(driver_node)
    except (ConnectionError, Exception) as e:
        print(f"Node startup failed or was interrupted: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
