import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot.mycobot import MyCobot
import time
import threading

class MyCobotDriver(Node):
    def __init__(self):
        super().__init__('mycobot_driver')
        
        # --- Parameters ---
        # Your myCobot serial port, needs to be changed according to the actual situation
        # Windows: "COM3"
        # Linux: "/dev/ttyUSB0"
        # MacOS: "/dev/tty.SLAB_USBtoUART"
        port = "/dev/ttyUSB0" 
        
        self.get_logger().info(f"Connecting to myCobot on port: '{port}'...")
        try:
            self.mc = MyCobot(port, 115200)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to myCobot: {e}")
            # Shutdown the node cleanly if connection fails
            self.destroy_node()
            rclpy.shutdown()
            # Raising an exception can be useful if another script is managing this node's lifecycle
            raise e

        if not self.mc.is_controller_connected():
            self.get_logger().error("myCobot controller not connected! Please check the port.")
            self.destroy_node()
            rclpy.shutdown()
            raise ConnectionError("myCobot controller not connected")

        self.get_logger().info("myCobot connected successfully!")

        # --- Publisher ---
        # Publishes the real-time joint angles of the robot
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # --- Subscriber ---
        # Subscribes to control commands
        self.joint_command_subscriber = self.create_subscription(
            JointState,
            'joint_commands',
            self.command_callback,
            10)

        # --- Start a separate thread to publish joint states ---
        self.publish_thread = threading.Thread(target=self.publish_states_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def publish_states_loop(self):
        """Loop to read and publish the robot's real joint states"""
        rate = self.create_rate(10) # 10 Hz
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

    def command_callback(self, msg: JointState):
        """Receive commands and control the robot"""
        if len(msg.position) == 6:
            self.get_logger().info(f"Received command, moving joints to: {[round(p, 2) for p in msg.position]}")
            angles_deg = [self.mc.radians_to_degrees(rad) for rad in msg.position]
            self.mc.send_angles(angles_deg, 80) # Set speed to 80

def main(args=None):
    rclpy.init(args=args)
    try:
        driver_node = MyCobotDriver()
        rclpy.spin(driver_node)
    except (ConnectionError, Exception) as e:
        # The logger might already be destroyed, so just print
        print(f"Node startup failed or was interrupted: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
