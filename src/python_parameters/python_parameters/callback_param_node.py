# Example modified from https://roboticsbackend.com/ros2-rclpy-parameter-callback/

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# Import the Bool interface from the example_interfaces package for testing only
# Create your own semantically meaningful message type for your application
from example_interfaces.msg import Bool

import time

class TestParamsCallback(Node):
    def __init__(self):
        super().__init__('callback_param_node')
        self.declare_parameter('camera_device_port', '/dev/ttyACM0')
        self.declare_parameter('autonomous_mode', False)
        self.declare_parameter('battery_percentage_warning', 15.0)
        
        self.camera_device_port_ = self.get_parameter('camera_device_port').value
        self.autonomous_mode_ = self.get_parameter('autonomous_mode').value
        self.battery_percentage_warning_ = self.get_parameter('battery_percentage_warning').value

        # Flag to indicate whether to restart the camera
        self.restart_camera = False
        
        # Register this callback to be called to inspect parameter(s) just PRIOR to setting them. 
        # You can prevent parameter(s) from being set by returning SetParametersResult(successful=False).
        self.add_on_set_parameters_callback(self.parameters_callback)

        # # Deregister the callback if you want to stop it from being called before parameters are set.
        # self.remove_on_set_parameters_callback(self.parameters_callback)

        # Create a reentrant callback group: Callbacks in this group MAY be called concurrently from 
        # multiple threads, including multiple instances of the same callback.
        self.reentrant_group_1 = ReentrantCallbackGroup()
        # Create a mutually exclusive callback group: Callbacks in this group MUST be 
        # called sequentially, including multiple instances of the same callback.
        self.mutually_exclusive_group_1 = MutuallyExclusiveCallbackGroup()

        # Create a periodic callback to simulate camera monitoring and restart,
        # with the timer callback in its own mutually exclusive callback group
        self.timer = self.create_timer(1, self.camera_monitoring_callback, callback_group=self.mutually_exclusive_group_1)
        # # with the timer callback in its own reentrant callback group
        # self.timer = self.create_timer(1, self.camera_monitoring_callback, callback_group=self.reentrant_group_1)
        # # with the timer callback in the node's default callback group (sharing the queue with other callbacks)
        # self.timer = self.create_timer(1, self.camera_monitoring_callback)


        self.autonomous_mode_publisher_ = self.create_publisher(Bool, 'autonomous_mode', 10)

    # Timer callback to simulate camera monitoring and restart
    def camera_monitoring_callback(self):
        if self.restart_camera:
            self.get_logger().info("Restarting camera with port: " + str(self.camera_device_port_))
            # Simulate camera restart 
            time.sleep(4.0)   # Takes four times as long as the timer callback period
            self.get_logger().info("Camera restarted!")
            self.restart_camera = False
		
    # Callback to be called just PRIOR to setting parameters. You can reject the parameters here.
    # The conditions for when it returns success=True may need to change if you ever use 'set_parameters_atomically' (set all at once) as 
    # the params list will be of size greater than 1 and it should return False if ANY one is NOT valid.
    # Otherwise, it will always be of size 1 and you only ever need to worry about dealing with one parameter in each callback instance.
    def parameters_callback(self, params):
        success = False

        # Keep count of how many parameters are being set in this callback instance to see if params are being set atomically.
        num_param_counter = 1   

        for param in params:
            # Keep count of how many parameters are being set in this callback instance to see if params are being set atomically.
            self.get_logger().info("Parameter " + str(num_param_counter) + ": " + param.name + " with value " + str(param.value))
            num_param_counter += 1

            if param.name == "camera_device_port":
                if param.type_ == Parameter.Type.STRING:
                    if param.value.startswith('/dev/tty'):
                        success = True
                        self.camera_device_port_ = param.value
                        self.restart_camera = True
                        self.get_logger().info("Setting camera device port: " + str(self.camera_device_port_))
            if param.name == "battery_percentage_warning":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    # See minimal_param_node.py to see how to enforce integer or float range parameter constraints automatically by declaring them in the parameter descriptor.
                    if param.value >= 0.0 and param.value < 100.0:  
                        success = True
                        self.battery_percentage_warning_ = param.value
                        self.get_logger().info("Setting battery percentage warning: " + str(self.battery_percentage_warning_))
            if param.name == "autonomous_mode":
                if param.type_ == Parameter.Type.BOOL:
                    success = True
                    self.autonomous_mode_ = param.value
                    self.get_logger().info("Setting autonomous mode: " + str(self.autonomous_mode_))

                    # Publish the autonomous mode status
                    msg = Bool()
                    msg.data = self.autonomous_mode_
                    self.autonomous_mode_publisher_.publish(msg)
                    self.get_logger().info("Published autonomous mode status: " + str(self.autonomous_mode_))
                    
        return SetParametersResult(successful=success)

def main(args=None):
    rclpy.init(args=args)
    node = TestParamsCallback()

    # Use MultiThreadedExecutor with the same number of threads as the maximum number of callbacks that may be called concurrently
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()