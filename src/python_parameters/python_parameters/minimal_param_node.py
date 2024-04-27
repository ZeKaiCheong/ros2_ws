import rclpy
import rclpy.node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from rclpy.parameter import Parameter

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node',
                         allow_undeclared_parameters=False, # Explanation and uses shown in highlighted section below
                         automatically_declare_parameters_from_overrides=False, # Explanation and uses shown in highlighted section below
                         ) 
        
        # --------------- IGNORE THIS SECTION IF YOU WANT TO FOLLOW BEST PRACTICES AND DECLARE ALL YOUR PARAMETERS BEFORE USING THEM ---------------

        # # Make allow_undeclared_parameters=True to allow undeclared parameters.
        # param_str = Parameter('my_str', Parameter.Type.STRING, 'Set from code')
        # self.set_parameters([param_str])    # Parameters WILL be implicitly declared before being set even if they were not declared beforehand. Parameter overrides are ignored by this method. For each successfully set parameter, a ParameterEvent message is published.
        # self.get_logger().info('Previously undeclared but now declared str: %s' % str(param_str.value))

        # Make automatically_declare_parameters_from_overrides=True to declare parameters from overrides such as:
        # ros2 run python_parameters minimal_param_node --ros-args -p nondeclared_str:="Set from command line"

        # Get undeclared parameters whilst keeping allow_undeclared_parameters=False
        # Provide a default backup value to use if the parameter is not declared.
        # Undeclared parameters will NOT be implicitly declared with backup value if it is used.
        param_int = self.get_parameter_or('my_int', Parameter('default_int', Parameter.Type.INTEGER, 5))
        self.get_logger().info('Undeclared int: %s' % param_int.value)

        # ------------------------------------------------------------- END OF SECTION -------------------------------------------------------------
        
        # Define parameter descriptors
        my_parameter_descriptor = ParameterDescriptor(description='This parameter is mine!')
        your_parameter_descriptor = ParameterDescriptor(description='This parameter is yours!', 
                                                        additional_constraints='These are constraints that aren\'t enforced automatically and you should implement yourself', 
                                                        read_only=False,
                                                        dynamic_typing=False,
                                                        # integer_range=[IntegerRange(from_value=0, to_value=10, step=2)])
                                                        floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0, step=2.0)])

        # Declare all parameters at once
        self.declare_parameters(
            namespace='',
            parameters=[
                ('my_parameter', 'me', my_parameter_descriptor),
                ('your_parameter', 8.0, your_parameter_descriptor),
                ('my_double_array', rclpy.Parameter.Type.DOUBLE_ARRAY)
            ]
        )
        # # Or one by one
        # self.declare_parameter('my_parameter', 'me', my_parameter_descriptor)
        # self.declare_parameter('your_parameter', 8.0, your_parameter_descriptor)

        # Undeclare parameters
        self.undeclare_parameter('my_double_array')


        self.timer = self.create_timer(1, self.timer_callback)


    def timer_callback(self):
        # Get all parameters at once
        (my_param, your_param) = self.get_parameters(['my_parameter', 'your_parameter'])
        # # Or one by one
        # my_param = self.get_parameter('my_parameter')
        # your_param = self.get_parameter('your_parameter')

        self.get_logger().info('My parameter is: %s!' % my_param.value)
        self.get_logger().info('Your parameter is: %s!' % your_param.value)

        # # Set parameters
        # my_new_param = rclpy.parameter.Parameter(
        #     'my_parameter',
        #     rclpy.Parameter.Type.STRING,
        #     'world'
        # )
        # all_new_parameters = [my_new_param]
        # self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()