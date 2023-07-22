import rclpy
from rclpy.node import Node

class GlobalParameterServer(Node):
    def __init__(self):
        super().__init__('global_parameter_server',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.get_logger().info('Global Parameter Server Started')

def main(args=None):
    rclpy.init(args=args)
    node = GlobalParameterServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
