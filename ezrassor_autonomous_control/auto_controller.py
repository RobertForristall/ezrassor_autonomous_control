import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import Float64, String

class AutoController(Node):

    def __init__(self, model):
        super().__init__('auto_controller')

        self.command_flag = 'hold'
        self.pub_msg = String()
        self.logger = self.get_logger()
        self.model = model

        self.result_sub = self.create_subscription(
            String,
            'command_result',
            self.handle_result_callback,
            1
        )

        self.command_pub = self.create_publisher(
            String,
            'test_controls',
            10
        )

        self.supervisor = self.create_timer(0.1, self.supervisor_callback)

    def handle_result_callback(self, msg):
        self.command_flag = msg.data

    def supervisor_callback(self):

        # Hold: wait for next command flag
        if self.command_flag == 'hold' or self.command_flag == 'ready':
            pass
        
        # Begin/Lift: send the lift command to prep for driving
        elif self.command_flag == 'begin' or self.command_flag == 'lift':
            self.command_flag = 'hold'
            if (self.model == 'basic'):
                self.pub_msg.data = 'lift_both'
            elif (self.model == 'paver'):
                self.pub_msg.data = 'lift_back'
            self.command_pub.publish(self.pub_msg)

        # Turn: send the turn command to face the target position
        elif self.command_flag == 'turn':
            self.command_flag = 'hold'
            self.pub_msg.data = 'turn'
            self.command_pub.publish(self.pub_msg)

        # Drive: send the drive command to drive the alloted movement increment
        elif self.command_flag == 'drive':
            self.command_flag = 'hold'
            self.pub_msg.data = 'drive'
            self.command_pub.publish(self.pub_msg)

def main(args=None):

    try:
        rclpy.init(args=args)
        model = sys.argv[1]
        node = AutoController(model)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()