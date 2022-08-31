import rclpy
from rclpy.node import Node
import sys

from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from ezrassor_interfaces.msg import WheelCommand

from ezrassor_autonomous_control import uti_objects

class TestController(Node):

    def __init__(self, spawn_x, spawn_y, target_x, target_y):
        super().__init__('test_controller')

        self.logger = self.get_logger()
        self.world_state = uti_objects.WorldState(spawn_x, spawn_y, target_x, target_y)

        self.joint_state_sub = self.create_subscription(
            JointState,
            self.world_state.joint_state_topic,
            self.world_state.handle_joint_changes,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.world_state.imu_topic,
            self.world_state.handle_orientation_changes,
            10
        )

        self.sub = self.create_subscription(
            String,
            'test_controls',
            self.handle_test_command,
            10
        )

        self.drum_arm_front_pub = self.create_publisher(
            Float64,
            'drum_arm_front_command',
            10
        )

        self.drum_arm_back_pub = self.create_publisher(
            Float64,
            'drum_arm_back_command',
            10
        )

        self.wheel_pub = self.create_publisher(
            Twist,
            "diff_drive_base_controller/cmd_vel_unstamped",
            10
        )
    
    def handle_test_command(self, msg):
        if (msg.data == 'lift'):

            if (self.world_state.front_arm_angle >= 0.7):
                self.logger.info("Front arm already lifted...")
            else:
                self.logger.info("Lifting front drum arm...")

                self.timer_front_arm = self.create_timer(0.1, self.timer_callback_front_arm_raise)
                
            
            if (self.world_state.back_arm_angle >= 0.7):
                self.logger.info("Back arm already lifted...")
            else:
                self.logger.info("Lifting back drum arm...")

                self.timer_back_arm = self.create_timer(0.1, self.timer_callback_back_arm_raise)
        
        elif (msg.data == 'lower'):
            if (self.world_state.front_arm_angle <= 0.1):
                self.logger.info("Front arm already lowered...")
            else:
                self.logger.info("Lowering front drum arm...")
                self.timer_front_arm = self.create_timer(0.1, self.timer_callback_front_arm_lower)

            if (self.world_state.back_arm_angle <= 0.1):
                self.logger.info("Back arm already lowered...")
            else:
                self.logger.info("Lowering back drum arm...")
                self.timer_back_arm = self.create_timer(0.1, self.timer_callback_back_arm_lower)

        elif (msg.data == 'turn'):
            self.logger.info(
                "Beginning turn to face "
                + str(self.world_state.target[0])
                + ', '
                + str(self.world_state.target[1])
                + ' ...'
            )

            self.timer_turning = self.create_timer(0.1, self.timer_callback_turning)
            


    def timer_callback_front_arm_raise(self):
        if (self.world_state.front_arm_angle < 0.7):
            lift_msg = Float64()
            lift_msg.data = 8.0
            self.drum_arm_front_pub.publish(lift_msg)
        else:
            self.timer_front_arm.destroy()

    def timer_callback_back_arm_raise(self):
        if (self.world_state.back_arm_angle < 0.7):
            lift_msg = Float64()
            lift_msg.data = 8.0
            self.drum_arm_back_pub.publish(lift_msg)
        else:
            self.timer_back_arm.destroy()
    
    def timer_callback_front_arm_lower(self):
        if (self.world_state.front_arm_angle > 0.1):
            lift_msg = Float64()
            lift_msg.data = 5.75
            self.drum_arm_front_pub.publish(lift_msg)
        else:
            halt_msg = Float64()
            halt_msg.data = 0.0
            self.drum_arm_front_pub.publish(halt_msg)
            self.timer_front_arm.destroy()

    def timer_callback_back_arm_lower(self):
        if (self.world_state.back_arm_angle > 0.1):
            lift_msg = Float64()
            lift_msg.data = 5.75
            self.drum_arm_back_pub.publish(lift_msg)
        else:
            halt_msg = Float64()
            halt_msg.data = 0.0
            self.drum_arm_back_pub.publish(halt_msg)
            self.timer_back_arm.destroy()

    def timer_callback_turning(self):
        self.logger.info("Current heading: " + str(self.world_state.heading))
        
        turn_angle = uti_objects.get_turn_angle(
            self.world_state.heading,
            self.world_state.target[0],
            self.world_state.target[1]
        )

        #self.logger.info("Amount to turn: " + str(turn_angle))

        if (turn_angle[1] > 0):
            # Left turn
            #msg = WheelCommand()
            #msg.action = 'turn'
            #msg.value = 10.0

            msg = Twist()
            msg.angular.z = 1.0

            if (self.world_state.heading < turn_angle[0]):
                self.wheel_pub.publish(msg)
            else:
                msg.angular.z = 0.0
                self.wheel_pub.publish(msg)

        else:
            # Right turn
            #msg = WheelCommand()
            #msg.action = 'turn'
            #msg.value = -10.0

            msg = Twist()
            msg.angular.z = -1.0

            if (self.world_state.heading > turn_angle[0]):
                self.wheel_pub.publish(msg)
            else:
                msg.angular.z = 0.0
                self.wheel_pub.publish(msg)

        


def main(args=None):

    try:
        rclpy.init(args=args)

        spawn_x = float(sys.argv[1])
        spawn_y = float(sys.argv[2])
        target_x = float(sys.argv[3])
        target_y = float(sys.argv[4])

        test_controller = TestController(
            spawn_x,
            spawn_y,
            target_x,
            target_y
        )

        rclpy.spin(test_controller)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()


