import numpy as np
from transforms3d.euler import quat2euler

class WorldState:

    def __init__(self, spawn_x, spawn_y, target_x, target_y):
        self.front_arm_angle = 0
        self.back_arm_angle = 0

        self.spawn = (spawn_x, spawn_y)
        self.target = (target_x, target_y)
        self.current_position = [0, 0]

        self.joint_state_topic = "/joint_states"
        self.imu_topic = "/imu_plugin/out"
        self.heading = 0

    def handle_joint_changes(self, msg):
        self.front_arm_angle = msg.position[2]
        self.back_arm_angle = msg.position[4]

    def handle_orientation_changes(self, msg):
        quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        self.heading = quat2euler(quat)[0]

def cart2pol(x, y):
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)
    return(r, theta)

def get_turn_angle(heading, x, y):
    angle = cart2pol(x, y)[1]
    angle = np.rad2deg(angle)
    return (angle, angle-heading)

    
