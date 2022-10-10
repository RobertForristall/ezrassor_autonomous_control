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
        #self.imu_topic = "/imu_plugin/out"
        #self.imu_topic = "/demo/imu"
        self.model_state_topic = "/demo/model_states_demo"
        self.model_index = -1
        self.heading = 0
        self.movement_increment = 0.5
        self.current_moved = 0.0
        self.start_frame = [0,0]

    def handle_joint_changes(self, msg):
        self.front_arm_angle = msg.position[2]
        self.back_arm_angle = msg.position[4]

    def handle_orientation_changes(self, msg):    
        quat = [
            msg.pose[1].orientation.w,
            msg.pose[1].orientation.x,
            msg.pose[1].orientation.y,
            msg.pose[1].orientation.z
        ]
        self.heading = np.rad2deg(quat2euler(quat)[2])

        self.current_position = [msg.pose[1].position.x, msg.pose[1].position.y]

def cart2pol(x, y):
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)
    return(r, theta)

def get_turn_angle(heading, x, y):
    angle = cart2pol(x, y)[1]
    angle = np.rad2deg(angle)
    return (angle, angle-heading)

def euclidean_distance(x1, x2, y1, y2):
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)

    
