class WorldState:

    def __init__(self):
        self.front_arm_angle = 0
        self.back_arm_angle = 0

        self.joint_state_topic = "/joint_states"

    def handle_joint_changes(self, msg):
        self.front_arm_angle = msg.position[2]
        self.back_arm_angle = msg.position[4]
