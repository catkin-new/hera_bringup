import actionlib
import rospy
import math
import time
import tf

from agent.abstract_action import AbstractAction
from agent.util.enuns import LogColor
from agent.util.enuns import Side

from hera_msgs.msg import TeleopFeedback, TeleopResult, TeleopAction, TeleopGoal

from geometry_msgs.msg import Twist

class Teleop(AbstractAction):
    """
        Action to teleoperate the robot
    """
    def __init__(self, robot):
        super(Teleop, self).__init__(robot)
        self.robot_namespace = rospy.get_namespace()

        self.action_name = 'teleop'
        self.action_server = actionlib.SimpleActionServer(
            self.action_name,
            TeleopAction,
            execute_cb=self.goal_cb,
            auto_start=False
        )
        self.action_server.start()

    def goal_cb(self, goal):
        """
            Callback to receive the goal
        """
        result = self.execute(goal.cmd, goal.velocity, goal.time_in_seconds)
        self.action_server.set_succeeded(TeleopResult(result=result))

    def execute(self, cmd, velocity, time_in_seconds):
        twist_msg = Twist()

        if cmd == 'up_left':
            twist_msg.linear.x = velocity
            twist_msg.angular.y = velocity
        elif cmd == 'spin_left_front':
            twist_msg.linear.x = velocity
            twist_msg.angular.z = velocity
        elif cmd == 'forward':
            twist_msg.linear.x = velocity
        elif cmd == 'spin_right_front':
            twist_msg.linear.x = velocity
            twist_msg.angular.z = -velocity
        elif cmd == 'up_right':
            twist_msg.linear.x = velocity
            twist_msg.linear.y = -velocity
        elif cmd == 'left':
            twist_msg.linear.y = velocity
        elif cmd == 'spin_left':
            twist_msg.angular.z = velocity
        elif cmd == 'spin_right':
            twist_msg.angular.z = -velocity
        elif cmd == 'right':
            twist_msg.linear.y = -velocity
        elif cmd == 'down_left':
            twist_msg.linear.x = -velocity
            twist_msg.linear.y = velocity
        elif cmd == 'spin_left_back':
            twist_msg.linear.x = -velocity
            twist_msg.angular.z = -velocity
        elif cmd == 'backward':
            twist_msg.linear.x = -velocity
        elif cmd == 'spin_right_back':
            twist_msg.linear.x = -velocity
            twist_msg.angular.z = velocity
        elif cmd == 'down_right':
            twist_msg.linear.x = -velocity
            twist_msg.linear.y = -velocity
        elif cmd == 'stop':
            twist_msg.linear.x = velocity - velocity
            twist_msg.linear.y = velocity - velocity

        self.robot.get_actuators().teleop(twist_msg)

        if seconds > 0:
            time.sleep(seconds)
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            self.robot.get_actuators().teleop(twist_msg)




