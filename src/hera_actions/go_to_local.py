import actionlib
import rospy
import tf

from agent.abstract_action import AbstractAction
from agent.util.enuns import LogColor

from hera_msgs.msg import PoseFeedback, PoseResult, PoseAction, PoseGoal

from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped


class GoToLocal(AbstractAction):
    """
        Action to move the robot to a specified local position
    """
    def __init__(self, robot):
        super(GoToLocal, self).__init__(robot)
        self.robot_namespace = rospy.get_namespace()

        self.action_name = 'go_to_local'

        self.action_server = actionlib.SimpleActionServer(
            self.action_name,
            PoseAction,
            execute_cb=self.goal_cb,
            auto_start=False
        )
        self.action_server.start()

        self.tf_listener = tf.TransformListener()

    def goal_cb(self, goal):
        """
            Callback to receive the goal
        """
        result = self.execute(goal.location, goal)
        self.action_server.set_succeeded(PoseResult(result=result))

    def execute(self, target_location):
        """
        Execute the action

        :param self: The object pointer
        :param target_location: The target_location to move the robot
        :return: The result of the action

        """
        self.robot.add_log('Moving to local position: {}'.format(target_location), LogColor.GREEN)

        # Get the position and orientation of the target_location
        (target_position, target_orientation) = self.tf_listener.lookupTransform('/map', target_location, rospy.Time(0))

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'

        goal_pose.pose.position.x = target_position[0]
        goal_pose.pose.position.y = target_position[1]

        goal_pose.orientation.z = target_orientation[2]
        goal_pose.orientation.w = target_orientation[3]

        # Create the goal
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = 'map'
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose = goal_pose.pose

        return self.robot.get_actuators().go_to(move_base_goal)
