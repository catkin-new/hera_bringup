import actionlib
import rospy
import tf

from agent.abstract_action import AbstractAction
from agent.util.enuns import LogColor

from hera_msgs.msg import PoseFeedback, PoseResult, PoseAction, PoseGoal

from geometry_msgs.msg import PoseWithCovarianceStamped


class SetPose(AbstractAction):
    """
        Action to get the current pose of the robot
    """

    def __init__(self, robot):
        super(SetPose, self).__init__(robot)
        self.robot_namespace = rospy.get_namespace()

        self.action_name = 'get_pose'

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
        result = self.execute(goal)
        self.action_server.set_succeeded(PoseResult(result=result))

    def execute(self, location):
        """
        Execute the action

        :param self: The object pointer
        :param location: The location to move the robot
        :return: The result of the action

        """
        self.robot.add_log('Getting pose', LogColor.GREEN)

        # Get the position and orientation of the location
        (target_position, target_orientation) = self.tf_listener.lookupTransform('/map', location, rospy.Time(0))

        goal_pose = PoseWithCovarianceStamped()
        goal_pose.header.frame_id = 'map'

        goal_pose.pose.pose.position.x = target_position[0]
        goal_pose.pose.pose.position.y = target_position[1]
        goal_pose.pose.pose.orientation.z = target_orientation[2]
        goal_pose.pose.pose.orientation.w = target_orientation[3]

        self.robot.get_actuators().set_pose(goal_pose)
        return 'Pose set'
