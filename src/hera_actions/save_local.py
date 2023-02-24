import actionlib
import rospy
import tf

from agent.abstract_action import AbstractAction
from agent.util.enuns import LogColor

from hera_msgs.msg import PoseFeedback, PoseResult, PoseAction, PoseGoal

from geometry_msgs.msg import Pose

class SaveLocal(AbstractAction):
    """
        Action to save a local position
    """

    def __init__(self, robot):
        super(SaveLocal, self).__init__(robot)
        self.robot_namespace = rospy.get_namespace()

        self.action_name = 'save_local'

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

    def execute(self, local):
        """
        Execute the action

        :param self: The object pointer
        :param local: Local to be saved
        :return: The result of the action

        """
        self.robot.add_log('Saving local position: {}'.format(goal.location), LogColor.GREEN)

        # Get the position and orientation of the location
        (target_position, target_orientation) = self.tf_listener.lookupTransform('/map', goal.location, rospy.Time(0))

        goal_pose = Pose()
        goal_pose.position.x = target_position[0]
        goal_pose.position.y = target_position[1]
        goal_pose.orientation.z = target_orientation[2]
        goal_pose.orientation.w = target_orientation[3]

        self.robot.get_actuators().save_local(goal.location, goal_pose)
        return 'Local position saved'
