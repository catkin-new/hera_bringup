import actionnlib
import rospy
import tf

from agent.abstract_action import AbstractAction
from agent.util.enuns import LogColor

from hera_msgs.msg import GoToPoseAction, GoToPoseGoal, GoToPoseResult, GoToPoseFeedback

from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped


class GoToPose(AbstractAction):
    """
        Action to move the robot to a specified local pose
    """

    def __init__(self, robot):
        super(GoToPose, self).__init__(robot)
        self.robot_namespace = rospy.get_namespace()

        self.action_name = 'go_to_pose'
        self.action_server = actionlib.SimpleActionServer(
            self.action_name,
            GoToPoseAction,
            execute_cb=self.goal_cb,
            auto_start=False
        )
        self.action_server.start()

        self.tf_listener = tf.TransformListener()

    def goal_cb(self, goal):
        """
            Callback to receive the goal
        """
        result = self.execute(goal.location, goal.reference)
        self.action_server.set_succeeded(GoToPoseResult(result=result))

    def execute(self, target_location, reference):
        """
        Execute the action
        :param self: The object pointer
        :param target_location: The target_location to move the robot
        :param reference: The reference frame of the location
        :return: The result of the action

        """
        self.robot.add_log('Moving to pose: {} in {}'.format(target_location, reference), LogColor.GREEN)

        # Get target position
        (target_position, target_orientation) = self.tf_listener.lookupTransform('/map', reference, rospy.Time(0))

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = target_position[0] + target_location.position.x
        goal_pose.pose.position.y = target_position[1] + target_location.position.y

        goal_pose.pose.orientation.z = target_orientation[2]
        goal_pose.pose.orientation.w = target_orientation[3]

        # Create goal
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = 'map'
        move_base_goal.target_pose.header.frame_id = rospy.Time.now()
        move_base_goal.target_pose.pose = goal_pose.pose

        return self.robot.get_actuators().go_to(move_base_goal)
