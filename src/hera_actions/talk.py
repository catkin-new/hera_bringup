from translate import Translator

import actionlib
import rospy

from abstract_action import AbstractAction
from agent.util.enuns import LogColor

from hera_msgs.msg import TalkFeedback, TalkResult, TalkAction, TalkGoal
from std_msgs.msg import String

class Talk(AbstractAction):
    """
        Action to the robot talk to the user in any language
    """
    def __init__(self, robot):
        super(Talk, self).__init__(robot)
        self.robot_namespace = rospy.get_namespace()

        self.action_name = 'talk'
        self.action_server = actionlib.SimpleActionServer(
            self.action_name,
            TalkAction,
            execute_cb=self.goal_cb,
            auto_start=False
        )
        self.action_server.start()

        self.publish_vizbox_operator = rospy.Publisher('/vizbox/operator_text', String, queue_size=80)

    def goal_cb(self, goal):
        """
            Callback to receive the goal
        """
        result = self.execute(goal.text, goal.from_language, goal.to_language)
        self.action_server.set_succeeded(TalkResult(result=result))

    def execute(self, text, from_language = "en", to_language = "en"):
        """
        Execute the action

        :param self: The object pointer
        :param text: The text to be spoken
        :param from_language: The language of the text
        :param to_language: The language to be spoken
        :return: True if the action was executed successfully

        """
        translator = Translator(from_lang=from_language, to_lang=to_language)
        translated_text = translator.translate(text)
        self.publish_vizbox_operator.publish(translated_text)

        goal = TalkGoal(text=translated_text)
        goal.phrase = translated_text

        return self.robot.get_actuators().talk.execute(goal)

