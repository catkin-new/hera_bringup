import actionlib
import rospy

from agent.abstract_action import AbstractAction
from agent.util.enuns import LogColor

from hera_msgs.msg import ListenFeedback, ListenResult, ListenAction, ListenGoal

from hera_msgs.srv import StartSpeech
from hera_msgs.msg import SpeechResults
from std_msgs.msg import String


class Listen(AbstractAction):
    """
        Action to listen to the user
    """

    def __init__(self, robot):
        super(Listen, self).__init__(robot)
        self.robot_namespace = rospy.get_namespace()

        self.action_name = 'listen'
        self.action_server = actionlib.SimpleActionServer(
            self.action_name,
            ListenAction,
            execute_cb=self.goal_cb,
            auto_start=False
        )
        self.action_server.start()

        self.publish_vizbox_operator = rospy.Publisher('/vizbox/operator_text', String, queue_size=80)

        def goal_cb(self, goal):
            """
                Callback to receive the goal
            """
            speech = self.execute(goal)
            self.action_server.set_succeeded(ListenResult(result=result, speech_results=speech.results))

        def repeat_text(self, text):
            if text != "":
                self.publish_vizbox_operator.publish(text)
                robot.get_actuators().talk.execute("I heard you say: " + str(text))

        def confirm_text(self, text):
            if text != "":
                self.repeat_text(text)
                robot.get_actuators().talk.execute("Is that correct?")
                expected_answer = self.execute(["yes", "no"])
                if expected_answer.result == "yes":
                    return True
                else:
                    return False
            else:
                return False

        def execute(self, request, ask_confirm=False, repeat=False):
            """
            Wait for the robot to recognize any speech in the list

            :param self: The object pointer
            :param request: The request to listen to the user
            :param ask_confirm: If the robot should ask for confirmation
            :param repeat: If the robot should repeat the text
            :return: The result of the action

            """
            self.robot.add_log('Listening to the user', LogColor.CYAN)

            speech = self.robot.get_actuators().listen(request.specific_request, request.results)
            recognition_result = speech.result

            for i in speech.results:
                recognition_result = recognition.replace('<' + i.id + '>', i.values[0], 1)

            if recognitin_result != "":
                self.publish_vizbox_operator.publish(recognition_result)
                self.robot.add_log('Heard: {}'.format(recognition_result), LogColor.CYAN)
                if ask_confirm and repeat:
                    self.confirm_text(recognition_result)
                elif ask_confirm and (self.confirm_text(recognition_result) is False):
                    speech.result = ""

            return speech
