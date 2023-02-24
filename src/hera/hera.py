from agent.abstract_agent import AbstractAgent

from hera_actions import *

from hera_msgs.srv import RobotRequest, RobotRequestResponse

import rospy
import json


class Hera(AbstractAgent):
    """
        Agent to control the robot
    """

    def __init__(self):
        super(Hera, self).__init__()
        rospy.init_node('Hera', anonymous=True)
        self.robot_namespace = rospy.get_namespace()

        self.hera_actions.add_action('GoToLocal', GoToLocal(self))
        self.hera_actions.add_action('GoToPose', GoToPose(self))
        self.hera_actions.add_action('SaveLocal', SaveLocal(self))
        self.hera_actions.add_action('Listen', Listen(self))
        self.hera_actions.add_action('Talk', Talk(self))
        self.hera_actions.add_action('Teleop', Teleop(self))
        self.hera_actions.add_action('SetPose', SetPose(self))

        request_service = rospy.Service('robot_request', RobotRequest, self.request_cb)

    def request_cb(self, request):
        """
            Callback to receive and handle requests
        """
        if request.robot_request == 'known_places':
            known_places = [loc.name for loc in self.get_sensors().locals()]
            return RobotRequestResponse(json.dumps(known_places))
        elif request.robot_request == 'nearest_person':
            nearest_person = self.get_sensors('people').people
            if len(nearest_person) > 0:
                return RobotRequestResponse(json.dumps(nearest_person[0]))
            else:
                return RobotRequestResponse(json.dumps(None))
        elif request.robot_request == 'is_front_free':
            maximum_distance = 2.0
            alpha = 5
            half_fov = len(self.get_sensors('laser').ranges) / 2

            minimum_distance = min(self.get_sensors('laser').ranges[int(half_fov - alpha):int(half_fov + alpha)])
            if minimum_distance > maximum_distance:
                return RobotRequestResponse(json.dumps(True))
            else:
                return RobotRequestResponse(json.dumps(False))


if __name__ == '__main__':
    hera = Hera()
    rospy.spin()
