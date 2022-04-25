import rospy
import actionlib

from std_srvs.srv import Trigger
from spot_msgs.msg import OpenDoorAction
from spot_msgs.srv import OpenDoor
from spot_driver.arm.arm_utilities.door_opener import open_door_main

from bosdyn.client.manipulation_api_client import ManipulationApiClient


class ArmWrapper:
    def __init__(self, robot, wrapper, logger):
        self._logger = logger
        self._spot_wrapper = wrapper

        self._robot = robot
        assert (
            self._robot.has_arm()
        ), "You've tried using the arm on your Spot, but no arm was detected!"

        self.open_door_srv = rospy.Service(
            "open_door",
            Trigger,
            self.handle_open_door,
        )

        self._init_bosdyn_clients()
        self._init_actionservers()

    def _init_bosdyn_clients(self):
        self._manip_client = self._robot.ensure_client(
            ManipulationApiClient.default_service_name
        )

    def _init_actionservers(self):
        self.open_door_as = actionlib.SimpleActionServer(
            "open_door",
            OpenDoorAction,
            execute_cb=self.handle_open_door,
            auto_start=False,
        )
        self.open_door_as.start()

    def handle_open_door(self, goal):
        del goal
        rospy.loginfo("Got a open door request")
        open_door_main(self._robot, self._spot_wrapper)
