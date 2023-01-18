import asyncio
import roslibpy
import json

from kivy.config import Config

Config.set("graphics", "width", "1280")
Config.set("graphics", "height", "800")

from kivymd.app import MDApp
from kivy.uix.screenmanager import ScreenManager
import kivy
from kivy.properties import NumericProperty, StringProperty

import MovoConfig
from screen_wrapper import ScreenWrapper

from screens.screen_config import ScreenConfig 
from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import RobotStatus, BridgeConfig, UserInputs


class MainApp(MDApp, ScreenWrapper):
    _other_task = None
    coms_enabled = True

    # ros things
    client = None
    ros_action_topic = None
    interface_screen_topic = None
    estop_topic = None

    # task queue things
    task_queue = [] # dict of { "task": Int, "delay": Int, "args":""}

    # counters for items
    water_count = NumericProperty(0)
    ice_count = NumericProperty(0)
    blanket_count = NumericProperty(0)

    ros_set_screen = ""

    def main(self):
        self._other_task = asyncio.ensure_future(self.backend())

        async def run_wrapper():
            await self.async_run(async_lib="asyncio")
            print("App done")
            self._other_task.cancel()

        return asyncio.gather(run_wrapper(), self._other_task)

    # add a task to do after a delay
    def queue(self, task, delay=0, args=None):
        self.task_queue.append({"task": task, "delay": delay, "args":args})

    # backend polling to read from ros coms
    async def backend(self):
        await asyncio.sleep(0.5)
        self.root.transition = kivy.uix.screenmanager.FadeTransition()

        # set the initial screen
        self.root.current = ScreenConfig.FACE_SCREEN_NAME 

        try:
            while True:

                # update screen upon new state
                # need better way. probably just another state
                if len(self.ros_set_screen) > 6:
                    self.root.current = self.ros_set_screen
                    self.ros_set_screen = ""

                # execute a queued task
                if len(self.task_queue):
                    current = self.task_queue.pop() 
                    func = current["task"]
                    await asyncio.sleep(current["delay"])
                    func_args = current["args"]

                    if args:
                        func(func_args)
                    else:
                        func()

                await asyncio.sleep(0.2)

        except asyncio.CancelledError:
            pass
        finally:
            print("Backend Done")

    # kivy screen set up
    def build(self):
        self.theme_cls.primary_palette = "Blue"
        self.theme_cls.theme_style = "Dark"
        return self.build_wrapper()

    def init_ros(self):
        # initialize the ros bridge client

        #self.client = roslibpy.Ros(
        #    MovoConfig.Config["Movo2"]["IP"], MovoConfig.Config["RosBridgePort"]
        #)

        # temporary offline computer testing
        self.client = roslibpy.Ros(
            host='localhost', port=MovoConfig.Config["RosBridgePort"]
        )
        # temporary offline computer testing

        self.client.run()
        asyncio.sleep(0.5)

        # publisher back to interface
        self.ros_action_topic = roslibpy.Topic(self.client, BridgeConfig.USER_INPUT_TOPIC, "std_msgs/String")
        self.estop_topic = roslibpy.Topic(self.client, BridgeConfig.ESTOP_TOPIC, "std_msgs/String", latch=True)

        # subscriber to interface messages
        self.interface_screen_topic = roslibpy.Topic(self.client, BridgeConfig.ROBOT_STATUS_TOPIC, "std_msgs/String")
        self.interface_screen_topic.subscribe(self.process_robot_status)

    # override
    def call_ros_action(self, action: int, args: dict = {}) -> bool:
        """
        :param action: Enum for which action to be called
        :param args: optional additional arguments
        :return: True if successful
        """
        if not type(args) is dict:
            args = {}
        args['action'] = str(action)
        json_str = json.dumps(args)
        msg = roslibpy.Message({"data": json_str})
        print(f"send mesage {msg}")
        try:
            # different topic if engaging estop
            if action == UserInputs.ESTOP:
                self.estop_action_topic.publish(msg)
            else:
                self.ros_action_topic.publish(msg)
            return True
        except:
            return False

    def process_robot_status(self, msg):
        """
        :param msg: Message to be sent
        :return: True if successful
        """
        # take in status received from master and react to it 
        print(f"recieved {msg}")
        status = int(msg['data']) # enum

        next_screen = ScreenConfig.HUB_SCREEN_NAME
        if status == RobotStatus.IDLE_HOME or status == RobotStatus.DRIVING:
            # instantly return since no user input expected
            self.call_ros_action(UserInputs.NO_ROS_ACTION)
            next_screen = ScreenConfig.FACE_SCREEN_NAME

        elif status == RobotStatus.BEDSIDE_IDLE:
            next_screen = ScreenConfig.HUB_SCREEN_NAME
        elif status == RobotStatus.BEDSIDE_DELIVER:
            # initiate arm extend
            next_screen = ScreenConfig.EXTEND_ARM_SCREEN_NAME
        elif status == RobotStatus.ITEM_STOCK_REACHED:
            # show admin what to stock
            next_screen = ScreenConfig.ITEM_FILL_SCREEN_NAME
        elif status == RobotStatus.ARM_EXTENDED:
            next_screen = ScreenConfig.WAIT_ITEM_GET_SCREEN_NAME
        elif status == RobotStatus.ARM_RETRACTED:
            # reset robot state for next request
            self.call_ros_action(UserInputs.NO_ROS_ACTION)
            next_screen = ScreenConfig.HUB_SCREEN_NAME

        # statuses which do not change screens
        # for now
        elif status == RobotStatus.EXTENDING_ARM:
            # show popup or message that arm is extending
            self.call_ros_action(UserInputs.NO_ROS_ACTION)
            return True
        elif status == RobotStatus.RETRACTING_ARM:
            # show popup or message that arm is retracting
            self.call_ros_action(UserInputs.NO_ROS_ACTION)
            return True

        else:
            print(f"CODE {status} UNKNOWN")
            return False
 
        self.ros_set_screen = next_screen
        return True


if __name__ == "__main__":
    # initialize the app and event loop
    loop = asyncio.get_event_loop()
    app = MainApp()

    # initializes everything related to ros bridge
    if app.coms_enabled:
        app.init_ros()

    # start the app event loop
    loop.run_until_complete(app.main())
    loop.close()

    # clean up ros bridge connections
    if app.coms_enabled:
        app.client.terminate()
