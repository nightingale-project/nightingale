# follow environment variable is needed to be set so Kivy doesn't take
# over the commmand line for reading arguments
import os

os.environ["KIVY_NO_ARGS"] = "1"

import MovoConfig
from screen_wrapper import ScreenWrapper
from screens.screen_config import ScreenConfig
from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import (
    RobotStatus,
    BridgeConfig,
    UserInputs,
)

import json
import asyncio
import roslibpy
from functools import partial
import argparse
import kivy
from kivy.config import Config
from kivy.clock import Clock

Config.set("graphics", "width", "1280")
Config.set("graphics", "height", "800")

from kivymd.app import MDApp
from kivy.uix.screenmanager import ScreenManager
from kivy.properties import NumericProperty, StringProperty


class MainApp(MDApp, ScreenWrapper):
    _other_task = None
    _wd_task = None

    # ros things
    client = None
    ros_action_topic = None
    interface_screen_topic = None
    estop_topic = None
    arm_topic = None
    collision_topic = None

    # task queue things
    task_queue = []  # dict of { "task": Int, "delay": Int, "args":""}

    # counters for items
    water_count = NumericProperty(0)
    ice_count = NumericProperty(0)
    blanket_count = NumericProperty(0)

    watchdog_timer = ScreenConfig.WATCHDOG_TIMER_SECONDS
    watchdog2_exited = False

    # stack for screens to be able to return to them
    screen_stack = []
    current_action = "0"
    pending_action = "0"
    payload = ""

    def main(self):
        self._other_task = asyncio.ensure_future(self.backend())
        #self._wd_task = asyncio.ensure_future(self.watchdog_run())

        async def run_wrapper():
            await self.async_run(async_lib="asyncio")
            print("App done")
            self._other_task.cancel()
            #self._wd_task.cancel()

        #return asyncio.gather(run_wrapper(), self._other_task, self._wd_task)
        return asyncio.gather(run_wrapper(), self._other_task)


    # add a task to do after a delay
    def queue(self, task, delay=0, args=None):
        self.task_queue.append({"task": task, "delay": delay, "args": args})

    # backend polling to read from ros coms
    async def backend(self):
        await asyncio.sleep(0.5)
        self.root.transition = kivy.uix.screenmanager.FadeTransition()

        # set the initial screen
        self.root.current = ScreenConfig.SYMPOSIUM_HOME_SCREEN_NAME
        self.screen_stack.append(self.root.current)

        try:
            while True:
                # execute a queued task
                if len(self.task_queue) > 0:
                    current = self.task_queue.pop()
                    func = current["task"]
                    await asyncio.sleep(current["delay"])
                    func_args = current["args"]

                    if func_args:
                        func(func_args)
                    else:
                        func()

                await asyncio.sleep(0.2)

        except asyncio.CancelledError:
            pass
        finally:
            print("Backend Done")

    async def watchdog_run(self):
        await asyncio.sleep(0.5)
        try:
            while True:
                # need to only activate on user input screens
                if self.root.current in [
                    ScreenConfig.HUB_SCREEN_NAME,
                    ScreenConfig.CONFIRMATION_SCREEN_NAME,
                    ScreenConfig.ITEM_SELECT_SCREEN_NAME,
                ]:
                    self.watchdog_timer -= 1
                    # print(f"wd1 timer {self.watchdog_timer}")
                    await asyncio.sleep(1)
                    if self.watchdog_timer <= 0:
                        # store screen to return to if input recieved
                        ScreenConfig.last_screen = self.root.current
                        self.root.current = ScreenConfig.WATCHDOG_TIMEOUT_SCREEN_NAME
                        watchdog2 = ScreenConfig.WATCHDOG_TIMER_SECONDS
                        while True:
                            await asyncio.sleep(1)
                            watchdog2 -= 1
                            # print(f"wd2 timer {watchdog2}")
                            if watchdog2 <= 0:
                                # watchtime time up, send robot home
                                # print("WD2 timeout")
                                self.call_ros_action(UserInputs.WD_TIMEOUT)
                                self.root.current = ScreenConfig.FACE_SCREEN_NAME
                                # restart watchdog since no longer waiting on user input
                                self.reset_wd()
                                break
                            elif self.watchdog2_exited == True:
                                # user responded they are here
                                self.reset_wd()
                                self.watchdog2_exited = False
                                break
                else:
                    # on certain screens wd does not run
                    self.reset_wd()

                    # give back control to other threads
                    await asyncio.sleep(0.5)
        except asyncio.CancelledError:
            pass
        finally:
            print("Watchdog")

    # kivy screen set up
    def build(self):
        self.theme_cls.primary_palette = "Blue"
        self.theme_cls.theme_style = "Dark"
        return self.build_wrapper()

    def init_ros(self, on_movo=False):
        # initialize the ros bridge client

        self.client = None
        if on_movo == True:
            self.client = roslibpy.Ros(
                MovoConfig.Config["Movo2"]["IP"], MovoConfig.Config["RosBridgePort"]
            )
        else:
            self.client = roslibpy.Ros(
                host="localhost", port=MovoConfig.Config["RosBridgePort"]
            )

        self.client.run()
        asyncio.sleep(0.5)

        # publisher back to interface
        self.ros_action_topic = roslibpy.Topic(
            self.client, BridgeConfig.USER_INPUT_TOPIC, "std_msgs/String"
        )
        self.estop_topic = roslibpy.Topic(
            self.client, BridgeConfig.ESTOP_TOPIC, "std_msgs/String", latch=True
        )

        # subscriber to interface messages
        self.interface_screen_topic = roslibpy.Topic(
            self.client, BridgeConfig.ROBOT_STATUS_TOPIC, "std_msgs/String"
        )
        self.interface_screen_topic.subscribe(self.process_robot_status)

        # subscriber to collision topic
        self.collision_topic = roslibpy.Topic(
            self.client, BridgeConfig.ARM_COLLISION_TOPIC, "std_msgs/String"
        )
        self.collision_topic.subscribe(self.process_collision_status)

    # override
    def call_ros_action(self, action: int, args: dict = {}) -> bool:
        """
        :param action: Enum for which action to be called
        :param args: optional additional arguments
        :return: True if successful
        """
        if not type(args) is dict:
            args = {}
        args["action"] = str(action)
        json_str = json.dumps(args)
        msg = roslibpy.Message({"data": json_str})
        print(f"send mesage {msg}")
        try:
            # different topic if engaging estop
            if action == UserInputs.ESTOP:
                self.estop_action_topic.publish(msg)
            else:
                if action == UserInputs.WD_TIMEOUT or action == UserInputs.RETURN_HOME:
                    # clear screen history for next patient/task loop
                    self.screen_stack.clear()

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
        status = int(msg["data"])  # enum

        next_screen = ScreenConfig.HUB_SCREEN_NAME
        if status == RobotStatus.IDLE_HOME:
            # instantly return since no user input expected
            self.call_ros_action(UserInputs.NO_ROS_ACTION)
            next_screen = ScreenConfig.FACE_SCREEN_NAME

        elif status == RobotStatus.DRIVING:
            # instantly return since no user input expected
            # switch to start driving screen for 5 secs and then switch to face screen
            self.call_ros_action(UserInputs.NO_ROS_ACTION)
            next_screen = ScreenConfig.START_DRIVE_SCREEN_NAME
            Clock.schedule_once(
                partial(self.set_screen_delayed, ScreenConfig.FACE_SCREEN_NAME), 5
            )

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

        self.queue(self.robot_state_screen_change_task, 0, next_screen)
        return True

    def set_screen_delayed(self, screen_name, dt):
        # able to set any screen name after a delay.
        # meant to be used with Kivy.Clock scheduling calls
        self.root.current = screen_name

    def robot_state_screen_change_task(self, next_screen):
        # update screen upon new state
        if next_screen is not None and self.get_screen(next_screen):
            self.root.current = next_screen
            self.screen_stack.append(self.root.current)

    def reset_wd(self):
        # reset watchdog to max time
        self.watchdog_timer = ScreenConfig.WATCHDOG_TIMER_SECONDS


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ros_comms",
        action="store_true",
        help="If set then True to use ROS bridge, False otherwise",
    )
    parser.add_argument(
        "--on_movo",
        action="store_true",
        help="If set then True to run on movo, False if on own computer",
    )

    args = parser.parse_args()
    # initialize the app and event loop
    loop = asyncio.get_event_loop()
    app = MainApp()

    # initializes everything related to ros bridge
    if args.ros_comms == True:
        app.init_ros(args.on_movo)

    # start the app event loop
    loop.run_until_complete(app.main())
    loop.close()

    # clean up ros bridge connections
    if app.coms_enabled:
        app.client.terminate()
