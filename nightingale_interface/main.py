import asyncio
import roslibpy
import json

from kivy.config import Config

Config.set("graphics", "width", "1280")
Config.set("graphics", "height", "800")

from kivymd.app import MDApp
from kivy.uix.screenmanager import ScreenManager
import kivy
from kivy.properties import NumericProperty

import MovoConfig
from screen_wrapper import ScreenWrapper
from screens.screen_config import ScreenConfig as cfg


class MainApp(MDApp, ScreenWrapper):
    _other_task = None
    coms_enabled = True

    # ros things
    client = None
    ros_action_topic = None
    interface_screen_topic = None

    # task queue things
    queue_task = []
    queue_delay = []
    queue_args = []

    # counters for items
    water_count = NumericProperty(0)
    ice_count = NumericProperty(0)
    blanket_count = NumericProperty(0)

    def main(self):
        self._other_task = asyncio.ensure_future(self.backend())

        async def run_wrapper():
            await self.async_run(async_lib="asyncio")
            print("App done")
            self._other_task.cancel()

        return asyncio.gather(run_wrapper(), self._other_task)

    # add a task to do after a delay
    def queue(self, task, delay=0, args=None):
        self.queue_delay.append(delay)
        self.queue_task.append(task)
        self.queue_args.append(args)

    # backend polling to read from ros coms
    async def backend(self):
        await asyncio.sleep(0.5)
        self.root.transition = kivy.uix.screenmanager.FadeTransition()

        # set the initial screen
        self.root.current = "facescreen"

        try:
            while True:

                # execute a queued task
                if len(self.queue_task):
                    func = self.queue_task.pop()
                    await asyncio.sleep(self.queue_delay.pop())
                    args = self.queue_args.pop()
                    if args:
                        func(args)
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
        self.client = roslibpy.Ros(
            MovoConfig.Config["Movo2"]["IP"], MovoConfig.Config["RosBridgePort"]
        )
        self.client.run()
        asyncio.sleep(0.5)

        self.ros_action_topic = roslibpy.Topic(self.client, "ui/robot/call_action", "std_msgs/String")

        self.interface_screen_topic = roslibpy.Topic(self.client, "ui/app/set_screen", "std_msgs/String")
        self.interface_screen_topic.subscribe(self.set_screen_callback)

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
        try:
            self.ros_action_topic.publish(msg)
            return True
        except:
            return False

    def set_screen_callback(self, msg):
        # if the screen requested exists
        if self.get_screen(str(msg['data'])):
            # def a function for setting the screen
            def func(args):
                self.root.current = args
            # queue the function
            self.queue(func, args=str(msg['data']))
            return True
        return False


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
