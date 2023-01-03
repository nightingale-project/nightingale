import asyncio
import roslibpy

from kivy.config import Config
Config.set('graphics', 'width', '1280')
Config.set('graphics', 'height', '800')

from kivymd.app import MDApp
from kivy.uix.screenmanager import ScreenManager
import kivy

import MovoConfig
from screen_wrapper import ScreenWrapper


class MainApp(MDApp, ScreenWrapper):
    _other_task = None
    coms_enabled = False

    # ros things
    client = None
    dummy_topic = None

    # task queue things
    queue_task = []
    queue_delay = []

    def main(self):
        self._other_task = asyncio.ensure_future(self.backend())

        async def run_wrapper():
            await self.async_run(async_lib='asyncio')
            print('App done')
            self._other_task.cancel()

        return asyncio.gather(run_wrapper(), self._other_task)

    # add a task to do after a delay
    def queue(self, task, delay=0):
        self.queue_delay.append(delay)
        self.queue_task.append(task)

    # backend polling to read from ros coms
    async def backend(self):
        await asyncio.sleep(0.5)
        self.root.transition = kivy.uix.screenmanager.FadeTransition()
        self.root.current = 'facescreen'
        try:
            while True:

                # execute a queued task
                if len(self.queue_task):
                    func = self.queue_task.pop()
                    await asyncio.sleep(self.queue_delay.pop())
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
        self.client = roslibpy.Ros(MovoConfig.Config['Movo2']["IP"], MovoConfig.Config['port'])
        self.client.run()
        asyncio.sleep(0.5)

        # initialize the /qt_robot/speech/say topic and stop service
        self.dummy_topic = roslibpy.Topic(self.client, '/qt_robot/behavior/talkText', 'std_msgs/String')


if __name__ == '__main__':
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
