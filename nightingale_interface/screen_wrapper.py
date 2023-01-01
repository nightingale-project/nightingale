from screens import empty
from screens.screen_1 import Screen1
from screens.screen_2 import Screen2

from kivy.uix.screenmanager import ScreenManager


class ScreenWrapper(Screen1, Screen2):
    def get_screen(self, name):
        for i in range(len(self.root.screen_names)):
            if self.root.screen_names[i] == name:
                return self.root.screens[i]
        return False

    def build_wrapper(self):
        empty_screen = empty.build()
        screen1 = self.screen1_build()
        screen2 = self.screen2_build()

        sm = ScreenManager()
        sm.add_widget(empty_screen)
        sm.add_widget(screen1)
        sm.add_widget(screen2)

        return sm
