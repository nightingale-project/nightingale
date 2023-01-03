from screens.home_screen import HomeScreen
from screens.nurse_alert_screen import NurseAlertScreen
from screens.face_screen import FaceScreen
from screens.estopped_screen import EStoppedScreen

from kivy.uix.screenmanager import ScreenManager


class ScreenWrapper(HomeScreen, NurseAlertScreen, FaceScreen, EStoppedScreen):
    def get_screen(self, name):
        for i in range(len(self.root.screen_names)):
            if self.root.screen_names[i] == name:
                return self.root.screens[i]
        return False

    def build_wrapper(self):
        homescreen = self.home_build()
        nursealertscreen = self.nurse_alert_build()
        facescreen = self.face_build()
        estoppedscreen = self.estopped_build()

        sm = ScreenManager()
        sm.add_widget(facescreen)
        sm.add_widget(homescreen)
        sm.add_widget(nursealertscreen)
        sm.add_widget(estoppedscreen)

        return sm
