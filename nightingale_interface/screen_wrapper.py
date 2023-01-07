from screens.home_screen import HomeScreen
from screens.nurse_alert_screen import NurseAlertScreen
from screens.face_screen import FaceScreen
from screens.estopped_screen import EStoppedScreen
from screens.admin_screen import AdminScreen
from screens.extend_arm_screen import ExtendArmScreen
from screens.retract_arm_screen import RetractArmScreen
from screens.wait_item_get_screen import WaitItemGetScreen
from screens.video_call_screen import VideoCallScreen
from screens.item_select_screen import ItemSelectScreen
from screens.item_fill_screen import ItemFillScreen
from screens.confirmation_screen import ConfirmationScreen

from kivy.uix.screenmanager import ScreenManager
from kivy.properties import ListProperty


class ScreenWrapper(
    HomeScreen,
    NurseAlertScreen,
    FaceScreen,
    EStoppedScreen,
    AdminScreen,
    ExtendArmScreen,
    RetractArmScreen,
    WaitItemGetScreen,
    VideoCallScreen,
    ItemSelectScreen,
    ItemFillScreen,
    ConfirmationScreen,
):
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
        adminscreen = self.admin_build()
        extendarmscreen = self.extend_arm_build()
        retractarmscreen = self.retract_arm_build()
        waititemgetscreen = self.wait_item_get_build()
        videocallscreen = self.video_call_build()
        itemselectionscreen = self.item_select_build()
        itemfillscreen = self.item_fill_build()
        confirmationscreen = self.confirmation_build()

        sm = ScreenManager()
        # id's of screens are in the order they are added, facescreen = 0, last is -1
        sm.add_widget(facescreen)
        sm.add_widget(homescreen)
        sm.add_widget(nursealertscreen)
        sm.add_widget(estoppedscreen)
        sm.add_widget(adminscreen)
        sm.add_widget(extendarmscreen)
        sm.add_widget(retractarmscreen)
        sm.add_widget(waititemgetscreen)
        sm.add_widget(videocallscreen)
        sm.add_widget(itemselectionscreen)
        sm.add_widget(itemfillscreen)
        sm.add_widget(confirmationscreen)
        return sm
