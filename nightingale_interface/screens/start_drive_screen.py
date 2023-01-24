from kivy.uix.screenmanager import Screen
from kivy.uix.screenmanager import NoTransition
from kivy.uix.button import Button

from kivymd.uix.label import MDLabel
from screens.screen_config import ScreenConfig as cfg


class StartDriveScreen:

    def drive_to_homescreen_override(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        self.screen_stack.append(button_data.parent.manager.current)
        button_data.parent.manager.current = cfg.HUB_SCREEN_NAME
        # should look to implement cancellation of face screen switch but might be difficult
        # since kivy Clock scheduled in another thread. Could use more variables to store
        # not critical since can tap face screen to go to hub screen

    def start_drive_build(self):
        screen = Screen(name=cfg.START_DRIVE_SCREEN_NAME)

        # remove press color animation
        screen.add_widget(
            Button(
                background_normal="images/wheel_moving.jpg",
                pos_hint={
                    "center_x": cfg.SCREEN_X_CENTER,
                    "center_y": cfg.SCREEN_Y_CENTER,
                },
                size_hint=(1, 1),
                on_release=self.drive_to_homescreen_override,
            )
        )

        return screen
