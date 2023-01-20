from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image
from kivy.uix.label import Label

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import NoTransition
from screens.screen_config import ScreenConfig as cfg
from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import (
    BridgeConfig,
    UserInputs,
)


class WatchdogScreen:
    def exit_wd(self, button_data):
        # returns to previous screen that came before wd activated
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = self.screen_stack.pop()
        self.watchdog2_exited = True

    def watchdog_build(self):
        screen = Screen(name=cfg.WATCHDOG_TIMEOUT_SCREEN_NAME)

        # ADD IMAGE FOR STOPPED STATE
        stopped_label = MDLabel(
            text="Are you there?",
            font_style="H4",
            halign="center",
            pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": cfg.SCREEN_Y_CENTER},
        )
        stopped_label.font_size = "100sp"

        screen.add_widget(stopped_label)
        screen.add_widget(
            MDRectangleFlatButton(
                text="Yes",
                font_style="H4",
                pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": 0.2},
                size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
                on_release=self.exit_wd,
            )
        )

        return screen
