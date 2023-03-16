from kivy.uix.screenmanager import Screen
from kivy.uix.button import Button
from kivy.uix.screenmanager import SlideTransition, NoTransition
from kivy.uix.popup import Popup
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.clock import Clock

from kivymd.uix.button import MDRectangleFlatButton
from screens.screen_config import ScreenConfig as cfg

# TEMPORARY UNTIL INCLUDES ARE FIXED
import sys

sys.path.append("..")
# TEMPORARY UNTIL INCLUDES ARE FIXED

from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import (
    UserInputs,
)


class SymposiumHomeScreen:
    def exit(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = cfg.FACE_SCREEN_NAME

    def arm_extend(self, button_data):
        # set action to extend
        self.call_ros_action(UserInputs.START_EXTEND_ARM)

    def arm_retract(self, button_data):
        # set action to extend
        self.call_ros_action(UserInputs.START_RETRACT_ARM)

    def symposium_home_build(self):
        screen = Screen(name=cfg.SYMPOSIUM_HOME_SCREEN_NAME)

        screen.add_widget(
            MDRectangleFlatButton(
                text="EXTEND ARM",
                font_style="H4",
                pos_hint={
                    "center_x": cfg.SCREEN_X_CENTER,
                    "center_y": cfg.SCREEN_Y_CENTER,
                },
                size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
                on_release=self.arm_extend,
            )
        )
        screen.add_widget(
            MDRectangleFlatButton(
                text="RETRACT ARM",
                font_style="H4",
                pos_hint={
                    "center_x": cfg.SCREEN_X_CENTER,
                    "center_y": cfg.SCREEN_Y_CENTER - 0.25,
                },
                size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
                on_release=self.arm_retract,
            )
        )

        screen.add_widget(
            MDRectangleFlatButton(
                text="Exit",
                font_style="H4",
                pos_hint={"center_x": 0.125, "center_y": 0.1},
                size_hint=(cfg.SHORT_RECT_WIDTH, cfg.SHORT_RECT_HEIGHT),
                on_release=self.exit,
            )
        )


        return screen



