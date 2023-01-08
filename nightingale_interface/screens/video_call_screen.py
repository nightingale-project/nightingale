from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg


class VideoCallScreen:
    video_call_name = "videocallscreen"

    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        cfg.LAST_SCREEN = self.video_call_name
        button_data.parent.manager.current = "estoppedscreen"

    def exit_call(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        cfg.LAST_SCREEN = self.video_call_name
        button_data.parent.manager.current = "homescreen"

    def video_call_build(self):
        screen = Screen(name=self.video_call_name)

        # estop button
        screen.add_widget(
            Button(
                background_normal="images/stop.png",
                size_hint_x=cfg.ESTOP_XHINT,
                size_hint_y=cfg.ESTOP_YHINT,
                pos_hint={"center_x": cfg.ESTOP_XPOS, "center_y": cfg.ESTOP_YPOS},
                on_release=self.estop,
            )
        )

        # exit call screen
        screen.add_widget(
            MDRectangleFlatButton(
                text="Exit",
                font_size=cfg.CANCEL_BUTTON_FONTSIZE,
                pos_hint={"center_x": 0.125, "center_y": 0.9},
                size_hint=(0.2, 0.1),
                on_release=self.exit_call,
            )
        )

        return screen
