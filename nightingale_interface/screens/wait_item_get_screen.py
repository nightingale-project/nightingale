from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image
from kivy.uix.videoplayer import VideoPlayer

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg


class WaitItemGetScreen:
    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = "estoppedscreen"

    def retract_arm(self, button_data):
        # publishes message to stop to retract arm
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.current = "retractarmscreen"

    def wait_item_get_build(self):
        screen = Screen(name="waititemgetscreen")

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

        # retract arm
        screen.add_widget(
            MDRectangleFlatButton(
                text="Finish",
                font_style="H4",
                pos_hint={"center_x": 0.85, "center_y": 0.15},
                size_hint=(0.4, 0.12),
                on_release=self.retract_arm,
            )
        )

        # Video player of robot moving
        # screen.add_widget(
        #    VideoPlayer(
        #        source='wait_item_get.mkv',
        #        state='play',
        #        size_hint_x=0.15,
        #        pos_hint={"center_x": 0.3, "center_y": 0.5},
        # )

        return screen
