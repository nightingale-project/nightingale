from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image
from kivy.uix.videoplayer import VideoPlayer

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg


class RetractArmScreen:
    retract_arm_name = "retractarmscreen"
    # wait for rostopic published of when arm is normal state
    # and switch screens on that

    def retract_arm_build(self):
        screen = Screen(name=self.retract_arm_name)

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

        # Video player of robot moving
        # screen.add_widget(
        #    VideoPlayer(
        #        source='retract_arm.mkv',
        #        state='play',
        #        size_hint_x=0.15,
        #        pos_hint={"center_x": 0.3, "center_y": 0.5},
        # )

        return screen