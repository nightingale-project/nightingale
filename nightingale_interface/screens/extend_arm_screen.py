from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image
from kivy.uix.videoplayer import VideoPlayer

from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg


class ExtendArmScreen:
    extend_arm_name = "extendarmscreen"

    # when arm is extended to goal rostopic should publish to tell interface to switch to waititemget screen

    def retract_arm(self, button_data):
        # publishes message to stop to retract arm
        button_data.parent.manager.transition = NoTransition()
        cfg.last_screen = button_data.parent.manager.current
        cfg.pending_action = cfg.RETRACT_ARM
        button_data.parent.manager.current = "confirmation_screen"

    def extend_arm(self, button_data):
        # starts robot arm extend when patient is ready
        button_data.parent.manager.transition = NoTransition()
        cfg.last_screen = button_data.parent.manager.current
        cfg.pending_action = cfg.EXTEND_ARM
        button_data.parent.manager.current = "confirmationscreen"

    def extend_arm_build(self):
        screen = Screen(name=self.extend_arm_name)

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
                text="Cancel",
                font_style="H4",
                pos_hint={"center_x": 0.85, "center_y": 0.35},
                size_hint=(cfg.SHORT_RECT_WIDTH, cfg.SHORT_RECT_HEIGHT),
                on_release=self.retract_arm,
            )
        )

        # start extend arm
        screen.add_widget(
            MDRectangleFlatButton(
                text="Start",
                font_style="H4",
                pos_hint={"center_x": 0.85, "center_y": 0.15},
                size_hint=(cfg.SHORT_RECT_WIDTH, cfg.SHORT_RECT_HEIGHT),
                on_release=self.extend_arm,
            )
        )

        # Video player of robot moving
        # screen.add_widget(
        #    VideoPlayer(
        #        source='extend_arm.mkv',
        #        state='play',
        #        size_hint_x=0.15,
        #        pos_hint={"center_x": 0.3, "center_y": 0.5},
        # )

        return screen
