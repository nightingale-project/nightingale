from kivy.uix.screenmanager import Screen
from kivy.uix.screenmanager import SlideTransition, NoTransition
from kivy.uix.video import Video
from kivy.uix.button import Button

from kivymd.uix.button import MDRectangleFlatButton

from screens.screen_config import ScreenConfig as cfg
from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import (
    UserInputs,
)


class RetractArmScreen:
    def retract_arm(self, button_data):
        # publishes message to stop to retract arm
        # button_data.parent.manager.transition = NoTransition()
        self.screen_stack.append(button_data.parent.manager.current)
        # self.pending_action = UserInputs.START_RETRACT_ARM
        # button_data.parent.manager.current = cfg.CONFIRMATION_SCREEN_NAME
        # respond to M.P to extend arm
        self.call_ros_action(UserInputs.START_RETRACT_ARM)
        # show pop up

    def retract_arm_build(self):
        screen = Screen(name=cfg.RETRACT_ARM_SCREEN_NAME)

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

        # start extend arm
        screen.add_widget(
            MDRectangleFlatButton(
                text="Start retract arm",
                font_style="H4",
                pos_hint={"center_x": 0.85, "center_y": 0.15},
                size_hint=(cfg.SHORT_RECT_WIDTH, cfg.SHORT_RECT_HEIGHT),
                on_release=self.retract_arm,
            )
        )

        # Video player of robot moving
        screen.add_widget(
            Video(
                source="videos/clock.mp4",
                state="play",
                options={"eos": "loop"},
                size_hint_x=cfg.VIDEO_PLAYER_WIDTH,
                size_hint_y=cfg.VIDEO_PLAYER_HEIGHT,
                pos_hint={
                    "center_x": cfg.VIDEO_PLAYER_XPOS,
                    "center_y": cfg.VIDEO_PLAYER_YPOS,
                },
            )
        )

        return screen
