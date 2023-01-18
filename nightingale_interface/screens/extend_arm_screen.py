from kivy.uix.screenmanager import Screen
from kivy.uix.image import Image
from kivy.uix.video import Video


from kivymd.uix.label import MDLabel
from kivymd.uix.button import MDRectangleFlatButton
from kivy.uix.button import Button

from kivy.uix.screenmanager import SlideTransition, NoTransition
from screens.screen_config import ScreenConfig as cfg
from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import UserInputs

class ExtendArmScreen:

    def cancel_extend_arm(self, button_data):
        pass
        # publishes message to stop to retract arm
        #button_data.parent.manager.transition = NoTransition()
        #cfg.last_screen = button_data.parent.manager.current
        #cfg.pending_action = UserInputs.START_RETRACT_ARM
        #button_data.parent.manager.current = cfg.CONFIRMATION_SCREEN_NAME

    def extend_arm(self, button_data):
        # starts robot arm extend when patient is ready
        #button_data.parent.manager.transition = NoTransition()
        #cfg.last_screen = button_data.parent.manager.current
        #cfg.pending_action = UserInputs.START_EXTEND_ARM
        #button_data.parent.manager.current = cfg.CONFIRMATION_SCREEN_NAME

        # respond to M.P to extend arm
        self.call_ros_action(UserInputs.START_EXTEND_ARM)
        # show pop up

    def extend_arm_build(self):
        screen = Screen(name=cfg.EXTEND_ARM_SCREEN_NAME)

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
                on_release=self.cancel_extend_arm,
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
        screen.add_widget(
            Video(
                source='videos/clock.mp4',
                state='play',
                options = {'eos': 'loop'},
                size_hint_x=cfg.VIDEO_PLAYER_WIDTH,
                size_hint_y=cfg.VIDEO_PLAYER_HEIGHT,
                pos_hint={"center_x": cfg.VIDEO_PLAYER_XPOS, "center_y": cfg.VIDEO_PLAYER_YPOS},
            )
        )

        return screen
