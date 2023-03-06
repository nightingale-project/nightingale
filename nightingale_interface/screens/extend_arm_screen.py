from kivy.uix.screenmanager import Screen
from kivy.uix.video import Video
from kivy.uix.button import Button
from kivy.uix.screenmanager import SlideTransition, NoTransition
from kivy.uix.popup import Popup
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.clock import Clock

from kivymd.uix.button import MDRectangleFlatButton

from screens.screen_config import ScreenConfig as cfg
from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import (
    UserInputs,
)


class ExtendArmScreen:
    def cancel_extend_arm(self, button_data):
        pass
        # publishes message to stop to retract arm
        # button_data.parent.manager.transition = NoTransition()
        # cfg.last_screen = button_data.parent.manager.current
        # cfg.pending_action = UserInputs.START_RETRACT_ARM
        # button_data.parent.manager.current = cfg.CONFIRMATION_SCREEN_NAME

    def extend_arm(self, button_data):
        # starts robot arm extend when patient is ready
        # button_data.parent.manager.transition = NoTransition()
        # self.pending_action = UserInputs.START_EXTEND_ARM
        # button_data.parent.manager.current = cfg.CONFIRMATION_SCREEN_NAME
        self.screen_stack.append(button_data.parent.manager.current)
        self.reset_wd()
        # respond to M.P to extend arm
        self.call_ros_action(UserInputs.START_EXTEND_ARM)

        # popup feedback
        layout = GridLayout(cols=1, padding=10)
        popupLabel = Label(text="Extending arm!\nPlease wait until arm is extended")
        layout.add_widget(popupLabel)
        # Instantiate the modal popup and display
        popup = Popup(
            title="Nightingale Action Center",
            content=layout,
            size_hint=(None, None),
            size=(300, 100),
            pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": 0.9},
        )
        popup.open()
        # Schedule pop up auto dismiss for 2 seconds
        Clock.schedule_once(popup.dismiss, 2)

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
        # screen.add_widget(
        #    MDRectangleFlatButton(
        #        text="Cancel",
        #        font_style="H4",
        #        pos_hint={"center_x": 0.85, "center_y": 0.35},
        #        size_hint=(cfg.SHORT_RECT_WIDTH, cfg.SHORT_RECT_HEIGHT),
        #        on_release=self.cancel_extend_arm,
        #    )
        # )

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
                source="videos/arm_extendx3.mp4",
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
