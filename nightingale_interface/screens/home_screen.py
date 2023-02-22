from kivy.uix.screenmanager import Screen
from kivy.uix.button import Button
from kivy.uix.screenmanager import SlideTransition, NoTransition

from kivymd.uix.button import MDRectangleFlatButton
from screens.screen_config import ScreenConfig as cfg

# TEMPORARY UNTIL INCLUDES ARE FIXED
import sys

sys.path.append("..")
# TEMPORARY UNTIL INCLUDES ARE FIXED

from nightingale_ros_bridge.src.nightingale_ros_bridge.bridge_interface_config import (
    UserInputs,
)


class HomeScreen:
    def estop(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        self.screen_stack.append(button_data.parent.manager.current)
        self.reset_wd()
        # engage estop on ROS topic
        self.call_ros_action(UserInputs.ESTOP)
        button_data.parent.manager.current = cfg.ESTOP_SCREEN_NAME

    def to_nurse_alert(self, button_data):
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "left"
        self.reset_wd()
        self.screen_stack.append(button_data.parent.manager.current)
        button_data.parent.manager.current = cfg.NURSE_ALERT_SCREEN_NAME

    def to_video_call(self, button_data):
        # publish to topic to tell robot not to move
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "left"
        self.reset_wd()
        self.screen_stack.append(button_data.parent.manager.current)
        button_data.parent.manager.current = cfg.VIDEO_CALL_SCREEN_NAME
        # open web browser to specific size of scree

    def to_item_select(self, button_data):
        # publish to topic to tell robot not to move
        button_data.parent.manager.transition = SlideTransition()
        button_data.parent.manager.transition.direction = "left"
        self.reset_wd()
        self.screen_stack.append(button_data.parent.manager.current)
        button_data.parent.manager.current = cfg.ITEM_SELECT_SCREEN_NAME

    def dismiss(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        button_data.parent.manager.transition.direction = "left"
        self.reset_wd()
        self.screen_stack.append(button_data.parent.manager.current)
        self.pending_action = UserInputs.RETURN_HOME
        button_data.parent.manager.current = cfg.CONFIRMATION_SCREEN_NAME

    def to_admin_control(self, button_data):
        button_data.parent.manager.transition = NoTransition()
        self.reset_wd()
        self.screen_stack.append(button_data.parent.manager.current)
        button_data.parent.manager.current = cfg.ADMIN_SCREEN_NAME

    def home_build(self):
        screen = Screen(name=cfg.HUB_SCREEN_NAME)

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

        screen.add_widget(
            MDRectangleFlatButton(
                text="Request Items",
                font_style="H4",
                pos_hint={
                    "center_x": cfg.SCREEN_X_CENTER,
                    "center_y": cfg.SCREEN_Y_CENTER,
                },
                size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
                on_release=self.to_item_select,
            )
        )

        screen.add_widget(
            MDRectangleFlatButton(
                text="Dismiss",
                font_style="H4",
                pos_hint={"center_x": 0.125, "center_y": 0.1},
                size_hint=(cfg.SHORT_RECT_WIDTH, cfg.SHORT_RECT_HEIGHT),
                on_release=self.dismiss,
            )
        )

        # screen.add_widget(
        #    MDRectangleFlatButton(
        #        text="Admin Control",
        #        font_size=cfg.CANCEL_BUTTON_FONTSIZE,
        #        pos_hint={"center_x": 0.125, "center_y": 0.9},
        #        size_hint=(cfg.SHORT_RECT_WIDTH, cfg.SHORT_RECT_HEIGHT),
        #        on_release=self.to_admin_control,
        #    )
        # )

        # screen.add_widget(
        #    MDRectangleFlatButton(
        #        text="Chat with Nurse",
        #        font_style="H4",
        #        pos_hint={
        #            "center_x": cfg.SCREEN_X_CENTER,
        #            "center_y": cfg.SCREEN_Y_CENTER,
        #        },
        #        size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
        #        on_release=self.to_video_call,
        #    )
        # )

        # screen.add_widget(
        #    MDRectangleFlatButton(
        #        text="Nurse Assistance",
        #        font_style="H4",
        #        pos_hint={"center_x": cfg.SCREEN_X_CENTER, "center_y": 0.75},
        #        size_hint=(cfg.LONG_RECT_WIDTH, cfg.LONG_RECT_HEIGHT),
        #        on_release=self.to_nurse_alert,
        #    )
        # )

        return screen
